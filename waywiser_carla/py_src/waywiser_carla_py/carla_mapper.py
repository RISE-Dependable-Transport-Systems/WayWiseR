#!/usr/bin/env python3

import concurrent.futures
import math
import os
import time

import carla
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from PIL.PngImagePlugin import PngInfo
from scipy.ndimage import map_coordinates

# Constants
COLOR_ALUMINIUM = (186, 189, 182)
COLOR_ROAD = (46, 52, 54)
COLOR_WHITE = (255, 255, 255)

TILE_SIZE = 256  # Standard tile size is 256x256 pixels

EARTH_REF_LATITUDE = 57.71495867
EARTH_REF_LONGITUDE = 12.89134921

EARTH_CIRCUMFERENCE = 40075000  # in meters
METERS_PER_DEG_LATITUDE = 111320
Image.MAX_IMAGE_PIXELS = 250e6


class CarlaMapper(object):
    """
    Render a 2D image from the top view of a Carla world.

    Note:
    ----
        This class uses a cache system; if a rendered aerial image for the given town and zoom
        level already exists on disk, it reuses that image instead of re-capturing it.

    """

    def __init__(
        self,
        host,
        port,
        timeout,
        town,
        logger,
        carla_world_origin_offset=(0, 0),
        aerial_view_camera_resolution=1920,
        aerial_view_camera_fov=5.0,
        aerial_view_camera_height=1500.0,
        reset_base_map_image=False,
        enuref=[0.0, 0.0, 0.0],
        rendering_wait_ticks=10,
        map_region=None,
        image_processing_workers=2,
    ):
        """
        Initialize the CarlaMapper.

        Render the map image based on the world, its map, and additional flags that provide extra
        information about the road network.
        """
        self.logger = logger
        self.host = host
        self.port = port
        self.timeout = timeout
        self.rendering_wait_ticks = rendering_wait_ticks
        self.image_processing_workers = max(1, int(image_processing_workers))

        self.meters_per_degree_lat = METERS_PER_DEG_LATITUDE
        self.carla_world_origin_offset = carla.Location(
            carla_world_origin_offset[0], carla_world_origin_offset[1], 0
        )

        self.map_image = None
        self.aerial_image = None
        self.aerial_fetch_cancelled = False

        aerial_view_camera_focus_length = aerial_view_camera_resolution / (
            2 * math.tan(math.radians(aerial_view_camera_fov) / 2)
        )
        aerial_view_camera_image_width_meters = (
            aerial_view_camera_height * aerial_view_camera_resolution
        ) / aerial_view_camera_focus_length

        self.aerial_view_camera_image_meters_per_pixel = (
            aerial_view_camera_image_width_meters / aerial_view_camera_resolution
        )

        self.earth_max_zoom_level = math.floor(
            math.log2(
                EARTH_CIRCUMFERENCE / (self.aerial_view_camera_image_meters_per_pixel * TILE_SIZE)
            )
        )
        self.aerial_view_camera_fov = aerial_view_camera_fov
        self.aerial_view_camera_height = aerial_view_camera_height
        self.aerial_view_camera_resolution = aerial_view_camera_resolution

        # Build path for saving or loading the cached rendered map
        filename = town + '_' + str(self.earth_max_zoom_level) + '.png'
        dirname = os.path.join('carla_map')
        full_path = str(os.path.join(dirname, filename))

        if os.path.isfile(full_path) and not reset_base_map_image:
            self.map_image = Image.open(full_path)
            self.aerial_image = self.map_image.copy()
            metadata = self.map_image.text

            # Initialize variables from metadata
            world_bounds_string = metadata['world_bounds']
            min_x, max_x, min_y, max_y = tuple(float(x) for x in world_bounds_string.split(', '))
            if 'ref_lat' in metadata and 'ref_lon' in metadata:
                self.earth_ref_lat = float(metadata['ref_lat'])
                self.earth_ref_lon = float(metadata['ref_lon'])
            else:
                self.earth_ref_lat = enuref[0]
                self.earth_ref_lon = enuref[1]

            self.world_bounds = (min_x, max_x, min_y, max_y)
            self.width = max_x - min_x
            self.height = max_y - min_y
            self.world_offset = (min_x, min_y)
            self.min_meters_per_pixel = (
                EARTH_CIRCUMFERENCE
                * math.cos(math.radians(self.earth_ref_lat))
                / ((2**self.earth_max_zoom_level) * TILE_SIZE)
            )
            self.width_in_pixels = math.ceil(self.width / self.min_meters_per_pixel)
            self.height_in_pixels = math.ceil(self.height / self.min_meters_per_pixel)

            self.logger.info(
                f'Loaded cached map from {full_path} with ENU ref: '
                f'Lat={self.earth_ref_lat}, Lon={self.earth_ref_lon}'
            )

            if self.width <= 0 or self.height <= 0:
                raise ValueError(
                    'Invalid cached map bounds produce non-positive image size. '
                    f'world_bounds={self.world_bounds}. '
                    'Delete cached image or launch with reset_base_map_image:=true.'
                )
        else:
            self.logger.info('Connecting to carla server to create Map image.')

            self.client = carla.Client(host, port)
            self.client.set_timeout(timeout)
            carla_world = self._call_with_timeout_retry('client.get_world', self.client.get_world)
            carla_map = self._call_with_timeout_retry('world.get_map', carla_world.get_map)

            carla_georef = carla_map.transform_to_geolocation(carla.Location(0, 0, 0))
            if carla_georef.latitude != 0.0 and carla_georef.longitude != 0.0:
                self.earth_ref_lat = carla_georef.latitude
                self.earth_ref_lon = carla_georef.longitude
                self.logger.info(
                    'Using ENU reference from carla map: '
                    f'Lat={carla_georef.latitude}, Lon={carla_georef.longitude}'
                )
            else:
                self.earth_ref_lat = enuref[0]
                self.earth_ref_lon = enuref[1]
                self.logger.info(
                    f'Using ENU reference from params: Lat={self.earth_ref_lat}, '
                    f'Lon={self.earth_ref_lon}'
                )

            carla_map_name = carla_map.name.split('/')[-1]
            if town != carla_map_name:
                raise ValueError(
                    f'Carla map "{carla_map_name}" does not match the town param: {town}'
                )

            self.min_meters_per_pixel = (
                EARTH_CIRCUMFERENCE
                * math.cos(math.radians(self.earth_ref_lat))
                / ((2**self.earth_max_zoom_level) * TILE_SIZE)
            )

            waypoints = self._call_with_timeout_retry(
                'map.generate_waypoints', lambda: carla_map.generate_waypoints(1)
            )
            margin = 50
            precision_digits = 4

            max_x = round(
                max(waypoints, key=lambda x: x.transform.location.x).transform.location.x + margin,
                precision_digits,
            )
            max_y = round(
                max(waypoints, key=lambda x: x.transform.location.y).transform.location.y + margin,
                precision_digits,
            )
            min_x = round(
                min(waypoints, key=lambda x: x.transform.location.x).transform.location.x - margin,
                precision_digits,
            )
            min_y = round(
                min(waypoints, key=lambda x: x.transform.location.y).transform.location.y - margin,
                precision_digits,
            )
            auto_world_bounds = (min_x, max_x, min_y, max_y)

            if map_region and len(map_region) == 4 and any(v != 0.0 for v in map_region):
                # Extract user bounds
                # Convert User Y-Up (North+, South-) to CARLA Y-Down (North-, South+)
                u_min_y = -map_region[0]
                u_max_x = map_region[1]
                u_max_y = map_region[2]
                u_min_x = -map_region[3]

                min_x, max_x, min_y, max_y = u_min_x, u_max_x, u_min_y, u_max_y

                # Log overlap information for diagnostics only
                overlap_min_x = max(u_min_x, auto_world_bounds[0])
                overlap_max_x = min(u_max_x, auto_world_bounds[1])
                overlap_min_y = max(u_min_y, auto_world_bounds[2])
                overlap_max_y = min(u_max_y, auto_world_bounds[3])
                if overlap_max_x <= overlap_min_x or overlap_max_y <= overlap_min_y:
                    self.logger.warning(
                        f'User map_region {map_region} does not overlap with auto-detected bounds '
                        f'{auto_world_bounds}. The map may be empty.'
                    )
                else:
                    self.logger.info(
                        f'Using user-specified map_region. Overlap with auto bounds: '
                        f'x=[{overlap_min_x:.1f}, {overlap_max_x:.1f}], '
                        f'y=[{overlap_min_y:.1f}, {overlap_max_y:.1f}]'
                    )

            self.world_bounds = (min_x, max_x, min_y, max_y)
            self.logger.info(f'Calculated world bounds: {self.world_bounds}')
            self.width = max_x - min_x
            self.height = max_y - min_y
            if self.width <= 0 or self.height <= 0:
                raise ValueError(
                    'World bounds produce non-positive image size: '
                    f'width={self.width}, height={self.height}. '
                    f'world_bounds={self.world_bounds}'
                )
            self.world_offset = (min_x, min_y)
            self.width_in_pixels = math.ceil(self.width / self.min_meters_per_pixel)
            self.height_in_pixels = math.ceil(self.height / self.min_meters_per_pixel)

            self.logger.info('Resetting the weather to ClearNoon to capture aerial images.')
            clear_noon_weather = carla.WeatherParameters.ClearNoon
            carla_world.set_weather(clear_noon_weather)
            self._call_with_timeout_retry('map.get_topology', carla_map.get_topology)

            self.map_image = Image.new(
                'RGBA',
                (
                    self.width_in_pixels,
                    self.height_in_pixels,
                ),
                color=(255, 255, 255, 0),
            )

            # Capture aerial view
            self.capture_aerial_view(carla_world)

            # Save the rendered map to disk
            os.makedirs(dirname, exist_ok=True)
            metadata = PngInfo()
            metadata.add_text(
                'world_bounds',
                (
                    f'{self.world_bounds[0]}, {self.world_bounds[1]}, '
                    f'{self.world_bounds[2]}, {self.world_bounds[3]}'
                ),
            )
            metadata.add_text('ref_lat', str(self.earth_ref_lat))
            metadata.add_text('ref_lon', str(self.earth_ref_lon))

            self.map_image.save(full_path, format='PNG', pnginfo=metadata)
            self.logger.info(f'Saved map to {full_path}')

        self.carla_world_center = self.pixel_to_carla_world(
            (self.map_image.width / 2, self.map_image.height / 2)
        )

        map_tile = Image.new('RGB', (TILE_SIZE, TILE_SIZE), COLOR_WHITE)
        self.draw_text_on_tile(map_tile, 'Out of Range')
        self.out_of_range_tile = map_tile

        self.carla_max_zoom_level = self.earth_max_zoom_level - math.floor(
            math.log2(EARTH_CIRCUMFERENCE / (10 * max(self.width, self.height)))
        )

    def _call_with_timeout_retry(self, operation_name, operation, retries=5, retry_delay=1.0):
        last_error = None
        for attempt in range(1, retries + 1):
            try:
                return operation()
            except RuntimeError as error:
                last_error = error
            except Exception as error:
                message = str(error).lower()
                if 'timeout' not in message and 'time-out' not in message:
                    raise
                last_error = error

            if attempt < retries:
                self.logger.warning(
                    f'{operation_name} failed ({attempt}/{retries}) due to timeout: {last_error}. '
                    f'Retrying in {retry_delay:.1f}s...'
                )
                time.sleep(retry_delay)

        raise RuntimeError(
            f'{operation_name} failed after {retries} attempts due to timeout: {last_error}'
        )

    def get_carla_world(self):
        """Get the CARLA world object."""
        if not hasattr(self, 'client') or self.client is None:
            self.client = carla.Client(self.host, self.port)
            self.client.set_timeout(self.timeout)
        return self.client.get_world()

    def capture_aerial_view(self, carla_world, progress_callback=None):
        """
        Capture aerial view of the map area.

        progress_callback: function(count, total, partial_image)
        """
        self.aerial_fetch_cancelled = False

        # Initialize an empty image if we don't have one
        if self.aerial_image is None or self.aerial_image.size != (
            self.width_in_pixels,
            self.height_in_pixels,
        ):
            self.aerial_image = Image.new(
                'RGBA', (self.width_in_pixels, self.height_in_pixels), (0, 0, 0, 0)
            )
        # Save original settings
        original_settings = carla_world.get_settings()
        settings = carla_world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        carla_world.apply_settings(settings)

        camera_count_x = math.ceil(self.width_in_pixels / self.aerial_view_camera_resolution)
        camera_count_y = math.ceil(self.height_in_pixels / self.aerial_view_camera_resolution)
        base_image = Image.new(
            'RGBA',
            (
                camera_count_x * self.aerial_view_camera_resolution,
                camera_count_y * self.aerial_view_camera_resolution,
            ),
            color=(255, 255, 255, 0),
        )
        logger = self.logger

        def get_camera_spawn_locations(image_width_meters, image_height_meters, camera_height):
            # Generate waypoints that will cover the whole map area
            min_x, max_x, min_y, max_y = self.world_bounds
            spawn_locations = []

            x_count = max(1, math.ceil((max_x - min_x) / image_width_meters))
            y_count = max(1, math.ceil((max_y - min_y) / image_height_meters))

            x_start = min_x + image_width_meters / 2
            y_start = min_y + image_height_meters / 2

            for ix in range(x_count):
                x = x_start + ix * image_width_meters
                for iy in range(y_count):
                    y = y_start + iy * image_height_meters
                    spawn_locations.append(carla.Location(x, y, camera_height))

            return spawn_locations

        def perspective_to_orthographic(
            camera_resolution,
            rgb_image,
            depth_image,
            image_plane_coords,
            intrinsic_matrix,
            eps=1e-6,
        ):
            world_coords = image_plane_coords * depth_image.flatten()

            ortho_coords = intrinsic_matrix @ world_coords
            ortho_coords[2] = np.where(ortho_coords[2] == 0, eps, ortho_coords[2])
            ortho_coords /= ortho_coords[2]

            ortho_x = ortho_coords[0].reshape(camera_resolution, camera_resolution)
            ortho_y = ortho_coords[1].reshape(camera_resolution, camera_resolution)

            ortho_image = np.zeros_like(rgb_image)
            for c in range(4):  # For each channel (RGBA)
                ortho_image[:, :, c] = map_coordinates(
                    rgb_image[:, :, c], [ortho_y, ortho_x], order=1, mode='constant'
                )
            return ortho_image

        image_width_meters = (
            self.aerial_view_camera_resolution * self.aerial_view_camera_image_meters_per_pixel
        )
        image_center = (
            int(self.aerial_view_camera_resolution / 2),
            int(self.aerial_view_camera_resolution / 2),
        )
        executor = None
        pending_pastes = []
        rgb_camera = None
        depth_camera = None
        rgb_camera_listening = False
        depth_camera_listening = False

        try:
            rgb_camera_bp = carla_world.get_blueprint_library().find('sensor.camera.rgb')
            rgb_camera_bp.set_attribute('image_size_x', str(self.aerial_view_camera_resolution))
            rgb_camera_bp.set_attribute('image_size_y', str(self.aerial_view_camera_resolution))
            rgb_camera_bp.set_attribute('fov', str(self.aerial_view_camera_fov))
            rgb_camera_bp.set_attribute('role_name', 'hero')

            depth_camera_bp = carla_world.get_blueprint_library().find('sensor.camera.depth')
            depth_camera_bp.set_attribute('image_size_x', str(self.aerial_view_camera_resolution))
            depth_camera_bp.set_attribute('image_size_y', str(self.aerial_view_camera_resolution))
            depth_camera_bp.set_attribute('fov', str(self.aerial_view_camera_fov))
            depth_camera_bp.set_attribute('role_name', 'hero')

            # Get spawn locations to cover the whole world
            camera_spawn_locations = get_camera_spawn_locations(
                image_width_meters, image_width_meters, self.aerial_view_camera_height
            )
            num_camera_spawn_locations = len(camera_spawn_locations)
            logger.info(
                f'Number of images to cover the world at {self.aerial_view_camera_height}m '
                f'camera height: {num_camera_spawn_locations}'
            )

            if self.image_processing_workers > 1:
                executor = concurrent.futures.ThreadPoolExecutor(
                    max_workers=self.image_processing_workers
                )

            def flush_completed_pastes(block=False, current_count=0):
                nonlocal pending_pastes
                if not pending_pastes:
                    return

                if block:
                    concurrent.futures.wait(
                        [future for future, _, _ in pending_pastes],
                        return_when=concurrent.futures.FIRST_COMPLETED,
                    )

                remaining = []
                pasted_count = 0
                for future, position, image_index in pending_pastes:
                    if future.done():
                        ortho_image = future.result()
                        img_pil = Image.fromarray(ortho_image, 'RGBA')
                        logger.info(f'Pasting image {image_index}/{num_camera_spawn_locations}...')
                        Image.Image.paste(base_image, img_pil, position)
                        pasted_count += 1
                    else:
                        remaining.append((future, position, image_index))
                pending_pastes = remaining

                if pasted_count > 0:
                    final_crop = base_image.crop(
                        (0, 0, self.width_in_pixels, self.height_in_pixels)
                    )
                    self.aerial_image = final_crop
                    if progress_callback:
                        progress_callback(current_count, num_camera_spawn_locations, final_crop)

            # Define a dictionary to store captured images
            images = {'rgb': None, 'depth': None}

            # Create pixel coordinates
            y, x = np.indices(
                (self.aerial_view_camera_resolution, self.aerial_view_camera_resolution)
            )
            pixel_coords = np.stack((x.flatten(), y.flatten(), np.ones_like(x.flatten())), axis=0)

            intrinsic_matrix = np.identity(3)
            focus_length = self.aerial_view_camera_resolution / (
                2 * np.tan(np.radians(self.aerial_view_camera_fov) / 2)
            )
            intrinsic_matrix[0, 0] = intrinsic_matrix[1, 1] = focus_length
            intrinsic_matrix[0, 2] = self.aerial_view_camera_resolution / 2.0
            intrinsic_matrix[1, 2] = self.aerial_view_camera_resolution / 2.0

            inverse_instrinsic_matrix = np.linalg.inv(intrinsic_matrix)
            image_plane_coords = inverse_instrinsic_matrix @ pixel_coords

            # Spawn cameras once
            initial_transform = carla.Transform(
                camera_spawn_locations[0], carla.Rotation(pitch=-90.0, yaw=-90.0, roll=0.0)
            )
            rgb_camera = carla_world.spawn_actor(rgb_camera_bp, initial_transform)
            depth_camera = carla_world.spawn_actor(depth_camera_bp, initial_transform)

            def on_rgb_image_received(image):
                nonlocal images
                img_array = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
                    (image.height, image.width, 4)
                )
                images['rgb'] = img_array[:, :, [2, 1, 0, 3]]  # Reorder BGRA to RGBA

            def on_depth_image_received(image):
                nonlocal images
                img_array = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
                    (image.height, image.width, 4)
                )
                img_array = img_array[:, :, [2, 1, 0, 3]]  # Reorder BGRA to RGBA

                r_channel = img_array[:, :, 0].astype(np.float32)
                g_channel = img_array[:, :, 1].astype(np.float32)
                b_channel = img_array[:, :, 2].astype(np.float32)

                normalized_depth = (r_channel + g_channel * 256 + b_channel * 256 * 256) / (
                    256 * 256 * 256 - 1
                )
                images['depth'] = 1000 * normalized_depth  # Convert to meters

            rgb_camera.listen(on_rgb_image_received)
            depth_camera.listen(on_depth_image_received)
            rgb_camera_listening = True
            depth_camera_listening = True

            for index, location in enumerate(camera_spawn_locations):
                if self.aerial_fetch_cancelled:
                    break

                transform = carla.Transform(
                    location, carla.Rotation(pitch=-90.0, yaw=-90.0, roll=0.0)
                )
                logger.info(
                    f'Capturing image {index + 1}/{num_camera_spawn_locations} '
                    f'at {transform.location}'
                )

                rgb_camera.set_transform(transform)
                depth_camera.set_transform(transform)

                for _ in range(self.rendering_wait_ticks):
                    if self.aerial_fetch_cancelled:
                        break
                    while True:
                        try:
                            carla_world.tick()
                            break
                        except RuntimeError as e:
                            logger.warn(
                                f'Simulation tick timed out during settling: {e}. Retrying...'
                            )
                            time.sleep(1.0)

                if self.aerial_fetch_cancelled:
                    break

                images['rgb'] = None
                images['depth'] = None

                while images['rgb'] is None or images['depth'] is None:
                    if self.aerial_fetch_cancelled:
                        break
                    try:
                        carla_world.tick()
                    except RuntimeError as e:
                        logger.warn(f'Simulation tick timed out: {e}. Retrying...')
                        time.sleep(1.0)

                if self.aerial_fetch_cancelled:
                    break

                logger.info(f'Processing image {index + 1}/{num_camera_spawn_locations}...')
                position = self.carla_world_to_pixel(
                    transform.location,
                    offset=image_center,
                    meters_per_pixel=self.aerial_view_camera_image_meters_per_pixel,
                )

                if executor is None:
                    ortho_image = perspective_to_orthographic(
                        self.aerial_view_camera_resolution,
                        images['rgb'],
                        images['depth'],
                        image_plane_coords,
                        intrinsic_matrix,
                    )
                    img_pil = Image.fromarray(ortho_image, 'RGBA')
                    logger.info(f'Pasting image {index + 1}/{num_camera_spawn_locations}...')
                    Image.Image.paste(base_image, img_pil, position)
                    final_crop = base_image.crop(
                        (0, 0, self.width_in_pixels, self.height_in_pixels)
                    )
                    self.aerial_image = final_crop
                    if progress_callback:
                        progress_callback(index + 1, num_camera_spawn_locations, final_crop)
                else:
                    future = executor.submit(
                        perspective_to_orthographic,
                        self.aerial_view_camera_resolution,
                        images['rgb'].copy(),
                        images['depth'].copy(),
                        image_plane_coords,
                        intrinsic_matrix,
                    )
                    pending_pastes.append((future, position, index + 1))
                    if len(pending_pastes) >= self.image_processing_workers * 2:
                        flush_completed_pastes(block=True, current_count=index + 1)
                    else:
                        flush_completed_pastes(block=False, current_count=index + 1)

            while pending_pastes:
                flush_completed_pastes(block=True, current_count=num_camera_spawn_locations)

            if not self.aerial_fetch_cancelled:
                corrected_width = (
                    self.width_in_pixels
                    * self.min_meters_per_pixel
                    / self.aerial_view_camera_image_meters_per_pixel
                )
                corrected_height = (
                    self.height_in_pixels
                    * self.min_meters_per_pixel
                    / self.aerial_view_camera_image_meters_per_pixel
                )
                base_image = base_image.crop((0, 0, int(corrected_width), int(corrected_height)))
                base_image = base_image.resize(
                    (self.width_in_pixels, self.height_in_pixels), Image.Resampling.LANCZOS
                )
                self.aerial_image = base_image
                # Initially, map_image is just the aerial view
                self.map_image = self.aerial_image.copy()
            else:
                logger.info('Aerial view capture cancelled.')
        finally:
            if executor is not None:
                executor.shutdown(wait=False)

            if rgb_camera is not None:
                if rgb_camera_listening:
                    try:
                        rgb_camera.stop()
                    except Exception:
                        pass
                try:
                    rgb_camera.destroy()
                except Exception:
                    pass

            if depth_camera is not None:
                if depth_camera_listening:
                    try:
                        depth_camera.stop()
                    except Exception:
                        pass
                try:
                    depth_camera.destroy()
                except Exception:
                    pass

            carla_world.apply_settings(original_settings)

    def get_map_tile(self, x, y, zoom, layer='default'):
        def tilex2long_deg(x, z):
            return x / (2.0**z) * 360.0 - 180.0

        def tiley2lat_deg(y, z):
            n = math.pi - 2.0 * math.pi * y / (2.0**z)
            return math.degrees(math.atan(math.sinh(n)))

        carla_zoom_level = self.carla_max_zoom_level - (self.earth_max_zoom_level - zoom)

        if carla_zoom_level < 0:
            return self.out_of_range_tile

        # Select the target image based on the layer requested
        target_image = self.map_image
        if layer == 'aerial' and self.aerial_image:
            target_image = self.aerial_image

        lon_deg = tilex2long_deg(x, zoom)
        lat_deg = tiley2lat_deg(y, zoom)

        carla_location = self.earth_to_carla_transform(lon_deg, lat_deg)

        pixel_x, pixel_y = self.carla_world_to_pixel(carla_location)

        tile_size_pixels = int(TILE_SIZE * (2 ** (self.carla_max_zoom_level - carla_zoom_level)))

        left = pixel_x
        top = pixel_y
        right = left + tile_size_pixels
        bottom = top + tile_size_pixels

        if left >= target_image.width or top >= target_image.height or right <= 0 or bottom <= 0:
            return self.out_of_range_tile

        crop_left = int(max(0, left))
        crop_top = int(max(0, top))
        crop_right = int(min(target_image.width, right))
        crop_bottom = int(min(target_image.height, bottom))

        cropped_image = target_image.crop((crop_left, crop_top, crop_right, crop_bottom))

        map_tile = Image.new('RGB', (tile_size_pixels, tile_size_pixels), COLOR_WHITE)
        paste_x = int(max(0, -left))
        paste_y = int(max(0, -top))
        map_tile.paste(cropped_image, (paste_x, paste_y))

        map_tile = map_tile.resize((TILE_SIZE, TILE_SIZE), Image.Resampling.LANCZOS)

        return map_tile

    def draw_text_on_tile(self, map_tile, text, color_str='red'):
        draw = ImageDraw.Draw(map_tile)

        font_size = 20
        try:
            font = ImageFont.truetype('arial.ttf', font_size)
        except IOError:
            font = ImageFont.load_default()

        bbox = draw.textbbox((0, 0), text, font=font)
        text_width = bbox[2] - bbox[0]
        text_height = bbox[3] - bbox[1]

        text_x = (TILE_SIZE - text_width) // 2
        text_y = (TILE_SIZE - text_height) // 2

        draw.text((text_x, text_y), text, fill=color_str, font=font)

    def carla_world_to_pixel(self, location, offset=(0, 0), meters_per_pixel=None):
        if meters_per_pixel is None:
            meters_per_pixel = self.min_meters_per_pixel
        x = (location.x - self.world_offset[0]) / meters_per_pixel
        y = (location.y - self.world_offset[1]) / meters_per_pixel
        return (int(x - offset[0]), int(y - offset[1]))

    def pixel_to_carla_world(self, pixel_location, offset=(0, 0)):
        x_pixel = pixel_location[0] + offset[0]
        y_pixel = pixel_location[1] + offset[1]

        x_world = (x_pixel) * self.min_meters_per_pixel + self.world_offset[0]
        y_world = (y_pixel) * self.min_meters_per_pixel + self.world_offset[1]

        return carla.Location(x_world, y_world, 0)

    def earth_to_carla_transform(self, lon, lat):
        x_offset_meters = (
            (lon - self.earth_ref_lon)
            * self.meters_per_degree_lat
            * math.cos(math.radians(self.earth_ref_lat))
        )
        y_offset_meters = (lat - self.earth_ref_lat) * self.meters_per_degree_lat

        carla_x = self.carla_world_origin_offset.x + x_offset_meters
        carla_y = self.carla_world_origin_offset.y - y_offset_meters

        return carla.Location(carla_x, carla_y, 0)

    def carla_to_earth_transform(self, carla_location):
        x_offset_meters = carla_location.x - self.carla_world_origin_offset.x
        y_offset_meters = carla_location.y - self.carla_world_origin_offset.y

        lon = (
            x_offset_meters
            / (self.meters_per_degree_lat * math.cos(math.radians(self.earth_ref_lat)))
        ) + self.earth_ref_lon
        lat = self.earth_ref_lat - (y_offset_meters / self.meters_per_degree_lat)

        return lon, lat
