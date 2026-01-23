#!/usr/bin/env python3

import io
import math
import os
import socket
import threading

import carla
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from PIL.PngImagePlugin import PngInfo
import rclpy
from rclpy.node import Node
from scipy.ndimage import map_coordinates

# Constants
COLOR_ALUMINIUM = (186, 189, 182)
COLOR_ROAD = (46, 52, 54)
COLOR_WHITE = (255, 255, 255)

BUFFER_SIZE = 1024

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
        This class uses a cache system; if the OpenDrive content of a Carla town has not changed,
        it reuses the previously rendered image.

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
    ):
        """
        Initialize the CarlaMapper.

        Render the map image based on the world, its map, and additional flags that provide extra
        information about the road network.
        """
        self.logger = logger

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
        self.min_meters_per_pixel = (
            EARTH_CIRCUMFERENCE
            * math.cos(math.radians(EARTH_REF_LATITUDE))
            / ((2**self.earth_max_zoom_level) * TILE_SIZE)
        )
        self.aerial_view_camera_fov = aerial_view_camera_fov
        self.aerial_view_camera_height = aerial_view_camera_height
        self.aerial_view_camera_resolution = aerial_view_camera_resolution

        # Build path for saving or loading the cached rendered map
        filename = town + '_' + str(self.earth_max_zoom_level) + '.png'
        dirname = os.path.join('carla_map')
        full_path = str(os.path.join(dirname, filename))

        if os.path.isfile(full_path) and not reset_base_map_image:
            # Load Image
            self.map_image = Image.open(full_path)
            metadata = self.map_image.text
            world_bounds_string = metadata['world_bounds']
            min_x, max_x, min_y, max_y = tuple(float(x) for x in world_bounds_string.split(', '))

            self.world_bounds = (min_x, max_x, min_y, max_y)
            self.width = max_x - min_x
            self.height = max_y - min_y
            self.world_offset = (min_x, min_y)
            self.width_in_pixels = math.ceil(self.width / self.min_meters_per_pixel)
            self.height_in_pixels = math.ceil(self.height / self.min_meters_per_pixel)
        else:
            self.logger.info('Connecting to carla server to create Map image.')

            self.client = carla.Client(host, port)
            self.client.set_timeout(timeout)
            carla_world = self.client.get_world()
            carla_map = carla_world.get_map()
            carla_map_name = carla_map.name.split('/')[-1]
            if town != carla_map_name:
                raise ValueError(
                    f'Carla map "{carla_map_name}" does not match the town param: {town}'
                )

            waypoints = carla_map.generate_waypoints(1)
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

            self.world_bounds = (min_x, max_x, min_y, max_y)
            self.width = max_x - min_x
            self.height = max_y - min_y
            self.world_offset = (min_x, min_y)
            self.width_in_pixels = math.ceil(self.width / self.min_meters_per_pixel)
            self.height_in_pixels = math.ceil(self.height / self.min_meters_per_pixel)

            self.logger.info('Resetting the weather to ClearNoon to capture aerial images.')
            clear_noon_weather = carla.WeatherParameters.ClearNoon
            carla_world.set_weather(clear_noon_weather)

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

            # Render road network map
            self.draw_road_map(carla_world, carla_map)

            self.logger.info('Completed rendering map image.')

            # If folders path does not exist, create it
            if not os.path.exists(dirname):
                os.makedirs(dirname)

            # Save rendered map for next executions of same map
            self.map_image = self.map_image.convert('RGB')

            metadata = PngInfo()
            world_bounds_string = ', '.join(f'{x:.{precision_digits}f}' for x in self.world_bounds)
            metadata.add_text('world_bounds', world_bounds_string)
            self.logger.info(f'Saving the map image to {full_path}')

            self.map_image.save(full_path, format='PNG', pnginfo=metadata)

        self.meters_per_degree_lat = METERS_PER_DEG_LATITUDE
        self.earth_ref_lat = EARTH_REF_LATITUDE
        self.earth_ref_lon = EARTH_REF_LONGITUDE
        self.carla_world_center = self.pixel_to_carla_world(
            (self.map_image.width / 2, self.map_image.height / 2)
        )
        self.carla_world_origin_offset = carla.Location(
            carla_world_origin_offset[0], carla_world_origin_offset[1], 0
        )

        map_tile = Image.new('RGB', (TILE_SIZE, TILE_SIZE), COLOR_WHITE)
        self.draw_text_on_tile(map_tile, 'Out of Range')
        self.out_of_range_tile = map_tile

        self.carla_max_zoom_level = self.earth_max_zoom_level - math.floor(
            math.log2(EARTH_CIRCUMFERENCE / (10 * max(self.width, self.height)))
        )

    def capture_aerial_view(self, carla_world):
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

            x = min_x + image_width_meters / 2
            while x <= max_x + image_width_meters:
                y = min_y + image_width_meters / 2
                while y <= max_y + image_width_meters:
                    location = carla.Location(x, y, camera_height)
                    spawn_locations.append(location)
                    y += image_height_meters
                x += image_width_meters

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

        rgb_camera_bp = carla_world.get_blueprint_library().find('sensor.camera.rgb')
        rgb_camera_bp.set_attribute('image_size_x', str(self.aerial_view_camera_resolution))
        rgb_camera_bp.set_attribute('image_size_y', str(self.aerial_view_camera_resolution))
        rgb_camera_bp.set_attribute('fov', str(self.aerial_view_camera_fov))

        depth_camera_bp = carla_world.get_blueprint_library().find('sensor.camera.depth')
        depth_camera_bp.set_attribute('image_size_x', str(self.aerial_view_camera_resolution))
        depth_camera_bp.set_attribute('image_size_y', str(self.aerial_view_camera_resolution))
        depth_camera_bp.set_attribute('fov', str(self.aerial_view_camera_fov))

        # Get spawn locations to cover the whole world
        camera_spawn_locations = get_camera_spawn_locations(
            image_width_meters, image_width_meters, self.aerial_view_camera_height
        )
        num_camera_spawn_locations = len(camera_spawn_locations)
        logger.info(
            f'Number of images to cover the world at {self.aerial_view_camera_height}m '
            f'camera height: {num_camera_spawn_locations}'
        )

        # Define a dictionary to store captured images
        images = {'rgb': None, 'depth': None}

        # Create pixel coordinates
        y, x = np.indices((self.aerial_view_camera_resolution, self.aerial_view_camera_resolution))
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

        for index, location in enumerate(camera_spawn_locations):
            transform = carla.Transform(location, carla.Rotation(pitch=-90.0, yaw=-90.0, roll=0.0))
            logger.info(
                f'Capturing image {index + 1}/{num_camera_spawn_locations} at {transform.location}'
            )
            # Update camera transform
            rgb_camera = carla_world.spawn_actor(rgb_camera_bp, transform)
            depth_camera = carla_world.spawn_actor(depth_camera_bp, transform)

            def on_rgb_image_received(image):
                nonlocal images
                # Convert CARLA image to NumPy array
                img_array = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
                    (image.height, image.width, 4)
                )
                images['rgb'] = img_array[:, :, [2, 1, 0, 3]]  # Reorder BGRA to RGBA

            def on_depth_image_received(image):
                nonlocal images
                # Convert CARLA image to NumPy array
                img_array = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
                    (image.height, image.width, 4)
                )
                img_array = img_array[:, :, [2, 1, 0, 3]]  # Reorder BGRA to RGBA

                # Split the image into R, G, B channels
                r_channel = img_array[:, :, 0].astype(np.float32)
                g_channel = img_array[:, :, 1].astype(np.float32)
                b_channel = img_array[:, :, 2].astype(np.float32)

                # Decode depth
                normalized_depth = (r_channel + g_channel * 256 + b_channel * 256 * 256) / (
                    256 * 256 * 256 - 1
                )
                images['depth'] = 1000 * normalized_depth  # Convert to meters

            rgb_camera.listen(on_rgb_image_received)
            depth_camera.listen(on_depth_image_received)

            # Wait for images to be captured
            images['rgb'] = None
            images['depth'] = None
            while images['rgb'] is None or images['depth'] is None:
                carla_world.wait_for_tick()

            # Remove the cameras after capturing image
            rgb_camera.destroy()
            depth_camera.destroy()

            ortho_image = perspective_to_orthographic(
                self.aerial_view_camera_resolution,
                images['rgb'],
                images['depth'],
                image_plane_coords,
                intrinsic_matrix,
            )
            img_pil = Image.fromarray(ortho_image, 'RGBA')

            position = self.carla_world_to_pixel(
                transform.location,
                offset=image_center,
                meters_per_pixel=self.aerial_view_camera_image_meters_per_pixel,
            )
            Image.Image.paste(base_image, img_pil, position)

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
        base_image = base_image.crop((0, 0, corrected_width - 1, corrected_height - 1))
        base_image = base_image.resize(
            (self.width_in_pixels, self.height_in_pixels), Image.Resampling.LANCZOS
        )
        self.map_image = base_image

    def draw_road_map(self, carla_world, carla_map, add_traffic_signs=False):
        """Draw all roads, including lane markings, arrows, and traffic signs."""
        # Adapted from
        # https://github.com/carla-simulator/carla/blob/b23c01ae4a3bd3ec15
        # 01084db10f889d424cfadf/PythonAPI/examples/no_rendering_mode.py#L727

        logger = self.logger
        base_image = self.map_image
        road_map_image = Image.new('RGBA', base_image.size, color=(255, 255, 255, 0))
        draw = ImageDraw.Draw(road_map_image)
        precision = 0.01

        def draw_traffic_signs(draw, font, actor, color=COLOR_ALUMINIUM):
            """Draw stop and yield traffic signs and their bounding boxes if enabled."""
            transform = actor.get_transform()
            waypoint = carla_map.get_waypoint(transform.location)

            angle = -waypoint.transform.rotation.yaw - 90.0

            # Rotate the text
            font_surface = Image.new('RGBA', (100, 100))  # Create a dummy surface
            temp_draw = ImageDraw.Draw(font_surface)
            text = 'STOP' if 'stop' in actor.type_id else 'YIELD'
            temp_draw.text((10, 10), text, font=font, fill=color)
            font_surface = font_surface.rotate(angle, expand=1)

            # Compute the pixel position and offset
            pixel_pos = self.carla_world_to_pixel(waypoint.transform.location)
            offset = (
                pixel_pos[0] - font_surface.width // 2,
                pixel_pos[1] - font_surface.height // 2,
            )

            # Paste the rotated text onto the main surface
            draw.bitmap(offset, font_surface, fill=color)

            # Draw line in front of stop
            forward_vector = carla.Location(waypoint.transform.get_forward_vector())
            left_vector = (
                carla.Location(-forward_vector.y, forward_vector.x, forward_vector.z)
                * waypoint.lane_width
                / 2
                * 0.7
            )

            line = [
                (waypoint.transform.location + (forward_vector * 1.5) + (left_vector)),
                (waypoint.transform.location + (forward_vector * 1.5) - (left_vector)),
            ]

            line_pixel = [self.carla_world_to_pixel(p) for p in line]
            draw.line(line_pixel, fill=color, width=2)

        def lateral_shift(transform, shift):
            """Shift the transform laterally based on its forward vector."""
            rotation = carla.Rotation(
                pitch=transform.rotation.pitch,
                yaw=transform.rotation.yaw + 90,
                roll=transform.rotation.roll,
            )
            lateral_vector = carla.Transform(rotation=rotation).get_forward_vector()
            return transform.location + shift * lateral_vector

        # Draw Roads
        logger.info('Drawing road network from opendrive data')
        carla_topology = carla_map.get_topology()
        topology = [x[0] for x in carla_topology]
        topology = sorted(topology, key=lambda w: w.transform.location.z)
        set_waypoints = []

        for waypoint in topology:
            waypoints = [waypoint]

            # Generate waypoints of a road id. Stop when road id differs
            nxt = waypoint.next(precision)
            if len(nxt) > 0:
                nxt = nxt[0]
                while nxt.road_id == waypoint.road_id:
                    waypoints.append(nxt)
                    nxt = nxt.next(precision)
                    if len(nxt) > 0:
                        nxt = nxt[0]
                    else:
                        break
            set_waypoints.append(waypoints)

        road_color_with_alpha = COLOR_ROAD + (128,)
        for waypoints in set_waypoints:
            waypoint = waypoints[0]
            road_left_side = [lateral_shift(w.transform, -w.lane_width * 0.5) for w in waypoints]
            road_right_side = [lateral_shift(w.transform, w.lane_width * 0.5) for w in waypoints]

            polygon = road_left_side + list(reversed(road_right_side))
            polygon = [self.carla_world_to_pixel(x) for x in polygon]

            if len(polygon) > 2:
                draw.polygon(polygon, fill=road_color_with_alpha)

        if add_traffic_signs:
            actors = carla_world.get_actors()
            # Find and Draw Traffic Signs: Stops and Yields
            font_size = int(1.0 / self.min_meters_per_pixel * 1)
            font = ImageFont.truetype('arial.ttf', font_size)

            stops = [actor for actor in actors if 'stop' in actor.type_id]
            yields = [actor for actor in actors if 'yield' in actor.type_id]

            for ts_stop in stops:
                draw_traffic_signs(draw, font, ts_stop)

            for ts_yield in yields:
                draw_traffic_signs(draw, font, ts_yield)

        self.map_image = Image.alpha_composite(base_image, road_map_image)

    def get_map_tile(self, x, y, zoom):
        def tilex2long_deg(x, z):
            return x / (2.0**z) * 360.0 - 180.0

        def tiley2lat_deg(y, z):
            n = math.pi - 2.0 * math.pi * y / (2.0**z)
            return math.degrees(math.atan(math.sinh(n)))

        carla_zoom_level = self.carla_max_zoom_level - (self.earth_max_zoom_level - zoom)

        if carla_zoom_level < 0:
            return self.out_of_range_tile

        # Convert tile coordinates to lon/lat
        lon_deg = tilex2long_deg(x, zoom)
        lat_deg = tiley2lat_deg(y, zoom)

        # Convert lon/lat to Carla world coordinates
        carla_location = self.earth_to_carla_transform(lon_deg, lat_deg)

        # Convert Carla world coordinates to pixel coordinates in the map image
        pixel_x, pixel_y = self.carla_world_to_pixel(carla_location)

        # Calculate tile size in pixels for the current zoom level
        tile_size_pixels = TILE_SIZE * (2 ** (self.carla_max_zoom_level - carla_zoom_level))

        # Calculate the bounds of the tile in the map image
        left = pixel_x
        top = pixel_y
        right = left + tile_size_pixels
        bottom = top + tile_size_pixels

        # Check if the tile is completely out of the map bounds
        if (
            left >= self.map_image.width
            or top >= self.map_image.height
            or right <= 0
            or bottom <= 0
        ):
            return self.out_of_range_tile

        # Adjust cropping to stay within image bounds
        crop_left = max(0, left)
        crop_top = max(0, top)
        crop_right = min(self.map_image.width, right)
        crop_bottom = min(self.map_image.height, bottom)

        # Crop the relevant part of the map image
        cropped_image = self.map_image.crop((crop_left, crop_top, crop_right, crop_bottom))

        # Create a new tile and paste the cropped image
        map_tile = Image.new('RGB', (tile_size_pixels, tile_size_pixels), COLOR_WHITE)
        paste_x = max(0, -left)
        paste_y = max(0, -top)
        map_tile.paste(cropped_image, (paste_x, paste_y))

        # Resize to standard tile size
        map_tile = map_tile.resize((TILE_SIZE, TILE_SIZE), Image.Resampling.LANCZOS)

        return map_tile

    def draw_text_on_tile(self, map_tile, text, color_str='red'):
        # Create a drawing context
        draw = ImageDraw.Draw(map_tile)

        # Set the text font
        font_size = 20  # Adjust the size as needed
        try:
            # Load a font, or use the default one if not available
            font = ImageFont.truetype('arial.ttf', font_size)
        except IOError:
            font = ImageFont.load_default()

        # Calculate the size of the text
        bbox = draw.textbbox((0, 0), text, font=font)
        text_width = bbox[2] - bbox[0]
        text_height = bbox[3] - bbox[1]

        # Calculate the position to center the text on the tile
        text_x = (TILE_SIZE - text_width) // 2
        text_y = (TILE_SIZE - text_height) // 2

        # Draw the text on the tile
        draw.text((text_x, text_y), text, fill=color_str, font=font)

    def carla_world_to_pixel(self, location, offset=(0, 0), meters_per_pixel=None):
        if meters_per_pixel is None:
            meters_per_pixel = self.min_meters_per_pixel
        x = (location.x - self.world_offset[0]) / meters_per_pixel
        y = (location.y - self.world_offset[1]) / meters_per_pixel
        return (int(x - offset[0]), int(y - offset[1]))

    def pixel_to_carla_world(self, pixel_location, offset=(0, 0)):
        # Adjust for offset
        x_pixel = pixel_location[0] + offset[0]
        y_pixel = pixel_location[1] + offset[1]

        # Convert from pixel coordinates to world coordinates
        x_world = (x_pixel) * self.min_meters_per_pixel + self.world_offset[0]
        y_world = (y_pixel) * self.min_meters_per_pixel + self.world_offset[1]

        return carla.Location(x_world, y_world, 0)

    def earth_to_carla_transform(self, lon, lat):
        # Offset from reference location
        x_offset_meters = (
            (lon - self.earth_ref_lon)
            * self.meters_per_degree_lat
            * math.cos(math.radians(self.earth_ref_lat))
        )
        y_offset_meters = (lat - self.earth_ref_lat) * self.meters_per_degree_lat

        # Convert offsets to CARLA world coordinates
        carla_x = self.carla_world_origin_offset.x + x_offset_meters
        carla_y = self.carla_world_origin_offset.y - y_offset_meters

        return carla.Location(carla_x, carla_y, 0)

    def carla_to_earth_transform(self, carla_location):
        # Calculate the offsets in meters from the CARLA reference location
        x_offset_meters = carla_location.x - self.carla_world_origin_offset.x
        y_offset_meters = carla_location.y - self.carla_world_origin_offset.y

        # Calculate the longitude and latitude
        lon = (
            x_offset_meters
            / (self.meters_per_degree_lat * math.cos(math.radians(self.earth_ref_lat)))
        ) + self.earth_ref_lon
        lat = self.earth_ref_lat - (y_offset_meters / self.meters_per_degree_lat)

        return lon, lat


class CarlaOsmTileServer(Node):
    """ROS2 node to serve map tiles from a Carla world using OpenStreetMap data."""

    def __init__(self):
        super().__init__('carla_osm_tile_server')

        self.declare_parameter('reset_base_map_image', False)
        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 2000)
        self.declare_parameter('timeout', 2)
        self.declare_parameter('town', 'Town10HD_Opt')

        self.declare_parameter('tcp_server_ip', 'localhost')
        self.declare_parameter('tcp_server_port', 8080)
        self.declare_parameter('carla_world_origin_offset_x', 0.0)
        self.declare_parameter('carla_world_origin_offset_y', 0.0)
        self.declare_parameter('aerial_view_camera_resolution', 1920)
        self.declare_parameter('aerial_view_camera_fov', 5.0)
        self.declare_parameter('aerial_view_camera_height', 1500.0)

        self.reset_base_map_image = (
            self.get_parameter('reset_base_map_image').get_parameter_value().bool_value
        )
        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().integer_value
        self.town = self.get_parameter('town').get_parameter_value().string_value

        self.tcp_server_ip = self.get_parameter('tcp_server_ip').get_parameter_value().string_value
        self.tcp_server_port = (
            self.get_parameter('tcp_server_port').get_parameter_value().integer_value
        )
        self.carla_world_origin_offset_x = (
            self.get_parameter('carla_world_origin_offset_x').get_parameter_value().double_value
        )
        self.carla_world_origin_offset_y = (
            self.get_parameter('carla_world_origin_offset_y').get_parameter_value().double_value
        )
        self.aerial_view_camera_resolution = (
            self.get_parameter('aerial_view_camera_resolution').get_parameter_value().integer_value
        )
        self.aerial_view_camera_fov = (
            self.get_parameter('aerial_view_camera_fov').get_parameter_value().double_value
        )
        self.aerial_view_camera_height = (
            self.get_parameter('aerial_view_camera_height').get_parameter_value().double_value
        )

        self.logger = rclpy.logging.get_logger(self.get_name())

        # Load the map and create a CarlaMapper instance
        carla_world_origin_offset = (
            self.carla_world_origin_offset_x,
            self.carla_world_origin_offset_y,
        )
        self.carla_mapper = CarlaMapper(
            self.host,
            self.port,
            self.timeout,
            self.town,
            self.logger,
            carla_world_origin_offset,
            self.aerial_view_camera_resolution,
            self.aerial_view_camera_fov,
            self.aerial_view_camera_height,
            self.reset_base_map_image,
        )

        # Start TCP server in a separate thread
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        threading.Thread(target=self.start_tcp_server, daemon=True).start()

    def start_tcp_server(self):
        """Start a TCP server to listen for incoming requests for map tiles."""
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.tcp_server_ip, self.tcp_server_port))
        self.socket.listen(1)

        # Log the server address and port
        self.get_logger().info(
            f'Map server is listening on {self.tcp_server_ip}:{self.tcp_server_port}. '
            f'Max. supported zoom level is {self.carla_mapper.earth_max_zoom_level} with '
            f'm/px {self.carla_mapper.min_meters_per_pixel} '
        )

        while True:
            conn, _ = self.socket.accept()
            self.handle_client(conn)

    def handle_client(self, conn):
        """Handle incoming client connections and sends the requested tile."""
        try:
            # Receive the entire request data from the client
            request_data = conn.recv(BUFFER_SIZE).decode().strip()
            if not request_data:
                self.get_logger().warning('Received empty request')
                return

            lines = request_data.split('\r\n')
            if not lines:
                self.get_logger().error('Invalid request format: No lines in request')
                return

            request_line = lines[0]
            parts = request_line.split()
            if len(parts) != 3 or not parts[1].endswith('.png'):
                self.get_logger().error(f'Invalid request format: {request_line}')
                return

            try:
                # Extract zoom, x, and y from the URL
                url_path = parts[1]
                url_parts = url_path.split('/')
                if len(url_parts) != 4:
                    self.get_logger().error(f'Invalid URL path: {url_path}')
                    return

                zoom = int(url_parts[1])
                x = int(url_parts[2])
                y = int(url_parts[3].replace('.png', ''))

                # self.get_logger().error(f'Processing request: {zoom}/{x}/{y}')

                # Generate the requested tile
                map_tile = self.carla_mapper.get_map_tile(x, y, zoom)

                if map_tile:
                    # Convert the tile image to binary
                    with io.BytesIO() as output:
                        map_tile.save(output, format='PNG')
                        image_data = output.getvalue()
                        content_length = len(image_data)

                        # Add header
                        conn.sendall(b'HTTP/1.1 200 OK\r\n')
                        conn.sendall(b'Content-Type: image/png\r\n')
                        conn.sendall(f'Content-Length: {content_length}\r\n'.encode())
                        conn.sendall(b'\r\n')

                        # Send the tile image back to the client
                        conn.sendall(image_data)
                else:
                    raise ValueError('Generated tile is None or invalid.')
            except ValueError as e:
                self.get_logger().error(f'Error processing request: {e}')
            except Exception as e:
                self.get_logger().error(f'Unexpected error: {e}')
        finally:
            conn.close()

    def destroy(self):
        # self._action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    carla_osm_tile_server_node = CarlaOsmTileServer()
    rclpy.spin(carla_osm_tile_server_node)
    carla_osm_tile_server_node.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
