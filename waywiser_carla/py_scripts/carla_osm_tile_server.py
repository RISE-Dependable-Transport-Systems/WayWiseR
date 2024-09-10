#!/usr/bin/env python3

import glob
import hashlib
import io
import math
import os
import socket
import threading

import carla
import numpy as np
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
import rclpy
from rclpy.node import Node

# Constants
COLOR_ALUMINIUM = (186, 189, 182)
COLOR_ROAD = (46, 52, 54)
COLOR_WHITE = (255, 255, 255)

BUFFER_SIZE = 1024

MAX_ZOOM_LEVEL = 22
TILE_SIZE = 256  # Standard tile size is 256x256 pixels

EARTH_REF_LATITUDE = 57.71495867
EARTH_REF_LONGITUDE = 12.89134921

EARTH_CIRCUMFERENCE = 40075000  # in meters
METERS_PER_DEG_LATITUDE = 111320


class CarlaMapper(object):
    """Class that renders a 2D image from top view of a carla world. Please note that a cache system is used, so if the OpenDrive content
    of a Carla town has not changed, it will read and use the stored image if it was rendered in a previous execution"""

    def __init__(self, carla_world, carla_map, logger, carla_world_origin_offset=(0, 0)):
        """Renders the map image generated based on the world, its map and additional flags that provide extra information about the road network"""
        self.scale = 1.0
        self.logger = logger

        waypoints = carla_map.generate_waypoints(1)
        margin = 50
        max_x = max(waypoints, key=lambda x: x.transform.location.x).transform.location.x + margin
        max_y = max(waypoints, key=lambda x: x.transform.location.y).transform.location.y + margin
        min_x = min(waypoints, key=lambda x: x.transform.location.x).transform.location.x - margin
        min_y = min(waypoints, key=lambda x: x.transform.location.y).transform.location.y - margin

        self.world_bounds = (min_x, max_x, min_y, max_y)
        self.width = max(max_x - min_x, max_y - min_y)
        self._world_offset = (min_x, min_y)

        self.earth_max_zoom_level = MAX_ZOOM_LEVEL
        self.min_meters_per_pixel = EARTH_CIRCUMFERENCE / (
            2**self.earth_max_zoom_level * TILE_SIZE
        )
        self.width_in_pixels = math.ceil(self.width / self.min_meters_per_pixel)
        self.carla_zoom_level_offset = self.earth_max_zoom_level - math.ceil(
            math.log2(self.width / (self.min_meters_per_pixel * TILE_SIZE))
        )

        self.map_image = Image.new(
            'RGBA', (self.width_in_pixels, self.width_in_pixels), color=(255, 255, 255, 0)
        )

        # Load OpenDrive content
        opendrive_content = carla_map.to_opendrive()

        # Get hash based on content
        hash_func = hashlib.sha1()
        hash_func.update(opendrive_content.encode('UTF-8'))
        opendrive_hash = str(hash_func.hexdigest())

        # Build path for saving or loading the cached rendered map
        filename = (
            carla_map.name.split('/')[-1]
            + '_'
            + str(self.width_in_pixels)
            + '_'
            + opendrive_hash
            + '.tga'
        )
        dirname = os.path.join('carla_map')
        full_path = str(os.path.join(dirname, filename))

        if os.path.isfile(full_path):
            # Load Image
            self.map_image = Image.open(full_path)
        else:
            # Capture aerial view
            self.capture_aerial_view(carla_world, int(self.width_in_pixels))

            # Render road network map
            self.draw_road_map(carla_world, carla_map)
            self.map_image = self.map_image.convert('RGB')
            self.logger.info('Completed rendering map image.')
            # If folders path does not exist, create it
            if not os.path.exists(dirname):
                os.makedirs(dirname)

            # Remove files if selected town had a previous version saved
            list_filenames = glob.glob(os.path.join(dirname, carla_map.name) + '*')
            for town_filename in list_filenames:
                os.remove(town_filename)

            # Save rendered map for next executions of same map
            self.map_image.save(full_path)

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

    def capture_aerial_view(self, carla_world, image_res_in_pixels):
        base_image = self.map_image
        logger = self.logger

        def get_camera_spawn_locations(image_width_meters, image_height_meters, camera_height):
            # Generate waypoints that will cover the whole map area
            min_x, max_x, min_y, max_y = self.world_bounds
            spawn_locations = []

            x = min_x
            while x <= max_x + image_width_meters:
                y = min_y
                while y <= max_y + image_width_meters:
                    location = carla.Location(x, y, camera_height)
                    spawn_locations.append(location)
                    y += image_height_meters
                x += image_width_meters

            return spawn_locations

        camera_fov = 10.0
        image_width_meters = image_res_in_pixels * self.min_meters_per_pixel
        camera_height = (image_width_meters / 2) / math.tan(math.radians(camera_fov / 2))
        camera_bp = carla_world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(image_res_in_pixels))
        camera_bp.set_attribute('image_size_y', str(image_res_in_pixels))
        camera_bp.set_attribute('fov', str(camera_fov))

        # Get waypoints to cover the whole world
        camera_spawn_locations = get_camera_spawn_locations(
            image_width_meters, image_width_meters, camera_height
        )
        num_waypoints = len(camera_spawn_locations)
        logger.info(
            f'Number of images to cover the world at {camera_height}m camera height: {num_waypoints}'
        )

        # Spawn the camera and draw the images on the map
        for index, location in enumerate(camera_spawn_locations):
            transform = carla.Transform(location, carla.Rotation(pitch=-90.0, yaw=-90.0, roll=0.0))

            camera = carla_world.spawn_actor(camera_bp, transform)
            logger.info(f'Capturing image {index + 1}/{num_waypoints} at {transform.location}')
            image_captured = False

            def on_image_received(image):
                nonlocal image_captured
                image_captured = True
                # Convert CARLA image to NumPy array
                img_array = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
                    (image.height, image.width, 4)
                )
                img_array = img_array[:, :, [2, 1, 0, 3]]  # Reorder BGRA to RGBA
                img_pil = Image.fromarray(img_array, 'RGBA')

                position = self.carla_world_to_pixel(
                    transform.location,
                    offset=(int(image_res_in_pixels / 2), int(image_res_in_pixels / 2)),
                )
                Image.Image.paste(base_image, img_pil, position)

            camera.listen(on_image_received)

            # Wait for image to be captured
            while not image_captured:
                carla_world.wait_for_tick()

            # Remove the camera after capturing image
            camera.destroy()

    def draw_road_map(self, carla_world, carla_map):
        """Draws all the roads, including lane markings, arrows and traffic signs"""
        # Adapted from
        # https://github.com/carla-simulator/carla/blob/b23c01ae4a3bd3ec1501084db10f889d424cfadf/PythonAPI/examples/no_rendering_mode.py#L727

        logger = self.logger
        base_image = self.map_image
        road_map_image = Image.new('RGBA', base_image.size, color=(255, 255, 255, 0))
        draw = ImageDraw.Draw(road_map_image)
        precision = 0.01

        def draw_traffic_signs(draw, font, actor, color=COLOR_ALUMINIUM):
            """Draw stop traffic signs and its bounding box if enabled"""
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
            """Makes a lateral shift of the forward vector of a transform"""
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

            polygon = road_left_side + [x for x in reversed(road_right_side)]
            polygon = [self.carla_world_to_pixel(x) for x in polygon]

            if len(polygon) > 2:
                draw.polygon(polygon, fill=road_color_with_alpha)

        actors = carla_world.get_actors()

        # Find and Draw Traffic Signs: Stops and Yields
        font_size = int(self.scale / self.min_meters_per_pixel * 1)
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
            return ((x / math.pow(2.0, z)) * 360.0) - 180.0

        def tiley2lat_deg(y, z):
            n = math.pi - (2.0 * math.pi * y / math.pow(2.0, z))
            return (180.0 / math.pi) * math.atan(0.5 * (math.exp(n) - math.exp(-n)))

        carla_zoom_level = zoom - self.carla_zoom_level_offset

        if carla_zoom_level < 0:
            return self.out_of_range_tile
        else:
            lon_deg = tilex2long_deg(x, zoom)
            lat_deg = tiley2lat_deg(y, zoom)
            carla_location = self.earth_to_carla_transform(lon_deg, lat_deg)
            carla_location_pixel_idx = self.carla_world_to_pixel(carla_location)

            tiles_count = 2**carla_zoom_level
            tile_width = math.floor(self.map_image.width / tiles_count)
            tile_height = math.floor(self.map_image.height / tiles_count)

            left = carla_location_pixel_idx[0]
            top = carla_location_pixel_idx[1]
            right = left + tile_width
            bottom = top + tile_height

            if (
                left < -tile_width
                or top < -tile_width
                or left >= self.map_image.width
                or top >= self.map_image.height
            ):
                return self.out_of_range_tile

            map_tile = Image.new('RGB', (tile_width, tile_height), COLOR_WHITE)
            cropped_image = self.map_image.crop(
                (
                    max(left, 0),
                    max(top, 0),
                    min(right, self.map_image.width - 1),
                    min(bottom, self.map_image.height - 1),
                )
            )

            left_px_in_tile = int((abs(left) - left) / 2)
            top_px_in_tile = int((abs(top) - top) / 2)
            map_tile.paste(cropped_image, (left_px_in_tile, top_px_in_tile))

            map_tile = map_tile.resize((TILE_SIZE, TILE_SIZE), Image.ANTIALIAS)

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
        text_width, text_height = draw.textsize(text, font=font)

        # Calculate the position to center the text on the tile
        text_x = (TILE_SIZE - text_width) // 2
        text_y = (TILE_SIZE - text_height) // 2

        # Draw the text on the tile
        draw.text((text_x, text_y), text, fill=color_str, font=font)

    def carla_world_to_pixel(self, location, offset=(0, 0)):
        """Converts the world coordinates to pixel coordinates"""
        x = self.scale * (location.x - self._world_offset[0]) / self.min_meters_per_pixel
        y = self.scale * (location.y - self._world_offset[1]) / self.min_meters_per_pixel
        return (int(x - offset[0]), int(y - offset[1]))

    def pixel_to_carla_world(self, pixel_location, offset=(0, 0)):
        """Converts pixel coordinates to world coordinates"""
        # Adjust for offset
        x_pixel = pixel_location[0] + offset[0]
        y_pixel = pixel_location[1] + offset[1]

        # Convert from pixel coordinates to world coordinates
        x_world = (x_pixel / self.scale) * self.min_meters_per_pixel + self._world_offset[0]
        y_world = (y_pixel / self.scale) * self.min_meters_per_pixel + self._world_offset[1]

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
    def __init__(self):
        super().__init__('carla_osm_tile_server')

        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 2000)
        self.declare_parameter('timeout', 2)

        self.declare_parameter('tcp_server_ip', 'localhost')
        self.declare_parameter('tcp_server_port', 8080)
        self.declare_parameter('carla_world_origin_offset_x', 0.0)
        self.declare_parameter('carla_world_origin_offset_y', 0.0)

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().integer_value

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

        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(self.timeout)
        self.world = self.client.get_world()

        self.logger = rclpy.logging.get_logger(self.get_name())

        # Load the map and create a CarlaMapper instance
        carla_world_origin_offset = (
            self.carla_world_origin_offset_x,
            self.carla_world_origin_offset_y,
        )
        self.carla_mapper = CarlaMapper(
            self.world, self.world.get_map(), self.logger, carla_world_origin_offset
        )

        # Start TCP server in a separate thread
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        threading.Thread(target=self.start_tcp_server, daemon=True).start()

    def start_tcp_server(self):
        """Starts a TCP server to listen for incoming requests for map tiles"""
        self.socket.bind((self.tcp_server_ip, self.tcp_server_port))
        self.socket.listen(1)

        # Log the server address and port
        self.get_logger().info(
            f'Map server is listening on {self.tcp_server_ip}:{self.tcp_server_port}. Max. supported zoom level is {self.carla_mapper.earth_max_zoom_level} with m/px {self.carla_mapper.min_meters_per_pixel} '
        )

        while True:
            conn, _ = self.socket.accept()
            self.handle_client(conn)

    def handle_client(self, conn):
        """Handles incoming client connections and sends the requested tile"""
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
