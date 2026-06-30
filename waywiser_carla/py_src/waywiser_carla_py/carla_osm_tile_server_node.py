#!/usr/bin/env python3

import io
import socket
import threading

from PIL import Image, ImageDraw
import rclpy
from rclpy.node import Node

try:
    from waywiser_carla_py.carla_mapper import CarlaMapper
except ImportError:
    from carla_mapper import CarlaMapper

BUFFER_SIZE = 1024
TILE_SIZE = 256  # standard OSM tile size


def _make_out_of_range_tile() -> bytes:
    """Return a 256x256 PNG tile with a white background and 'Out of range' label."""
    img = Image.new('RGBA', (TILE_SIZE, TILE_SIZE), (255, 255, 255, 220))
    draw = ImageDraw.Draw(img)
    # diagonal cross
    draw.line([(0, 0), (TILE_SIZE, TILE_SIZE)], fill=(200, 200, 200, 255), width=1)
    draw.line([(TILE_SIZE, 0), (0, TILE_SIZE)], fill=(200, 200, 200, 255), width=1)
    # label
    text = 'Out of range'
    try:
        bbox = draw.textbbox((0, 0), text)
        tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
    except AttributeError:  # Pillow < 9
        tw, th = draw.textsize(text)
    draw.text(
        ((TILE_SIZE - tw) // 2, (TILE_SIZE - th) // 2),
        text,
        fill=(100, 100, 100, 255),
    )
    buf = io.BytesIO()
    img.save(buf, format='PNG')
    return buf.getvalue()


class CarlaOsmTileServerNode(Node):
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
        self.declare_parameter('carla_bev_view_camera_resolution', 1920)
        self.declare_parameter('carla_bev_view_camera_fov', 5.0)
        self.declare_parameter('carla_bev_view_camera_height', 1500.0)
        self.declare_parameter('enuref', [0.0, 0.0, 0.0])
        self.declare_parameter('rendering_wait_ticks', 10)
        self.declare_parameter('map_region', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('image_processing_workers', 2)
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
        self.carla_bev_view_camera_resolution = (
            self.get_parameter(
                'carla_bev_view_camera_resolution'
            ).get_parameter_value().integer_value
        )
        self.carla_bev_view_camera_fov = (
            self.get_parameter('carla_bev_view_camera_fov').get_parameter_value().double_value
        )
        self.carla_bev_view_camera_height = (
            self.get_parameter('carla_bev_view_camera_height').get_parameter_value().double_value
        )
        self.enuref = self.get_parameter('enuref').get_parameter_value().double_array_value
        self.rendering_wait_ticks = (
            self.get_parameter('rendering_wait_ticks').get_parameter_value().integer_value
        )
        self.map_region = self.get_parameter('map_region').get_parameter_value().double_array_value
        self.image_processing_workers = (
            self.get_parameter('image_processing_workers').get_parameter_value().integer_value
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
            self.carla_bev_view_camera_resolution,
            self.carla_bev_view_camera_fov,
            self.carla_bev_view_camera_height,
            self.reset_base_map_image,
            enuref=self.enuref,
            rendering_wait_ticks=self.rendering_wait_ticks,
            map_region=self.map_region,
            image_processing_workers=self.image_processing_workers,
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
                # self.get_logger().warning('Received empty request')
                return

            lines = request_data.split('\r\n')
            if not lines:
                self.get_logger().error('Invalid request format: No lines in request')
                return

            request_line = lines[0]
            parts = request_line.split()
            if len(parts) != 3:
                self.get_logger().error(f'Invalid request format: {request_line}')
                return

            url_path = parts[1]
            url_parts = [p for p in url_path.split('/') if p]

            if not url_parts:
                return

            # Handle non-tile endpoints
            if url_parts[0] == 'metadata':
                import json

                meta = {
                    'town': self.town,
                    'mpp': self.carla_mapper.min_meters_per_pixel,
                    'world_offset': self.carla_mapper.world_offset,
                    'world_bounds': self.carla_mapper.world_bounds,
                    'width_px': self.carla_mapper.width_in_pixels,
                    'height_px': self.carla_mapper.height_in_pixels,
                    'max_zoom': self.carla_mapper.earth_max_zoom_level,
                    'ref_lat': self.carla_mapper.earth_ref_lat,
                    'ref_lon': self.carla_mapper.earth_ref_lon,
                }
                data = json.dumps(meta).encode()
                conn.sendall(b'HTTP/1.1 200 OK\r\n')
                conn.sendall(b'Content-Type: application/json\r\n')
                conn.sendall(f'Content-Length: {len(data)}\r\n'.encode())
                conn.sendall(b'\r\n')
                conn.sendall(data)
                return

            if not parts[1].endswith('.png'):
                self.get_logger().error(f'Invalid request format (expected .png): {request_line}')
                return

            try:
                # Extract zoom, x, and y from the URL
                layer = 'default'
                if url_parts[0] == 'carla_bev':
                    layer = 'carla_bev'
                    url_parts = url_parts[1:]

                if len(url_parts) != 3:
                    self.get_logger().error(f'Invalid URL path: {url_path}')
                    return

                zoom = int(url_parts[0])
                x = int(url_parts[1])
                y = int(url_parts[2].replace('.png', ''))

                # Generate the requested tile based on the layer
                map_tile = self.carla_mapper.get_map_tile(x, y, zoom, layer=layer)

                if map_tile:
                    # Convert the tile image to binary
                    with io.BytesIO() as output:
                        map_tile.save(output, format='PNG')
                        image_data = output.getvalue()
                else:
                    # Tile is outside the available map area — serve a placeholder
                    image_data = _make_out_of_range_tile()

                content_length = len(image_data)
                conn.sendall(b'HTTP/1.1 200 OK\r\n')
                conn.sendall(b'Content-Type: image/png\r\n')
                conn.sendall(f'Content-Length: {content_length}\r\n'.encode())
                conn.sendall(b'\r\n')
                conn.sendall(image_data)
            except ValueError as e:
                self.get_logger().error(f'Error processing request: {e}')
            except Exception as e:
                self.get_logger().error(f'Unexpected error: {e}')
        finally:
            conn.close()

    def destroy(self):
        try:
            self.socket.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CarlaOsmTileServerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
