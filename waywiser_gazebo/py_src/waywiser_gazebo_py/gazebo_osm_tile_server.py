#!/usr/bin/env python3

"""Serve Gazebo top-down renders through an OSM-compatible tile API."""

import io
import json
import math
import os
import shutil
import socket
import threading
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw
from PIL.PngImagePlugin import PngInfo
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication  # noqa: E402

try:
    from waywiser_teleop_py.gazebo_world_map import render_static_gazebo_world  # noqa: E402
except ImportError:
    from gazebo_world_map import render_static_gazebo_world  # type: ignore[no-redef] # noqa: E402


BUFFER_SIZE = 1024
EARTH_RADIUS_M = 6378137.0
TILE_SIZE = 256
BASE_MAP_CACHE_FILENAME = 'base_map.png'
BASE_MAP_CACHE_FORMAT_VERSION = 2
Image.MAX_IMAGE_PIXELS = 250e6


def _make_placeholder_tile(text, background=(255, 255, 255, 220)) -> bytes:
    img = Image.new('RGBA', (TILE_SIZE, TILE_SIZE), background)
    draw = ImageDraw.Draw(img)
    draw.line([(0, 0), (TILE_SIZE, TILE_SIZE)], fill=(200, 200, 200, 255), width=1)
    draw.line([(TILE_SIZE, 0), (0, TILE_SIZE)], fill=(200, 200, 200, 255), width=1)
    try:
        bbox = draw.textbbox((0, 0), text)
        text_width, text_height = bbox[2] - bbox[0], bbox[3] - bbox[1]
    except AttributeError:
        text_width, text_height = draw.textsize(text)
    draw.text(
        ((TILE_SIZE - text_width) // 2, (TILE_SIZE - text_height) // 2),
        text,
        fill=(100, 100, 100, 255),
    )
    buf = io.BytesIO()
    img.save(buf, format='PNG')
    return buf.getvalue()


def _make_out_of_range_tile() -> bytes:
    return _make_placeholder_tile('Out of range')


def _make_busy_tile() -> bytes:
    return _make_placeholder_tile('Map busy', (255, 251, 235, 230))


class GazeboOsmTileServer(Node):
    """ROS 2 node serving Gazebo world renders as OSM-style slippy tiles."""

    def __init__(self):
        super().__init__('gazebo_osm_tile_server')

        self.declare_parameter('world_sdf', '')
        self.declare_parameter('base_map_cache_dir', '')
        self.declare_parameter('reset_base_map_image', False)
        self.declare_parameter('tcp_server_ip', 'localhost')
        self.declare_parameter('tcp_server_port', 8081)
        self.declare_parameter('enuref', [57.71495867, 12.89134921, 0.0])
        self.declare_parameter('max_image_size_px', 4096)
        self.declare_parameter('gazebo_bev_view_camera_fov', 10.0)
        self.declare_parameter('gazebo_bev_view_camera_height', 0.0)
        self.declare_parameter('render_timeout_s', 20.0)
        self.declare_parameter('map_region', [0.0, 0.0, 0.0, 0.0])

        self.world_sdf = self.get_parameter('world_sdf').get_parameter_value().string_value
        self.base_map_cache_dir = (
            self.get_parameter('base_map_cache_dir').get_parameter_value().string_value
        )
        self.reset_base_map_image = (
            self.get_parameter('reset_base_map_image').get_parameter_value().bool_value
        )
        self.tcp_server_ip = self.get_parameter('tcp_server_ip').get_parameter_value().string_value
        self.tcp_server_port = (
            self.get_parameter('tcp_server_port').get_parameter_value().integer_value
        )
        self.enuref = list(self.get_parameter('enuref').get_parameter_value().double_array_value)
        self.max_image_size_px = (
            self.get_parameter('max_image_size_px').get_parameter_value().integer_value
        )
        self.gazebo_bev_view_camera_fov = (
            self.get_parameter('gazebo_bev_view_camera_fov').get_parameter_value().double_value
        )
        self.gazebo_bev_view_camera_height = (
            self.get_parameter('gazebo_bev_view_camera_height').get_parameter_value().double_value
        )
        self.render_timeout_s = (
            self.get_parameter('render_timeout_s').get_parameter_value().double_value
        )
        self.map_region = list(
            self.get_parameter('map_region').get_parameter_value().double_array_value
        )
        self._placeholder_tile = _make_out_of_range_tile()
        self._busy_tile = _make_busy_tile()
        self.processing_status = 'Busy'
        self.processing_detail = 'Starting'
        self.processing_current_capture = 0
        self.processing_total_captures = 0
        self.map_image = None
        self.world_bounds = None
        self.min_meters_per_pixel = 0.0
        self.max_zoom = 0
        self._runtime_cache_dir_notice_logged = False

        self._qt_app = QApplication.instance() or QApplication([])
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        threading.Thread(target=self.start_tcp_server, daemon=True).start()
        self.map_image, self.world_bounds = self._render_world_map()
        self.min_meters_per_pixel = self._min_meters_per_pixel()
        self.max_zoom = self._zoom_for_meters_per_pixel(self.min_meters_per_pixel)
        self.processing_status = 'Ready'
        self.processing_detail = 'Ready'
        self.get_logger().info(
            f'Gazebo OSM tile server ready. Max useful zoom is {self.max_zoom}; '
            f'base m/px is {self.min_meters_per_pixel:.3f}.'
        )

    def _render_world_map(self):
        if not self.world_sdf:
            raise ValueError('world_sdf parameter is required')

        cached_map = self._load_cached_base_map()
        if cached_map is not None:
            return cached_map

        self._clear_cached_tiles()
        rendered_map = render_static_gazebo_world(
            self.world_sdf,
            max_image_size_px=self.max_image_size_px,
            render_timeout_s=self.render_timeout_s,
            camera_fov_deg=self.gazebo_bev_view_camera_fov,
            camera_height_m=self.gazebo_bev_view_camera_height,
            map_region=self.map_region,
            progress_callback=self._render_progress,
        )
        self.rendered_camera_fov_deg = rendered_map.camera_fov_deg
        self.rendered_camera_height_m = rendered_map.camera_height_m
        self.rendered_capture_count = rendered_map.capture_count
        qimage = rendered_map.pixmap.toImage()
        buffer = qimage.bits().asstring(qimage.byteCount())
        image = Image.frombytes(
            'RGBA',
            (qimage.width(), qimage.height()),
            buffer,
            'raw',
            'BGRA',
        )
        self.get_logger().info(
            f'Rendered Gazebo tile base map from {self.world_sdf} using gazebo '
            f'({qimage.width()}x{qimage.height()} px, '
            f'fov={rendered_map.camera_fov_deg:.2f} deg, '
            f'height={rendered_map.camera_height_m:.2f} m, '
            f'captures={rendered_map.capture_count}, '
            f'bounds={tuple(round(value, 3) for value in rendered_map.bounds)}).'
        )
        self._save_cached_base_map(image, rendered_map)
        return image, rendered_map.bounds

    def _render_progress(self, current_capture, total_captures, capture_bounds):
        self.processing_current_capture = int(current_capture)
        self.processing_total_captures = int(total_captures)
        self.processing_detail = f'Capturing image {current_capture}/{total_captures}'
        self.get_logger().info(
            f'Capturing Gazebo BEV image {current_capture}/{total_captures} '
            f'bounds={tuple(round(value, 3) for value in capture_bounds)}'
        )

    def _base_map_cache_path(self):
        cache_dir = os.path.expanduser(os.path.expandvars(self.base_map_cache_dir))
        if cache_dir and self._is_runtime_gazebo_world_cache_dir(cache_dir):
            default_cache_dir = self._default_base_map_cache_dir()
            if default_cache_dir:
                if not self._runtime_cache_dir_notice_logged:
                    self.get_logger().info(
                        f'Ignoring runtime-generated Gazebo cache directory {cache_dir}; '
                        f'using {default_cache_dir}.'
                    )
                    self._runtime_cache_dir_notice_logged = True
                cache_dir = default_cache_dir
        if not cache_dir:
            cache_dir = self._default_base_map_cache_dir()
        if not cache_dir:
            return ''
        return os.path.join(cache_dir, BASE_MAP_CACHE_FILENAME)

    def _base_map_cache_dir(self):
        cache_path = self._base_map_cache_path()
        if not cache_path:
            return ''
        return os.path.dirname(cache_path)

    def _clear_cached_tiles(self):
        cache_dir = self._base_map_cache_dir()
        if not cache_dir or not os.path.isdir(cache_dir):
            return

        removed = 0
        for entry in os.listdir(cache_dir):
            path = os.path.join(cache_dir, entry)
            if os.path.isdir(path) and entry.isdigit():
                try:
                    shutil.rmtree(path)
                    removed += 1
                except OSError as exc:
                    self.get_logger().warning(f'Failed to remove cached tile dir {path}: {exc}')
        if removed:
            self.get_logger().info(
                f'Removed {removed} cached tile zoom directories from {cache_dir}.'
            )

    def _default_base_map_cache_dir(self):
        if not self.world_sdf:
            return ''
        return os.path.join(
            os.environ.get('WAYWISER_WS', os.getcwd()),
            'resources',
            'control_tower',
            'gazebo',
            self._world_name(),
        )

    def _is_runtime_gazebo_world_cache_dir(self, cache_dir):
        cache_name = os.path.basename(os.path.normpath(cache_dir))
        return cache_name.startswith('waywiser_harmonic_')

    def _load_cached_base_map(self):
        cache_path = self._base_map_cache_path()
        if not cache_path:
            self.get_logger().info('Gazebo tile base map cache is disabled.')
            return None
        if self.reset_base_map_image:
            self.get_logger().info(f'Ignoring Gazebo base map cache due reset flag: {cache_path}')
            return None
        if not os.path.isfile(cache_path):
            self.get_logger().info(f'No cached Gazebo tile base map found at {cache_path}.')
            return None

        try:
            cached_image = Image.open(cache_path)
            metadata = cached_image.text
            mismatch = self._cache_metadata_mismatch(metadata)
            if mismatch:
                self.get_logger().info(
                    f'Ignoring stale Gazebo base map cache {cache_path}: {mismatch}'
                )
                return None
            image = cached_image.convert('RGBA')
            world_bounds = tuple(json.loads(metadata['world_bounds']))
            self.rendered_camera_fov_deg = float(metadata.get('camera_fov_deg', 0.0))
            self.rendered_camera_height_m = float(metadata.get('camera_height_m', 0.0))
            self.rendered_capture_count = int(metadata.get('capture_count', 1))
        except Exception as exc:
            self.get_logger().warning(
                f'Ignoring unreadable Gazebo base map cache {cache_path}: {exc}'
            )
            return None

        self.get_logger().info(
            f'Loaded cached Gazebo tile base map from {cache_path} '
            f'({image.width}x{image.height} px, captures={self.rendered_capture_count}).'
        )
        return image, world_bounds

    def _save_cached_base_map(self, image, rendered_map):
        cache_path = self._base_map_cache_path()
        if not cache_path:
            self.get_logger().info('Gazebo tile base map cache is disabled; not saving.')
            return

        try:
            os.makedirs(os.path.dirname(cache_path), exist_ok=True)
            metadata = PngInfo()
            metadata.add_text('cache_format_version', str(BASE_MAP_CACHE_FORMAT_VERSION))
            metadata.add_text('world_sdf', self.world_sdf)
            metadata.add_text('world_name', self._world_name())
            metadata.add_text('max_image_size_px', str(int(self.max_image_size_px)))
            metadata.add_text(
                'requested_camera_fov_deg', str(float(self.gazebo_bev_view_camera_fov))
            )
            metadata.add_text(
                'requested_camera_height_m', str(float(self.gazebo_bev_view_camera_height))
            )
            metadata.add_text('map_region', json.dumps([float(value) for value in self.map_region]))
            metadata.add_text('world_bounds', json.dumps(rendered_map.bounds))
            metadata.add_text('camera_fov_deg', str(float(rendered_map.camera_fov_deg)))
            metadata.add_text('camera_height_m', str(float(rendered_map.camera_height_m)))
            metadata.add_text('capture_count', str(int(rendered_map.capture_count)))
            image.save(cache_path, format='PNG', pnginfo=metadata)
            self.get_logger().info(f'Saved Gazebo tile base map cache to {cache_path}.')
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to save Gazebo base map cache to {cache_path}: {exc}'
            )

    def _cache_metadata_mismatch(self, metadata):
        expected = {
            'cache_format_version': str(BASE_MAP_CACHE_FORMAT_VERSION),
            'world_name': self._world_name(),
            'max_image_size_px': str(int(self.max_image_size_px)),
        }
        for key, expected_value in expected.items():
            actual_value = metadata.get(key)
            if actual_value != expected_value:
                return f'{key}={actual_value!r}, expected {expected_value!r}'

        float_expected = {
            'requested_camera_fov_deg': float(self.gazebo_bev_view_camera_fov),
            'requested_camera_height_m': float(self.gazebo_bev_view_camera_height),
        }
        for key, expected_value in float_expected.items():
            try:
                actual_value = float(metadata[key])
            except (KeyError, TypeError, ValueError):
                return f'{key} missing or invalid'
            if not math.isclose(actual_value, expected_value, rel_tol=1e-9, abs_tol=1e-9):
                return f'{key}={actual_value!r}, expected {expected_value!r}'

        try:
            actual_map_region = [float(value) for value in json.loads(metadata['map_region'])]
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return 'map_region missing or invalid'
        expected_map_region = [float(value) for value in self.map_region]
        if len(actual_map_region) != len(expected_map_region) or any(
            not math.isclose(actual, expected, rel_tol=1e-9, abs_tol=1e-9)
            for actual, expected in zip(actual_map_region, expected_map_region)
        ):
            return f'map_region={actual_map_region!r}, expected {expected_map_region!r}'

        return ''

    def _world_name(self):
        world_sdf = os.path.expanduser(os.path.expandvars(self.world_sdf))
        if os.path.isfile(world_sdf):
            try:
                root = ET.parse(world_sdf).getroot()
                world_element = root if root.tag == 'world' else root.find('world')
                if world_element is not None and world_element.get('name'):
                    return str(world_element.get('name'))
            except (ET.ParseError, OSError):
                pass
        return os.path.splitext(os.path.basename(world_sdf))[0]

    def _min_meters_per_pixel(self):
        min_x, max_x, min_y, max_y = self.world_bounds
        return max(
            (max_x - min_x) / max(self.map_image.width, 1),
            (max_y - min_y) / max(self.map_image.height, 1),
        )

    def _zoom_for_meters_per_pixel(self, meters_per_pixel):
        lat0 = max(min(float(self.enuref[0]), 85.0), -85.0)
        zoom = math.log2(
            156543.03392 * max(math.cos(math.radians(lat0)), 0.01)
            / max(meters_per_pixel, 1e-9)
        )
        return max(0, int(math.floor(zoom)))

    def start_tcp_server(self):
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.tcp_server_ip, self.tcp_server_port))
        self.socket.listen(8)
        self.get_logger().info(
            f'Gazebo OSM tile server listening on '
            f'{self.tcp_server_ip}:{self.tcp_server_port}.'
        )

        while rclpy.ok():
            conn, _ = self.socket.accept()
            self.handle_client(conn)

    def handle_client(self, conn):
        try:
            request_data = conn.recv(BUFFER_SIZE).decode().strip()
            if not request_data:
                return

            request_line = request_data.split('\r\n')[0]
            parts = request_line.split()
            if len(parts) != 3:
                self.get_logger().error(f'Invalid request format: {request_line}')
                return

            url_path = parts[1]
            url_parts = [part for part in url_path.split('/') if part]
            if not url_parts:
                return

            if url_parts[0] == 'metadata':
                self._send_json_metadata(conn)
                return

            if not url_path.endswith('.png') or len(url_parts) != 3:
                self.get_logger().error(f'Invalid tile URL: {url_path}')
                return

            zoom = int(url_parts[0])
            tile_x = int(url_parts[1])
            tile_y = int(url_parts[2].replace('.png', ''))
            if self.processing_status != 'Ready' or self.map_image is None:
                self._send_response(
                    conn,
                    b'image/png',
                    self._busy_tile,
                    status=b'503 Service Unavailable',
                    extra_headers=[b'Retry-After: 1'],
                )
                return
            image_data = self._tile_png(tile_x, tile_y, zoom)
            self._send_response(conn, b'image/png', image_data)
        except ValueError as exc:
            self.get_logger().error(f'Error processing tile request: {exc}')
        except Exception as exc:
            self.get_logger().error(f'Unexpected tile server error: {exc}')
        finally:
            conn.close()

    def _send_json_metadata(self, conn):
        import json

        if self.processing_status != 'Ready' or self.map_image is None or self.world_bounds is None:
            payload = {
                'status': self.processing_status,
                'detail': self.processing_detail,
                'current_capture': self.processing_current_capture,
                'total_captures': self.processing_total_captures,
            }
            self._send_response(conn, b'application/json', json.dumps(payload).encode())
            return

        min_x, max_x, min_y, max_y = self.world_bounds
        payload = {
            'status': self.processing_status,
            'detail': self.processing_detail,
            'current_capture': self.processing_current_capture,
            'total_captures': self.processing_total_captures,
            'world_sdf': self.world_sdf,
            'mpp': self.min_meters_per_pixel,
            'world_bounds': self.world_bounds,
            'width_px': self.map_image.width,
            'height_px': self.map_image.height,
            'camera_fov_deg': float(self.rendered_camera_fov_deg),
            'camera_height_m': float(self.rendered_camera_height_m),
            'capture_count': int(self.rendered_capture_count),
            'map_region': [float(value) for value in self.map_region],
            'max_zoom': self.max_zoom,
            'ref_lat': float(self.enuref[0]),
            'ref_lon': float(self.enuref[1]),
            'south_west_north_east': self._lat_lon_bounds(min_x, max_x, min_y, max_y),
        }
        self._send_response(conn, b'application/json', json.dumps(payload).encode())

    def _tile_png(self, tile_x, tile_y, zoom):
        tile_bounds = self._tile_enu_bounds(tile_x, tile_y, zoom)
        crop_box = self._image_crop_box(tile_bounds)
        if crop_box is None:
            return self._placeholder_tile

        crop, paste_box = crop_box
        source = self.map_image.crop(crop)
        source = source.resize(
            (paste_box[2] - paste_box[0], paste_box[3] - paste_box[1]),
            _image_resampling_lanczos(),
        )
        tile = Image.new('RGBA', (TILE_SIZE, TILE_SIZE), (255, 255, 255, 0))
        tile.paste(source, paste_box)
        buf = io.BytesIO()
        tile.save(buf, format='PNG')
        return buf.getvalue()

    def _tile_enu_bounds(self, tile_x, tile_y, zoom):
        north = self._tile_y_to_lat(tile_y, zoom)
        south = self._tile_y_to_lat(tile_y + 1, zoom)
        west = self._tile_x_to_lon(tile_x, zoom)
        east = self._tile_x_to_lon(tile_x + 1, zoom)
        west_x, north_y = self._llh_to_enu(north, west)
        east_x, south_y = self._llh_to_enu(south, east)
        return west_x, east_x, south_y, north_y

    def _image_crop_box(self, tile_bounds):
        min_x, max_x, min_y, max_y = self.world_bounds
        tile_min_x, tile_max_x, tile_min_y, tile_max_y = tile_bounds
        intersect_min_x = max(min_x, tile_min_x)
        intersect_max_x = min(max_x, tile_max_x)
        intersect_min_y = max(min_y, tile_min_y)
        intersect_max_y = min(max_y, tile_max_y)
        if intersect_min_x >= intersect_max_x or intersect_min_y >= intersect_max_y:
            return None

        source = (
            self._world_x_to_image_x(intersect_min_x),
            self._world_y_to_image_y(intersect_max_y),
            self._world_x_to_image_x(intersect_max_x),
            self._world_y_to_image_y(intersect_min_y),
        )
        paste = (
            round((intersect_min_x - tile_min_x) / (tile_max_x - tile_min_x) * TILE_SIZE),
            round((tile_max_y - intersect_max_y) / (tile_max_y - tile_min_y) * TILE_SIZE),
            round((intersect_max_x - tile_min_x) / (tile_max_x - tile_min_x) * TILE_SIZE),
            round((tile_max_y - intersect_min_y) / (tile_max_y - tile_min_y) * TILE_SIZE),
        )
        return tuple(round(value) for value in source), paste

    def _world_x_to_image_x(self, x):
        min_x, max_x, _, _ = self.world_bounds
        return (x - min_x) / (max_x - min_x) * self.map_image.width

    def _world_y_to_image_y(self, y):
        _, _, min_y, max_y = self.world_bounds
        return (max_y - y) / (max_y - min_y) * self.map_image.height

    def _llh_to_enu(self, lat, lon):
        lat0, lon0, _ = self.enuref
        x = math.radians(lon - lon0) * EARTH_RADIUS_M * math.cos(math.radians(lat0))
        y = math.radians(lat - lat0) * EARTH_RADIUS_M
        return x, y

    def _enu_to_llh(self, x, y):
        lat0, lon0, height0 = self.enuref
        lat = lat0 + math.degrees(y / EARTH_RADIUS_M)
        lon = lon0 + math.degrees(x / (EARTH_RADIUS_M * max(math.cos(math.radians(lat0)), 1e-9)))
        return lat, lon, height0

    def _lat_lon_bounds(self, min_x, max_x, min_y, max_y):
        south, west, _ = self._enu_to_llh(min_x, min_y)
        north, east, _ = self._enu_to_llh(max_x, max_y)
        return [south, west, north, east]

    @staticmethod
    def _tile_x_to_lon(tile_x, zoom):
        return tile_x / float(1 << zoom) * 360.0 - 180.0

    @staticmethod
    def _tile_y_to_lat(tile_y, zoom):
        n = math.pi - 2.0 * math.pi * tile_y / float(1 << zoom)
        return math.degrees(math.atan(0.5 * (math.exp(n) - math.exp(-n))))

    def _send_response(self, conn, content_type, data, status=b'200 OK', extra_headers=None):
        conn.sendall(b'HTTP/1.1 ' + status + b'\r\n')
        conn.sendall(b'Content-Type: ' + content_type + b'\r\n')
        for header in extra_headers or []:
            conn.sendall(header + b'\r\n')
        conn.sendall(f'Content-Length: {len(data)}\r\n'.encode())
        conn.sendall(b'\r\n')
        conn.sendall(data)

    def destroy(self):
        try:
            self.socket.close()
        except Exception:
            pass
        super().destroy_node()


def _image_resampling_lanczos():
    try:
        return Image.Resampling.LANCZOS
    except AttributeError:
        return Image.LANCZOS


def main(args=None):
    rclpy.init(args=args)
    node = GazeboOsmTileServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
