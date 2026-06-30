#!/usr/bin/env python3

"""Serve Gazebo top-down renders through an OSM-compatible tile API."""

from dataclasses import dataclass
import io
import json
import math
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
import uuid
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw
from PIL.PngImagePlugin import PngInfo
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtGui import QColor, QImage, QPainter, QPixmap  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402

# Gazebo static-world rendering

DEFAULT_WORLD_PADDING_M = 2.0
DEFAULT_MAX_IMAGE_SIZE_PX = 1800
DEFAULT_RENDER_TIMEOUT_S = 15.0
DEFAULT_CAMERA_FOV_DEG = 10.0
DEFAULT_CAMERA_HEIGHT_M = 0.0
MIN_GAZEBO_CAMERA_FOV_RAD = 0.1
MAX_STITCHED_IMAGE_PIXELS = 500_000_000
MAX_STITCHED_CAPTURE_COUNT = 512
TOP_DOWN_CAMERA_NAME = 'control_tower_top_down_camera'
TOP_DOWN_IMAGE_TOPIC = '/control_tower/top_down_map/image'
GAZEBO_TRANSPORT_ENV_VARS = (
    'GZ_PARTITION',
    'IGN_PARTITION',
    'GZ_DISCOVERY_MSG_PORT',
    'IGN_DISCOVERY_MSG_PORT',
    'GZ_DISCOVERY_SRV_PORT',
    'IGN_DISCOVERY_SRV_PORT',
)


@dataclass
class StaticWorldShape:
    kind: str
    points: list
    color: QColor
    radius: float = 0.0


@dataclass
class StaticWorldMap:
    pixmap: QPixmap
    bounds: tuple
    shape_count: int
    camera_height_m: float = 0.0
    camera_fov_deg: float = DEFAULT_CAMERA_FOV_DEG
    capture_count: int = 1


def render_static_gazebo_world(
    world_sdf_path,
    max_image_size_px=DEFAULT_MAX_IMAGE_SIZE_PX,
    render_timeout_s=DEFAULT_RENDER_TIMEOUT_S,
    camera_fov_deg=DEFAULT_CAMERA_FOV_DEG,
    camera_height_m=DEFAULT_CAMERA_HEIGHT_M,
    map_region=None,
    progress_callback=None,
):
    """Render a Gazebo world as a top-down pixmap.

    This starts a temporary, headless Gazebo instance with a nadir camera injected into
    the world. That produces a real Gazebo render while avoiding objects that are spawned
    later into the user's live simulation.
    """
    world_sdf_path = os.path.abspath(os.path.expanduser(world_sdf_path))
    shapes, bounds = _load_world_shapes_and_bounds(world_sdf_path)
    bounds = _bounds_from_map_region(map_region) or bounds

    if float(camera_height_m) > 0.0:
        pixmap, image_bounds, capture_count = _render_world_with_gazebo_camera_grid(
            world_sdf_path,
            bounds,
            max_image_size_px,
            render_timeout_s,
            camera_fov_deg,
            camera_height_m,
            progress_callback=progress_callback,
        )
    else:
        pixmap = _render_world_with_gazebo_camera(
            world_sdf_path,
            bounds,
            max_image_size_px,
            render_timeout_s,
            camera_fov_deg,
            camera_height_m,
        )
        image_bounds = _square_bounds_for_camera(bounds, camera_fov_deg, camera_height_m)
        capture_count = 1

    return StaticWorldMap(
        pixmap=pixmap,
        bounds=image_bounds,
        shape_count=len(shapes),
        camera_height_m=_camera_height_for_bounds(bounds, camera_fov_deg, camera_height_m),
        camera_fov_deg=_camera_fov_degrees(camera_fov_deg),
        capture_count=capture_count,
    )


def _square_bounds_for_camera(bounds, camera_fov_deg=DEFAULT_CAMERA_FOV_DEG, camera_height_m=0.0):
    min_x, max_x, min_y, max_y = bounds
    center_x = (min_x + max_x) * 0.5
    center_y = (min_y + max_y) * 0.5
    half_side = _camera_ortho_scale(bounds, camera_fov_deg, camera_height_m) * 0.5
    return (
        center_x - half_side,
        center_x + half_side,
        center_y - half_side,
        center_y + half_side,
    )


def _load_world_shapes_and_bounds(world_sdf_path):
    world_sdf_path = os.path.abspath(os.path.expanduser(world_sdf_path))
    if not os.path.exists(world_sdf_path):
        raise FileNotFoundError(f'Gazebo world SDF not found: {world_sdf_path}')

    root = ET.parse(world_sdf_path).getroot()
    world = _first_child(root, 'world') if _tag_name(root) != 'world' else root
    if world is None:
        raise ValueError(f'No <world> element found in {world_sdf_path}')

    shapes = []
    for model in _children(world, 'model'):
        model_pose = _parse_pose(_first_child(model, 'pose'))
        shapes.extend(_model_shapes(model, model_pose))

    for include in _children(world, 'include'):
        include_shape = _include_placeholder_shape(include)
        if include_shape is not None:
            shapes.append(include_shape)

    if not shapes:
        raise ValueError(f'No drawable visual geometry found in {world_sdf_path}')

    bounds = _shape_bounds(shapes)
    bounds = (
        bounds[0] - DEFAULT_WORLD_PADDING_M,
        bounds[1] + DEFAULT_WORLD_PADDING_M,
        bounds[2] - DEFAULT_WORLD_PADDING_M,
        bounds[3] + DEFAULT_WORLD_PADDING_M,
    )
    return shapes, bounds


def _bounds_from_map_region(map_region):
    if (
        not map_region
        or len(map_region) != 4
        or not any(float(value) != 0.0 for value in map_region)
    ):
        return None

    top, right, bottom, left = [float(value) for value in map_region]
    min_x, max_x = sorted((left, right))
    min_y, max_y = sorted((bottom, top))
    if min_x == max_x or min_y == max_y:
        raise ValueError(
            'Gazebo map_region must describe a non-empty area as '
            f'[top, right, bottom, left]; got {map_region} which resolves to '
            f'x=[{min_x}, {max_x}], y=[{min_y}, {max_y}]'
        )
    return min_x, max_x, min_y, max_y


def _render_world_with_gazebo_camera(
    world_sdf_path,
    bounds,
    max_image_size_px,
    render_timeout_s,
    camera_fov_deg,
    camera_height_m,
    gazebo_env=None,
    camera_name=TOP_DOWN_CAMERA_NAME,
    image_topic=TOP_DOWN_IMAGE_TOPIC,
):
    gz_executable = shutil.which('gz')
    if gz_executable is None:
        raise RuntimeError('gz executable was not found')

    image_size = max(256, int(max_image_size_px))
    camera_height = _camera_height_for_bounds(bounds, camera_fov_deg, camera_height_m)
    ortho_scale = _camera_ortho_scale(bounds, camera_fov_deg, camera_height)

    with tempfile.TemporaryDirectory(prefix='waywiser_gazebo_map_') as temp_dir:
        render_world_path = os.path.join(temp_dir, 'control_tower_top_down.world.sdf')
        _write_top_down_camera_world(
            world_sdf_path,
            render_world_path,
            bounds,
            image_size,
            camera_height,
            ortho_scale,
            camera_name,
            image_topic,
        )

        owns_gazebo_env = gazebo_env is None
        if owns_gazebo_env:
            gazebo_env = _private_gazebo_transport_env()
            previous_partition_env = _apply_gazebo_transport_env(gazebo_env)
        else:
            previous_partition_env = None
        try:
            image_capture = _GazeboImageCapture(image_topic)
            process = subprocess.Popen(
                [gz_executable, 'sim', '-s', '-r', '--headless-rendering', render_world_path],
                cwd=os.path.dirname(world_sdf_path) or None,
                env=gazebo_env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            try:
                qimage = image_capture.wait_for_image(process, render_timeout_s)
            finally:
                process.terminate()
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2.0)
        finally:
            if owns_gazebo_env:
                _restore_gazebo_transport_env(previous_partition_env)

        pixmap = QPixmap.fromImage(qimage)
        if pixmap.isNull():
            raise RuntimeError('Gazebo published an unreadable top-down frame')
        return pixmap.copy()


def _render_world_with_gazebo_camera_grid(
    world_sdf_path,
    bounds,
    max_image_size_px,
    render_timeout_s,
    camera_fov_deg,
    camera_height_m,
    progress_callback=None,
):
    image_size = max(256, int(max_image_size_px))
    camera_height = _camera_height_for_bounds(bounds, camera_fov_deg, camera_height_m)
    footprint_side_m = _camera_ortho_scale(bounds, camera_fov_deg, camera_height)
    min_x, max_x, min_y, max_y = bounds
    world_width_m = max_x - min_x
    world_height_m = max_y - min_y
    columns = max(1, math.ceil(world_width_m / footprint_side_m))
    rows = max(1, math.ceil(world_height_m / footprint_side_m))
    capture_count = columns * rows
    meters_per_pixel = footprint_side_m / image_size
    stitched_width = max(1, math.ceil(world_width_m / meters_per_pixel))
    stitched_height = max(1, math.ceil(world_height_m / meters_per_pixel))
    _validate_stitching_plan(
        capture_count,
        stitched_width,
        stitched_height,
        footprint_side_m,
        camera_height,
        camera_fov_deg,
    )

    gazebo_env = _private_gazebo_transport_env()
    previous_partition_env = _apply_gazebo_transport_env(gazebo_env)
    stitched_pixmap = QPixmap(stitched_width, stitched_height)
    stitched_pixmap.fill(QColor(255, 255, 255, 0))
    painter = QPainter(stitched_pixmap)
    try:
        for row in range(rows):
            capture_max_y = max_y - row * footprint_side_m
            capture_min_y = capture_max_y - footprint_side_m
            target_y = round((max_y - capture_max_y) / meters_per_pixel)
            for column in range(columns):
                capture_min_x = min_x + column * footprint_side_m
                capture_max_x = capture_min_x + footprint_side_m
                target_x = round((capture_min_x - min_x) / meters_per_pixel)
                capture_bounds = (
                    capture_min_x,
                    capture_max_x,
                    capture_min_y,
                    capture_max_y,
                )
                try:
                    capture_index = row * columns + column + 1
                    image_topic = f'{TOP_DOWN_IMAGE_TOPIC}/{capture_index}'
                    if progress_callback is not None:
                        progress_callback(capture_index, capture_count, capture_bounds)
                    capture_pixmap = _render_world_with_gazebo_camera(
                        world_sdf_path,
                        capture_bounds,
                        image_size,
                        render_timeout_s,
                        camera_fov_deg,
                        camera_height,
                        gazebo_env=gazebo_env,
                        camera_name=f'{TOP_DOWN_CAMERA_NAME}_{capture_index}',
                        image_topic=image_topic,
                    )
                except Exception as exc:
                    raise RuntimeError(
                        'Failed to render Gazebo BEV stitch capture '
                        f'{capture_index}/{capture_count} '
                        f'at row={row}, column={column}, bounds={capture_bounds}'
                    ) from exc
                painter.drawPixmap(target_x, target_y, capture_pixmap)
    finally:
        painter.end()
        _restore_gazebo_transport_env(previous_partition_env)

    return stitched_pixmap.copy(), bounds, capture_count


def _private_gazebo_transport_env():
    env = os.environ.copy()
    partition = f'waywiser_gazebo_map_{os.getpid()}_{uuid.uuid4().hex}'
    msg_port = _free_udp_port()
    srv_port = _free_udp_port({msg_port})

    env['GZ_PARTITION'] = partition
    env['IGN_PARTITION'] = partition
    env['GZ_DISCOVERY_MSG_PORT'] = str(msg_port)
    env['IGN_DISCOVERY_MSG_PORT'] = str(msg_port)
    env['GZ_DISCOVERY_SRV_PORT'] = str(srv_port)
    env['IGN_DISCOVERY_SRV_PORT'] = str(srv_port)
    return env


def _apply_gazebo_transport_env(env):
    previous_env = {env_var: os.environ.get(env_var) for env_var in GAZEBO_TRANSPORT_ENV_VARS}
    for env_var in GAZEBO_TRANSPORT_ENV_VARS:
        os.environ[env_var] = env[env_var]
    return previous_env


def _restore_gazebo_transport_env(previous_env):
    for env_var, value in previous_env.items():
        if value is None:
            os.environ.pop(env_var, None)
        else:
            os.environ[env_var] = value


def _free_udp_port(exclude=None):
    exclude = set(exclude or [])
    for _ in range(16):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind(('127.0.0.1', 0))
            port = sock.getsockname()[1]
        if port not in exclude:
            return port
    raise RuntimeError('Could not allocate a private Gazebo discovery port')


def _write_top_down_camera_world(
    source_world_path,
    target_world_path,
    bounds,
    image_size,
    camera_height,
    ortho_scale,
    camera_name=TOP_DOWN_CAMERA_NAME,
    image_topic=TOP_DOWN_IMAGE_TOPIC,
):
    tree = ET.parse(source_world_path)
    root = tree.getroot()
    world = _first_child(root, 'world') if _tag_name(root) != 'world' else root
    if world is None:
        raise ValueError(f'No <world> element found in {source_world_path}')

    for model in list(_children(world, 'model')):
        if model.get('name') == camera_name:
            world.remove(model)

    if not _has_gazebo_sensors_system(world):
        world.append(_gazebo_sensors_system_plugin())

    center_x = (bounds[0] + bounds[1]) * 0.5
    center_y = (bounds[2] + bounds[3]) * 0.5
    camera_model = ET.Element('model', {'name': camera_name})
    ET.SubElement(camera_model, 'static').text = 'true'
    ET.SubElement(camera_model, 'pose').text = (
        f'{center_x:.6f} {center_y:.6f} {camera_height:.6f} 0 {math.pi / 2:.9f} {math.pi / 2:.9f}'
    )
    link = ET.SubElement(camera_model, 'link', {'name': 'link'})
    sensor = ET.SubElement(link, 'sensor', {'name': 'camera', 'type': 'camera'})
    ET.SubElement(sensor, 'always_on').text = 'true'
    ET.SubElement(sensor, 'update_rate').text = '5'
    ET.SubElement(sensor, 'topic').text = image_topic

    camera = ET.SubElement(sensor, 'camera')
    ET.SubElement(camera, 'horizontal_fov').text = _camera_fov_for_scale(
        ortho_scale, camera_height
    )
    image = ET.SubElement(camera, 'image')
    ET.SubElement(image, 'width').text = str(image_size)
    ET.SubElement(image, 'height').text = str(image_size)
    ET.SubElement(image, 'format').text = 'R8G8B8'
    clip = ET.SubElement(camera, 'clip')
    ET.SubElement(clip, 'near').text = '0.1'
    ET.SubElement(clip, 'far').text = f'{camera_height + 1000.0:.6f}'

    world.append(camera_model)
    tree.write(target_world_path, encoding='utf-8', xml_declaration=True)


def _gazebo_sensors_system_plugin():
    plugin = ET.Element(
        'plugin',
        {
            'filename': 'gz-sim-sensors-system',
            'name': 'gz::sim::systems::Sensors',
        },
    )
    ET.SubElement(plugin, 'render_engine').text = 'ogre2'
    return plugin


def _has_gazebo_sensors_system(world):
    for plugin in _children(world, 'plugin'):
        plugin_name = plugin.get('name', '')
        plugin_filename = plugin.get('filename', '')
        if (
            plugin_name == 'gz::sim::systems::Sensors'
            or plugin_name == 'ignition::gazebo::systems::Sensors'
            or plugin_filename == 'gz-sim-sensors-system'
            or plugin_filename == 'ignition-gazebo-sensors-system'
        ):
            return True
    return False


def _camera_fov_for_scale(scale, camera_height):
    fov = 2.0 * math.atan(max(float(scale), 0.001) * 0.5 / max(float(camera_height), 0.001))
    return f'{max(MIN_GAZEBO_CAMERA_FOV_RAD, min(fov, math.pi - 0.01)):.9f}'


def _camera_fov_radians(camera_fov_deg):
    return max(MIN_GAZEBO_CAMERA_FOV_RAD, math.radians(min(float(camera_fov_deg), 170.0)))


def _camera_fov_degrees(camera_fov_deg):
    return math.degrees(_camera_fov_radians(camera_fov_deg))


def _world_side_m(bounds):
    min_x, max_x, min_y, max_y = bounds
    return max(max_x - min_x, max_y - min_y, 1.0)


def _camera_height_for_bounds(bounds, camera_fov_deg, camera_height_m):
    requested_height = max(float(camera_height_m), 0.0)
    if requested_height > 0.0:
        return max(requested_height, 0.001)
    required_height = (
        _world_side_m(bounds) * 0.5 / math.tan(_camera_fov_radians(camera_fov_deg) * 0.5)
    )
    return max(required_height, 1.0)


def _camera_ortho_scale(bounds, camera_fov_deg, camera_height_m):
    camera_height = max(float(camera_height_m), 0.001)
    visible_side = 2.0 * camera_height * math.tan(_camera_fov_radians(camera_fov_deg) * 0.5)
    if float(camera_height_m) > 0.0:
        return max(visible_side, 0.001)
    return max(_world_side_m(bounds), 1.0)


def _validate_stitching_plan(
    capture_count,
    stitched_width,
    stitched_height,
    footprint_side_m,
    camera_height_m,
    camera_fov_deg,
):
    stitched_pixels = stitched_width * stitched_height
    if capture_count > MAX_STITCHED_CAPTURE_COUNT:
        raise ValueError(
            'Gazebo BEV stitching would require too many captures: '
            f'{capture_count} captures, footprint={footprint_side_m:.2f} m, '
            f'height={camera_height_m:.2f} m, fov={_camera_fov_degrees(camera_fov_deg):.2f} deg. '
            'Increase gazebo_bev_view_camera_height or gazebo_bev_view_camera_fov.'
        )
    if stitched_pixels > MAX_STITCHED_IMAGE_PIXELS:
        raise ValueError(
            'Gazebo BEV stitching would create an image that is too large: '
            f'{stitched_width}x{stitched_height} px ({stitched_pixels} pixels), '
            f'footprint={footprint_side_m:.2f} m. Increase camera height/FOV or lower '
            'max_image_size_px.'
        )


class _GazeboImageCapture:
    def __init__(self, topic):
        try:
            from gz.msgs10.image_pb2 import Image, PixelFormatType  # noqa: PLC0415
            from gz.transport13 import Node  # noqa: PLC0415
        except ImportError:
            _add_system_python_dist_packages()
            try:
                from gz.msgs10.image_pb2 import Image, PixelFormatType  # noqa: PLC0415
                from gz.transport13 import Node  # noqa: PLC0415
            except ImportError as retry_exc:
                raise RuntimeError(
                    'Gazebo Python transport bindings are not available. '
                    'Install the Gazebo Python bindings or launch Control Tower with '
                    'system site-packages enabled.'
                ) from retry_exc

        self._pixel_format_type = PixelFormatType
        self._image = None
        self._event = threading.Event()
        self._node = Node()
        self._node.subscribe(Image, topic, self._callback)

    def _callback(self, msg):
        image = self._qimage_from_gazebo_image(msg)
        if image is not None:
            self._image = image
            self._event.set()

    def wait_for_image(self, process, timeout_s):
        deadline = time.monotonic() + max(float(timeout_s), 1.0)
        while time.monotonic() < deadline:
            if self._event.wait(timeout=0.1):
                return self._image
            return_code = process.poll()
            if return_code is not None:
                raise RuntimeError(
                    f'Gazebo exited before publishing a top-down frame ({return_code})'
                )
        raise TimeoutError('Timed out waiting for Gazebo top-down camera frame')

    def _qimage_from_gazebo_image(self, msg):
        width = int(msg.width)
        height = int(msg.height)
        step = int(msg.step)
        if width <= 0 or height <= 0 or step <= 0:
            return None

        image_bytes = bytes(msg.data)
        pixel_format = msg.pixel_format_type
        formats = self._pixel_format_type

        if pixel_format == formats.RGB_INT8:
            return QImage(image_bytes, width, height, step, QImage.Format_RGB888).copy()
        if pixel_format == formats.RGBA_INT8:
            return QImage(image_bytes, width, height, step, QImage.Format_RGBA8888).copy()
        if pixel_format == formats.BGRA_INT8:
            return QImage(image_bytes, width, height, step, QImage.Format_ARGB32).copy()
        if pixel_format == formats.BGR_INT8:
            image = QImage(image_bytes, width, height, step, QImage.Format_RGB888).copy()
            return image.rgbSwapped()
        if pixel_format == formats.L_INT8:
            return QImage(image_bytes, width, height, step, QImage.Format_Grayscale8).copy()

        raise RuntimeError(f'Unsupported Gazebo image pixel format: {pixel_format}')


def _add_system_python_dist_packages():
    """Expose apt-installed Gazebo Python bindings when running inside a venv."""
    candidates = [
        '/usr/lib/python3/dist-packages',
        f'/usr/lib/python{sys.version_info.major}.{sys.version_info.minor}/dist-packages',
        f'/usr/local/lib/python{sys.version_info.major}.{sys.version_info.minor}/dist-packages',
    ]
    for path in candidates:
        if os.path.isdir(path) and path not in sys.path:
            sys.path.append(path)


def _model_shapes(model, parent_pose):
    shapes = []
    model_pose = _compose_pose(parent_pose, _parse_pose(_first_child(model, 'pose')))
    for link in _children(model, 'link'):
        link_pose = _compose_pose(model_pose, _parse_pose(_first_child(link, 'pose')))
        visuals = list(_children(link, 'visual'))
        if not visuals:
            visuals = list(_children(link, 'collision'))
        for visual in visuals:
            visual_pose = _compose_pose(link_pose, _parse_pose(_first_child(visual, 'pose')))
            geometry = _first_child(visual, 'geometry')
            if geometry is None:
                continue
            shape = _geometry_shape(geometry, visual_pose, _material_color(visual))
            if shape is not None:
                shapes.append(shape)

    for nested_model in _children(model, 'model'):
        shapes.extend(_model_shapes(nested_model, model_pose))
    return shapes


def _geometry_shape(geometry, pose, color):
    plane = _first_child(geometry, 'plane')
    if plane is not None:
        size = _float_list(_text(_first_child(plane, 'size')), [100.0, 100.0])
        return StaticWorldShape('polygon', _box_points(pose, size[0], size[1]), color)

    box = _first_child(geometry, 'box')
    if box is not None:
        size = _float_list(_text(_first_child(box, 'size')), [1.0, 1.0, 1.0])
        return StaticWorldShape('polygon', _box_points(pose, size[0], size[1]), color)

    cylinder = _first_child(geometry, 'cylinder')
    if cylinder is not None:
        radius = _float_text(_first_child(cylinder, 'radius'), 0.5)
        return StaticWorldShape('circle', [(pose[0], pose[1])], color, radius)

    sphere = _first_child(geometry, 'sphere')
    if sphere is not None:
        radius = _float_text(_first_child(sphere, 'radius'), 0.5)
        return StaticWorldShape('circle', [(pose[0], pose[1])], color, radius)

    mesh = _first_child(geometry, 'mesh')
    if mesh is not None:
        scale = _float_list(_text(_first_child(mesh, 'scale')), [1.0, 1.0, 1.0])
        return StaticWorldShape('polygon', _box_points(pose, scale[0], scale[1]), color)

    return None


def _include_placeholder_shape(include):
    pose = _parse_pose(_first_child(include, 'pose'))
    name_parts = (
        _text(_first_child(include, 'name')),
        _text(_first_child(include, 'uri')),
    )
    name_text = ' '.join(value for value in name_parts if value).lower()
    if not name_text:
        return None

    if 'oak tree' in name_text:
        return StaticWorldShape('oak_tree', [(pose[0], pose[1])], QColor('#15803d'), 3.2)
    if 'pine tree' in name_text:
        return StaticWorldShape('pine_tree', [(pose[0], pose[1])], QColor('#166534'), 2.2)
    if 'tree' in name_text:
        return StaticWorldShape('tree', [(pose[0], pose[1])], QColor('#15803d'), 2.5)
    if 'grass' in name_text or 'ground' in name_text:
        return StaticWorldShape('polygon', _box_points(pose, 10.0, 10.0), QColor('#86a35d'))
    if 'water' in name_text or 'coast' in name_text:
        return StaticWorldShape('polygon', _box_points(pose, 12.0, 12.0), QColor('#93c5fd'))
    return StaticWorldShape('circle', [(pose[0], pose[1])], QColor('#6b7280'), 0.5)


def _shape_bounds(shapes):
    xs = []
    ys = []
    for shape in shapes:
        if shape.kind in ('circle', 'tree', 'pine_tree', 'oak_tree'):
            x, y = shape.points[0]
            xs.extend([x - shape.radius, x + shape.radius])
            ys.extend([y - shape.radius, y + shape.radius])
        else:
            xs.extend(point[0] for point in shape.points)
            ys.extend(point[1] for point in shape.points)
    return (min(xs), max(xs), min(ys), max(ys))


def _points_bounds(points):
    return (
        min(point[0] for point in points),
        max(point[0] for point in points),
        min(point[1] for point in points),
        max(point[1] for point in points),
    )


def _box_points(pose, size_x, size_y):
    x, y, yaw = pose
    half_x = max(abs(size_x) * 0.5, 0.05)
    half_y = max(abs(size_y) * 0.5, 0.05)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    points = []
    for local_x, local_y in (
        (-half_x, -half_y),
        (half_x, -half_y),
        (half_x, half_y),
        (-half_x, half_y),
    ):
        points.append(
            (
                x + local_x * cos_yaw - local_y * sin_yaw,
                y + local_x * sin_yaw + local_y * cos_yaw,
            )
        )
    return points


def _compose_pose(parent, child):
    px, py, pyaw = parent
    cx, cy, cyaw = child
    cos_yaw = math.cos(pyaw)
    sin_yaw = math.sin(pyaw)
    return (
        px + cx * cos_yaw - cy * sin_yaw,
        py + cx * sin_yaw + cy * cos_yaw,
        pyaw + cyaw,
    )


def _parse_pose(pose_elem):
    values = _float_list(_text(pose_elem), [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    values += [0.0] * (6 - len(values))
    return (values[0], values[1], values[5])


def _material_color(visual):
    material = _first_child(visual, 'material')
    if material is None:
        return QColor('#9ca3af')
    for tag in ('diffuse', 'ambient'):
        color_elem = _first_child(material, tag)
        if color_elem is not None:
            values = _float_list(_text(color_elem), [0.6, 0.6, 0.6, 1.0])
            values += [1.0] * (4 - len(values))
            return QColor(
                int(max(0.0, min(1.0, values[0])) * 255),
                int(max(0.0, min(1.0, values[1])) * 255),
                int(max(0.0, min(1.0, values[2])) * 255),
                int(max(0.0, min(1.0, values[3])) * 255),
            )
    return QColor('#9ca3af')


def _float_text(elem, default):
    try:
        return float(_text(elem))
    except (TypeError, ValueError):
        return default


def _float_list(text, default):
    if not text:
        return list(default)
    try:
        return [float(value) for value in text.split()]
    except ValueError:
        return list(default)


def _text(elem):
    return elem.text.strip() if elem is not None and elem.text else ''


def _children(parent, tag):
    return [child for child in list(parent) if _tag_name(child) == tag]


def _first_child(parent, tag):
    if parent is None:
        return None
    for child in list(parent):
        if _tag_name(child) == tag:
            return child
    return None


def _tag_name(element):
    return element.tag.split('}')[-1]


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


class GazeboOsmTileServerNode(Node):
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
    node = GazeboOsmTileServerNode()
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
