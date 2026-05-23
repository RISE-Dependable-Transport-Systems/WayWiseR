"""Gazebo world top-down map rendering for Control Tower."""

from dataclasses import dataclass
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

from PyQt5.QtGui import QColor, QImage, QPainter, QPixmap

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
