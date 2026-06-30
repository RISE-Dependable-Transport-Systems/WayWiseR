#!/usr/bin/env python3
"""ROS 2 node for vehicle teleoperation using a keyboard and PyQt5 GUI."""

from contextlib import contextmanager
from datetime import datetime
import json
import os
import subprocess
import sys
import tempfile
import time
import wave


@contextmanager
def suppress_stderr():
    """Redirect stderr to devnull at the C level."""
    # Save original stderr file descriptor (hardcode fd 2 for C-level)
    save_fd = os.dup(2)
    try:
        # Open devnull
        with open(os.devnull, 'w', encoding='utf-8') as devnull:
            # Redirect stderr (fd 2) to devnull
            os.dup2(devnull.fileno(), 2)
            yield
    finally:
        # Restore stderr
        os.dup2(save_fd, 2)
        os.close(save_fd)


with suppress_stderr():
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    import numpy as np
    from rcl_interfaces.msg import Parameter as ParameterMsg
    from rcl_interfaces.msg import ParameterType, ParameterValue
    from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
    import rclpy
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState, Joy
    from std_msgs.msg import Bool, Header, String
    import tf2_ros
    from tf_transformations import euler_from_quaternion
    from visualization_msgs.msg import MarkerArray

try:
    with suppress_stderr():
        from PyQt5.QtCore import QEvent, QPoint, Qt, QTimer, QUrl
        from PyQt5.QtGui import QKeySequence
        from PyQt5.QtMultimedia import QAudio, QAudioDeviceInfo, QSoundEffect
        from PyQt5.QtNetwork import QNetworkAccessManager, QNetworkReply, QNetworkRequest
        from PyQt5.QtWidgets import (
            QActionGroup,
            QApplication,
            QDialog,
            QFrame,
            QGraphicsOpacityEffect,
            QGridLayout,
            QHBoxLayout,
            QLabel,
            QMainWindow,
            QMenu,
            QMessageBox,
            QPushButton,
            QSizePolicy,
            QSplitter,
            QToolTip,
            QWidget,
        )
        from PyQt5.uic import loadUi
    _PYQT5_AVAILABLE = True
except ImportError:
    _PYQT5_AVAILABLE = False

    # Stub base classes so that class definitions below do not raise NameError
    # at import time.  Instantiating them will raise RuntimeError at runtime.
    class _QtStub:
        """Placeholder for Qt classes when PyQt5 is not installed."""

        def __init_subclass__(cls, **kwargs):
            super().__init_subclass__(**kwargs)

        def __init__(self, *args, **kwargs):
            raise RuntimeError('PyQt5 is not installed. Install python3-pyqt5 to use this class.')

    QDialog = _QtStub  # type: ignore[misc,assignment]
    QFrame = _QtStub  # type: ignore[misc,assignment]
    QGridLayout = _QtStub  # type: ignore[misc,assignment]
    QMainWindow = _QtStub  # type: ignore[misc,assignment]
    QWidget = _QtStub  # type: ignore[misc,assignment]
    QEvent = None
    QKeySequence = None
    QPoint = _QtStub  # type: ignore[misc,assignment]
    Qt = None
    QTimer = _QtStub  # type: ignore[misc,assignment]
    QUrl = None
    QActionGroup = _QtStub  # type: ignore[misc,assignment]
    QAudio = None
    QAudioDeviceInfo = None
    QSoundEffect = _QtStub  # type: ignore[misc,assignment]
    QApplication = _QtStub  # type: ignore[misc,assignment]
    QGraphicsOpacityEffect = _QtStub  # type: ignore[misc,assignment]
    QHBoxLayout = _QtStub  # type: ignore[misc,assignment]
    QPushButton = _QtStub  # type: ignore[misc,assignment]
    QMenu = _QtStub  # type: ignore[misc,assignment]
    QMessageBox = _QtStub  # type: ignore[misc,assignment]
    QSizePolicy = None
    QSplitter = _QtStub  # type: ignore[misc,assignment]
    QToolTip = _QtStub  # type: ignore[misc,assignment]
    loadUi = None


from waywiser_core.msg import (  # noqa: E402
    BatteryState,
    HeartbeatRxState,
    MissionState,
    NavSatFixExtended,
    PathWithTwists,
    QuadcopterState,
)
from waywiser_py.waywiser_utils import RELIABLE_TRANSIENT_LOCAL_QOS, RosUtils  # noqa: E402
from waywiser_twist_safety.msg import EmergencyStopState  # noqa: E402

try:
    from waywiser_teleop_py.vehicle_overlay import VehicleOverlayModel  # noqa: E402
except ModuleNotFoundError:
    import importlib.util  # noqa: E402

    overlay_module_path = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        'vehicle_overlay.py',
    )
    overlay_spec = importlib.util.spec_from_file_location(
        'waywiser_teleop_py.vehicle_overlay',
        overlay_module_path,
    )
    overlay_module = importlib.util.module_from_spec(overlay_spec)
    sys.modules[overlay_spec.name] = overlay_module
    overlay_spec.loader.exec_module(overlay_module)
    VehicleOverlayModel = overlay_module.VehicleOverlayModel

if _PYQT5_AVAILABLE:
    UI_BASE_PATH = os.path.join(
        get_package_share_directory('waywiser_teleop'), 'user_interface', 'control_tower'
    )
    from waywiser_teleop_py.mission_planner import (  # noqa: E402
        ACTIVE_BUTTON_STYLE,
        build_path_with_twists,
        MissionPlannerWidget,
        OPENSTREETMAP_CACHE_DIR,
        OPENSTREETMAP_TILE_SERVER_URL,
        POPUP_MENU_STYLE,
        UpMenuButton,
    )
else:
    UI_BASE_PATH = None
    MissionPlannerWidget = None
    UpMenuButton = None
    ACTIVE_BUTTON_STYLE = ''
    OPENSTREETMAP_CACHE_DIR = ''
    OPENSTREETMAP_TILE_SERVER_URL = ''
    POPUP_MENU_STYLE = ''
    build_path_with_twists = None

AUTOPILOT_MARKER_TTL_SECONDS = 1.0


class ControlTowerNode(Node):
    """Publish twist messages using keypresses from the keyboard."""

    def __init__(self):
        super().__init__('control_tower', allow_undeclared_parameters=True)

        # Initialize emergency_stop_target_state_msg
        self.emergency_stop_target_state_msg = EmergencyStopState()
        self.emergency_stop_target_state_msg.sender_id = self.get_name()
        self.emergency_stop_target_state_msg.state = EmergencyStopState.ACTIVE

        # Declare parameters
        self.declare_parameter('control_vehicle_node_fqn', '')
        self.control_vehicle_node_fqn = (
            self.get_parameter('control_vehicle_node_fqn').get_parameter_value().string_value
        )
        self.vehicle_namespace = RosUtils.parent_namespace_from_fqn(self.control_vehicle_node_fqn)

        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('startup_linear_speed', 0.5)
        self.declare_parameter('startup_angular_speed', 1.0)
        self.declare_parameter('linear_speed_increment', 0.1)
        self.declare_parameter('angular_speed_increment', 0.1)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('vehicle_control_enabled', True)
        self.declare_parameter('publish_control_tower_heartbeat', True)
        self.declare_parameter('control_tower_heartbeat_topic', 'control_tower_heartbeat')
        self.declare_parameter('control_tower_heartbeat_rate', 5.0)

        # Joy emergency stop parameters
        self.declare_parameter('emergency_stop_set_joy_button_index', 5)
        self.declare_parameter('emergency_stop_clear_joy_button_index', 7)
        self.declare_parameter('joy_timeout', 1.0)

        # Set initial linear and angular speeds
        self.linear_speed = (
            self.get_parameter('startup_linear_speed').get_parameter_value().double_value
        )
        self.angular_speed = (
            self.get_parameter('startup_angular_speed').get_parameter_value().double_value
        )

        self.max_linear_speed = (
            self.get_parameter('max_linear_speed').get_parameter_value().double_value
        )
        self.max_angular_speed = (
            self.get_parameter('max_angular_speed').get_parameter_value().double_value
        )

        self.linear_speed_increment = (
            self.get_parameter('linear_speed_increment').get_parameter_value().double_value
        )
        self.angular_speed_increment = (
            self.get_parameter('angular_speed_increment').get_parameter_value().double_value
        )

        self.emergency_stop_set_joy_button_index = (
            self.get_parameter('emergency_stop_set_joy_button_index')
            .get_parameter_value()
            .integer_value
        )
        self.emergency_stop_clear_joy_button_index = (
            self.get_parameter('emergency_stop_clear_joy_button_index')
            .get_parameter_value()
            .integer_value
        )
        self.joy_timeout = self.get_parameter('joy_timeout').get_parameter_value().double_value
        self.vehicle_control_enabled = (
            self.get_parameter('vehicle_control_enabled').get_parameter_value().bool_value
        )
        self.publish_control_tower_heartbeat = (
            self.get_parameter('publish_control_tower_heartbeat').get_parameter_value().bool_value
        )
        self.control_tower_heartbeat_topic = (
            self.get_parameter('control_tower_heartbeat_topic').get_parameter_value().string_value
        )
        self.control_tower_heartbeat_rate = max(
            0.1,
            self.get_parameter('control_tower_heartbeat_rate').get_parameter_value().double_value,
        )

        # Wait for sim time if needed
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        if self.use_sim_time:
            if rclpy.ok() and self.get_clock().now().nanoseconds == 0:
                self.get_logger().warn('Waiting for /clock to be published...')
            while rclpy.ok() and self.get_clock().now().nanoseconds == 0:
                time.sleep(1.0)
                rclpy.spin_once(self)
            self.get_logger().warn('Receiving /clock msgs now.')

        # Vehicle parameters (to be fetched from vehicle node)
        self.odom_topic = ''
        self.vehicle_pose_topic = ''
        self.battery_state_topic = ''
        self.arm_command_topic = ''
        self.quadcopter_state_topic = ''
        self.mission_status_topic = ''
        self.control_tower_heartbeat_rx_state_topic = ''
        self.control_tower_heartbeat_timeout = 0.0
        self.nav_sat_fix_extended_topic = ''
        self.emergency_stop_status_topic = ''
        self.emergency_stop_update_topic = ''
        self.joint_states_topic = ''
        self.home_pose_topic = ''
        self.route_topic = ''
        self.autopilot_state_control_topic = ''
        self.enuref = [57.71495867, 12.89134921, 0.0]
        self.waywise_object_type = 'generic'
        self.vehicle_connected = False

        # Subscribers (will be created after fetching topics)
        self.odom_subscriber = None
        self.vehicle_pose_subscriber = None
        self.home_pose_subscriber = None
        self.battery_state_subscriber = None
        self.quadcopter_state_subscriber = None
        self.mission_status_subscriber = None
        self.control_tower_heartbeat_rx_state_subscriber = None
        self.nav_sat_fix_extended_subscriber = None
        self.emergency_stop_state_subscriber = None
        self.robot_description_subscriber = None
        self.joint_states_subscriber = None
        self.marker_subscribers = []
        self.emergency_stop_request_publisher = None
        self.arm_command_publisher = None
        self.mux_publisher = None
        self.route_publisher = None
        self.autopilot_state_control_publisher = None
        self.control_tower_heartbeat_publisher = None

        # Mux initialization
        self.mux_output_topic = ''
        self.declare_parameter('mux_output_topic', 'teleop_mux_vel')
        self.mux_output_topic = (
            self.get_parameter('mux_output_topic').get_parameter_value().string_value
        )
        self.key_vel_publisher = self.create_publisher(Twist, 'key_vel', 10)
        self.declare_parameter('route_topic', 'waywiser_path')
        self.route_topic = self.get_parameter('route_topic').get_parameter_value().string_value
        self.declare_parameter('startup_route_file', '')
        self.startup_route_file = (
            self.get_parameter('startup_route_file').get_parameter_value().string_value
        )
        self.declare_parameter('autopilot_state_control_topic', '/autopilot_state_control')
        self.autopilot_state_control_topic = (
            self.get_parameter('autopilot_state_control_topic').get_parameter_value().string_value
        )
        self.declare_parameter('osm_tile_server_url', OPENSTREETMAP_TILE_SERVER_URL)
        self.osm_tile_server_url = (
            self.get_parameter('osm_tile_server_url').get_parameter_value().string_value
        )

        # Determine default cache directory based on environment variable or current workspace
        workspace_root = os.environ.get('WAYWISER_WS')
        if not workspace_root:
            workspace_root = os.path.abspath(
                os.path.join(os.path.dirname(__file__), '../../../../..')
            )
        default_cache_dir = OPENSTREETMAP_CACHE_DIR or os.path.join(
            workspace_root, 'resources', 'control_tower', 'osm'
        )
        self.declare_parameter('osm_tile_cache_dir', default_cache_dir)
        self.osm_tile_cache_dir = (
            self.get_parameter('osm_tile_cache_dir').get_parameter_value().string_value
        )

        self.declare_parameter('map_source', 'OpenStreetMap')
        self.map_source = self._normalize_map_source(
            self.get_parameter('map_source').get_parameter_value().string_value
        )

        self.declare_parameter('world_frame', 'map')
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value

        # TF2 buffer and listener for transforming vehicle poses to world_frame
        self._tf_listener = None
        self._tf_listener_suspended = False
        self._tf_resume_timer = None
        self._create_tf_listener()
        self._last_clock_msg_ns = None
        self._waiting_for_sim_time_reset = False
        self._clock_reset_subscriber = None
        if self.use_sim_time:
            self._clock_reset_subscriber = self.create_subscription(
                Clock,
                '/clock',
                self.clock_callback,
                10,
            )
        self.declare_parameter('setup_request_topic', '/setup_request')
        self.setup_request_topic = (
            self.get_parameter('setup_request_topic').get_parameter_value().string_value
        )
        self.setup_request_subscriber = self.create_subscription(
            String,
            self.setup_request_topic,
            self.setup_request_callback,
            10,
        )

        self.mux_sources = {}  # source_name -> {topic, priority, timeout, last_msg, last_stamp}
        self.mux_source_subscribers = []
        self.active_mux_source = 'None'
        self._init_mux_sources()

        # Joy subscribers and timers
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.joy_watchdog_timer = self.create_timer(self.joy_timeout, self.joy_watchdog_callback)
        self.joy_watchdog_timer.cancel()
        self.control_tower_heartbeat_timer = self.create_timer(
            1.0 / self.control_tower_heartbeat_rate,
            self.publish_control_tower_heartbeat_msg,
        )

        # State variables
        self.last_emergency_stop_state = {'msg': None, 'stamp': self.get_clock().now()}
        self.last_control_tower_heartbeat_rx_state = {
            'msg': None,
            'stamp': self.get_clock().now(),
        }
        self.last_odom = {
            'pose': None,
            'twist': None,
            'stamp': self.get_clock().now(),
            'frame_id': '',
        }
        self.last_vehicle_pose = {
            'pose': None,
            'stamp': self.get_clock().now(),
            'frame_id': '',
        }
        self.last_home_pose = {'pose': None, 'stamp': self.get_clock().now(), 'frame_id': ''}
        self.last_battery_state = {'msg': None, 'stamp': self.get_clock().now()}
        self.last_quadcopter_state = {'msg': None, 'stamp': self.get_clock().now()}
        self.last_quadcopter_state_code = None
        self.last_mission_state = {'msg': None, 'stamp': self.get_clock().now()}
        self.last_nav_sat_fix_extended = {'msg': None, 'stamp': self.get_clock().now()}
        self.last_robot_description = ''
        self.vehicle_overlay_model = VehicleOverlayModel()
        self.visual_marker_store = {}
        self.visual_markers = []
        self.vehicle_status_reset_pending = False
        self.vehicle_status_cleared = False
        self.vehicle_status_reset_generation = 0

        # Current twist command
        self.current_twist = Twist()

        # Auto arm state
        self.auto_arm_enabled = True
        self.hold_position_on_idle_enabled = True
        self.auto_lift_off_enabled = True
        self.auto_lift_off_active = False
        self.arm_request_debounce_period = 1.0
        self.last_arm_request = {'arm': None, 'time': 0.0}

        # Key states
        self.keys_pressed = set()
        self.is_actuation_requested = False
        self.auto_landing_active = False

        # RTCM correction age mapping
        self.rtcm_correction_age_mapping = {
            0: 'NA',
            1: '0-1 s',
            2: '1-2 s',
            3: '2-5 s',
            4: '5-10 s',
            5: '10-15 s',
            6: '15-20 s',
            7: '20-30 s',
            8: '30-45 s',
            9: '45-60 s',
            10: '60-90 s',
            11: '90-120 s',
            12: '≥120 s',
        }

    def clock_callback(self, msg):
        clock_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        if self._last_clock_msg_ns is not None and clock_ns < self._last_clock_msg_ns:
            self.handle_sim_time_reset(self._last_clock_msg_ns, clock_ns)
            self._waiting_for_sim_time_reset = False
        self._last_clock_msg_ns = clock_ns

    def setup_request_callback(self, msg):
        try:
            request = json.loads(msg.data) if msg.data else {}
        except json.JSONDecodeError:
            request = {}
        reset_all = self._request_bool(request, 'reset_all', False)
        if reset_all and self.use_sim_time:
            self.get_logger().warn(
                'Gazebo full reset requested. Suspending Control Tower TF listener '
                'until simulation time restarts.'
            )
            self.suspend_tf_listener()
            self._waiting_for_sim_time_reset = True
            self.reset_runtime_state_after_time_reset()
            self.schedule_tf_listener_resume(12.0)

    def handle_sim_time_reset(self, previous_clock_ns, current_clock_ns):
        self.get_logger().warn(
            'Simulation time moved backwards from '
            f'{previous_clock_ns / 1e9:.3f} s to {current_clock_ns / 1e9:.3f} s. '
            'Clearing Control Tower TF and runtime state.'
        )
        self.reset_runtime_state_after_time_reset()
        self.schedule_tf_listener_resume(0.5)

    def reset_runtime_state_after_time_reset(self):
        self._tf_buffer.clear()
        now = self.get_clock().now()
        self.last_emergency_stop_state = {'msg': None, 'stamp': now}
        self.last_control_tower_heartbeat_rx_state = {'msg': None, 'stamp': now}
        self.last_odom = {
            'pose': None,
            'twist': None,
            'stamp': now,
            'frame_id': '',
        }
        self.last_vehicle_pose = {'pose': None, 'stamp': now, 'frame_id': ''}
        self.last_home_pose = {'pose': None, 'stamp': now, 'frame_id': ''}
        self.last_battery_state = {'msg': None, 'stamp': now}
        self.last_quadcopter_state = {'msg': None, 'stamp': now}
        self.last_quadcopter_state_code = None
        self.last_mission_state = {'msg': None, 'stamp': now}
        self.last_nav_sat_fix_extended = {'msg': None, 'stamp': now}
        self.current_twist = Twist()
        self.auto_landing_active = False
        self.auto_lift_off_active = False
        self.vehicle_status_reset_pending = True
        self.vehicle_status_cleared = True
        self.vehicle_status_reset_generation += 1
        self._reset_vehicle_overlay_model()
        self.visual_marker_store.clear()
        self.visual_markers = []

    def _create_tf_listener(self):
        try:
            self._tf_buffer = tf2_ros.Buffer(node=self)
        except TypeError:
            self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tf_listener_suspended = False

    def suspend_tf_listener(self):
        if self._tf_listener is not None:
            try:
                self._tf_listener.unregister()
            except Exception as exc:
                self.get_logger().warn(f'Could not unregister Control Tower TF listener: {exc}')
            self._tf_listener = None
        self._tf_buffer.clear()
        self._tf_listener_suspended = True

    def schedule_tf_listener_resume(self, delay_sec):
        if self._tf_resume_timer is not None:
            self.destroy_timer(self._tf_resume_timer)
        self._tf_resume_timer = self.create_timer(
            max(0.1, float(delay_sec)),
            self.resume_tf_listener_once,
        )

    def resume_tf_listener_once(self):
        if self._tf_resume_timer is not None:
            self.destroy_timer(self._tf_resume_timer)
            self._tf_resume_timer = None
        if self._tf_listener is None:
            self.get_logger().info('Resuming Control Tower TF listener after simulation reset.')
            self._create_tf_listener()
        else:
            self._tf_buffer.clear()

    @staticmethod
    def _request_bool(request, name, default):
        value = request.get(name, default) if isinstance(request, dict) else default
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def request_params_from_vehicle_node(self):
        """Request params from the current vehicle node."""
        self.vehicle_connected = False
        service_name = f'/{self.control_vehicle_node_fqn}/get_parameters'
        service_name = service_name.replace('//', '/')
        self.get_logger().info(f"Requesting parameters from '{self.control_vehicle_node_fqn}'")

        client = self.create_client(GetParameters, service_name)
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                f"Parameter service of '{self.control_vehicle_node_fqn}' not available"
            )
            self.destroy_client(client)
            return

        request = GetParameters.Request()
        request.names = [
            'odom_topic',
            'vehicle_pose_topic',
            'battery_state_topic',
            'fused_nav_sat_fix_extended_topic',
            'emergency_stop_status_topic',
            'emergency_stop_update_topic',
            'enuref',
            'waywise_object_type',
            'autopilot_state_control_topic',
            'joint_states_topic',
            'mission_status_topic',
            'control_tower_heartbeat_rx_state_topic',
            'control_tower_heartbeat_timeout',
            'home_pose_topic',
        ]

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            vals = future.result().values
            if len(vals) >= 8:
                self.odom_topic = self._prefix_with_vehicle_namespace(
                    vals[0].string_value or self.odom_topic
                )
                self.vehicle_pose_topic = self._prefix_with_vehicle_namespace(
                    vals[1].string_value or self.vehicle_pose_topic
                )
                self.battery_state_topic = self._prefix_with_vehicle_namespace(
                    vals[2].string_value or self.battery_state_topic
                )
                self.nav_sat_fix_extended_topic = self._prefix_with_vehicle_namespace(
                    vals[3].string_value or self.nav_sat_fix_extended_topic
                )
                self.emergency_stop_status_topic = self._prefix_with_vehicle_namespace(
                    vals[4].string_value or self.emergency_stop_status_topic
                )
                self.emergency_stop_update_topic = self._prefix_with_vehicle_namespace(
                    vals[5].string_value or self.emergency_stop_update_topic
                )
                self.enuref = vals[6].double_array_value or self.enuref
                self.waywise_object_type = vals[7].string_value or self.waywise_object_type
                if len(vals) >= 9 and vals[8].string_value:
                    self.autopilot_state_control_topic = self._prefix_with_vehicle_namespace(
                        vals[8].string_value
                    )
                self.joint_states_topic = self._prefix_with_vehicle_namespace(
                    vals[9].string_value
                    if len(vals) >= 10 and vals[9].string_value
                    else 'joint_states'
                )
                self.mission_status_topic = self._prefix_with_vehicle_namespace(
                    vals[10].string_value
                    if len(vals) >= 11 and vals[10].string_value
                    else 'mission_status'
                )
                if len(vals) >= 12:
                    self.control_tower_heartbeat_rx_state_topic = (
                        self._prefix_with_vehicle_namespace(vals[11].string_value)
                        if vals[11].string_value
                        else ''
                    )
                if len(vals) >= 13 and vals[12].type == ParameterType.PARAMETER_DOUBLE:
                    self.control_tower_heartbeat_timeout = vals[12].double_value
                self.home_pose_topic = self._prefix_with_vehicle_namespace(
                    vals[13].string_value
                    if len(vals) >= 14 and vals[13].string_value
                    else 'home_pose'
                )
                if self.waywise_object_type == 'quadcopter':
                    qc_request = GetParameters.Request()
                    qc_request.names = [
                        'arm_command_topic',
                        'quadcopter_state_topic',
                        'auto_lift_off_enabled',
                    ]
                    qc_future = client.call_async(qc_request)
                    rclpy.spin_until_future_complete(self, qc_future, timeout_sec=5.0)

                    if qc_future.result() is not None:
                        qc_vals = qc_future.result().values
                        if len(qc_vals) >= 3:
                            self.arm_command_topic = self._prefix_with_vehicle_namespace(
                                qc_vals[0].string_value or self.arm_command_topic
                            )
                            self.quadcopter_state_topic = self._prefix_with_vehicle_namespace(
                                qc_vals[1].string_value or self.quadcopter_state_topic
                            )
                            if qc_vals[2].type == ParameterType.PARAMETER_BOOL:
                                self.auto_lift_off_enabled = qc_vals[2].bool_value
                        else:
                            self.get_logger().warn(
                                f'Received {len(qc_vals)} quadcopter params instead of '
                                f"at least 3 from '{self.control_vehicle_node_fqn}'"
                            )

                # Create subscribers
                self._create_subscribers()

                # Create publishers
                self._create_publishers()
                self.vehicle_connected = True

                self.get_logger().info(
                    f"Updated topics from '{self.control_vehicle_node_fqn}' "  # noqa: Q000
                    f'(waywise_object_type={self.waywise_object_type})'  # noqa: Q000
                )

                # We don't reset the joy_watchdog_timer here; we wait for the first /joy message
                self.set_hover_hold_enabled(self.hold_position_on_idle_enabled)

            else:
                self.get_logger().warn(
                    f'Received {len(vals)} topics instead of at least 8 from '
                    f"'{self.control_vehicle_node_fqn}'"
                )

        self.destroy_client(client)

    def _prefix_with_vehicle_namespace(self, topic):
        if not topic:
            return ''
        return RosUtils.prefix_topic_with_namespace(topic, self.vehicle_namespace)

    def set_remote_node_parameter(self, node_fqn, name, value):
        """Set a single parameter on a remote node."""
        if not node_fqn:
            return

        service_name = f'/{node_fqn}/set_parameters'.replace('//', '/')
        client = self.create_client(SetParameters, service_name)
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"Parameter service of '{node_fqn}' not available")
            self.destroy_client(client)
            return

        request = SetParameters.Request()
        param_msg = ParameterMsg()
        param_msg.name = name
        if isinstance(value, bool):
            param_msg.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
        elif isinstance(value, int):
            param_msg.value = ParameterValue(
                type=ParameterType.PARAMETER_INTEGER, integer_value=value
            )
        elif isinstance(value, float):
            param_msg.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=value
            )
        elif isinstance(value, str):
            param_msg.value = ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value=value
            )
        request.parameters.append(param_msg)

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        self.destroy_client(client)

    def set_hover_hold_enabled(self, enabled):
        """Update hover hold behavior on the vehicle node."""
        self.hold_position_on_idle_enabled = bool(enabled)

        if self.waywise_object_type != 'quadcopter':
            return

        self.set_remote_node_parameter(
            self.control_vehicle_node_fqn,
            'hold_position_on_idle',
            self.hold_position_on_idle_enabled,
        )

    def set_auto_lift_off_enabled(self, enabled):
        """Update auto lift-off availability on the vehicle node."""
        self.auto_lift_off_enabled = bool(enabled)
        if not self.auto_lift_off_enabled and self.auto_lift_off_active:
            self.auto_lift_off_active = False
        self.set_remote_node_parameter(
            self.control_vehicle_node_fqn,
            'auto_lift_off_enabled',
            self.auto_lift_off_enabled,
        )
        if not self.auto_lift_off_enabled:
            self.set_remote_node_parameter(self.control_vehicle_node_fqn, 'auto_lift_off', False)

    def start_auto_lift_off(self):
        """Request automatic lift-off from the vehicle node."""
        if self.waywise_object_type != 'quadcopter':
            self.get_logger().warn('Auto lift-off is only available for quadcopters.')
            return
        if not self.auto_lift_off_enabled:
            self.get_logger().warn(
                'Auto lift-off is disabled. Enable the Auto lift off option first.'
            )
            return

        if self.auto_landing_active:
            self.set_auto_landing_active(False)

        self.auto_lift_off_active = True
        self.set_remote_node_parameter(self.control_vehicle_node_fqn, 'auto_lift_off', True)
        self.get_logger().info('Auto lift-off requested.')

    def set_auto_arm_enabled(self, enabled):
        """Update auto arm behavior on the vehicle node."""
        self.auto_arm_enabled = bool(enabled)
        self.set_remote_node_parameter(
            self.control_vehicle_node_fqn, 'auto_arm', self.auto_arm_enabled
        )

    def set_auto_landing_active(self, active):
        """Update auto landing behavior on the vehicle node."""
        self.auto_landing_active = bool(active)
        self.set_remote_node_parameter(
            self.control_vehicle_node_fqn, 'auto_landing', self.auto_landing_active
        )

    def _create_subscribers(self):
        """Create or recreate subscribers based on topic names."""
        self._reset_vehicle_overlay_model()

        if self.odom_topic:
            self.get_logger().info(f'Creating subscriber for odom topic {self.odom_topic}')
            if self.odom_subscriber:
                self.destroy_subscription(self.odom_subscriber)
            self.odom_subscriber = self.create_subscription(
                Odometry, self.odom_topic, self.odom_callback, 10
            )

        if self.vehicle_pose_topic:
            if self.vehicle_pose_subscriber:
                self.destroy_subscription(self.vehicle_pose_subscriber)
            self.vehicle_pose_subscriber = self.create_subscription(
                PoseStamped, self.vehicle_pose_topic, self.vehicle_pose_callback, 10
            )

        if self.home_pose_topic:
            if self.home_pose_subscriber:
                self.destroy_subscription(self.home_pose_subscriber)
            self.home_pose_subscriber = self.create_subscription(
                PoseStamped,
                self.home_pose_topic,
                self.home_pose_callback,
                RELIABLE_TRANSIENT_LOCAL_QOS,
            )

        if self.battery_state_topic:
            if self.battery_state_subscriber:
                self.destroy_subscription(self.battery_state_subscriber)
            self.battery_state_subscriber = self.create_subscription(
                BatteryState,
                self.battery_state_topic,
                self.battery_state_callback,
                10,
            )

        if self.quadcopter_state_topic:
            if self.quadcopter_state_subscriber:
                self.destroy_subscription(self.quadcopter_state_subscriber)
            self.quadcopter_state_subscriber = self.create_subscription(
                QuadcopterState,
                self.quadcopter_state_topic,
                self.quadcopter_state_callback,
                RELIABLE_TRANSIENT_LOCAL_QOS,
            )

        if self.mission_status_topic:
            if self.mission_status_subscriber:
                self.destroy_subscription(self.mission_status_subscriber)
            self.mission_status_subscriber = self.create_subscription(
                MissionState,
                self.mission_status_topic,
                self.mission_status_callback,
                RELIABLE_TRANSIENT_LOCAL_QOS,
            )

        if self.control_tower_heartbeat_rx_state_topic:
            if self.control_tower_heartbeat_rx_state_subscriber:
                self.destroy_subscription(self.control_tower_heartbeat_rx_state_subscriber)
            self.control_tower_heartbeat_rx_state_subscriber = self.create_subscription(
                HeartbeatRxState,
                self.control_tower_heartbeat_rx_state_topic,
                self.control_tower_heartbeat_rx_state_callback,
                RELIABLE_TRANSIENT_LOCAL_QOS,
            )

        if self.nav_sat_fix_extended_topic:
            if self.nav_sat_fix_extended_subscriber:
                self.destroy_subscription(self.nav_sat_fix_extended_subscriber)
            self.nav_sat_fix_extended_subscriber = self.create_subscription(
                NavSatFixExtended,
                self.nav_sat_fix_extended_topic,
                self.nav_sat_fix_extended_callback,
                RELIABLE_TRANSIENT_LOCAL_QOS,
            )

        if self.emergency_stop_status_topic:
            if self.emergency_stop_state_subscriber:
                self.destroy_subscription(self.emergency_stop_state_subscriber)
            self.emergency_stop_state_subscriber = self.create_subscription(
                EmergencyStopState,
                self.emergency_stop_status_topic,
                self.emergency_stop_state_subscriber_callback,
                10,
            )

        robot_description_topic = self._prefix_with_vehicle_namespace('robot_description')
        if robot_description_topic:
            if self.robot_description_subscriber:
                self.destroy_subscription(self.robot_description_subscriber)
            self.robot_description_subscriber = self.create_subscription(
                String,
                robot_description_topic,
                self.robot_description_callback,
                RELIABLE_TRANSIENT_LOCAL_QOS,
            )

        if self.joint_states_topic:
            if self.joint_states_subscriber:
                self.destroy_subscription(self.joint_states_subscriber)
            self.joint_states_subscriber = self.create_subscription(
                JointState,
                self.joint_states_topic,
                self.joint_states_callback,
                10,
            )

        for marker_subscriber in self.marker_subscribers:
            self.destroy_subscription(marker_subscriber)
        self.marker_subscribers = []
        self.visual_marker_store.clear()
        self.visual_markers = []
        for topic in ('waypoint_markers', 'autopilot_markers'):
            topic_with_ns = self._prefix_with_vehicle_namespace(topic)
            self.marker_subscribers.append(
                self.create_subscription(
                    MarkerArray,
                    topic_with_ns,
                    self.visual_marker_array_callback,
                    RELIABLE_TRANSIENT_LOCAL_QOS,
                )
            )

    def _create_publishers(self):
        if self.emergency_stop_update_topic:
            if self.emergency_stop_request_publisher:
                self.destroy_publisher(self.emergency_stop_request_publisher)
            self.emergency_stop_request_publisher = self.create_publisher(
                EmergencyStopState,
                self.emergency_stop_update_topic,
                RELIABLE_TRANSIENT_LOCAL_QOS,
            )

        if self.arm_command_topic:
            if self.arm_command_publisher:
                self.destroy_publisher(self.arm_command_publisher)
            self.arm_command_publisher = self.create_publisher(Bool, self.arm_command_topic, 10)
        elif self.arm_command_publisher:
            self.destroy_publisher(self.arm_command_publisher)
            self.arm_command_publisher = None

        if self.mux_output_topic:
            if self.mux_publisher:
                self.destroy_publisher(self.mux_publisher)
            mux_output_topic_with_ns = self._prefix_with_vehicle_namespace(self.mux_output_topic)
            self.mux_publisher = self.create_publisher(Twist, mux_output_topic_with_ns, 10)

        if self.route_topic:
            if self.route_publisher:
                self.destroy_publisher(self.route_publisher)
            route_topic_with_ns = self._prefix_with_vehicle_namespace(self.route_topic)
            self.route_publisher = self.create_publisher(
                PathWithTwists, route_topic_with_ns, RELIABLE_TRANSIENT_LOCAL_QOS
            )

        if self.autopilot_state_control_topic:
            if self.autopilot_state_control_publisher:
                self.destroy_publisher(self.autopilot_state_control_publisher)
            self.autopilot_state_control_publisher = self.create_publisher(
                Bool, self.autopilot_state_control_topic, RELIABLE_TRANSIENT_LOCAL_QOS
            )

        if self.control_tower_heartbeat_publisher:
            self.destroy_publisher(self.control_tower_heartbeat_publisher)
            self.control_tower_heartbeat_publisher = None
        if self.publish_control_tower_heartbeat and self.control_tower_heartbeat_topic:
            heartbeat_topic_with_ns = self._prefix_with_vehicle_namespace(
                self.control_tower_heartbeat_topic
            )
            self.control_tower_heartbeat_publisher = self.create_publisher(
                Header, heartbeat_topic_with_ns, 10
            )

    def publish_control_tower_heartbeat_msg(self):
        """Publish the operator heartbeat consumed by the vehicle-side failsafe."""
        if (
            not self.publish_control_tower_heartbeat
            or not self.vehicle_control_enabled
            or not self.vehicle_connected
            or self.control_tower_heartbeat_publisher is None
        ):
            return

        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = self.get_name()
        self.control_tower_heartbeat_publisher.publish(msg)

    def publish_mission(self, mission_points, altitude, speed):
        """Publish a planned mission to the selected vehicle and enable autopilot."""
        if not self.vehicle_control_enabled:
            self.get_logger().warn('Vehicle control is disabled in passive monitoring mode.')
            return False

        if not self.vehicle_connected:
            self.get_logger().warn('Connect a vehicle node before sending a mission.')
            return False

        if len(mission_points) < 1:
            self.get_logger().warn('Mission is empty. Add at least one waypoint before sending.')
            return False

        if self.waywise_object_type == 'quadcopter' and self._vehicle_heartbeat_timed_out():
            self.get_logger().warn(
                'Ignoring mission request: vehicle reports Control Tower heartbeat timeout.'
            )
            return False

        if self.route_publisher is None or self.autopilot_state_control_publisher is None:
            self._create_publishers()

        if self.route_publisher is None:
            self.get_logger().warn('Mission route publisher is not available.')
            return False

        mission_altitude = float(altitude) if self.waywise_object_type == 'quadcopter' else 0.0
        mission_speed = max(float(speed), 0.0)
        msg = build_path_with_twists(
            mission_points,
            self.get_clock().now().to_msg(),
            frame_id='map',
            altitude=mission_altitude,
            speed=mission_speed,
        )
        self.route_publisher.publish(msg)

        if self.autopilot_state_control_publisher is not None:
            autopilot_msg = Bool()
            autopilot_msg.data = True
            self.autopilot_state_control_publisher.publish(autopilot_msg)

        self.get_logger().info(
            f'Sent mission with {len(mission_points)} waypoint(s), speed={mission_speed:.2f} m/s, '
            f'z={mission_altitude:.2f} m to {self._prefix_with_vehicle_namespace(self.route_topic)}'
        )
        return True

    @staticmethod
    def _normalize_map_source(source):
        source_by_key = {
            'osm': 'OpenStreetMap',
            'openstreetmap': 'OpenStreetMap',
            'open street map': 'OpenStreetMap',
            'open streetmap': 'OpenStreetMap',
            'local': 'Local OSM server',
            'local osm': 'Local OSM server',
            'local osm server': 'Local OSM server',
            'none': 'None',
        }
        return source_by_key.get(str(source).strip().lower(), 'OpenStreetMap')

    def _vehicle_heartbeat_timed_out(self):
        rx_msg = self.last_control_tower_heartbeat_rx_state.get('msg')
        return rx_msg is not None and rx_msg.state == HeartbeatRxState.TIMEOUT

    def odom_callback(self, msg):
        """Handle odometry messages."""
        if self._waiting_for_sim_time_reset:
            return
        self.vehicle_status_cleared = False
        # Nav_msgs/Odometry: pose and twist are nested in PoseWithCovariance / TwistWithCovariance
        self.last_odom['pose'] = msg.pose.pose
        self.last_odom['twist'] = msg.twist.twist
        self.last_odom['stamp'] = self.get_clock().now()
        self.last_odom['frame_id'] = msg.header.frame_id

    def vehicle_pose_callback(self, msg):
        """Handle vehicle pose messages."""
        if self._waiting_for_sim_time_reset:
            return
        self.vehicle_status_cleared = False
        # Geometry_msgs/PoseStamped: world pose is in the 'pose' field
        self.last_vehicle_pose['pose'] = msg.pose
        self.last_vehicle_pose['stamp'] = self.get_clock().now()
        self.last_vehicle_pose['frame_id'] = msg.header.frame_id

    def home_pose_callback(self, msg):
        """Handle the vehicle's reported home pose."""
        if self._waiting_for_sim_time_reset:
            return
        self.vehicle_status_cleared = False
        self.last_home_pose['pose'] = msg.pose
        self.last_home_pose['stamp'] = self.get_clock().now()
        self.last_home_pose['frame_id'] = msg.header.frame_id

    def robot_description_callback(self, msg):
        """Parse robot_description into a top-view overlay model."""
        if not msg.data:
            return
        self.last_robot_description = msg.data
        self._load_vehicle_overlay_model(msg.data)

    def _reset_vehicle_overlay_model(self):
        self.vehicle_overlay_model = VehicleOverlayModel()
        if self.last_robot_description:
            self._load_vehicle_overlay_model(self.last_robot_description)

    def _load_vehicle_overlay_model(self, robot_description):
        try:
            self.vehicle_overlay_model.load_urdf(robot_description)
        except Exception as exc:
            self.get_logger().warn(f'Could not parse robot_description for map overlay: {exc}')

    def joint_states_callback(self, msg):
        """Update the top-view overlay with the latest joint positions."""
        self.vehicle_overlay_model.update_joint_states(msg.name, msg.position)

    def visual_marker_array_callback(self, msg):
        """Cache vehicle visualization markers for drawing on the mission map."""
        stamp = self.get_clock().now()
        for marker in msg.markers:
            action = marker.action
            if action == 3:  # DELETEALL
                if marker.ns:
                    for key in list(self.visual_marker_store):
                        if key[0] == marker.ns:
                            self.visual_marker_store.pop(key, None)
                else:
                    self.visual_marker_store.clear()
                continue
            key = (marker.ns, marker.id)
            if action == 2:  # DELETE
                self.visual_marker_store.pop(key, None)
                continue
            self.visual_marker_store[key] = {'marker': marker, 'stamp': stamp}
        self._refresh_visual_markers()

    def _refresh_visual_markers(self):
        """Drop short-lived autopilot markers that have stopped updating."""
        now = self.get_clock().now()
        for key, entry in list(self.visual_marker_store.items()):
            marker = entry['marker']
            if marker.ns.endswith('/autopilot_markers'):
                age = (now - entry['stamp']).nanoseconds / 1e9
                if age > AUTOPILOT_MARKER_TTL_SECONDS:
                    self.visual_marker_store.pop(key, None)
        self.visual_markers = [entry['marker'] for entry in self.visual_marker_store.values()]

    def battery_state_callback(self, msg):
        """Handle battery state messages."""
        self.vehicle_status_cleared = False
        self.last_battery_state['msg'] = msg
        self.last_battery_state['stamp'] = self.get_clock().now()

    def quadcopter_state_callback(self, msg):
        """Handle quadcopter high-level state updates."""
        self.vehicle_status_cleared = False
        previous_state_code = self.last_quadcopter_state_code
        now = self.get_clock().now()
        self.last_quadcopter_state['msg'] = msg
        self.last_quadcopter_state['stamp'] = now
        self.last_quadcopter_state_code = msg.state_code

        # Sync auto landing state from vehicle node
        self.auto_landing_active = msg.state_code == QuadcopterState.LANDING
        self.auto_lift_off_active = msg.state_code == QuadcopterState.AUTO_LIFTING_OFF

    def mission_status_callback(self, msg):
        """Handle mission state updates."""
        self.vehicle_status_cleared = False
        self.last_mission_state['msg'] = msg
        self.last_mission_state['stamp'] = self.get_clock().now()

    def control_tower_heartbeat_rx_state_callback(self, msg):
        """Handle vehicle-side Control Tower heartbeat receive state updates."""
        self.vehicle_status_cleared = False
        self.last_control_tower_heartbeat_rx_state['msg'] = msg
        self.last_control_tower_heartbeat_rx_state['stamp'] = self.get_clock().now()

    def nav_sat_fix_extended_callback(self, msg):
        """Handle extended GPS/FIX messages."""
        self.vehicle_status_cleared = False
        self.last_nav_sat_fix_extended['msg'] = msg
        self.last_nav_sat_fix_extended['stamp'] = self.get_clock().now()

    def emergency_stop_state_subscriber_callback(self, msg):
        """Handle emergency stop state updates."""
        self.vehicle_status_cleared = False
        self.last_emergency_stop_state['msg'] = msg
        self.last_emergency_stop_state['stamp'] = self.get_clock().now()

    def joy_callback(self, msg):
        """Handle joystick messages."""
        if self.joy_watchdog_timer.is_canceled():
            self.get_logger().info(
                'Receiving messages from /joy topic now. Monitoring emergency_stop button.'
            )
        self.joy_watchdog_timer.reset()

        if msg.buttons[self.emergency_stop_set_joy_button_index] == 1:
            self.set_emergency_stop(active=True)
        elif msg.buttons[self.emergency_stop_clear_joy_button_index] == 1:
            self.set_emergency_stop(active=False)

    def joy_watchdog_callback(self):
        """Handle joystick watchdog timer."""
        self.set_emergency_stop(active=True)
        self.get_logger().warn(
            f'/joy topic has stopped publishing for {self.joy_timeout:.2f} seconds. '
            'emergency_stop ACTIVATED.'
        )
        self.joy_watchdog_timer.cancel()

    def _init_mux_sources(self):
        """Initialize mux sources from parameters."""
        for subscriber in self.mux_source_subscribers:
            self.destroy_subscription(subscriber)
        self.mux_source_subscribers = []
        self.mux_sources = {}

        # Get the list of sources
        if not self.has_parameter('mux_input.sources'):
            self.declare_parameter('mux_input.sources', [''])
        try:
            source_names = (
                self.get_parameter('mux_input.sources').get_parameter_value().string_array_value
            )
        except Exception as e:
            self.get_logger().warn(f'Could not get mux_input.sources parameter: {e}')
            return

        if len(source_names) == 0 or source_names[0] == '':
            self.get_logger().warn('No mux sources configured!')
            return

        self.get_logger().info(f'Found mux sources: {source_names}')

        for name in source_names:
            try:
                if not self.has_parameter(f'mux_input.{name}.topic'):
                    self.declare_parameter(f'mux_input.{name}.topic', '')
                if not self.has_parameter(f'mux_input.{name}.timeout'):
                    self.declare_parameter(f'mux_input.{name}.timeout', 0.0)
                if not self.has_parameter(f'mux_input.{name}.priority'):
                    self.declare_parameter(f'mux_input.{name}.priority', 0)
                if not self.has_parameter(f'mux_input.{name}.prepend_vehicle_namespace'):
                    self.declare_parameter(f'mux_input.{name}.prepend_vehicle_namespace', False)
                topic = (
                    self.get_parameter(f'mux_input.{name}.topic')
                    .get_parameter_value()
                    .string_value
                )
                timeout = (
                    self.get_parameter(f'mux_input.{name}.timeout')
                    .get_parameter_value()
                    .double_value
                )
                priority = (
                    self.get_parameter(f'mux_input.{name}.priority')
                    .get_parameter_value()
                    .integer_value
                )
                prepend_vehicle_namespace = (
                    self.get_parameter(f'mux_input.{name}.prepend_vehicle_namespace')
                    .get_parameter_value()
                    .bool_value
                )
            except Exception as e:
                self.get_logger().warn(f'Incomplete configuration for source {name}: {e}')
                continue

            topic_with_ns = (
                self._prefix_with_vehicle_namespace(topic) if prepend_vehicle_namespace else topic
            )
            self.mux_sources[name] = {
                'topic': topic_with_ns,
                'timeout': timeout,
                'priority': priority,
                'last_msg': Twist(),
                'last_stamp': self.get_clock().now(),
                'active': False,
            }

            if name != 'keyboard':
                self.get_logger().info(f'Creating subscriber for {name} on topic {topic_with_ns}')
                self.mux_source_subscribers.append(
                    self.create_subscription(
                        Twist, topic_with_ns, lambda msg, n=name: self._mux_callback(msg, n), 10
                    )
                )

        self.get_logger().info(
            f'Initialized {len(self.mux_sources)} mux sources: {list(self.mux_sources.keys())}'
        )

    def _mux_callback(self, msg, source_name):
        """Handle generic mux source callbacks."""
        self._update_mux_source(source_name, msg)

    def _update_mux_source(self, source_name, msg):
        """Update the internal buffer for a mux source."""
        if source_name in self.mux_sources:
            self.mux_sources[source_name]['last_msg'] = msg
            self.mux_sources[source_name]['last_stamp'] = self.get_clock().now()
            self.mux_sources[source_name]['active'] = True

    def _deactivate_mux_source(self, source_name):
        """Let a source go quiet so lower-priority sources can win."""
        if source_name in self.mux_sources:
            self.mux_sources[source_name]['active'] = False

    def _evaluate_mux_and_publish(self):
        """Evaluate priorities and publish the winning command."""
        now = self.get_clock().now()
        winner = None
        winner_name = 'None'

        # Sort sources by priority descending
        # Uses internal dict structure, no waywiser_utils function needed here
        sorted_sources = sorted(
            self.mux_sources.items(), key=lambda x: x[1]['priority'], reverse=True
        )

        for name, data in sorted_sources:
            if not data.get('active', False):
                continue
            time_since_last = (now - data['last_stamp']).nanoseconds / 1e9
            if time_since_last <= data['timeout']:
                # Found the highest priority valid command
                winner = data['last_msg']
                winner_name = name
                break

        if winner is None:
            self.active_mux_source = winner_name
            return

        self.active_mux_source = winner_name
        if self.mux_publisher:
            self.mux_publisher.publish(winner)

    def update_speed(self, linear_delta=0, angular_delta=0):
        """Update linear and angular speed."""
        if linear_delta != 0:
            self.linear_speed += linear_delta
            self.linear_speed = max(
                self.linear_speed_increment, min(self.linear_speed, self.max_linear_speed)
            )

        if angular_delta != 0:
            self.angular_speed += angular_delta
            self.angular_speed = max(
                self.angular_speed_increment, min(self.angular_speed, self.max_angular_speed)
            )

    def process_keys_and_publish(self):
        """Process currently pressed keys and publish twist message."""
        twist = Twist()
        if not self.vehicle_control_enabled:
            self._deactivate_mux_source('keyboard')
            self.active_mux_source = 'None'
            self.is_actuation_requested = False
            self.current_twist = twist
            return

        # Check for actuation keys
        actuation_keys = {Qt.Key.Key_W, Qt.Key.Key_X, Qt.Key.Key_A, Qt.Key.Key_D, Qt.Key.Key_S}
        if self.waywise_object_type == 'quadcopter':
            actuation_keys.update({Qt.Key.Key_Q, Qt.Key.Key_E, Qt.Key.Key_R, Qt.Key.Key_F})
        is_actuation_requested_now = bool(self.keys_pressed & actuation_keys)

        if is_actuation_requested_now:
            if Qt.Key.Key_S not in self.keys_pressed:
                if Qt.Key.Key_W in self.keys_pressed:
                    twist.linear.x += self.linear_speed
                if Qt.Key.Key_X in self.keys_pressed:
                    twist.linear.x -= self.linear_speed
                if Qt.Key.Key_A in self.keys_pressed:
                    if abs(twist.linear.x) > 0.01:
                        twist.angular.z += (
                            self.angular_speed if twist.linear.x > 0 else -self.angular_speed
                        )
                    else:
                        twist.angular.z += self.angular_speed
                if Qt.Key.Key_D in self.keys_pressed:
                    if abs(twist.linear.x) > 0.01:
                        twist.angular.z -= (
                            self.angular_speed if twist.linear.x > 0 else +self.angular_speed
                        )
                    else:
                        twist.angular.z -= self.angular_speed
                if self.waywise_object_type == 'quadcopter':
                    if Qt.Key.Key_Q in self.keys_pressed:
                        twist.linear.y += self.linear_speed
                    if Qt.Key.Key_E in self.keys_pressed:
                        twist.linear.y -= self.linear_speed
                    if Qt.Key.Key_R in self.keys_pressed:
                        twist.linear.z += self.linear_speed
                    if Qt.Key.Key_F in self.keys_pressed:
                        twist.linear.z -= self.linear_speed

        publish_keyboard_cmd = is_actuation_requested_now or self.is_actuation_requested
        if publish_keyboard_cmd:
            self._update_mux_source('keyboard', twist)
            self.key_vel_publisher.publish(twist)
        else:
            self._deactivate_mux_source('keyboard')

        # Select winner and publish to muxed output
        self._evaluate_mux_and_publish()

        self.is_actuation_requested = is_actuation_requested_now
        self.current_twist = twist

    def set_emergency_stop(self, active=True):
        """Set or clear emergency stop."""
        if active:
            self.emergency_stop_target_state_msg.state = EmergencyStopState.ACTIVE
        else:
            self.emergency_stop_target_state_msg.state = EmergencyStopState.CLEAR

        self.emergency_stop_target_state_msg.stamp = self.get_clock().now().to_msg()
        if self.emergency_stop_request_publisher is not None:
            self.emergency_stop_request_publisher.publish(self.emergency_stop_target_state_msg)

    def request_arm_state(self, arm=True):
        """Request arm or disarm through the selected vehicle node."""
        if not self.vehicle_control_enabled:
            self.get_logger().warn('Vehicle control is disabled in passive monitoring mode.')
            return

        if self.waywise_object_type != 'quadcopter':
            self.get_logger().warn('Arm/disarm keys are only available for quadcopters.')
            return

        if arm:
            if self._vehicle_heartbeat_timed_out():
                self.get_logger().warn(
                    'Ignoring arm request: vehicle reports Control Tower heartbeat timeout.'
                )
                return
            # Check if PX4 is ready for takeoff before requesting arm
            state_msg = self.last_quadcopter_state.get('msg')
            if not state_msg or state_msg.state_code != QuadcopterState.READY_TO_ARM:
                self.get_logger().warn(
                    'Ignoring arm request: PX4 is not ready for offboard arming.'
                )
                return
        else:
            # Check if PX4 is in air before requesting disarm
            state_msg = self.last_quadcopter_state.get('msg')
            if state_msg and state_msg.state_code == QuadcopterState.IN_FLIGHT:
                self.get_logger().warn('Ignoring disarm request: vehicle is still in flight.')
                return

        if self.arm_command_publisher is None:
            self.get_logger().warn('Arm/disarm command topic is not available for this vehicle.')
            return

        now_monotonic = time.monotonic()
        if (
            self.last_arm_request['arm'] == bool(arm)
            and (now_monotonic - self.last_arm_request['time']) < self.arm_request_debounce_period
        ):
            return

        msg = Bool()
        msg.data = bool(arm)
        self.arm_command_publisher.publish(msg)
        self.last_arm_request = {'arm': bool(arm), 'time': now_monotonic}
        if arm:
            self.get_logger().info('Requested arm through waywiser_copter_node.')
        else:
            self.get_logger().info('Requested disarm through waywiser_copter_node.')

    def is_battery_low(self):
        """Check if battery is low."""
        return (
            self.last_battery_state['msg'] is not None
            and self.last_battery_state['msg'].state == BatteryState.LOW_VOLTAGE
        )


class VehicleNodeDialog(QDialog):
    """Dialog to prompt user for vehicle node name."""

    def __init__(
        self, node: Node, default_name='waywiser_car_node', auto_connect=False, parent=None
    ):
        super().__init__(parent)
        self.node = node
        self.vehicle_node_name = None
        self.default_name = default_name
        self.auto_connect = auto_connect

        # Load UI from file
        ui_path = os.path.join(UI_BASE_PATH, 'vehicle_node_selection.ui')
        loadUi(ui_path, self)

        # Connect signals
        self.input_field.setText(default_name)
        self.input_field.returnPressed.connect(self.check_node)
        self.fetch_button.clicked.connect(self.fetch_nodes)
        self.node_list.itemDoubleClicked.connect(self.on_node_double_clicked)
        self.check_button.clicked.connect(self.check_node)
        self.cancel_button.clicked.connect(self.reject)

        # Set focus to input field
        self.input_field.setFocus()
        if self.auto_connect:
            QTimer.singleShot(100, self.attempt_auto_connect)
        else:
            QTimer.singleShot(100, self.fetch_nodes)

    def attempt_auto_connect(self):
        """Attempt to connect directly to the default node without scanning."""
        if not self.default_name:
            self.fetch_nodes()
            return

        self.set_status(f"Checking default node '{self.default_name}'...", '#f59e0b')
        QApplication.processEvents()

        if self.check_node_has_vehicle_param(self.default_name):
            self.set_status(f"Node '{self.default_name}' available! Connecting...", '#10b981')
            QApplication.processEvents()
            self.vehicle_node_name = self.default_name
            self.accept()
        else:
            self.fetch_nodes()

    def fetch_nodes(self):
        """Fetch and display all available ROS nodes that have vehicle_interface_type parameter."""
        self.set_status('Fetching available vehicle nodes...', '#f59e0b')
        QApplication.processEvents()

        try:
            # Get all node names and namespaces
            names_and_namespaces = self.node.get_node_names_and_namespaces()

            # Clear the list
            self.node_list.clear()

            # Check each node for vehicle_interface_type parameter
            vehicle_nodes = []
            for name, namespace in names_and_namespaces:
                # Format the node name
                if namespace == '/':
                    full_name = name
                else:
                    full_name = (
                        f'{namespace}/{name}' if not name.startswith('/') else f'{namespace}{name}'
                    )

                # Remove leading slash if present
                full_name = full_name.lstrip('/')

                # Skip our own node
                if full_name == self.node.get_name():
                    continue

                # Check if node has vehicle_interface_type parameter
                if self.check_node_has_vehicle_param(full_name):
                    vehicle_nodes.append(full_name)

            # Sort the node names
            vehicle_nodes.sort()

            # Add to list widget
            for node_name in vehicle_nodes:
                self.node_list.addItem(node_name)

            # Highlight default node if it exists in the list
            self.highlight_default_node()

            if len(vehicle_nodes) > 0:
                self.set_status(f'Found {len(vehicle_nodes)} vehicle node(s).', '#10b981')
            else:
                self.set_status(
                    'No vehicle nodes found. Make sure vehicle nodes are running.', '#f59e0b'
                )

        except Exception as e:
            self.set_status(f'Error fetching nodes: {str(e)}', '#ef4444')
            self.node.get_logger().error(f'Error fetching nodes: {e}')

    def check_node_has_vehicle_param(self, node_name):
        """Check if a node has the vehicle_interface_type parameter."""
        parameter_names = self.list_node_parameters(node_name)
        if parameter_names is None or 'vehicle_interface_type' not in parameter_names:
            return False
        if 'control_tower_selectable' not in parameter_names:
            return True

        request = GetParameters.Request()
        request.names = ['control_tower_selectable']
        response = self.call_parameter_service(node_name, 'get_parameters', GetParameters, request)
        if response is None or not response.values:
            return False
        value = response.values[0]
        return value.type != ParameterType.PARAMETER_NOT_SET and value.bool_value

    def list_node_parameters(self, node_name):
        """List a node's declared parameters without requesting undeclared values."""
        request = ListParameters.Request()
        request.depth = ListParameters.Request.DEPTH_RECURSIVE
        response = self.call_parameter_service(
            node_name, 'list_parameters', ListParameters, request
        )
        return None if response is None else set(response.result.names)

    def call_parameter_service(self, node_name, service_suffix, service_type, request):
        """Call a parameter service and cleanly cancel requests that time out."""
        service_name = f'/{node_name}/{service_suffix}'.replace('//', '/')
        client = self.node.create_client(service_type, service_name)
        future = None
        try:
            if not client.wait_for_service(timeout_sec=0.5):
                return None

            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=0.5)
            if not future.done():
                future.cancel()
                return None
            return future.result()
        except Exception:
            if future is not None and not future.done():
                future.cancel()
            return None
        finally:
            self.node.destroy_client(client)

    def highlight_default_node(self):
        """Highlight the default node name in the list if it exists."""
        for i in range(self.node_list.count()):
            item = self.node_list.item(i)
            if item.text() == self.default_name:
                # Set background color to highlight
                item.setSelected(True)
                # Scroll to the item
                self.node_list.scrollToItem(item)
                return True
        return False

    def on_node_double_clicked(self, item):
        """Handle double-click on a node in the list."""
        node_name = item.text()
        self.input_field.setText(node_name)
        self.input_field.setFocus()
        self.check_node()

    def check_node(self):
        """Check if the node exists and fetch parameters."""
        node_name = self.input_field.text().strip()

        if not node_name:
            self.set_status('Please enter a valid node name.', '#ef4444')
            return

        self.set_status(f"Checking if node '{node_name}' is available...", '#f59e0b')
        QApplication.processEvents()

        # Try to connect to parameter service
        service_name = f'/{node_name}/get_parameters'
        client = self.node.create_client(GetParameters, service_name)

        if not client.wait_for_service(timeout_sec=5.0):
            self.set_status(f"Node '{node_name}' is not available. Please retry.", '#ef4444')
            self.node.destroy_client(client)
            return

        self.set_status(f"Node '{node_name}' is available! Connecting...", '#10b981')
        QApplication.processEvents()
        time.sleep(1.0)

        self.vehicle_node_name = node_name
        self.node.destroy_client(client)
        self.accept()

    def set_status(self, text, color):
        """Set status label text and color."""
        self.status_label.setText(text)
        self.status_label.setStyleSheet(f'color: {color}; font-weight: 600;')

    def get_vehicle_node_name(self):
        """Return the selected vehicle node name."""
        return self.vehicle_node_name


class UsageGuideDialog(QDialog):
    """Dialog to show usage guide for the control tower."""

    def __init__(self, parent=None):
        super().__init__(parent)

        # Load UI from file
        ui_path = os.path.join(UI_BASE_PATH, 'usage_guide.ui')
        loadUi(ui_path, self)

        # Connect close button
        self.close_button.clicked.connect(self.accept)


class ControlTowerUI(QMainWindow):
    """PyQt5 GUI for ControlTowerNode node using .ui file."""

    def __init__(self, node: ControlTowerNode):
        super().__init__()
        self.node = node
        self._startup_mission_loaded = False
        self._handled_vehicle_status_reset_generation = node.vehicle_status_reset_generation

        # Load the main UI from file
        ui_path = os.path.join(UI_BASE_PATH, 'twist_control.ui')
        loadUi(ui_path, self)

        # Recompose loaded UI into a control-tower layout with mission planning on the left.
        self.setup_mission_planner_shell()
        self.setup_heartbeat_status_row()
        self.setup_vehicle_status_layout()
        self.setup_status_copy_buttons()
        self.setup_selectable_status_values()

        # Setup audio for low battery warning
        self.setup_audio()

        # Connect signals
        self.setup_connections()

        # Timer for ROS spinning
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(10)

        # Timer for publishing commands
        publish_rate = self.node.get_parameter('publish_rate').get_parameter_value().double_value
        self.publish_timer = QTimer()
        self.publish_timer.timeout.connect(self.publish_command)
        self.publish_timer.start(int(1000 / publish_rate))

        # Timer for updating display
        self.display_timer = QTimer()
        self.display_timer.timeout.connect(self.update_display)
        self.display_timer.start(100)

        # UI colors
        self.gray_color = 'gray'
        self.green_color = '#26D07C'
        self.yellow_color = '#EFA90B'
        self.red_color = '#D32F2F'
        self.blue_color = '#0076CE'
        self.inactive_muted_color = '#6b7280'

        self.control_group_opacity = QGraphicsOpacityEffect(self.control_group)
        self.control_group.setGraphicsEffect(self.control_group_opacity)

        self.status_group_opacity = QGraphicsOpacityEffect(self.status_group)
        self.status_group.setGraphicsEffect(self.status_group_opacity)

        self.update_control_group_state()

        # Track if we've already played warning sound
        self.last_battery_warning = False

        # Auto-connect only when launch/config provided an explicit vehicle node name.
        if self.node.control_vehicle_node_fqn.strip():
            QTimer.singleShot(100, lambda: self.show_vehicle_node_dialog(auto_connect=True))

    def setup_mission_planner_shell(self):
        """Mount the mission planner next to the existing twist control panel."""
        self._move_button_layout_to_top()

        self.mission_planner_button = QPushButton('MISSION PLANNER')
        self.mission_planner_button.setCheckable(True)
        self.mission_planner_button.setStyleSheet(ACTIVE_BUTTON_STYLE)
        self.button_layout.insertWidget(1, self.mission_planner_button)

        self.fit_right_panel_button = QPushButton('FIT WIDTH')
        self.fit_right_panel_button.setCheckable(True)
        self.fit_right_panel_button.setFixedHeight(22)
        self.fit_right_panel_button.setMinimumWidth(78)
        self.fit_right_panel_button.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.fit_right_panel_button.setStyleSheet(
            'QPushButton {'
            ' font-size: 9pt; font-weight: 600; padding: 2px 8px;'
            '}' + ACTIVE_BUTTON_STYLE
        )
        self.fit_right_panel_button.setToolTip(
            'Keep the control panel wide enough to show its complete contents'
        )
        self.ros_time_label = QLabel('ROS Time: [N/A]')
        self.ros_time_label.setStyleSheet(
            'QLabel {'
            ' color: #d1d5db; font-size: 9pt; font-weight: 600;'
            ' background: transparent; border: none; padding: 0 6px;'
            '}'
        )
        self.ros_time_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.fit_right_panel_layout = QHBoxLayout()
        self.fit_right_panel_layout.setContentsMargins(2, 0, 2, 0)
        self.fit_right_panel_layout.setSpacing(8)
        self.fit_right_panel_layout.addWidget(self.fit_right_panel_button)
        self.fit_right_panel_layout.addSpacing(14)
        self.fit_right_panel_layout.addWidget(self.ros_time_label)
        self.fit_right_panel_layout.addStretch(1)
        self.verticalLayout_2.insertLayout(0, self.fit_right_panel_layout)
        self._fitting_right_panel = False
        self._right_panel_fit_initialized = False
        self._right_panel_fit_padding = 36
        self._right_panel_fit_grow_threshold = 14
        self._right_panel_fit_shrink_threshold = 52
        self.right_panel_fit_timer = QTimer(self)
        self.right_panel_fit_timer.setInterval(250)
        self.right_panel_fit_timer.timeout.connect(self._fit_right_panel_width)

        twist_control_widget = self.takeCentralWidget()
        self.twist_control_widget = twist_control_widget
        central_widget = QWidget(self)
        central_layout = QHBoxLayout(central_widget)
        central_layout.setContentsMargins(0, 0, 0, 0)

        self.main_splitter = QSplitter(Qt.Horizontal, central_widget)
        self._main_splitter_handle_width = self.main_splitter.handleWidth()
        self.mission_planner = MissionPlannerWidget(UI_BASE_PATH, self.main_splitter)
        self.mission_planner.set_vehicle_type(self.node.waywise_object_type)
        self.mission_planner.set_vehicle_connected(self.node.vehicle_connected)
        self.mission_planner.set_tile_server_url(self.node.osm_tile_server_url)
        self.mission_planner.set_tile_cache_dir(self.node.osm_tile_cache_dir)
        self.local_osm_tile_server_url = self.node.osm_tile_server_url
        self.local_osm_tile_cache_dir = self.node.osm_tile_cache_dir
        self.openstreetmap_tile_cache_dir = (
            self.node.osm_tile_cache_dir
            if self.node.map_source == 'OpenStreetMap'
            else OPENSTREETMAP_CACHE_DIR
        )
        self.osm_status_network = QNetworkAccessManager(self)
        self.osm_status_timer = QTimer(self)
        self.osm_status_timer.setInterval(1000)
        self.osm_status_timer.timeout.connect(self.poll_osm_server_status)
        self.osm_status_reply = None
        self.osm_server_status = None
        self.osm_ready_refresh_pending = False
        self._setup_map_config_button()
        self.node.get_logger().info(
            f'Control Tower map source: {self.node.map_source}; '
            f'OSM tile server: {self.node.osm_tile_server_url}'
        )
        self.on_map_source_selected(self.node.map_source)
        self.mission_planner.setMinimumWidth(120)
        self.main_splitter.addWidget(self.mission_planner)
        self.main_splitter.addWidget(twist_control_widget)
        self.main_splitter.setStretchFactor(0, 1)
        self.main_splitter.setStretchFactor(1, 0)
        self.main_splitter.setCollapsible(0, True)
        self.main_splitter.setCollapsible(1, False)

        self.scrollArea.setWidgetResizable(True)
        self.scrollArea.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.scrollArea.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        twist_control_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        central_layout.addWidget(self.main_splitter)
        self.setCentralWidget(central_widget)
        self._update_right_pane_min_width(force=True)
        self._schedule_panel_fit()
        QTimer.singleShot(0, self._try_load_startup_mission)

    def setup_heartbeat_status_row(self):
        """Add operator heartbeat status to the General vehicle status group."""
        label_style = 'color: #9ca3af; font-weight: 600;'
        value_style = 'color: #6b7280; font-weight: 700;'
        time_style = 'color: #6b7280; font-size: 9pt;'

        self.heartbeat_rx_static_label = QLabel('Heartbeat Rx State:')
        self.heartbeat_rx_static_label.setStyleSheet(label_style)
        self.heartbeat_rx_label = QLabel('UNKNOWN')
        self.heartbeat_rx_label.setStyleSheet(value_style)
        self.heartbeat_rx_time_label = QLabel('Last updated: N/A')
        self.heartbeat_rx_time_label.setStyleSheet(time_style)

        self.gridLayout_5.addWidget(self.heartbeat_rx_static_label, 7, 0)
        self.gridLayout_5.addWidget(self.heartbeat_rx_label, 7, 1)
        self.gridLayout_5.addWidget(self.heartbeat_rx_time_label, 7, 2)

    def setup_vehicle_status_layout(self):
        """Reflow vehicle status groups for the compact control-tower panel."""
        compact_group_style_template = (
            'QGroupBox#GROUP_NAME {'
            ' border: 1px solid #2d3748; border-radius: 12px;'
            ' margin-top: 16px; padding-top: 10px;'
            ' font-weight: 600;'
            ' background: qlineargradient(x1:0, y1:0, x2:0, y2:1,'
            ' stop:0 #232936, stop:1 #1a1d29);'
            ' font-size: 11pt;'
            '}'
            'QGroupBox#GROUP_NAME::title {'
            ' subcontrol-origin: margin; left: 20px; padding: 0 8px;'
            ' color: #60a5fa;'
            '}'
        )
        self.control_group.setStyleSheet(
            compact_group_style_template.replace('GROUP_NAME', 'control_group')
        )
        self.status_group.setStyleSheet(
            compact_group_style_template.replace('GROUP_NAME', 'status_group')
        )
        self.general_group.setTitle('')
        self.general_group.setFlat(True)
        self.general_group.setStyleSheet(
            'QGroupBox#general_group {'
            ' border: 0px; margin-top: 0px; padding-top: 0px; background: transparent;'
            '}'
            'QGroupBox#general_group::title {'
            ' height: 0px; margin: 0px; padding: 0px;'
            '}'
        )
        for group in (
            self.general_group,
            self.odom_group,
            self.world_pose_group,
            self.gnssfix_group,
        ):
            self.gridLayout_4.removeWidget(group)

        self.gridLayout_4.addWidget(self.general_group, 0, 0, 1, 2)
        self.odom_group.hide()
        self.world_pose_group.hide()
        self.gnssfix_group.hide()

        self.gridLayout_4.setContentsMargins(10, 0, 10, 10)
        self.gridLayout_4.setHorizontalSpacing(10)
        self.gridLayout_4.setVerticalSpacing(10)

        self.odom_vel_time_label = QLabel('Last updated: N/A')
        self.odom_vel_time_label.setStyleSheet('color: #6b7280; font-size: 9pt;')
        self.home_pose_static_label = QLabel('Home Pose [(x, y, z); (r, p, y)]:')
        self.home_pose_static_label.setStyleSheet('color: #9ca3af; font-weight: 600;')
        self.home_pose_label = QLabel('N/A')

        self.gridLayout_5.setHorizontalSpacing(14)
        self.gridLayout_5.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_5.setColumnStretch(0, 0)
        self.gridLayout_5.setColumnStretch(1, 1)
        self.gridLayout_5.setColumnStretch(2, 0)

        general_static_labels = (
            self.node_static_label,
            self.type_static_label,
            self.enuref_static_label,
            self.home_pose_static_label,
            self.estop_static_label,
            self.battery_static_label,
            self.copter_state_static_label,
            self.mission_state_static_label,
            self.heartbeat_rx_static_label,
            self.world_pose_static_label,
            self.odom_pos_static_label,
            self.odom_vel_static_label,
        )
        general_value_labels = (
            self.vehicle_node_label,
            self.vehicle_type_label,
            self.enuref_label,
            self.home_pose_label,
            self.estop_label,
            self.battery_label,
            self.copter_state_label,
            self.mission_state_label,
            self.heartbeat_rx_label,
            self.world_pose_pos_label,
            self.odom_pos_label,
            self.odom_vel_label,
        )
        general_time_labels = (
            self.estop_time_label,
            self.battery_time_label,
            self.copter_state_time_label,
            self.mission_state_time_label,
            self.heartbeat_rx_time_label,
            self.world_pose_time_label,
            self.odom_time_label,
            self.odom_vel_time_label,
        )
        for label in general_static_labels + general_time_labels:
            label.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Preferred)
            label.setContentsMargins(2, 0, 2, 0)
        for label in general_value_labels:
            label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
            label.setMinimumWidth(0)

        for row, widgets in enumerate(
            (
                (self.node_static_label, self.vehicle_node_label, None),
                (self.type_static_label, self.vehicle_type_label, None),
                (self.enuref_static_label, self.enuref_label, None),
                (self.home_pose_static_label, self.home_pose_label, None),
                (self.estop_static_label, self.estop_label, self.estop_time_label),
                (self.battery_static_label, self.battery_label, self.battery_time_label),
                (
                    self.copter_state_static_label,
                    self.copter_state_label,
                    self.copter_state_time_label,
                ),
                (
                    self.mission_state_static_label,
                    self.mission_state_label,
                    self.mission_state_time_label,
                ),
                (
                    self.heartbeat_rx_static_label,
                    self.heartbeat_rx_label,
                    self.heartbeat_rx_time_label,
                ),
            )
        ):
            static_label, value_label, time_label = widgets
            self.gridLayout_5.addWidget(static_label, row, 0)
            value_column_span = 2 if time_label is None else 1
            self.gridLayout_5.addWidget(value_label, row, 1, 1, value_column_span)
            if time_label is not None:
                self.gridLayout_5.addWidget(time_label, row, 2)

        odom_widgets = (
            self.odom_pos_static_label,
            self.odom_pos_label,
            self.odom_yaw_static_label,
            self.odom_yaw_label,
            self.odom_vel_static_label,
            self.odom_vel_label,
            self.odom_time_static_label,
            self.odom_time_label,
        )
        for widget in odom_widgets:
            self.gridLayout_8.removeWidget(widget)
        self.odom_pos_static_label.setText('Odom Pose [(x, y, z); (r, p, y)]:')
        self.odom_pos_label.setWordWrap(False)
        self.odom_pos_label.setMinimumWidth(0)
        self.odom_yaw_static_label.hide()
        self.odom_yaw_label.hide()
        self.odom_time_static_label.hide()
        self.odom_time_label.setText('Last updated: N/A')
        self.odom_time_label.setStyleSheet('color: #6b7280; font-size: 9pt;')
        self.odom_time_label.show()
        self.odom_vel_static_label.setText(
            'Odom Velocity [(v<sub>x</sub>, v<sub>y</sub>, v<sub>z</sub>); v<sub>yaw</sub>]:'
        )
        self.gridLayout_5.addWidget(self.odom_pos_static_label, 10, 0)
        self.gridLayout_5.addWidget(self.odom_pos_label, 10, 1)
        self.gridLayout_5.addWidget(self.odom_time_label, 10, 2)
        self.gridLayout_5.addWidget(self.odom_vel_static_label, 11, 0)
        self.gridLayout_5.addWidget(self.odom_vel_label, 11, 1)
        self.gridLayout_5.addWidget(self.odom_vel_time_label, 11, 2)

        self.gnss_title_label = QLabel('GNSS (fused)')
        self.gnss_title_label.setStyleSheet('color: #60a5fa; font-weight: 700;')
        self.gnss_time_label.setText('Last updated: N/A')
        self.gnss_time_label.setStyleSheet('color: #6b7280; font-size: 9pt;')
        self.gnss_fused_on_chip_static_label = QLabel('Fused on chip:')
        self.gnss_fused_on_chip_static_label.setStyleSheet('color: #9ca3af; font-weight: 600;')
        self.gnss_fused_on_chip_label = QLabel('N/A')
        self.gnss_fused_on_chip_label.setStyleSheet('color: #6b7280; font-weight: 700;')

        self.gnss_top_separator = QFrame()
        self.gnss_top_separator.setFrameShape(QFrame.HLine)
        self.gnss_top_separator.setFrameShadow(QFrame.Plain)
        self.gnss_top_separator.setStyleSheet('color: #2d3748; background-color: #2d3748;')
        self.gnss_bottom_separator = QFrame()
        self.gnss_bottom_separator.setFrameShape(QFrame.HLine)
        self.gnss_bottom_separator.setFrameShadow(QFrame.Plain)
        self.gnss_bottom_separator.setStyleSheet('color: #2d3748; background-color: #2d3748;')
        self.gnss_fields_widget = QWidget()
        self.gnss_fields_widget.setObjectName('gnss_fields_widget')
        self.gnss_fields_widget.setStyleSheet(
            'QWidget#gnss_fields_widget {'
            ' border: 1px solid #2d3748; border-radius: 12px;'
            ' background: qlineargradient(x1:0, y1:0, x2:0, y2:1,'
            ' stop:0 #232936, stop:1 #1a1d29);'
            '}'
        )
        self.gnss_fields_layout = QGridLayout(self.gnss_fields_widget)
        self.gnss_fields_layout.setContentsMargins(10, 10, 10, 10)
        self.gnss_fields_layout.setHorizontalSpacing(14)
        self.gnss_fields_layout.setVerticalSpacing(10)

        gnss_fields = (
            (self.gnss_fix_static_label, self.gnss_fix_label, 0, 0),
            (self.gnss_pos_static_label, self.gnss_pos_label, 0, 2),
            (
                self.gnss_last_rtcm_static_label,
                self.gnss_last_rtcm_correction_label,
                1,
                0,
            ),
            (
                self.gnss_accuracy_static_label,
                self.gnss_accuracy_label,
                1,
                2,
            ),
            (self.gnss_num_satellites_static_label, self.gnss_num_satellites_label, 2, 0),
            (self.gnss_fused_on_chip_static_label, self.gnss_fused_on_chip_label, 2, 2),
        )
        self.gnss_pos_static_label.setText('Position (lat, lon, alt, yaw):')
        self.gnss_accuracy_static_label.setText('Accuracy (horiz, vert, yaw):')
        self.gnss_head_static_label.hide()
        self.gnss_head_label.hide()
        self.gnss_time_static_label.hide()
        self.gridLayout_7.removeWidget(self.gnss_head_static_label)
        self.gridLayout_7.removeWidget(self.gnss_head_label)
        self.gridLayout_7.removeWidget(self.gnss_time_static_label)
        self.gridLayout_7.removeWidget(self.gnss_time_label)
        self.gridLayout_5.addWidget(self.gnss_top_separator, 12, 0, 1, 3)
        self.gridLayout_5.addWidget(self.gnss_title_label, 13, 0, 1, 2)
        self.gridLayout_5.addWidget(self.gnss_time_label, 13, 2)
        for static_label, value_label, row, column in gnss_fields:
            static_label.setStyleSheet('color: #9ca3af; font-weight: 600;')
            value_label.setStyleSheet('color: #6b7280; font-weight: 700;')
            static_label.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Preferred)
            value_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
            static_label.setContentsMargins(2, 0, 2, 0)
            self.gridLayout_7.removeWidget(static_label)
            self.gridLayout_7.removeWidget(value_label)
            self.gnss_fields_layout.addWidget(static_label, row, column)
            self.gnss_fields_layout.addWidget(value_label, row, column + 1)
        self.gridLayout_5.addWidget(self.gnss_fields_widget, 14, 0, 1, 3)
        self.gridLayout_5.addWidget(self.gnss_bottom_separator, 15, 0, 1, 3)
        self.gnss_time_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.gnss_time_label.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Preferred)
        self.gnss_time_label.setContentsMargins(2, 0, 2, 0)
        self.gnss_fields_layout.setColumnStretch(0, 0)
        self.gnss_fields_layout.setColumnStretch(1, 1)
        self.gnss_fields_layout.setColumnStretch(2, 0)
        self.gnss_fields_layout.setColumnStretch(3, 1)

        world_pose_widgets = (
            self.world_pose_static_label,
            self.world_pose_pos_label,
            self.world_pose_yaw_static_label,
            self.world_pose_yaw_label,
            self.world_pose_time_static_label,
            self.world_pose_time_label,
        )
        for widget in world_pose_widgets:
            self.gridLayout_6.removeWidget(widget)
        self.world_pose_static_label.setText('World Pose [(x, y, z); (r, p, y)]:')
        self.world_pose_pos_label.setWordWrap(False)
        self.world_pose_pos_label.setMinimumWidth(0)
        self.world_pose_yaw_static_label.hide()
        self.world_pose_yaw_label.hide()
        self.world_pose_time_static_label.hide()
        self.world_pose_time_label.setText('Last updated: N/A')
        self.world_pose_time_label.setStyleSheet('color: #6b7280; font-size: 9pt;')
        self.world_pose_time_label.show()
        self.gridLayout_5.addWidget(self.world_pose_static_label, 9, 0)
        self.gridLayout_5.addWidget(self.world_pose_pos_label, 9, 1)
        self.gridLayout_5.addWidget(self.world_pose_time_label, 9, 2)

    def setup_status_copy_buttons(self):
        """Add compact copy buttons beside high-value position fields."""
        for label in (
            self.enuref_label,
            self.home_pose_label,
            self.world_pose_pos_label,
            self.odom_pos_label,
            self.gnss_pos_label,
        ):
            self._wrap_label_with_copy_button(label)

    def _wrap_label_with_copy_button(self, value_label):
        target_layout = None
        target_position = None
        for layout in (self.gridLayout_5, self.gnss_fields_layout):
            index = layout.indexOf(value_label)
            if index < 0:
                continue
            target_layout = layout
            target_position = layout.getItemPosition(index)
            break
        if target_layout is None or target_position is None:
            return

        row, column, row_span, column_span = target_position
        target_layout.removeWidget(value_label)

        wrapper = QWidget()
        wrapper_layout = QHBoxLayout(wrapper)
        wrapper_layout.setContentsMargins(0, 0, 0, 0)
        wrapper_layout.setSpacing(4)
        wrapper_layout.addWidget(value_label, 1)

        copy_button = QPushButton('⧉')
        copy_button.setFixedSize(20, 18)
        copy_button.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        copy_button.setToolTip('Copy value')
        copy_button.setStyleSheet(
            'QPushButton {'
            ' color: #d1d5db; background-color: #374151; border: 1px solid #4b5563;'
            ' border-radius: 4px; padding: 0px; font-size: 10pt; font-weight: 600;'
            '}'
            'QPushButton:hover { background-color: #4b5563; }'
            'QPushButton:pressed { background-color: #1f2937; }'
        )

        copy_button.clicked.connect(
            lambda checked=False, label=value_label, button=copy_button: (
                self._copy_status_label_text(label, button)
            )
        )
        wrapper_layout.addWidget(copy_button)
        target_layout.addWidget(wrapper, row, column, row_span, column_span)

    def _copy_status_label_text(self, value_label, copy_button):
        copy_text = value_label.property('copy_text') or value_label.text()
        QApplication.clipboard().setText(copy_text)
        popup_position = copy_button.mapToGlobal(QPoint(copy_button.width() // 2, 0))
        QToolTip.showText(popup_position, 'copied', copy_button, copy_button.rect(), 1000)

    def setup_selectable_status_values(self):
        """Allow copying vehicle-status values without making field labels selectable."""
        value_label_names = (
            'vehicle_node_label',
            'vehicle_type_label',
            'enuref_label',
            'home_pose_label',
            'estop_label',
            'battery_label',
            'copter_state_label',
            'mission_state_label',
            'heartbeat_rx_label',
            'odom_pos_label',
            'odom_vel_label',
            'odom_yaw_label',
            'world_pose_pos_label',
            'world_pose_yaw_label',
            'gnss_fix_label',
            'gnss_pos_label',
            'gnss_head_label',
            'gnss_accuracy_label',
            'gnss_last_rtcm_correction_label',
            'gnss_num_satellites_label',
            'gnss_fused_on_chip_label',
        )
        for name in value_label_names:
            label = getattr(self, name, None)
            if label is None:
                continue
            label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            label.setCursor(Qt.IBeamCursor)
            label.installEventFilter(self)

    def eventFilter(self, watched, event):
        if (
            QEvent is not None
            and QKeySequence is not None
            and event.type() == QEvent.KeyPress
            and event.matches(QKeySequence.Copy)
        ):
            copy_text = watched.property('copy_text') if hasattr(watched, 'property') else None
            selected_text = watched.selectedText() if hasattr(watched, 'selectedText') else ''
            if copy_text and selected_text:
                QApplication.clipboard().setText(copy_text)
                return True
        return super().eventFilter(watched, event)

    def _setup_map_config_button(self):
        self.map_config_button = UpMenuButton('MAP CONFIG')
        self.map_config_button.setToolTip('Map source')
        self.map_config_button.setFixedWidth(132)
        self.map_config_menu = QMenu(self.map_config_button)
        self.map_config_menu.setStyleSheet(POPUP_MENU_STYLE)
        self.map_source_action_group = QActionGroup(self.map_config_menu)
        self.map_source_action_group.setExclusive(True)
        for source in ('OpenStreetMap', 'Local OSM server', 'None'):
            action = self.map_config_menu.addAction(source)
            action.setCheckable(True)
            action.setChecked(source == self.node.map_source)
            action.triggered.connect(
                lambda checked=False, selected_source=source: self.on_map_source_selected(
                    selected_source
                )
            )
            self.map_source_action_group.addAction(action)
        self.map_config_button.clicked.connect(self.show_map_config_menu)
        self.mission_planner.set_map_config_widget(self.map_config_button)
        self.mission_planner.osm_url_edit.editingFinished.connect(self.on_osm_config_edited)
        self.mission_planner.osm_cache_edit.editingFinished.connect(self.on_osm_config_edited)
        self.mission_planner.osm_cache_browse_button.clicked.connect(self.on_osm_config_edited)
        self.mission_planner.osm_refresh_requested.connect(self.on_osm_refresh_requested)

    def show_map_config_menu(self):
        menu_size = self.map_config_menu.sizeHint()
        popup_pos = self.map_config_button.mapToGlobal(QPoint(0, -menu_size.height()))
        self.map_config_menu.exec_(popup_pos)

    def on_osm_config_edited(self):
        if self.node.map_source == 'Local OSM server':
            self.node.osm_tile_server_url = self.mission_planner.osm_url_edit.text()
            self.node.osm_tile_cache_dir = self.mission_planner.osm_cache_edit.text()
            self.local_osm_tile_server_url = self.node.osm_tile_server_url
            self.local_osm_tile_cache_dir = self.node.osm_tile_cache_dir
            self.poll_osm_server_status()
        elif self.node.map_source == 'OpenStreetMap':
            self.node.osm_tile_server_url = OPENSTREETMAP_TILE_SERVER_URL
            self.node.osm_tile_cache_dir = self.mission_planner.osm_cache_edit.text()
            self.openstreetmap_tile_cache_dir = self.node.osm_tile_cache_dir

    def on_osm_refresh_requested(self):
        if self.node.map_source not in ('Local OSM server', 'OpenStreetMap'):
            return
        self.on_osm_config_edited()
        self.mission_planner.refresh_tiles(clear_disk=False)

    def poll_osm_server_status(self):
        if (
            self.node.map_source != 'Local OSM server'
            or self.osm_status_reply is not None
            or not self.node.osm_tile_server_url
        ):
            return
        metadata_url = self.node.osm_tile_server_url.rstrip('/') + '/metadata'
        request = QNetworkRequest(QUrl(metadata_url))
        request.setRawHeader(b'User-Agent', b'Waywiser-ControlTowerNode/1.0')
        self.osm_status_reply = self.osm_status_network.get(request)
        self.osm_status_reply.finished.connect(self.on_osm_status_reply)

    def on_osm_status_reply(self):
        reply = self.osm_status_reply
        self.osm_status_reply = None
        if reply is None:
            return
        try:
            status = 'Busy'
            if reply.error() == QNetworkReply.NoError:
                payload = json.loads(bytes(reply.readAll()).decode('utf-8'))
                status = payload.get('status', 'Ready')
                if status == 'Ready' and self.osm_server_status != 'Ready':
                    self.osm_status_timer.stop()
                    self.schedule_osm_ready_refresh()
            self.mission_planner.set_osm_server_status(status)
            self.osm_server_status = status
        except Exception:
            self.mission_planner.set_osm_server_status('Busy')
            self.osm_server_status = 'Busy'
        finally:
            reply.deleteLater()

    def schedule_osm_ready_refresh(self):
        if self.osm_ready_refresh_pending:
            return
        self.osm_ready_refresh_pending = True
        # Delay by 1 s so the vehicle's enuref is received and applied to the canvas
        # before tile coordinates are computed.  Firing immediately (0 ms) caused
        # the refresh to use the canvas default enuref (Gothenburg placeholder)
        # rather than the actual simulation location, resulting in 404 tile
        # responses that blocked the correct tiles for 30 s.
        QTimer.singleShot(1000, self.refresh_osm_tiles_after_ready)

    def refresh_osm_tiles_after_ready(self):
        self.osm_ready_refresh_pending = False
        if self.node.map_source != 'Local OSM server' or self.osm_server_status != 'Ready':
            return
        self.mission_planner.refresh_tiles(clear_disk=False)
        self.node.get_logger().info('OSM tile server is ready; refreshing visible map tiles.')

    def _try_load_startup_mission(self):
        if self._startup_mission_loaded:
            return

        mission_file = self.node.startup_route_file.strip()
        if not mission_file:
            self._startup_mission_loaded = True
            return

        if self.node.control_vehicle_node_fqn.strip() and not self.node.vehicle_connected:
            return

        mission_file = os.path.expanduser(os.path.expandvars(mission_file))
        try:
            self.mission_planner.set_enu_ref(self.node.enuref)
            loaded_count = self.mission_planner.load_mission_file(mission_file)
            self._startup_mission_loaded = True
            self.node.get_logger().info(
                f'Loaded startup mission with {loaded_count} points from: {mission_file}'
            )
        except Exception as exc:
            self._startup_mission_loaded = True
            self.node.get_logger().error(str(exc))

    def on_map_source_selected(self, source):
        """Switch the Mission Planner map background source."""
        if not hasattr(self, 'mission_planner'):
            return
        source = self.node._normalize_map_source(source)
        self.node.map_source = source
        if source == 'OpenStreetMap':
            self.node.osm_tile_server_url = OPENSTREETMAP_TILE_SERVER_URL
            self.node.osm_tile_cache_dir = self.openstreetmap_tile_cache_dir
        elif source == 'Local OSM server':
            self.node.osm_tile_server_url = self.local_osm_tile_server_url
            self.node.osm_tile_cache_dir = self.local_osm_tile_cache_dir
        self._set_checked_map_source(source)
        self.mission_planner.set_tile_server_url(self.node.osm_tile_server_url)
        self.mission_planner.set_tile_cache_dir(self.node.osm_tile_cache_dir)
        self.mission_planner.set_map_source(source)
        self.mission_planner.set_osm_config_mode(source)
        if source == 'Local OSM server':
            self.mission_planner.set_osm_server_status('Busy')
            self.osm_server_status = 'Busy'
            self.osm_status_timer.start()
            self.poll_osm_server_status()
            self.mission_planner.refresh_tiles()
        else:
            self.osm_status_timer.stop()

    def _set_checked_map_source(self, source):
        if not hasattr(self, 'map_source_action_group'):
            return
        for action in self.map_source_action_group.actions():
            action.setChecked(action.text() == source)

    def _update_right_pane_min_width(self, force=False):
        """Keep the control pane wide enough; grow the window minimum if needed."""
        if not hasattr(self, 'twist_control_widget') or not hasattr(self, 'main_splitter'):
            return
        if hasattr(self, 'fit_right_panel_button') and self.fit_right_panel_button.isChecked():
            self._fit_right_panel_width()
            return

        total_width = max(
            self.centralWidget().width() if self.centralWidget() else self.width(), 1
        )
        right_min_width = max(int(total_width * 0.25), 320)
        right_preferred_width = max(
            right_min_width,
            self.scrollAreaWidgetContents.minimumSizeHint().width() + 56,
        )
        map_min_width = self.mission_planner.minimumWidth()
        splitter_handle_width = max(self.main_splitter.handleWidth(), 1)
        target_right_width = right_preferred_width if force else right_min_width
        required_window_width = target_right_width + map_min_width + splitter_handle_width

        self.twist_control_widget.setMinimumWidth(right_min_width)
        self.twist_control_widget.setMaximumWidth(16777215)
        self.setMinimumWidth(required_window_width)

        sizes = self.main_splitter.sizes()
        if force or (len(sizes) >= 2 and sizes[1] < right_min_width):
            available = max(sum(sizes), total_width, required_window_width)
            self.main_splitter.setSizes(
                [max(map_min_width, available - target_right_width), target_right_width]
            )

    def _right_panel_required_width(self):
        """Return the width needed by the control panel's current contents."""
        contents_layout = self.scrollAreaWidgetContents.layout()
        if contents_layout is not None:
            contents_layout.activate()

        contents_width = max(
            self.scrollAreaWidgetContents.sizeHint().width(),
            self.scrollAreaWidgetContents.minimumSizeHint().width(),
            contents_layout.sizeHint().width() if contents_layout is not None else 0,
        )
        scroll_chrome_width = self.scrollArea.frameWidth() * 2 + 12
        if self.scrollArea.verticalScrollBarPolicy() != Qt.ScrollBarAlwaysOff:
            scroll_chrome_width += self.scrollArea.verticalScrollBar().sizeHint().width()

        top_layout_width = self.button_layout.sizeHint().width()
        outer_layout = self.twist_control_widget.layout()
        if outer_layout is not None:
            margins = outer_layout.contentsMargins()
            top_layout_width += margins.left() + margins.right()

        return max(320, contents_width + scroll_chrome_width, top_layout_width) + (
            self._right_panel_fit_padding
        )

    def _fit_right_panel_width(self):
        """Fit and maintain the control pane width while its toggle is latched."""
        if (
            self._fitting_right_panel
            or not hasattr(self, 'fit_right_panel_button')
            or not self.fit_right_panel_button.isChecked()
            or not hasattr(self, 'main_splitter')
        ):
            return

        self._fitting_right_panel = True
        try:
            desired_width = self._right_panel_required_width()
            map_min_width = max(self.mission_planner.minimumWidth(), 120)
            handle_width = max(self.main_splitter.handleWidth(), 1)

            sizes = self.main_splitter.sizes()
            current_width = sizes[1] if len(sizes) >= 2 else 0
            width_delta = desired_width - current_width
            should_resize = (
                not self._right_panel_fit_initialized
                or width_delta > self._right_panel_fit_grow_threshold
                or width_delta < -self._right_panel_fit_shrink_threshold
            )
            fitted_width = desired_width if should_resize else current_width
            required_window_width = fitted_width + map_min_width + handle_width

            screen = QApplication.primaryScreen()
            if screen is not None:
                required_window_width = min(
                    required_window_width, screen.availableGeometry().width()
                )

            if self.width() < required_window_width:
                self.resize(required_window_width, self.height())

            available_width = max(self.main_splitter.width(), 1)
            fitted_width = min(
                fitted_width,
                max(320, available_width - map_min_width - handle_width),
            )
            self.twist_control_widget.setMinimumWidth(fitted_width)
            self.setMinimumWidth(fitted_width + map_min_width + handle_width)
            if should_resize:
                self.main_splitter.setSizes(
                    [max(map_min_width, available_width - fitted_width), fitted_width]
                )
            self._right_panel_fit_initialized = True
        finally:
            self._fitting_right_panel = False

    def on_fit_right_panel_toggled(self, checked):
        """Latch automatic control-panel width fitting on or off."""
        if checked:
            self.fit_right_panel_button.setToolTip(
                'Automatic control-panel width fitting is enabled'
            )
            self.scrollArea.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            self.scrollArea.horizontalScrollBar().setValue(0)
            self.main_splitter.setHandleWidth(0)
            self._right_panel_fit_initialized = False
            self._fit_right_panel_width()
            self.right_panel_fit_timer.start()
            return

        self.right_panel_fit_timer.stop()
        self.scrollArea.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.fit_right_panel_button.setToolTip(
            'Keep the control panel wide enough to show its complete contents'
        )
        self.main_splitter.setHandleWidth(self._main_splitter_handle_width)
        self._right_panel_fit_initialized = False
        self._update_right_pane_min_width()

    def _fit_startup_panel_sizes(self):
        """Fit startup panes after Qt has computed real size hints."""
        if not hasattr(self, 'twist_control_widget') or not hasattr(self, 'main_splitter'):
            return
        if self.fit_right_panel_button.isChecked():
            self._fit_right_panel_width()
            return

        QApplication.processEvents()
        total_width = max(
            self.centralWidget().width() if self.centralWidget() else self.width(), 1
        )
        right_width = max(
            int(total_width * 0.25),
            320,
            self.scrollAreaWidgetContents.minimumSizeHint().width() + 72,
        )
        map_width = max(self.mission_planner.minimumWidth(), 120)
        required_width = right_width + map_width + max(self.main_splitter.handleWidth(), 1)

        screen = QApplication.primaryScreen()
        if screen is not None:
            required_width = min(required_width, screen.availableGeometry().width())

        if self.width() < required_width:
            self.resize(required_width, self.height())

        available = max(self.main_splitter.width(), required_width)
        right_width = min(right_width, max(320, available - map_width))
        self.main_splitter.setSizes([max(map_width, available - right_width), right_width])
        self.mission_planner.adjust_mission_controls_height()

    def _schedule_panel_fit(self):
        QTimer.singleShot(0, self._fit_startup_panel_sizes)
        QTimer.singleShot(150, self._fit_startup_panel_sizes)
        QTimer.singleShot(500, self._fit_startup_panel_sizes)
        QTimer.singleShot(1000, self._fit_startup_panel_sizes)

    def _move_button_layout_to_top(self):
        """Move the existing bottom button row above the control/status panels."""
        try:
            parent_layout = self.verticalLayout_2
            for index in range(parent_layout.count()):
                item = parent_layout.itemAt(index)
                if item.layout() is self.button_layout:
                    parent_layout.takeAt(index)
                    break
            parent_layout.insertLayout(0, self.button_layout)
        except Exception as exc:
            self.node.get_logger().warn(f'Could not move control buttons to the top: {exc}')

    def setup_connections(self):
        """Connect UI signals to slots."""
        self.vehicle_node_button.clicked.connect(self.show_vehicle_node_dialog)
        self.usage_button.clicked.connect(self.show_usage_guide)
        self.mission_planner_button.toggled.connect(self.on_mission_planner_toggled)
        self.fit_right_panel_button.toggled.connect(self.on_fit_right_panel_toggled)
        self.fit_right_panel_button.setChecked(True)
        self.mission_planner.send_mission_requested.connect(self.on_send_mission_requested)
        self.auto_arm_checkbox.stateChanged.connect(self.on_auto_arm_changed)
        self.hover_hold_checkbox.stateChanged.connect(self.on_hover_hold_changed)
        self.auto_lift_off_checkbox.stateChanged.connect(self.on_auto_lift_off_changed)

    def on_mission_planner_toggled(self, checked):
        """Enable or disable waypoint editing on the map."""
        if not self.node.vehicle_control_enabled:
            self.mission_planner_button.blockSignals(True)
            self.mission_planner_button.setChecked(False)
            self.mission_planner_button.blockSignals(False)
            self.mission_planner.set_planning_enabled(False)
            return
        self.mission_planner.set_planning_enabled(checked)
        self.mission_planner.adjust_mission_controls_height()
        self.mission_planner_button.setText('MISSION PLANNER')

    def on_send_mission_requested(self, points, altitude, speed):
        """Send the mission shown in the map to the selected vehicle."""
        if self.node.publish_mission(points, altitude, speed):
            self.mission_planner_button.setChecked(False)

    def on_auto_arm_changed(self, state):
        """Handle auto arm checkbox state change."""
        self.node.set_auto_arm_enabled(state == Qt.Checked)

    def on_hover_hold_changed(self, state):
        """Handle hover hold checkbox state change."""
        self.node.set_hover_hold_enabled(state == Qt.Checked)

    def on_auto_lift_off_changed(self, state):
        """Handle auto lift-off checkbox state change."""
        self.node.set_auto_lift_off_enabled(state == Qt.Checked)

    def sync_control_options_from_node(self):
        """Update control option widgets from the node without writing parameters back."""
        self.auto_lift_off_checkbox.blockSignals(True)
        self.auto_lift_off_checkbox.setChecked(self.node.auto_lift_off_enabled)
        self.auto_lift_off_checkbox.blockSignals(False)

    def setup_audio(self):
        """Set up audio for low battery warning beep."""
        self.sound_effect = None
        self.temp_wav_file = None

        # Check for available audio output devices and PulseAudio presence
        if not check_pulseaudio():
            self.node.get_logger().info('PulseAudio not reachable. Audio warning system disabled.')
            return

        with suppress_stderr():
            available_devices = QAudioDeviceInfo.availableDevices(QAudio.AudioOutput)

        if not available_devices:
            self.node.get_logger().info(
                'No audio output devices found. Audio warning system disabled.'
            )
            return

        try:
            # Generate a beep sound
            sample_rate = 44100
            duration = 0.5  # seconds
            frequency = 440.0  # Hz (A4 note)

            # Create sound wave
            t = np.linspace(0, duration, int(sample_rate * duration), False)
            wave_data = np.sin(2 * np.pi * frequency * t)

            # Convert to 16-bit PCM
            wave_data = (wave_data * 32767).astype(np.int16)

            # Create a temporary WAV file
            temp_dir = tempfile.mkdtemp()
            self.temp_wav_file = os.path.join(temp_dir, 'control_tower_beep.wav')

            # Write WAV file
            with wave.open(self.temp_wav_file, 'w') as wav_file:
                wav_file.setnchannels(1)  # Mono
                wav_file.setsampwidth(2)  # 2 bytes = 16 bits
                wav_file.setframerate(sample_rate)
                wav_file.writeframes(wave_data.tobytes())

            # Setup sound effect
            with suppress_stderr():
                self.sound_effect = QSoundEffect()
                self.sound_effect.setSource(QUrl.fromLocalFile(self.temp_wav_file))
                self.sound_effect.setVolume(0.5)

            self.node.get_logger().info('Audio warning system initialized.')

        except Exception as e:
            self.node.get_logger().warn(f'Failed to initialize audio: {e}')
            self.sound_effect = None

    def play_warning_beep(self):
        """Play warning beep sound."""
        if self.sound_effect is not None:
            try:
                if self.sound_effect.isLoaded():
                    self.sound_effect.play()
            except Exception as e:
                self.node.get_logger().warn(f'Failed to play warning sound: {e}')

    def show_usage_guide(self):
        """Show the usage guide dialog."""
        dialog = UsageGuideDialog(self)
        dialog.exec()

    def show_vehicle_node_dialog(self, auto_connect=False):
        """Show dialog to select vehicle node."""
        dialog = VehicleNodeDialog(
            self.node, self.node.control_vehicle_node_fqn, auto_connect, self
        )
        if dialog.exec() == QDialog.DialogCode.Accepted:
            vehicle_node = dialog.get_vehicle_node_name()
            if vehicle_node:
                self.node.control_vehicle_node_fqn = vehicle_node
                self.node.last_vehicle_pose = {
                    'pose': None,
                    'stamp': self.node.get_clock().now(),
                    'frame_id': '',
                }
                self.node.last_odom = {
                    'pose': None,
                    'twist': None,
                    'stamp': self.node.get_clock().now(),
                    'frame_id': '',
                }
                self.node.last_home_pose = {
                    'pose': None,
                    'stamp': self.node.get_clock().now(),
                    'frame_id': '',
                }
                self.mission_planner.prepare_for_selected_vehicle()
                self.node.vehicle_namespace = RosUtils.parent_namespace_from_fqn(
                    self.node.control_vehicle_node_fqn
                )
                self.node._init_mux_sources()
                self.node.request_params_from_vehicle_node()
                self.sync_control_options_from_node()
                self.update_ui_for_vehicle_type()
                self.update_control_group_state()
                self._schedule_panel_fit()
                self._try_load_startup_mission()

    def update_control_group_state(self):
        """Reflect whether the control pane has an active vehicle target."""
        has_vehicle_selected = bool(self.node.control_vehicle_node_fqn.strip())
        control_enabled = has_vehicle_selected and self.node.vehicle_control_enabled
        if hasattr(self, 'mission_planner'):
            self.mission_planner.set_vehicle_connected(
                self.node.vehicle_connected and self.node.vehicle_control_enabled
            )
            if not control_enabled:
                self.mission_planner.set_planning_enabled(False)

        # Control group dimming
        self.control_group.setEnabled(control_enabled)
        self.control_group_opacity.setOpacity(1.0 if control_enabled else 0.45)
        if not has_vehicle_selected:
            control_tooltip = 'Select a vehicle node to enable vehicle control.'
        elif not self.node.vehicle_control_enabled:
            control_tooltip = 'Passive monitoring mode: vehicle control is disabled.'
        else:
            control_tooltip = ''
        self.control_group.setToolTip(control_tooltip)

        if hasattr(self, 'mission_planner_button'):
            self.mission_planner_button.setEnabled(control_enabled)
            if not control_enabled and self.mission_planner_button.isChecked():
                self.mission_planner_button.blockSignals(True)
                self.mission_planner_button.setChecked(False)
                self.mission_planner_button.blockSignals(False)
            self.mission_planner_button.setToolTip(control_tooltip)

        # Status group dimming
        self.status_group.setEnabled(has_vehicle_selected)
        self.status_group_opacity.setOpacity(1.0 if has_vehicle_selected else 0.45)
        self.status_group.setToolTip(
            '' if has_vehicle_selected else 'Select a vehicle node to enable vehicle status.'
        )

        # Vehicle node button text
        if has_vehicle_selected:
            self.vehicle_node_button.setText('CHANGE VEHICLE NODE (F1)')
        else:
            self.vehicle_node_button.setText('SELECT VEHICLE NODE (F1)')

    def update_ui_for_vehicle_type(self):
        """Update UI elements based on the waywise_object_type."""
        self.mission_planner.set_vehicle_type(self.node.waywise_object_type)
        if self.node.waywise_object_type == 'quadcopter':
            self.auto_arm_status_box.show()
            self.hover_hold_status_box.show()
            self.auto_lift_off_status_box.show()
        else:
            self.auto_arm_status_box.hide()
            self.hover_hold_status_box.hide()
            self.auto_lift_off_status_box.hide()

    def keyPressEvent(self, event):
        """Handle key press events."""
        key = event.key()
        modifiers = event.modifiers()

        # Add key to pressed set
        self.node.keys_pressed.add(key)

        # Handle F1 for changing vehicle node
        if key == Qt.Key.Key_F1:
            self.show_vehicle_node_dialog()
            return

        # Handle F2 for showing usage guide
        if key == Qt.Key.Key_F12:
            self.show_usage_guide()
            return

        # Handle emergency stop even when Control Tower is in passive monitoring mode.
        if key == Qt.Key.Key_E and modifiers & Qt.KeyboardModifier.ControlModifier:
            if modifiers & Qt.KeyboardModifier.ShiftModifier:
                self.node.set_emergency_stop(active=False)
            else:
                self.node.set_emergency_stop(active=True)
            return

        if not self.node.vehicle_control_enabled:
            return

        # Handle speed adjustments
        if key == Qt.Key.Key_I:
            self.node.update_speed(linear_delta=self.node.linear_speed_increment)
        elif key == Qt.Key.Key_K:
            self.node.update_speed(linear_delta=-self.node.linear_speed_increment)
        elif key == Qt.Key.Key_O:
            self.node.update_speed(angular_delta=self.node.angular_speed_increment)
        elif key == Qt.Key.Key_L:
            if modifiers & Qt.KeyboardModifier.ControlModifier:
                if self.node.waywise_object_type == 'quadcopter':
                    if modifiers & Qt.KeyboardModifier.ShiftModifier:
                        self.node.set_auto_landing_active(not self.node.auto_landing_active)
                        state_str = 'ENABLED' if self.node.auto_landing_active else 'DISABLED'
                        self.node.get_logger().info(f'Auto landing {state_str}')
                    else:
                        self.node.start_auto_lift_off()
                else:
                    self.node.get_logger().warn(
                        'Auto lift-off/landing is only available for quadcopters.'
                    )
            else:
                self.node.update_speed(angular_delta=-self.node.angular_speed_increment)

        # Handle quadcopter arm/disarm
        if (
            not event.isAutoRepeat()
            and key == Qt.Key.Key_M
            and modifiers & Qt.KeyboardModifier.ControlModifier
        ):
            if modifiers & Qt.KeyboardModifier.ShiftModifier:
                self.node.request_arm_state(arm=False)
            else:
                self.node.request_arm_state(arm=True)

    def keyReleaseEvent(self, event):
        """Handle key release events."""
        key = event.key()
        self.node.keys_pressed.discard(key)

    def spin_ros(self):
        """Spin ROS node once."""
        try:
            if rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0)
            else:
                # ROS has been shutdown, close the application
                self.close()
        except Exception:
            # ROS context is invalid, stop trying to spin
            self.close()

    def publish_command(self):
        """Process keys and publish twist command."""
        try:
            if rclpy.ok():
                self.node.process_keys_and_publish()
        except Exception:
            # ROS context invalid, stop the timer
            self.publish_timer.stop()

    def update_display(self):
        """Update the status display with modern UI elements."""
        has_vehicle_selected = bool(self.node.control_vehicle_node_fqn.strip())
        self.update_control_group_state()

        if (
            self._handled_vehicle_status_reset_generation
            != self.node.vehicle_status_reset_generation
        ):
            self._reset_vehicle_status_fields()
            self._handled_vehicle_status_reset_generation = (
                self.node.vehicle_status_reset_generation
            )
            self.node.vehicle_status_reset_pending = False

        self.vehicle_node_label.setText(
            self.node.control_vehicle_node_fqn if has_vehicle_selected else 'No vehicle selected'
        )
        self.vehicle_node_label.setStyleSheet(
            f'color: {self.green_color}; font-weight: 700;'
            if has_vehicle_selected
            else f'color: {self.inactive_muted_color}; font-weight: 700;'
        )
        self.vehicle_type_label.setText(self.node.waywise_object_type.replace('_', ' ').title())
        self.vehicle_type_label.setStyleSheet(f'color: {self.green_color}; font-weight: 700;')

        if self.node.vehicle_status_cleared:
            self.enuref_label.setText('N/A')
            self.enuref_label.setProperty('copy_text', '')
            self.enuref_label.setStyleSheet(f'color: {self.gray_color}; font-weight: 700;')
        else:
            self.enuref_label.setText(
                f'({self.node.enuref[0]:.6f}°, {self.node.enuref[1]:.6f}°, {self.node.enuref[2]:.2f}m)'
            )
            self.enuref_label.setProperty(
                'copy_text',
                f'({self.node.enuref[0]:.15g}°, {self.node.enuref[1]:.15g}°, {self.node.enuref[2]:.15g}m)',
            )
            self.enuref_label.setStyleSheet(f'color: {self.green_color}; font-weight: 700;')

        self._update_home_pose_display()

        if self.node.vehicle_status_reset_pending:
            self._reset_vehicle_status_fields()
            self._handled_vehicle_status_reset_generation = (
                self.node.vehicle_status_reset_generation
            )
            self.node.vehicle_status_reset_pending = False

        self._update_estop_display()
        self._update_heartbeat_rx_display()
        self._update_quadcopter_state_display()
        self._update_mission_state_display()
        self._update_speed_display()
        self._update_battery_display()
        self._update_odom_display()
        self._update_world_pose_display()
        self._update_gnss_display()
        self._update_mission_planner_display()
        self._update_ros_time_display()
        self._update_right_pane_min_width()

        # Show/hide low battery warning
        is_battery_low = self.node.is_battery_low()

        self.warning_label.setText('[!!!] WARNING: LOW BATTERY VOLTAGE! [!!!]')
        self.warning_label.show()
        if is_battery_low:
            # Play beep sound (only once per low battery state)
            if not self.last_battery_warning:
                self.play_warning_beep()
                self.last_battery_warning = True
        else:
            self.warning_label.hide()
            self.last_battery_warning = False

    def _update_ros_time_display(self):
        """Show the current ROS clock in simulation or wall-clock form."""
        now_seconds = self.node.get_clock().now().nanoseconds / 1e9
        if self.node.use_sim_time:
            text = f'ROS Time: [{now_seconds:.3f} s]'
            tooltip = 'Current ROS simulation timestamp in seconds.'
        else:
            timestamp = datetime.fromtimestamp(now_seconds)
            text = f'ROS Time: [{timestamp:%Y-%m-%d %H:%M:%S}]'
            tooltip = 'Current ROS wall-clock timestamp.'
        self.ros_time_label.setText(text)
        self.ros_time_label.setToolTip(tooltip)

    def _reset_vehicle_status_fields(self):
        """Clear vehicle runtime status fields after a simulation reset."""
        value_style = f'color: {self.gray_color}; font-weight: 700;'
        time_style = f'color: {self.gray_color}; font-size: 9pt;'

        for label, text in (
            (self.enuref_label, 'N/A'),
            (self.home_pose_label, 'N/A'),
            (self.estop_label, 'UNKNOWN'),
            (self.battery_label, 'N/A'),
            (self.copter_state_label, 'UNKNOWN'),
            (self.mission_state_label, 'UNKNOWN'),
            (self.heartbeat_rx_label, 'UNKNOWN'),
            (self.world_pose_pos_label, 'N/A'),
            (self.odom_pos_label, 'N/A'),
            (self.odom_vel_label, 'N/A'),
            (self.gnss_fix_label, 'UNKNOWN'),
            (self.gnss_pos_label, 'N/A'),
            (self.gnss_accuracy_label, 'N/A'),
            (self.gnss_last_rtcm_correction_label, 'N/A'),
            (self.gnss_num_satellites_label, 'N/A'),
            (self.gnss_fused_on_chip_label, 'N/A'),
        ):
            label.setText(text)
            label.setStyleSheet(value_style)
            label.setProperty('copy_text', '')

        self.world_pose_yaw_label.setText('')
        self.odom_yaw_label.setText('')
        self.gnss_head_label.setText('')
        for label in (
            self.estop_time_label,
            self.battery_time_label,
            self.copter_state_time_label,
            self.mission_state_time_label,
            self.heartbeat_rx_time_label,
            self.world_pose_time_label,
            self.odom_time_label,
            self.odom_vel_time_label,
            self.gnss_time_label,
        ):
            label.setText('Last updated: N/A')
            label.setStyleSheet(time_style)

        self.warning_label.hide()
        self.last_battery_warning = False
        self.mission_planner.clear_vehicle_overlay()
        self.mission_planner.clear_home_position()
        self.mission_planner.set_visual_markers([])

    def _update_mission_planner_display(self):
        """Update Mission Planner context from the active vehicle state."""
        self.mission_planner.set_vehicle_type(self.node.waywise_object_type)
        self.mission_planner.set_enu_ref(self.node.enuref)
        if self.node.vehicle_status_cleared:
            self.mission_planner.clear_vehicle_overlay()
            self.mission_planner.clear_home_position()
            self.mission_planner.set_visual_markers([])
            return
        if self.node.last_home_pose['pose'] is not None:
            self.mission_planner.set_home_position(
                self.node.last_home_pose['pose'].position.x,
                self.node.last_home_pose['pose'].position.y,
            )
        else:
            self.mission_planner.clear_home_position()
        self._try_load_startup_mission()
        self.mission_planner.set_vehicle_overlay_model(self.node.vehicle_overlay_model)
        self.node._refresh_visual_markers()
        self.mission_planner.set_visual_markers(self.node.visual_markers)

        pose = None
        frame_id = ''
        if self.node.last_vehicle_pose['pose'] is not None:
            pose = self.node.last_vehicle_pose['pose']
            frame_id = self.node.last_vehicle_pose['frame_id']
        elif self.node.last_odom['pose'] is not None:
            pose = self.node.last_odom['pose']
            frame_id = self.node.last_odom['frame_id']

        if pose is None:
            self.mission_planner.clear_vehicle_pose()
            return

        orientation = pose.orientation
        _, _, yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        x = pose.position.x
        y = pose.position.y

        # Transform pose to world_frame if it arrives in a different frame.
        world_frame = self.node.world_frame
        if frame_id and frame_id != world_frame:
            try:
                t = self.node._tf_buffer.lookup_transform(
                    world_frame,
                    frame_id,
                    rclpy.time.Time(),
                )
                tx = t.transform.translation.x
                ty = t.transform.translation.y
                q = t.transform.rotation
                _, _, frame_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                import math

                cos_fy = math.cos(frame_yaw)
                sin_fy = math.sin(frame_yaw)
                orig_x, orig_y = x, y
                x = orig_x * cos_fy - orig_y * sin_fy + tx
                y = orig_x * sin_fy + orig_y * cos_fy + ty
                yaw += frame_yaw
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                pass  # No transform available yet — render at raw pose

        self.mission_planner.set_vehicle_pose(x, y, yaw)

    def _get_time_ago_and_color(self, stamp):
        """Format time since stamp and return a corresponding UI color."""
        time_since = int((self.node.get_clock().now() - stamp).nanoseconds / 1e9)
        if time_since > 60:
            return (f'{time_since // 60}m ago', self.red_color)
        elif time_since > 1:
            return (f'{time_since}s ago', self.yellow_color)
        else:
            return ('now', self.green_color)

    def _update_estop_display(self):
        """Update the emergency stop status on the UI."""
        if self.node.last_emergency_stop_state['msg'] is None:
            self.estop_label.setText('UNKNOWN')
            self.estop_label.setStyleSheet(f'color: {self.gray_color}; font-weight: 700;')
            self.estop_time_label.setText('Last updated: N/A')
            self.estop_time_label.setStyleSheet(f'color: {self.gray_color}; font-size: 9pt;')
            return

        time_label, time_based_color = self._get_time_ago_and_color(
            self.node.last_emergency_stop_state['stamp']
        )
        self.estop_time_label.setText(f'Last updated: {time_label}')
        self.estop_time_label.setStyleSheet(f'color: {time_based_color}; font-size: 9pt;')

        state = self.node.last_emergency_stop_state['msg'].state
        if state == EmergencyStopState.ACTIVE:
            self.estop_label.setText('[!] ACTIVE')
            self.estop_label.setStyleSheet(f'color: {self.red_color}; font-weight: 700;')
        elif state == EmergencyStopState.CLEAR:
            self.estop_label.setText('CLEAR')
            self.estop_label.setStyleSheet(f'color: {self.green_color}; font-weight: 700;')
        else:
            self.estop_label.setText('[?] UNKNOWN')
            self.estop_label.setStyleSheet(f'color: {self.gray_color}; font-weight: 700;')

    def _update_heartbeat_rx_display(self):
        """Update vehicle-reported Control Tower heartbeat receive state."""
        rx_msg = self.node.last_control_tower_heartbeat_rx_state.get('msg')
        time_text = 'Last updated: N/A'
        time_color = self.gray_color

        if rx_msg is None:
            state_text = 'UNKNOWN'
            color = self.gray_color
            tooltip = 'No vehicle heartbeat receive state has been received.'
        elif rx_msg.state == HeartbeatRxState.TIMEOUT:
            state_text = 'TIMEOUT'
            color = self.red_color
            tooltip = 'Vehicle has timed out receiving Control Tower heartbeat.'
        elif rx_msg.state == HeartbeatRxState.NO_HEARTBEAT:
            state_text = 'NO HEARTBEAT'
            color = self.gray_color
            tooltip = 'Vehicle has not received a Control Tower heartbeat yet.'
        elif rx_msg.state == HeartbeatRxState.ACTIVE:
            state_text = 'ACTIVE'
            color = self.green_color
            tooltip = 'Vehicle is receiving Control Tower heartbeat normally.'
        else:
            state_text = 'UNKNOWN'
            color = self.yellow_color
            tooltip = 'Vehicle reported an unknown heartbeat receive state.'

        if rx_msg is not None:
            report_stamp = self.node.last_control_tower_heartbeat_rx_state['stamp']
            time_str, time_color = self._get_time_ago_and_color(report_stamp)
            if rx_msg.age_s < 0.0:
                age_text = 'never'
            else:
                elapsed_since_report = max(
                    0.0,
                    (self.node.get_clock().now() - report_stamp).nanoseconds / 1e9,
                )
                estimated_age = rx_msg.age_s + elapsed_since_report
                age_text = f'{estimated_age:.2f}s'
            state_text = f'{state_text} | age: {age_text}'
            time_text = f'Last updated: {time_str}'

        self.heartbeat_rx_label.setText(state_text)
        self.heartbeat_rx_label.setStyleSheet(f'color: {color}; font-weight: 700;')
        self.heartbeat_rx_label.setToolTip(
            f'{tooltip} Age is measured from the heartbeat generation timestamp and '
            'extrapolated between vehicle state updates. '
            f'Vehicle timeout: {self.node.control_tower_heartbeat_timeout:.2f}s.'
        )
        self.heartbeat_rx_time_label.setText(time_text)
        self.heartbeat_rx_time_label.setStyleSheet(f'color: {time_color}; font-size: 9pt;')
        self.heartbeat_rx_time_label.setToolTip(tooltip)

    def _set_indicator_light(self, widget, is_on):
        """Set a status light to green, red, or gray."""
        if is_on is None:
            color = self.gray_color
        else:
            color = self.green_color if is_on else self.red_color
        widget.setText('●')
        widget.setStyleSheet(f'color: {color}; font-size: 14pt; font-weight: 700;')

    def _update_quadcopter_state_display(self):
        """Update quadcopter-only high-level status display."""
        is_quadcopter = self.node.waywise_object_type == 'quadcopter'

        self.copter_state_static_label.setVisible(is_quadcopter)
        self.copter_state_label.setVisible(is_quadcopter)
        self.copter_state_time_label.setVisible(is_quadcopter)

        if not is_quadcopter:
            return

        msg = self.node.last_quadcopter_state['msg']
        if msg is None:
            state_text = 'UNKNOWN'
            time_str, time_color = 'never', self.gray_color
            state_code = None
        else:
            state_code = msg.state_code
            state_text = msg.state_str.upper()
            time_str, time_color = self._get_time_ago_and_color(
                self.node.last_quadcopter_state['stamp']
            )

        self.copter_state_label.setText(state_text)
        self.copter_state_time_label.setText(f'Last updated: {time_str}')
        self.copter_state_time_label.setStyleSheet(f'color: {time_color}; font-size: 9pt;')

        # Set color based on state
        if state_code is not None:
            if state_code == QuadcopterState.EMERGENCY:
                color = self.red_color
            elif state_code in [
                QuadcopterState.ARMED,
                QuadcopterState.IN_FLIGHT,
                QuadcopterState.ON_MISSION,
                QuadcopterState.HOVERING,
            ]:
                color = self.green_color
            elif state_code in [
                QuadcopterState.ARMING,
                QuadcopterState.LANDING,
                QuadcopterState.LIFTING_OFF,
                QuadcopterState.AUTO_LIFTING_OFF,
                getattr(QuadcopterState, 'RETURNING_HOME', 13),
            ]:
                color = '#EFA90B'
            else:
                color = '#60a5fa'  # Blue
            self.copter_state_label.setStyleSheet(f'color: {color}; font-weight: 700;')

        if self.node.auto_landing_active:
            self.warning_label.setText('AUTO LANDING ACTIVE - MANUAL INPUT TO CANCEL')
            self.warning_label.show()
        elif self.node.auto_lift_off_active:
            self.warning_label.setText('AUTO LIFT-OFF ACTIVE - MANUAL INPUT TO CANCEL')
            self.warning_label.show()
        elif self.warning_label.text() == 'AUTO LANDING ACTIVE - MANUAL INPUT TO CANCEL':
            self.warning_label.hide()
            self.warning_label.setText('')
        elif self.warning_label.text() == 'AUTO LIFT-OFF ACTIVE - MANUAL INPUT TO CANCEL':
            self.warning_label.hide()
            self.warning_label.setText('')

    _MISSION_STATE_STRINGS = {
        MissionState.IDLE: ('Idle', None),
        MissionState.WAITING_FOR_ROUTE: ('Waiting for route', '#60a5fa'),
        MissionState.WAITING_FOR_VEHICLE_INIT: ('Waiting for init', '#60a5fa'),
        MissionState.WAITING_FOR_EMERGENCY_STOP_CLEAR: ('Waiting for E-stop', '#fbbf24'),
        MissionState.WAITING_FOR_GNSS_ACCURACY: ('Waiting for GNSS', '#fbbf24'),
        MissionState.WAITING_FOR_HEARTBEAT: ('Waiting for heartbeat', '#fbbf24'),
        MissionState.FOLLOW_ROUTE_INIT: ('Route: Init', None),
        MissionState.FOLLOW_ROUTE_LIFT_OFF: ('Route: Lift off', None),
        MissionState.FOLLOW_ROUTE_GOTO_BEGIN: ('Route: Go to start', None),
        MissionState.FOLLOW_ROUTE_FOLLOWING: ('Following route', None),
        MissionState.FOLLOW_ROUTE_APPROACHING_END_GOAL: ('Route: Approaching end', None),
        MissionState.FOLLOW_ROUTE_FINISHED: ('Route: Finished', None),
        MissionState.RETURN_HOME_INIT: ('RTH: Init', '#EFA90B'),
        MissionState.RETURN_HOME_LIFT_OFF: ('RTH: Lift off', '#EFA90B'),
        MissionState.RETURN_HOME_CRUISING: ('RTH: Cruising', '#EFA90B'),
        MissionState.RETURN_HOME_LANDING: ('RTH: Landing', '#EFA90B'),
    }

    def _update_mission_state_display(self):
        """Update mission state display for all vehicle types."""
        has_vehicle = bool(self.node.control_vehicle_node_fqn.strip())
        self.mission_state_static_label.setVisible(has_vehicle)
        self.mission_state_label.setVisible(has_vehicle)
        self.mission_state_time_label.setVisible(has_vehicle)

        if not has_vehicle:
            return

        msg = self.node.last_mission_state['msg']
        if msg is None:
            self.mission_state_label.setText('UNKNOWN')
            self.mission_state_label.setStyleSheet(f'color: {self.gray_color}; font-weight: 700;')
            self.mission_state_time_label.setText('Last updated: N/A')
            self.mission_state_time_label.setStyleSheet('color: #6b7280; font-size: 9pt;')
            return

        state_text, state_color = self._MISSION_STATE_STRINGS.get(msg.state, ('Unknown', None))
        time_str, time_color = self._get_time_ago_and_color(self.node.last_mission_state['stamp'])
        label_color = state_color or time_color
        self.mission_state_label.setText(state_text.upper())
        self.mission_state_label.setStyleSheet(f'color: {label_color}; font-weight: 700;')
        self.mission_state_time_label.setText(f'Last updated: {time_str}')
        self.mission_state_time_label.setStyleSheet(f'color: {time_color}; font-size: 9pt;')

    def _update_speed_display(self):
        """Update speed bars and labels on the UI."""
        max_lin = self.node.max_linear_speed
        max_ang = self.node.max_angular_speed

        self.linear_config_label.setText(f'> Configured: {self.node.linear_speed:.2f} m/s')
        self.angular_config_label.setText(f'> Configured: {self.node.angular_speed:.2f} rad/s')

        lin_pct = int((self.node.linear_speed / max_lin) * 100) if max_lin > 0 else 0
        ang_pct = int((self.node.angular_speed / max_ang) * 100) if max_ang > 0 else 0

        self.linear_progress_bg.setValue(lin_pct)
        self.angular_progress_bg.setValue(ang_pct)

        self._update_active_speed(lin_pct, ang_pct)

    def _update_active_speed(self, lin_pct, ang_pct):
        """Update active speed display based on keypresses."""
        is_lin = any(
            k in self.node.keys_pressed
            for k in [
                Qt.Key.Key_W,
                Qt.Key.Key_X,
                Qt.Key.Key_Q,
                Qt.Key.Key_E,
                Qt.Key.Key_R,
                Qt.Key.Key_F,
            ]
        )
        is_tl = Qt.Key.Key_A in self.node.keys_pressed
        is_tr = Qt.Key.Key_D in self.node.keys_pressed
        is_ang = is_tl or is_tr

        msg = self.node.last_emergency_stop_state['msg']
        if msg is not None and msg.state == EmergencyStopState.ACTIVE:
            self.linear_value_label.setText(f'{0:+.2f} m/s')
            self.angular_value_label.setText(f'{0:+.2f} rad/s')
            if is_lin or is_ang:
                self.start_estop_blink()
        else:
            self._update_progress_bars(is_lin, is_ang, lin_pct, ang_pct)
            self._update_joysticks(is_tl, is_tr, lin_pct, ang_pct)
            self.linear_value_label.setText(f'{self.node.current_twist.linear.x:+.2f} m/s')
            self.angular_value_label.setText(f'{self.node.current_twist.angular.z:+.2f} rad/s')

    def _update_progress_bars(self, is_lin, is_ang, lin_pct, ang_pct):
        """Update speed progress bars."""
        self.linear_progress_fg.setValue(lin_pct if is_lin else 0)
        self.angular_progress_fg.setValue(ang_pct if is_ang else 0)

    def _update_joysticks(self, is_tl, is_tr, lin_pct, ang_pct):
        """Update the position of the joystick thumbs based on current controls."""
        # Max displacement of the thumb stick from the center (56 is base, 18 is stick)
        # Center is at x=19, y=19.
        # Max movement is 19 pixels in any direction.
        max_displacement = 19

        # Left stick:
        # Y-axis (Forward/Back): W moves forward (up, -y), X moves backward (down, +y)
        # X-axis (Lateral): Q moves left (-x), E moves right (+x) [Quadcopter]
        l_x_disp = 0
        l_y_disp = 0

        if Qt.Key.Key_W in self.node.keys_pressed:
            l_y_disp = -int((lin_pct / 100.0) * max_displacement)
        elif Qt.Key.Key_X in self.node.keys_pressed:
            l_y_disp = int((lin_pct / 100.0) * max_displacement)

        if self.node.waywise_object_type == 'quadcopter':
            if Qt.Key.Key_Q in self.node.keys_pressed:
                l_x_disp = -int((lin_pct / 100.0) * max_displacement)
            elif Qt.Key.Key_E in self.node.keys_pressed:
                l_x_disp = int((lin_pct / 100.0) * max_displacement)

        self.left_joystick_stick.move(19 + l_x_disp, 19 + l_y_disp)

        # Right stick:
        # X-axis (Yaw): A moves left (-x), D moves right (+x)
        # Y-axis (Vertical): R moves up (-y), F moves down (+y) [Quadcopter]
        r_x_disp = 0
        r_y_disp = 0

        if is_tl:
            r_x_disp = -int((ang_pct / 100.0) * max_displacement)
        elif is_tr:
            r_x_disp = int((ang_pct / 100.0) * max_displacement)

        if self.node.waywise_object_type == 'quadcopter':
            if Qt.Key.Key_R in self.node.keys_pressed:
                r_y_disp = -int((lin_pct / 100.0) * max_displacement)
            elif Qt.Key.Key_F in self.node.keys_pressed:
                r_y_disp = int((lin_pct / 100.0) * max_displacement)

        self.right_joystick_stick.move(19 + r_x_disp, 19 + r_y_disp)

    def _update_battery_display(self):
        """Update battery information on the UI."""
        if self.node.last_battery_state['msg'] is None:
            self.battery_label.setText('N/A')
            self.battery_label.setStyleSheet(f'color: {self.gray_color}; font-weight: 700;')
            self.battery_time_label.setText('Last updated: N/A')
            self.battery_time_label.setStyleSheet(f'color: {self.gray_color}; font-size: 9pt;')
            return

        time_label, time_based_color = self._get_time_ago_and_color(
            self.node.last_battery_state['stamp']
        )
        self.battery_time_label.setText(f'Last updated: {time_label}')
        self.battery_time_label.setStyleSheet(f'color: {time_based_color}; font-size: 9pt;')

        voltage = self.node.last_battery_state['msg'].voltage
        is_low = self.node.last_battery_state['msg'].state == BatteryState.LOW_VOLTAGE
        txt = f'[!] {voltage:.2f} V' if is_low else f'{voltage:.2f} V'
        clr = self.red_color if is_low else time_based_color

        self.battery_label.setText(txt)
        self.battery_label.setStyleSheet(f'color: {clr}; font-weight: 700;')

    def _update_odom_display(self):
        """Update odometry information on the UI."""
        if self.node.last_odom['pose'] is None:
            self.odom_pos_label.setText('N/A')
            self.odom_pos_label.setProperty('copy_text', '')
            self.odom_vel_label.setText('N/A')
            self.odom_vel_label.setProperty('copy_text', '')
            self.odom_yaw_label.setText('')
            for label in (self.odom_pos_label, self.odom_vel_label):
                label.setStyleSheet(f'color: {self.gray_color}; font-weight: 700;')
            for label in (self.odom_time_label, self.odom_vel_time_label):
                label.setText('Last updated: N/A')
                label.setStyleSheet(f'color: {self.gray_color}; font-size: 9pt;')
            return

        time_label, time_based_color = self._get_time_ago_and_color(self.node.last_odom['stamp'])
        self._set_odom_ui_values(time_label, time_based_color)

    def _set_odom_ui_values(self, time_label, time_based_color):
        """Set odometry UI labels and styles."""
        p = self.node.last_odom['pose']
        roll, pitch, yaw = euler_from_quaternion(
            [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        )
        roll_deg = roll * 180.0 / 3.14159
        pitch_deg = pitch * 180.0 / 3.14159
        yaw_deg = yaw * 180.0 / 3.14159

        self.odom_pos_label.setText(
            f'[({p.position.x:.2f}, {p.position.y:.2f}, {p.position.z:.2f})m; '
            f'({roll_deg:.1f}, {pitch_deg:.1f}, {yaw_deg:.1f})°]'
        )
        self.odom_pos_label.setProperty(
            'copy_text',
            f'[({p.position.x:.15g}, {p.position.y:.15g}, {p.position.z:.15g})m; '
            f'({roll_deg:.15g}, {pitch_deg:.15g}, {yaw_deg:.15g})°]',
        )
        self.odom_pos_label.setMinimumWidth(self.odom_pos_label.sizeHint().width())
        self.odom_yaw_label.setText('')

        if self.node.last_odom['twist'] is not None:
            twist = self.node.last_odom['twist']
            vx = twist.linear.x
            vy = twist.linear.y
            vz = twist.linear.z
            yaw_rate_deg = twist.angular.z * 180.0 / 3.14159
        else:
            vx = 0.0
            vy = 0.0
            vz = 0.0
            yaw_rate_deg = 0.0
        self.odom_vel_label.setText(f'[({vx:.2f}, {vy:.2f}, {vz:.2f}) m/s; {yaw_rate_deg:.2f}°/s]')
        self.odom_vel_label.setProperty(
            'copy_text',
            f'[({vx:.15g}, {vy:.15g}, {vz:.15g}) m/s; {yaw_rate_deg:.15g}°/s]',
        )
        odom_time_text = f'Last updated: {time_label}'
        self.odom_time_label.setText(odom_time_text)
        self.odom_vel_time_label.setText(odom_time_text)

        style = f'color: {time_based_color}; font-weight: 700;'
        for lbl in [
            self.odom_pos_label,
            self.odom_yaw_label,
            self.odom_vel_label,
        ]:
            lbl.setStyleSheet(style)
        time_style = f'color: {time_based_color}; font-size: 9pt;'
        self.odom_time_label.setStyleSheet(time_style)
        self.odom_vel_time_label.setStyleSheet(time_style)

    def _update_world_pose_display(self):
        """Update world pose labels (ENU position and roll/pitch/yaw)."""
        if self.node.last_vehicle_pose['pose'] is None:
            self.world_pose_pos_label.setText('N/A')
            self.world_pose_pos_label.setProperty('copy_text', '')
            self.world_pose_yaw_label.setText('')
            self.world_pose_pos_label.setStyleSheet(f'color: {self.gray_color}; font-weight: 700;')
            self.world_pose_time_label.setText('Last updated: N/A')
            self.world_pose_time_label.setStyleSheet(f'color: {self.gray_color}; font-size: 9pt;')
            return

        time_label, time_based_color = self._get_time_ago_and_color(
            self.node.last_vehicle_pose['stamp']
        )
        self.world_pose_time_label.setText(f'Last updated: {time_label}')
        self.world_pose_time_label.setStyleSheet(f'color: {time_based_color}; font-size: 9pt;')

        pose = self.node.last_vehicle_pose['pose']
        roll_rad, pitch_rad, yaw_rad = euler_from_quaternion(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )
        roll_deg = roll_rad * 180.0 / 3.14159265359
        pitch_deg = pitch_rad * 180.0 / 3.14159265359
        yaw_deg = yaw_rad * 180.0 / 3.14159265359

        self.world_pose_pos_label.setText(
            f'[({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f})m; '
            f'({roll_deg:.1f}, {pitch_deg:.1f}, {yaw_deg:.1f})°]'
        )
        self.world_pose_pos_label.setProperty(
            'copy_text',
            f'[({pose.position.x:.15g}, {pose.position.y:.15g}, {pose.position.z:.15g})m; '
            f'({roll_deg:.15g}, {pitch_deg:.15g}, {yaw_deg:.15g})°]',
        )
        self.world_pose_pos_label.setMinimumWidth(self.world_pose_pos_label.sizeHint().width())
        self.world_pose_yaw_label.setText('')

        style = f'color: {time_based_color}; font-weight: 700;'
        self.world_pose_pos_label.setStyleSheet(style)
        self.world_pose_yaw_label.setStyleSheet(style)

    def _update_home_pose_display(self):
        """Update the drone-published home pose displayed under ENU reference."""
        pose = self.node.last_home_pose['pose']
        if self.node.vehicle_status_cleared or pose is None:
            self.home_pose_label.setText('N/A')
            self.home_pose_label.setProperty('copy_text', '')
            self.home_pose_label.setStyleSheet(f'color: {self.gray_color}; font-weight: 700;')
            return

        x, y, z = pose.position.x, pose.position.y, pose.position.z
        roll_rad, pitch_rad, yaw_rad = euler_from_quaternion(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )
        roll_deg = roll_rad * 180.0 / 3.14159265359
        pitch_deg = pitch_rad * 180.0 / 3.14159265359
        yaw_deg = yaw_rad * 180.0 / 3.14159265359

        self.home_pose_label.setText(
            f'[({x:.2f}, {y:.2f}, {z:.2f})m; ({roll_deg:.1f}, {pitch_deg:.1f}, {yaw_deg:.1f})°]'
        )
        self.home_pose_label.setProperty(
            'copy_text',
            f'[({x:.15g}, {y:.15g}, {z:.15g})m; '
            f'({roll_deg:.15g}, {pitch_deg:.15g}, {yaw_deg:.15g})°]',
        )
        self.home_pose_label.setMinimumWidth(self.home_pose_label.sizeHint().width())
        self.home_pose_label.setStyleSheet(f'color: {self.green_color}; font-weight: 700;')

    def _update_gnss_display(self):
        """Update GNSS fix and accuracy information on the UI."""
        if self.node.last_nav_sat_fix_extended['msg'] is None:
            for label, text in (
                (self.gnss_fix_label, 'UNKNOWN'),
                (self.gnss_pos_label, 'N/A'),
                (self.gnss_accuracy_label, 'N/A'),
                (self.gnss_last_rtcm_correction_label, 'N/A'),
                (self.gnss_num_satellites_label, 'N/A'),
                (self.gnss_fused_on_chip_label, 'N/A'),
            ):
                label.setText(text)
                label.setStyleSheet(f'color: {self.gray_color}; font-weight: 700;')
            self.gnss_pos_label.setProperty('copy_text', '')
            self.gnss_head_label.setText('')
            self.gnss_time_label.setText('Last updated: N/A')
            self.gnss_time_label.setStyleSheet(f'color: {self.gray_color}; font-size: 9pt;')
            return

        msg = self.node.last_nav_sat_fix_extended['msg']
        time_label, time_based_color = self._get_time_ago_and_color(
            self.node.last_nav_sat_fix_extended['stamp']
        )
        self.gnss_time_label.setText(f'Last updated: {time_label}')
        self.gnss_time_label.setStyleSheet(f'color: {time_based_color}; font-size: 9pt;')

        fix_type = msg.fix_type
        fix_text = 'UNKNOWN'
        fix_color = self.gray_color

        if fix_type == NavSatFixExtended.FIX_TYPE_NO_FIX:
            fix_text = 'NO FIX'
            fix_color = self.red_color
        elif fix_type == NavSatFixExtended.FIX_TYPE_2D_FIX:
            fix_text = '2D FIX'
            fix_color = self.blue_color
        elif fix_type == NavSatFixExtended.FIX_TYPE_3D_FIX:
            fix_text = '3D FIX'
            fix_color = self.yellow_color
        elif fix_type == NavSatFixExtended.FIX_TYPE_DEAD_RECKONING_ONLY_FIX:
            fix_text = 'DEAD RECKONING'
            fix_color = '#942478'
        elif fix_type == NavSatFixExtended.FIX_TYPE_GNSS_DR_COMBINED_FIX:
            fix_text = 'GNSS + DR'
            fix_color = self.green_color
        elif fix_type == NavSatFixExtended.FIX_TYPE_TIME_ONLY_FIX:
            fix_text = 'TIME ONLY'
            fix_color = self.red_color

        self.gnss_fix_label.setText(fix_text)
        self.gnss_fix_label.setStyleSheet(f'color: {fix_color}; font-weight: 700;')

        self.gnss_pos_label.setText(
            f'({msg.latitude:.2f}°, {msg.longitude:.2f}°, {msg.altitude:.2f}m, {msg.yaw:.1f}°)'
        )
        self.gnss_pos_label.setProperty(
            'copy_text',
            f'({msg.latitude:.15g}°, {msg.longitude:.15g}°, {msg.altitude:.15g}m, '
            f'{msg.yaw:.15g}°)',
        )
        self.gnss_pos_label.setStyleSheet(f'color: {time_based_color}; font-weight: 700;')

        self.gnss_head_label.setText('')
        self.gnss_head_label.setStyleSheet(f'color: {time_based_color}; font-weight: 700;')

        self.gnss_accuracy_label.setText(
            f'({msg.horizontal_accuracy:.2f}m, {msg.vertical_accuracy:.2f}m, '
            f'{msg.heading_accuracy:.2f}°)'
        )
        self.gnss_accuracy_label.setStyleSheet(f'color: {self.green_color}; font-weight: 700;')

        if msg.last_rtcm_correction_age > 12:
            msg.last_rtcm_correction_age = 12
        gnss_last_rtcm_correction_label_text = self.node.rtcm_correction_age_mapping[
            msg.last_rtcm_correction_age
        ]
        self.gnss_last_rtcm_correction_label.setText(gnss_last_rtcm_correction_label_text)
        self.gnss_last_rtcm_correction_label.setStyleSheet(
            f'color: {self.green_color}; font-weight: 700;'
        )
        self.gnss_num_satellites_label.setText(f'{msg.num_satellites}')
        self.gnss_num_satellites_label.setStyleSheet(
            f'color: {self.green_color}; font-weight: 700;'
        )
        self.gnss_fused_on_chip_label.setText('YES' if msg.is_fused_on_chip else 'NO')
        self.gnss_fused_on_chip_label.setStyleSheet(
            f'color: {time_based_color}; font-weight: 700;'
        )

    def start_estop_blink(self):
        if (
            hasattr(self, 'is_estop_label_blinking') and self.is_estop_label_blinking
        ):  # Prevent multiple timers
            return
        self.is_estop_label_blinking = True
        self.blink_count = 0
        self.blink_timer = QTimer()
        self.blink_timer.timeout.connect(self.toggle_estop_visibility)
        self.blink_timer.start(250)  # Blink every 250ms (4 times per second)

    def toggle_estop_visibility(self):
        # Toggle between visible and hidden
        self.estop_label.setVisible(not self.estop_label.isVisible())

        self.blink_count += 1
        if self.blink_count >= 8:  # 8 blinks × 250ms = 2 seconds
            self.blink_timer.stop()
            self.estop_label.setVisible(True)  # Make sure it ends visible
            self.is_estop_label_blinking = False  # Reset flag

    def closeEvent(self, event):
        """Handle window close event."""
        # Stop all timers
        self.ros_timer.stop()
        self.publish_timer.stop()
        self.display_timer.stop()

        # Cleanup audio
        if self.sound_effect is not None:
            self.sound_effect.stop()
            self.sound_effect = None

        # Remove temporary WAV file
        if self.temp_wav_file and os.path.exists(self.temp_wav_file):
            try:
                os.remove(self.temp_wav_file)
            except Exception:
                pass

        # Publish zero velocity before closing (if ROS is still active)
        try:
            if rclpy.ok():
                self.node.twist_publisher.publish(Twist())
        except Exception:
            # ROS context already shutdown, ignore
            pass

        event.accept()

    def resizeEvent(self, event):
        """Keep the splitter minimums aligned with the current window width."""
        super().resizeEvent(event)
        self._update_right_pane_min_width()

    def showEvent(self, event):
        """Re-fit panes once the window is actually shown."""
        super().showEvent(event)
        self._schedule_panel_fit()


def check_pulseaudio():
    """Check if PulseAudio is running and reachable."""
    try:
        # Quick check for PulseAudio server presence
        return (
            subprocess.run(
                ['pulseaudio', '--check'], capture_output=True, timeout=1.0, check=False
            ).returncode
            == 0
        )
    except (subprocess.SubprocessError, FileNotFoundError):
        return False


def main():
    if not _PYQT5_AVAILABLE:
        print(
            'ERROR: PyQt5 is not installed. control_tower requires PyQt5 (python3-pyqt5) to run.',
            file=sys.stderr,
        )
        sys.exit(1)

    # Install signal handler to close Qt app on Ctrl+C
    import signal

    # Suppress warnings during early initialization
    with suppress_stderr():
        rclpy.init()
        app = QApplication(sys.argv)

    node = ControlTowerNode()
    gui = ControlTowerUI(node)

    # Handle Ctrl+C gracefully
    def signal_handler(_sig, _frame):
        gui.close()
        app.quit()

    signal.signal(signal.SIGINT, signal_handler)

    # Allow Ctrl+C to work by processing events periodically
    timer = QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(100)

    gui.showMaximized()

    exit_code = app.exec()

    # Cleanup
    try:
        node.destroy_node()
    except Exception:
        pass

    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
