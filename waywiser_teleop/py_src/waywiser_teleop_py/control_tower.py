#!/usr/bin/env python3
"""ROS 2 node for vehicle teleoperation using a keyboard and PyQt5 GUI."""

from contextlib import contextmanager
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
    from rcl_interfaces.srv import GetParameters, SetParameters
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState, Joy
    from std_msgs.msg import Bool, String
    import tf2_ros
    from tf_transformations import euler_from_quaternion
    from visualization_msgs.msg import MarkerArray

try:
    with suppress_stderr():
        from PyQt5.QtCore import QPoint, Qt, QTimer, QUrl
        from PyQt5.QtMultimedia import QAudio, QAudioDeviceInfo, QSoundEffect
        from PyQt5.QtNetwork import QNetworkAccessManager, QNetworkReply, QNetworkRequest
        from PyQt5.QtWidgets import (
            QActionGroup,
            QApplication,
            QDialog,
            QGraphicsOpacityEffect,
            QHBoxLayout,
            QMainWindow,
            QMenu,
            QMessageBox,
            QPushButton,
            QSizePolicy,
            QSplitter,
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
    QMainWindow = _QtStub  # type: ignore[misc,assignment]
    QWidget = _QtStub  # type: ignore[misc,assignment]
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
    loadUi = None


from waywiser_core.msg import (  # noqa: E402
    BatteryState,
    MissionState,
    NavSatFixExtended,
    PathWithTwists,
    QuadcopterState,
)
from waywiser_py.waywiser_utils import RELIABLE_TRANSIENT_LOCAL_QOS, RosUtils  # noqa: E402
from waywiser_teleop_py.route_messages import build_path_with_twists  # noqa: E402
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
    from waywiser_teleop_py.route_planner import (  # noqa: E402
        ACTIVE_BUTTON_STYLE,
        OPENSTREETMAP_CACHE_DIR,
        OPENSTREETMAP_TILE_SERVER_URL,
        POPUP_MENU_STYLE,
        RoutePlannerWidget,
        UpMenuButton,
    )
else:
    UI_BASE_PATH = None
    RoutePlannerWidget = None
    UpMenuButton = None
    ACTIVE_BUTTON_STYLE = ''
    OPENSTREETMAP_CACHE_DIR = ''
    OPENSTREETMAP_TILE_SERVER_URL = ''
    POPUP_MENU_STYLE = ''


class ControlTower(Node):
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
        self.vehicle_namespace = RosUtils.parent_namespace_from_fqn(
            self.control_vehicle_node_fqn
        )

        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('startup_linear_speed', 0.5)
        self.declare_parameter('startup_angular_speed', 1.0)
        self.declare_parameter('linear_speed_increment', 0.1)
        self.declare_parameter('angular_speed_increment', 0.1)
        self.declare_parameter('publish_rate', 10.0)

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

        # Wait for sim time if needed
        use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        if use_sim_time:
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
        self.nav_sat_fix_extended_topic = ''
        self.emergency_stop_status_topic = ''
        self.emergency_stop_update_topic = ''
        self.joint_states_topic = ''
        self.route_topic = ''
        self.autopilot_state_control_topic = ''
        self.enuref = [57.71495867, 12.89134921, 0.0]
        self.waywise_object_type = 'generic'
        self.vehicle_connected = False

        # Subscribers (will be created after fetching topics)
        self.odom_subscriber = None
        self.vehicle_pose_subscriber = None
        self.battery_state_subscriber = None
        self.quadcopter_state_subscriber = None
        self.mission_status_subscriber = None
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
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.mux_sources = {}  # source_name -> {topic, priority, timeout, last_msg, last_stamp}
        self.mux_source_subscribers = []
        self.active_mux_source = 'None'
        self._init_mux_sources()

        # Joy subscribers and timers
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.joy_watchdog_timer = self.create_timer(self.joy_timeout, self.joy_watchdog_callback)
        self.joy_watchdog_timer.cancel()

        # State variables
        self.last_emergency_stop_state = {'msg': None, 'stamp': self.get_clock().now()}
        self.last_odom = {
            'pose': None,
            'twist': None,
            'stamp': self.get_clock().now(),
            'frame_id': '',
        }
        self.last_vehicle_pose = {'pose': None, 'stamp': self.get_clock().now(), 'frame_id': ''}
        self.last_battery_state = {'msg': None, 'stamp': self.get_clock().now()}
        self.last_quadcopter_state = {'msg': None, 'stamp': self.get_clock().now()}
        self.last_mission_state = {'msg': None, 'stamp': self.get_clock().now()}
        self.last_nav_sat_fix_extended = {'msg': None, 'stamp': self.get_clock().now()}
        self.vehicle_overlay_model = VehicleOverlayModel()
        self.visual_marker_store = {}
        self.visual_markers = []

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
        self.vehicle_overlay_model = VehicleOverlayModel()

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

    def publish_route(self, route_points, altitude, speed):
        """Publish a planned route to the selected vehicle and request autopilot enable."""
        if not self.vehicle_connected:
            self.get_logger().warn('Connect a vehicle node before sending a route.')
            return False

        if len(route_points) < 1:
            self.get_logger().warn('Route is empty. Add at least one waypoint before sending.')
            return False

        if self.route_publisher is None or self.autopilot_state_control_publisher is None:
            self._create_publishers()

        if self.route_publisher is None:
            self.get_logger().warn('Route publisher is not available.')
            return False

        route_altitude = float(altitude) if self.waywise_object_type == 'quadcopter' else 0.0
        route_speed = max(float(speed), 0.0)
        msg = build_path_with_twists(
            route_points,
            self.get_clock().now().to_msg(),
            frame_id='map',
            altitude=route_altitude,
            speed=route_speed,
        )
        self.route_publisher.publish(msg)

        if self.autopilot_state_control_publisher is not None:
            autopilot_msg = Bool()
            autopilot_msg.data = True
            self.autopilot_state_control_publisher.publish(autopilot_msg)

        self.get_logger().info(
            f'Sent route with {len(route_points)} waypoint(s), speed={route_speed:.2f} m/s, '
            f'z={route_altitude:.2f} m to {self._prefix_with_vehicle_namespace(self.route_topic)}'
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

    def odom_callback(self, msg):
        """Handle odometry messages."""
        # Nav_msgs/Odometry: pose and twist are nested in PoseWithCovariance / TwistWithCovariance
        self.last_odom['pose'] = msg.pose.pose
        self.last_odom['twist'] = msg.twist.twist
        self.last_odom['stamp'] = self.get_clock().now()
        self.last_odom['frame_id'] = msg.header.frame_id

    def vehicle_pose_callback(self, msg):
        """Handle vehicle pose messages."""
        # Geometry_msgs/PoseStamped: world pose is in the 'pose' field
        self.last_vehicle_pose['pose'] = msg.pose
        self.last_vehicle_pose['stamp'] = self.get_clock().now()
        self.last_vehicle_pose['frame_id'] = msg.header.frame_id

    def robot_description_callback(self, msg):
        """Parse robot_description into a top-view overlay model."""
        if not msg.data:
            return
        try:
            self.vehicle_overlay_model.load_urdf(msg.data)
        except Exception as exc:
            self.get_logger().warn(f'Could not parse robot_description for map overlay: {exc}')

    def joint_states_callback(self, msg):
        """Update the top-view overlay with the latest joint positions."""
        self.vehicle_overlay_model.update_joint_states(msg.name, msg.position)

    def visual_marker_array_callback(self, msg):
        """Cache vehicle visualization markers for drawing on the route map."""
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
            self.visual_marker_store[key] = marker
        self.visual_markers = list(self.visual_marker_store.values())

    def battery_state_callback(self, msg):
        """Handle battery state messages."""
        self.last_battery_state['msg'] = msg
        self.last_battery_state['stamp'] = self.get_clock().now()

    def quadcopter_state_callback(self, msg):
        """Handle quadcopter high-level state updates."""
        self.last_quadcopter_state['msg'] = msg
        self.last_quadcopter_state['stamp'] = self.get_clock().now()

        # Sync auto landing state from vehicle node
        self.auto_landing_active = msg.state_code == QuadcopterState.LANDING
        self.auto_lift_off_active = msg.state_code == QuadcopterState.AUTO_LIFTING_OFF

    def mission_status_callback(self, msg):
        """Handle mission state updates."""
        self.last_mission_state['msg'] = msg
        self.last_mission_state['stamp'] = self.get_clock().now()

    def nav_sat_fix_extended_callback(self, msg):
        """Handle extended GPS/FIX messages."""
        self.last_nav_sat_fix_extended['msg'] = msg
        self.last_nav_sat_fix_extended['stamp'] = self.get_clock().now()

    def emergency_stop_state_subscriber_callback(self, msg):
        """Handle emergency stop state updates."""
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
                self.declare_parameter(f'mux_input.{name}.topic', '')
                self.declare_parameter(f'mux_input.{name}.timeout', 0.0)
                self.declare_parameter(f'mux_input.{name}.priority', 0)
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
        if self.waywise_object_type != 'quadcopter':
            self.get_logger().warn('Arm/disarm keys are only available for quadcopters.')
            return

        if arm:
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
        try:
            service_name = f'/{node_name}/get_parameters'
            client = self.node.create_client(GetParameters, service_name)

            # Wait briefly for service
            if not client.wait_for_service(timeout_sec=0.5):
                self.node.destroy_client(client)
                return False

            # Request the vehicle_interface_type parameter
            request = GetParameters.Request()
            request.names = ['vehicle_interface_type']

            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=0.5)

            self.node.destroy_client(client)

            if future.result() is not None:
                values = future.result().values
                # Check if parameter exists and has a value
                if len(values) > 0 and values[0].type != 0:  # type 0 means NOT_SET
                    return True

            return False

        except Exception:
            return False

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
    """PyQt5 GUI for ControlTower node using .ui file."""

    def __init__(self, node: ControlTower):
        super().__init__()
        self.node = node
        self._startup_route_loaded = False

        # Load the main UI from file
        ui_path = os.path.join(UI_BASE_PATH, 'twist_control.ui')
        loadUi(ui_path, self)

        # Recompose loaded UI into a control-tower layout with route planning on the left.
        self.setup_route_planner_shell()

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

    def setup_route_planner_shell(self):
        """Mount the route planner next to the existing twist control panel."""
        self._move_button_layout_to_top()

        self.plan_route_button = QPushButton('MISSION PLANNER')
        self.plan_route_button.setCheckable(True)
        self.plan_route_button.setStyleSheet(ACTIVE_BUTTON_STYLE)
        self.button_layout.insertWidget(1, self.plan_route_button)

        twist_control_widget = self.takeCentralWidget()
        self.twist_control_widget = twist_control_widget
        central_widget = QWidget(self)
        central_layout = QHBoxLayout(central_widget)
        central_layout.setContentsMargins(0, 0, 0, 0)

        self.main_splitter = QSplitter(Qt.Horizontal, central_widget)
        self.route_planner = RoutePlannerWidget(UI_BASE_PATH, self.main_splitter)
        self.route_planner.set_vehicle_type(self.node.waywise_object_type)
        self.route_planner.set_vehicle_connected(self.node.vehicle_connected)
        self.route_planner.set_tile_server_url(self.node.osm_tile_server_url)
        self.route_planner.set_tile_cache_dir(self.node.osm_tile_cache_dir)
        self.osm_status_network = QNetworkAccessManager(self)
        self.osm_status_timer = QTimer(self)
        self.osm_status_timer.setInterval(1000)
        self.osm_status_timer.timeout.connect(self.poll_osm_server_status)
        self.osm_status_reply = None
        self.osm_server_status = None
        self._setup_map_config_button()
        self.node.get_logger().info(
            f'Control Tower map source: {self.node.map_source}; '
            f'OSM tile server: {self.node.osm_tile_server_url}'
        )
        self.on_map_source_selected(self.node.map_source)
        self.route_planner.setMinimumWidth(120)
        self.main_splitter.addWidget(self.route_planner)
        self.main_splitter.addWidget(twist_control_widget)
        self.main_splitter.setStretchFactor(0, 1)
        self.main_splitter.setStretchFactor(1, 0)
        self.main_splitter.setCollapsible(0, True)
        self.main_splitter.setCollapsible(1, False)

        self.scrollArea.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        twist_control_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        central_layout.addWidget(self.main_splitter)
        self.setCentralWidget(central_widget)
        self._update_right_pane_min_width(force=True)
        QTimer.singleShot(0, self._try_load_startup_route)

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
        self.route_planner.set_map_config_widget(self.map_config_button)
        self.route_planner.osm_url_edit.editingFinished.connect(self.on_osm_config_edited)
        self.route_planner.osm_cache_edit.editingFinished.connect(self.on_osm_config_edited)
        self.route_planner.osm_cache_browse_button.clicked.connect(self.on_osm_config_edited)
        self.route_planner.osm_refresh_requested.connect(self.on_osm_refresh_requested)

    def show_map_config_menu(self):
        menu_size = self.map_config_menu.sizeHint()
        popup_pos = self.map_config_button.mapToGlobal(QPoint(0, -menu_size.height()))
        self.map_config_menu.exec_(popup_pos)

    def on_osm_config_edited(self):
        if self.node.map_source == 'Local OSM server':
            self.node.osm_tile_server_url = self.route_planner.osm_url_edit.text()
            self.node.osm_tile_cache_dir = self.route_planner.osm_cache_edit.text()
            self.poll_osm_server_status()

    def on_osm_refresh_requested(self):
        if self.node.map_source != 'Local OSM server':
            return
        self.on_osm_config_edited()
        self.route_planner.refresh_tiles(clear_disk=False)

    def poll_osm_server_status(self):
        if (
            self.node.map_source != 'Local OSM server'
            or self.osm_status_reply is not None
            or not self.node.osm_tile_server_url
        ):
            return
        metadata_url = self.node.osm_tile_server_url.rstrip('/') + '/metadata'
        request = QNetworkRequest(QUrl(metadata_url))
        request.setRawHeader(b'User-Agent', b'Waywiser-ControlTower/1.0')
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
                    self.route_planner.refresh_tiles(clear_disk=True)
                    self.osm_status_timer.stop()
            self.route_planner.set_osm_server_status(status)
            self.osm_server_status = status
        except Exception:
            self.route_planner.set_osm_server_status('Busy')
            self.osm_server_status = 'Busy'
        finally:
            reply.deleteLater()

    def _try_load_startup_route(self):
        if self._startup_route_loaded:
            return

        route_file = self.node.startup_route_file.strip()
        if not route_file:
            self._startup_route_loaded = True
            return

        if self.node.control_vehicle_node_fqn.strip() and not self.node.vehicle_connected:
            return

        route_file = os.path.expanduser(os.path.expandvars(route_file))
        try:
            self.route_planner.set_enu_ref(self.node.enuref)
            loaded_count = self.route_planner.load_route_file(route_file)
            self._startup_route_loaded = True
            self.node.get_logger().info(
                f'Loaded startup route with {loaded_count} points from: {route_file}'
            )
        except Exception as exc:
            self._startup_route_loaded = True
            self.node.get_logger().error(str(exc))

    def on_map_source_selected(self, source):
        """Switch the route planner map background source."""
        if not hasattr(self, 'route_planner'):
            return
        source = self.node._normalize_map_source(source)
        self.node.map_source = source
        if source == 'OpenStreetMap':
            self.node.osm_tile_server_url = OPENSTREETMAP_TILE_SERVER_URL
            self.node.osm_tile_cache_dir = OPENSTREETMAP_CACHE_DIR
        self._set_checked_map_source(source)
        self.route_planner.set_tile_server_url(self.node.osm_tile_server_url)
        self.route_planner.set_tile_cache_dir(self.node.osm_tile_cache_dir)
        self.route_planner.set_map_source(source)
        self.route_planner.set_osm_config_mode(source)
        if source == 'Local OSM server':
            self.route_planner.set_osm_server_status('Busy')
            self.osm_server_status = 'Busy'
            self.osm_status_timer.start()
            self.poll_osm_server_status()
            self.route_planner.refresh_tiles()
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

        total_width = max(
            self.centralWidget().width() if self.centralWidget() else self.width(), 1
        )
        content_width = max(self.scrollAreaWidgetContents.minimumSizeHint().width(), 480)
        margin_and_scrollbar_width = 56
        right_min_width = max(int(total_width * 0.4), content_width + margin_and_scrollbar_width)
        map_min_width = self.route_planner.minimumWidth()
        splitter_handle_width = max(self.main_splitter.handleWidth(), 1)
        required_window_width = right_min_width + map_min_width + splitter_handle_width

        self.twist_control_widget.setMinimumWidth(right_min_width)
        self.twist_control_widget.setMaximumWidth(16777215)
        self.setMinimumWidth(required_window_width)

        sizes = self.main_splitter.sizes()
        if force or (len(sizes) >= 2 and sizes[1] < right_min_width):
            available = max(sum(sizes), total_width, required_window_width)
            self.main_splitter.setSizes(
                [max(map_min_width, available - right_min_width), right_min_width]
            )

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
        self.plan_route_button.toggled.connect(self.on_plan_route_toggled)
        self.route_planner.send_route_requested.connect(self.on_send_route_requested)
        self.auto_arm_checkbox.stateChanged.connect(self.on_auto_arm_changed)
        self.hover_hold_checkbox.stateChanged.connect(self.on_hover_hold_changed)
        self.auto_lift_off_checkbox.stateChanged.connect(self.on_auto_lift_off_changed)

    def on_plan_route_toggled(self, checked):
        """Enable or disable waypoint editing on the map."""
        self.route_planner.set_planning_enabled(checked)
        self.plan_route_button.setText('MISSION PLANNER')

    def on_send_route_requested(self, points, altitude, speed):
        """Send the route shown in the map to the selected vehicle."""
        if self.node.publish_route(points, altitude, speed):
            self.plan_route_button.setChecked(False)

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
                self.node.vehicle_namespace = RosUtils.parent_namespace_from_fqn(
                    self.node.control_vehicle_node_fqn
                )
                self.node._init_mux_sources()
                self.node.request_params_from_vehicle_node()
                self.sync_control_options_from_node()
                self.update_ui_for_vehicle_type()
                self.update_control_group_state()
                self._try_load_startup_route()

    def update_control_group_state(self):
        """Reflect whether the control pane has an active vehicle target."""
        has_vehicle_selected = bool(self.node.control_vehicle_node_fqn.strip())
        if hasattr(self, 'route_planner'):
            self.route_planner.set_vehicle_connected(self.node.vehicle_connected)

        # Control group dimming
        self.control_group.setEnabled(has_vehicle_selected)
        self.control_group_opacity.setOpacity(1.0 if has_vehicle_selected else 0.45)
        self.control_group.setToolTip(
            '' if has_vehicle_selected else 'Select a vehicle node to enable vehicle control.'
        )

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
        self.route_planner.set_vehicle_type(self.node.waywise_object_type)
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

        # Handle emergency stop
        if key == Qt.Key.Key_E and modifiers & Qt.KeyboardModifier.ControlModifier:
            if modifiers & Qt.KeyboardModifier.ShiftModifier:
                self.node.set_emergency_stop(active=False)
            else:
                self.node.set_emergency_stop(active=True)

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

        self.enuref_label.setText(
            f'({self.node.enuref[0]:.6f}°, {self.node.enuref[1]:.6f}°, {self.node.enuref[2]:.2f}m)'
        )
        self.enuref_label.setStyleSheet(f'color: {self.green_color}; font-weight: 700;')

        self._update_estop_display()
        self._update_quadcopter_state_display()
        self._update_mission_state_display()
        self._update_speed_display()
        self._update_battery_display()
        self._update_odom_display()
        self._update_world_pose_display()
        self._update_gnss_display()
        self._update_route_planner_display()
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

    def _update_route_planner_display(self):
        """Update route planner context from the active vehicle state."""
        self.route_planner.set_vehicle_type(self.node.waywise_object_type)
        self.route_planner.set_enu_ref(self.node.enuref)
        self._try_load_startup_route()
        self.route_planner.set_vehicle_overlay_model(self.node.vehicle_overlay_model)
        self.route_planner.set_visual_markers(self.node.visual_markers)

        # Animate rotor/propeller joints when the drone is armed or in-flight.
        # STARTING_UP and READY_TO_ARM are the only states where motors are still.
        qc_msg = self.node.last_quadcopter_state.get('msg')
        if qc_msg is not None and qc_msg.state_code not in (
            QuadcopterState.STARTING_UP,
            QuadcopterState.READY_TO_ARM,
        ):
            self.node.vehicle_overlay_model.tick(time.time())

        pose = None
        frame_id = ''
        if self.node.last_vehicle_pose['pose'] is not None:
            pose = self.node.last_vehicle_pose['pose']
            frame_id = self.node.last_vehicle_pose['frame_id']
        elif self.node.last_odom['pose'] is not None:
            pose = self.node.last_odom['pose']
            frame_id = self.node.last_odom['frame_id']

        if pose is None:
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

        self.route_planner.set_vehicle_pose(x, y, yaw)

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
        if self.node.last_emergency_stop_state['msg'] is not None:
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
        else:
            state_text = msg.state_str.upper()
            time_str, time_color = self._get_time_ago_and_color(
                self.node.last_quadcopter_state['stamp']
            )

        self.copter_state_label.setText(state_text)
        self.copter_state_time_label.setText(f'Last updated: {time_str}')
        self.copter_state_time_label.setStyleSheet(f'color: {time_color}; font-size: 9pt;')

        # Set color based on state
        if msg:
            if msg.state_code == QuadcopterState.EMERGENCY:
                color = self.red_color
            elif msg.state_code in [
                QuadcopterState.ARMED,
                QuadcopterState.IN_FLIGHT,
                QuadcopterState.ON_MISSION,
            ]:
                color = self.green_color
            elif msg.state_code in [
                QuadcopterState.ARMING,
                QuadcopterState.LANDING,
                QuadcopterState.LIFTING_OFF,
                QuadcopterState.AUTO_LIFTING_OFF,
            ]:
                color = '#fbbf24'  # Amber
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
        MissionState.FOLLOW_ROUTE_INIT: ('Route: Init', '#60a5fa'),
        MissionState.FOLLOW_ROUTE_GOTO_BEGIN: ('Route: Go to start', '#60a5fa'),
        MissionState.FOLLOW_ROUTE_FOLLOWING: ('Following route', None),
        MissionState.FOLLOW_ROUTE_APPROACHING_END_GOAL: ('Route: Approaching end', '#fbbf24'),
        MissionState.FOLLOW_ROUTE_FINISHED: ('Route: Finished', '#60a5fa'),
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
            self.mission_state_time_label.setText('Last updated: never')
            self.mission_state_time_label.setStyleSheet(
                f'color: {self.gray_color}; font-size: 9pt;'
            )
            return

        state_text, color = self._MISSION_STATE_STRINGS.get(msg.state, ('Unknown', None))
        if color is None:
            color = (
                self.green_color
                if msg.state == MissionState.FOLLOW_ROUTE_FOLLOWING
                else self.inactive_muted_color
            )
        time_str, time_color = self._get_time_ago_and_color(self.node.last_mission_state['stamp'])
        self.mission_state_label.setText(state_text.upper())
        self.mission_state_label.setStyleSheet(f'color: {color}; font-weight: 700;')
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
        if self.node.last_battery_state['msg'] is not None:
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
        if self.node.last_odom['pose'] is not None:
            time_label, time_based_color = self._get_time_ago_and_color(
                self.node.last_odom['stamp']
            )
            self._set_odom_ui_values(time_label, time_based_color)

    def _set_odom_ui_values(self, time_label, time_based_color):
        """Set odometry UI labels and styles."""
        p = self.node.last_odom['pose']
        roll, pitch, yaw = euler_from_quaternion(
            [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        )

        self.odom_pos_label.setText(
            f'({p.position.x:.2f}, {p.position.y:.2f}, {p.position.z:.2f}) m'
        )
        self.odom_yaw_label.setText(
            f'({roll * 180.0 / 3.14159:.1f}, {pitch * 180.0 / 3.14159:.1f}, '
            f'{yaw * 180.0 / 3.14159:.1f})°'
        )

        if self.node.last_odom['twist'] is not None:
            vx = self.node.last_odom['twist'].linear.x
            vy = self.node.last_odom['twist'].linear.y
            vz = self.node.last_odom['twist'].linear.z
        else:
            vx = 0.0
            vy = 0.0
            vz = 0.0
        self.odom_vel_label.setText(f'({vx:.2f}, {vy:.2f}, {vz:.2f}) m/s')
        self.odom_time_label.setText(time_label)

        style = f'color: {time_based_color}; font-weight: 700;'
        for lbl in [
            self.odom_pos_label,
            self.odom_yaw_label,
            self.odom_vel_label,
            self.odom_time_label,
        ]:
            lbl.setStyleSheet(style)

    def _update_world_pose_display(self):
        """Update world pose labels (ENU position and roll/pitch/yaw)."""
        if self.node.last_vehicle_pose['pose'] is None:
            return

        time_label, time_based_color = self._get_time_ago_and_color(
            self.node.last_vehicle_pose['stamp']
        )
        self.world_pose_time_label.setText(time_label)
        self.world_pose_time_label.setStyleSheet(f'color: {time_based_color}; font-weight: 700;')

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
            f'({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f}) m'
        )
        self.world_pose_yaw_label.setText(f'({roll_deg:.1f}, {pitch_deg:.1f}, {yaw_deg:.1f})°')

        style = f'color: {time_based_color}; font-weight: 700;'
        self.world_pose_pos_label.setStyleSheet(style)
        self.world_pose_yaw_label.setStyleSheet(style)

    def _update_gnss_display(self):
        """Update GNSS fix and accuracy information on the UI."""
        if self.node.last_nav_sat_fix_extended['msg'] is None:
            return

        msg = self.node.last_nav_sat_fix_extended['msg']
        time_label, time_based_color = self._get_time_ago_and_color(
            self.node.last_nav_sat_fix_extended['stamp']
        )
        self.gnss_time_label.setText(time_label)
        self.gnss_time_label.setStyleSheet(f'color: {time_based_color}; font-weight: 700;')

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
            f'({msg.latitude:.6f}°, {msg.longitude:.6f}°, {msg.altitude:.2f}m)'
        )
        self.gnss_pos_label.setStyleSheet(f'color: {time_based_color}; font-weight: 700;')

        self.gnss_head_label.setText(f'{self.node.last_nav_sat_fix_extended["msg"].yaw:.1f}°')
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

    node = ControlTower()
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
