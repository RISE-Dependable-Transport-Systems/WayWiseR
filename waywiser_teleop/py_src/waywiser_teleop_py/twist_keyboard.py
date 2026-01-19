#!/usr/bin/env python3
"""ROS 2 node for vehicle teleoperation using a keyboard and PyQt5 GUI."""

from contextlib import contextmanager
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
    from PyQt5.QtCore import Qt, QTimer, QUrl
    from PyQt5.QtMultimedia import QAudio, QAudioDeviceInfo, QSoundEffect

with suppress_stderr():
    from PyQt5.QtWidgets import QApplication, QDialog, QMainWindow
    from PyQt5.uic import loadUi
    from rcl_interfaces.srv import GetParameters
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Joy
    from tf_transformations import euler_from_quaternion


from waywiser_core.msg import BatteryState, NavSatFixExtended  # noqa: E402
from waywiser_py.waywiser_utils import RELIABLE_TRANSIENT_LOCAL_QOS, RosUtils  # noqa: E402
from waywiser_twist_safety.msg import EmergencyStopState  # noqa: E402

UI_BASE_PATH = os.path.join(
    get_package_share_directory('waywiser_teleop'), 'user_interface', 'twist_keyboard'
)


class TwistKeyboard(Node):
    """Publish twist messages using keypresses from the keyboard."""

    def __init__(self):
        super().__init__('twist_keyboard', allow_undeclared_parameters=True)

        # Initialize emergency_stop_target_state_msg
        self.emergency_stop_target_state_msg = EmergencyStopState()
        self.emergency_stop_target_state_msg.sender_id = self.get_name()
        self.emergency_stop_target_state_msg.state = EmergencyStopState.ACTIVE

        # Declare parameters
        self.declare_parameter('control_vehicle_node_fqn', 'waywiser_car_node')
        self.control_vehicle_node_fqn = (
            self.get_parameter('control_vehicle_node_fqn').get_parameter_value().string_value
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
        self.nav_sat_fix_extended_topic = ''
        self.emergency_stop_status_topic = ''
        self.emergency_stop_update_topic = ''
        self.enuref = [0.0, 0.0, 0.0]
        self.vehicle_namespace = ''

        # Subscribers (will be created after fetching topics)
        self.odom_subscriber = None
        self.vehicle_pose_subscriber = None
        self.battery_state_subscriber = None
        self.nav_sat_fix_extended_subscriber = None
        self.emergency_stop_state_subscriber = None
        self.emergency_stop_request_publisher = None
        self.mux_publisher = None

        # Mux initialization
        self.mux_output_topic = ''
        self.declare_parameter('mux_output_topic', 'teleop_mux_vel')
        self.mux_output_topic = (
            self.get_parameter('mux_output_topic').get_parameter_value().string_value
        )
        self.key_vel_publisher = self.create_publisher(Twist, 'key_vel', 10)

        self.mux_sources = {}  # source_name -> {topic, priority, timeout, last_msg, last_stamp}
        self.active_mux_source = 'None'
        self._init_mux_sources()

        # Joy subscribers and timers
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.joy_watchdog_timer = self.create_timer(self.joy_timeout, self.joy_watchdog_callback)

        # State variables
        self.last_emergency_stop_state = {'msg': None, 'stamp': self.get_clock().now()}
        self.last_odom = {'pose': None, 'twist': None, 'stamp': self.get_clock().now()}
        self.last_vehicle_pose = {'pose': None, 'stamp': self.get_clock().now()}
        self.last_battery_state = {'msg': None, 'stamp': self.get_clock().now()}
        self.last_nav_sat_fix_extended = {'msg': None, 'stamp': self.get_clock().now()}

        # Current twist command
        self.current_twist = Twist()

        # Key states
        self.keys_pressed = set()
        self.is_actuation_requested = False

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
        ]

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            vals = future.result().values
            if len(vals) >= 7:
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

                # Create subscribers
                self._create_subscribers()

                # Create publishers
                self._create_publishers()

                self.get_logger().info(f"Updated topics from '{self.control_vehicle_node_fqn}'")

                self.joy_watchdog_timer.reset()
            else:
                self.get_logger().warn(
                    f'Received {len(vals)} topics instead of 7 from '
                    f"'{self.control_vehicle_node_fqn}'"
                )

        self.destroy_client(client)

    def _prefix_with_vehicle_namespace(self, topic):
        return RosUtils.prefix_topic_with_namespace(topic, self.vehicle_namespace)

    def _create_subscribers(self):
        """Create or recreate subscribers based on topic names."""
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

    def _create_publishers(self):
        if self.emergency_stop_update_topic:
            if self.emergency_stop_request_publisher:
                self.destroy_publisher(self.emergency_stop_request_publisher)
            self.emergency_stop_request_publisher = self.create_publisher(
                EmergencyStopState,
                self.emergency_stop_update_topic,
                RELIABLE_TRANSIENT_LOCAL_QOS,
            )

        if self.mux_output_topic:
            if self.mux_publisher:
                self.destroy_publisher(self.mux_publisher)
            mux_output_topic_with_ns = self._prefix_with_vehicle_namespace(self.mux_output_topic)
            self.mux_publisher = self.create_publisher(Twist, mux_output_topic_with_ns, 10)

    def odom_callback(self, msg):
        """Handle odometry messages."""
        # Nav_msgs/Odometry: pose and twist are nested in PoseWithCovariance / TwistWithCovariance
        self.last_odom['pose'] = msg.pose.pose
        self.last_odom['twist'] = msg.twist.twist
        self.last_odom['stamp'] = self.get_clock().now()

    def vehicle_pose_callback(self, msg):
        """Handle vehicle pose messages."""
        # Geometry_msgs/PoseStamped: world pose is in the 'pose' field
        self.last_vehicle_pose['pose'] = msg.pose
        self.last_vehicle_pose['stamp'] = self.get_clock().now()

    def battery_state_callback(self, msg):
        """Handle battery state messages."""
        self.last_battery_state['msg'] = msg
        self.last_battery_state['stamp'] = self.get_clock().now()

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
            except Exception as e:
                self.get_logger().warn(f'Incomplete configuration for source {name}: {e}')
                continue

            self.mux_sources[name] = {
                'topic': topic,
                'timeout': timeout,
                'priority': priority,
                'last_msg': Twist(),
                'last_stamp': self.get_clock().now(),
            }

            if name != 'keyboard':
                self.get_logger().info(f'Creating subscriber for {name} on topic {topic}')
                self.create_subscription(
                    Twist, topic, lambda msg, n=name: self._mux_callback(msg, n), 10
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
            time_since_last = (now - data['last_stamp']).nanoseconds / 1e9
            if time_since_last <= data['timeout']:
                # Found the highest priority valid command
                winner = data['last_msg']
                winner_name = name
                break

        # If no winner or all timed out, publish zero velocity
        if winner is None:
            winner = Twist()

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
        is_actuation_requested_now = bool(self.keys_pressed & actuation_keys)

        if self.is_actuation_requested and not is_actuation_requested_now:
            # Just released all actuation keys - update keyboard source with zero twist
            pass
        if is_actuation_requested_now:
            if Qt.Key.Key_S not in self.keys_pressed:
                if Qt.Key.Key_W in self.keys_pressed:
                    twist.linear.x += self.linear_speed
                if Qt.Key.Key_X in self.keys_pressed:
                    twist.linear.x -= self.linear_speed
                if Qt.Key.Key_A in self.keys_pressed:
                    if twist.linear.x > 0:
                        twist.angular.z += self.angular_speed
                    else:
                        twist.angular.z -= self.angular_speed
                if Qt.Key.Key_D in self.keys_pressed:
                    if twist.linear.x > 0:
                        twist.angular.z -= self.angular_speed
                    else:
                        twist.angular.z += self.angular_speed

        # Always update keyboard source in mux
        self._update_mux_source('keyboard', twist)
        self.key_vel_publisher.publish(twist)

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
    """Dialog to show usage guide for the twist keyboard."""

    def __init__(self, parent=None):
        super().__init__(parent)

        # Load UI from file
        ui_path = os.path.join(UI_BASE_PATH, 'usage_guide.ui')
        loadUi(ui_path, self)

        # Connect close button
        self.close_button.clicked.connect(self.accept)


class TwistKeyboardUI(QMainWindow):
    """PyQt5 GUI for TwistKeyboard node using .ui file."""

    def __init__(self, node: TwistKeyboard):
        super().__init__()
        self.node = node

        # Load the main UI from file
        ui_path = os.path.join(UI_BASE_PATH, 'twist_keyboard.ui')
        loadUi(ui_path, self)

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

        # Track if we've already played warning sound
        self.last_battery_warning = False

        # Show vehicle node dialog on startup with auto-connect enabled
        QTimer.singleShot(100, lambda: self.show_vehicle_node_dialog(auto_connect=True))

    def setup_connections(self):
        """Connect UI signals to slots."""
        self.vehicle_node_button.clicked.connect(self.show_vehicle_node_dialog)
        self.usage_button.clicked.connect(self.show_usage_guide)

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
            self.temp_wav_file = os.path.join(temp_dir, 'twist_keyboard_beep.wav')

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
                self.node.request_params_from_vehicle_node()

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
            self.node.update_speed(angular_delta=-self.node.angular_speed_increment)

        # Handle emergency stop
        if key == Qt.Key.Key_E and modifiers & Qt.KeyboardModifier.ControlModifier:
            if modifiers & Qt.KeyboardModifier.ShiftModifier:
                self.node.set_emergency_stop(active=False)
            else:
                self.node.set_emergency_stop(active=True)

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
        self.vehicle_node_label.setText(self.node.control_vehicle_node_fqn)
        self.vehicle_node_label.setStyleSheet(f'color: {self.green_color}; font-weight: 700;')

        self.enuref_label.setText(
            f'({self.node.enuref[0]:.6f}°, {self.node.enuref[1]:.6f}°, {self.node.enuref[2]:.2f}m)'
        )
        self.enuref_label.setStyleSheet(f'color: {self.green_color}; font-weight: 700;')

        self._update_estop_display()
        self._update_speed_display()
        self._update_battery_display()
        self._update_odom_display()
        self._update_world_pose_display()
        self._update_gnss_display()

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

    def _update_speed_display(self):
        """Update speed bars and labels on the UI."""
        max_lin = self.node.max_linear_speed
        max_ang = self.node.max_angular_speed

        self.linear_config_label.setText(f'> Configured: {self.node.linear_speed:.2f} m/s')
        self.angular_config_label.setText(f'> Configured: {self.node.angular_speed:.2f} rad/s')

        lin_pct = int((self.node.linear_speed / max_lin) * 100) if max_lin > 0 else 0
        ang_pct = int((self.node.angular_speed / max_ang) * 100) if max_ang > 0 else 0

        self.linear_progress_bg.setValue(lin_pct)
        self.angular_progress_left_bg.setValue(ang_pct)
        self.angular_progress_right_bg.setValue(ang_pct)

        self._update_active_speed(lin_pct, ang_pct)

    def _update_active_speed(self, lin_pct, ang_pct):
        """Update active speed display based on keypresses."""
        is_th = Qt.Key.Key_W in self.node.keys_pressed or Qt.Key.Key_X in self.node.keys_pressed
        is_tl = Qt.Key.Key_A in self.node.keys_pressed
        is_tr = Qt.Key.Key_D in self.node.keys_pressed

        msg = self.node.last_emergency_stop_state['msg']
        if msg is not None and msg.state == EmergencyStopState.ACTIVE:
            self.linear_value_label.setText(f'{0:+.2f} m/s')
            self.angular_value_label.setText(f'{0:+.2f} rad/s')
            if is_th or is_tl or is_tr:
                self.start_estop_blink()
        else:
            self._update_progress_bars(is_th, is_tl, is_tr, lin_pct, ang_pct)
            self.linear_value_label.setText(f'{self.node.current_twist.linear.x:+.2f} m/s')
            self.angular_value_label.setText(f'{self.node.current_twist.angular.z:+.2f} rad/s')

    def _update_progress_bars(self, is_th, is_tl, is_tr, lin_pct, ang_pct):
        """Update speed progress bars."""
        self.linear_progress_fg.setValue(lin_pct if is_th else 0)

        l_val = ang_pct if (is_tl and not is_tr) else 0
        r_val = ang_pct if (is_tr and not is_tl) else 0

        self.angular_progress_left_fg.setValue(l_val)
        self.angular_progress_right_fg.setValue(r_val)

    def _update_battery_display(self):
        """Update battery voltage display on the UI."""
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
        yaw = euler_from_quaternion(
            [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        )[2]

        self.odom_pos_label.setText(f'({p.position.x:.2f}, {p.position.y:.2f}) m')
        self.odom_yaw_label.setText(f'{yaw * 180.0 / 3.14159:.1f}°')

        if self.node.last_odom['twist'] is not None:
            vx = self.node.last_odom['twist'].linear.x
            vy = self.node.last_odom['twist'].linear.y
        else:
            vx = 0.0
            vy = 0.0
        self.odom_vel_label.setText(f'({vx:.2f}, {vy:.2f}) m/s')
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
        """Update world pose labels (ENU position and heading)."""
        if self.node.last_vehicle_pose['pose'] is None:
            return

        time_label, time_based_color = self._get_time_ago_and_color(
            self.node.last_vehicle_pose['stamp']
        )
        self.world_pose_time_label.setText(time_label)
        self.world_pose_time_label.setStyleSheet(f'color: {time_based_color}; font-weight: 700;')

        pose = self.node.last_vehicle_pose['pose']
        yaw_rad = euler_from_quaternion(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )[2]
        yaw_deg = yaw_rad * 180.0 / 3.14159265359

        self.world_pose_pos_label.setText(f'({pose.position.x:.2f}, {pose.position.y:.2f}) m')
        self.world_pose_yaw_label.setText(f'{yaw_deg:.1f}°')

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
    # Install signal handler to close Qt app on Ctrl+C
    import signal

    # Suppress warnings during early initialization
    with suppress_stderr():
        rclpy.init()
        app = QApplication(sys.argv)

    node = TwistKeyboard()
    gui = TwistKeyboardUI(node)

    # Handle Ctrl+C gracefully
    def signal_handler(_sig, _frame):
        gui.close()
        app.quit()

    signal.signal(signal.SIGINT, signal_handler)

    # Allow Ctrl+C to work by processing events periodically
    timer = QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(100)

    gui.show()

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
