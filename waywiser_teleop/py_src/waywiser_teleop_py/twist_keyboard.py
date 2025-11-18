#!/usr/bin/env python3

import os
import sys
import tempfile
import time
import wave

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import numpy as np
from PyQt5.QtCore import Qt, QTimer, QUrl
from PyQt5.QtMultimedia import QSoundEffect
from PyQt5.QtWidgets import QApplication, QDialog, QMainWindow
from PyQt5.uic import loadUi
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32
from tf_transformations import euler_from_quaternion

from waywiser_core.msg import BatteryState, NavSatDiagnostics
from waywiser_py.waywiser_utils import RELIABLE_TRANSIENT_LOCAL_QOS
from waywiser_twist_safety.msg import EmergencyStopState

UI_BASE_PATH = os.path.join(
    get_package_share_directory('waywiser_teleop'), 'user_interface', 'twist_keyboard'
)


class TwistKeyboard(Node):
    """Node that publishes twist messages using keypresses from the keyboard."""

    def __init__(self):
        super().__init__('twist_keyboard')

        # Initialize emergency_stop_target_state_msg
        self.emergency_stop_target_state_msg = EmergencyStopState()
        self.emergency_stop_target_state_msg.sender_id = self.get_name()
        self.emergency_stop_target_state_msg.state = EmergencyStopState.ACTIVE

        # Declare parameters
        self.declare_parameter('control_vehicle_node', 'waywiser_car_node')
        self.control_vehicle_node = (
            self.get_parameter('control_vehicle_node').get_parameter_value().string_value
        )

        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('startup_linear_speed', 0.5)
        self.declare_parameter('startup_angular_speed', 1.0)
        self.declare_parameter('linear_speed_increment', 0.1)
        self.declare_parameter('angular_speed_increment', 0.1)
        self.declare_parameter('publish_rate', 10.0)

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

        # Wait for sim time if needed
        use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        if use_sim_time:
            if rclpy.ok() and self.get_clock().now().nanoseconds == 0:
                self.get_logger().warn('Waiting for /clock to be published...')
            while rclpy.ok() and self.get_clock().now().nanoseconds == 0:
                time.sleep(1.0)
                rclpy.spin_once(self)
            self.get_logger().info('Receiving /clock msgs now.')

        # Publishers
        self.twist_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.emergency_stop_request_publisher = self.create_publisher(
            EmergencyStopState, '/emergency_stop/target_state', RELIABLE_TRANSIENT_LOCAL_QOS
        )

        # Subscribers
        self.emergency_stop_state_subscriber = self.create_subscription(
            EmergencyStopState,
            '/emergency_stop/current_state',
            self.emergency_stop_state_subscriber_callback,
            10,
        )

        # Topic names (to be fetched from vehicle node)
        self.odom_topic = ''
        self.vehicle_pose_topic = ''
        self.battery_state_topic = ''
        self.rtcm_frequency_topic = ''
        self.nav_sat_diagnostics_topic = ''
        self.nav_sat_fix_topic = ''

        # Subscribers (will be created after fetching topics)
        self.odom_subscriber = None
        self.vehicle_pose_subscriber = None
        self.battery_state_subscriber = None
        self.rtcm_frequency_subscriber = None
        self.nav_sat_diagnostics_subscriber = None
        self.nav_sat_fix_subscriber = None

        # State variables
        self.last_emergency_stop_state = {'msg': None, 'stamp': self.get_clock().now()}
        self.last_odom = {'pose': None, 'twist': None, 'stamp': self.get_clock().now()}
        self.last_vehicle_pose = {'pose': None, 'stamp': self.get_clock().now()}
        self.last_battery_state = {'msg': None, 'stamp': self.get_clock().now()}
        self.last_nav_sat_diagnostics = {'msg': None, 'stamp': self.get_clock().now()}
        self.last_nav_sat_fix = {'msg': None, 'stamp': self.get_clock().now()}
        self.last_rtcm_frequency = {'frequency': None, 'stamp': self.get_clock().now()}

        # Current twist command
        self.current_twist = Twist()

        # Key states
        self.keys_pressed = set()
        self.is_actuation_requested = False

    def request_topics_from_vehicle_node(self):
        """Request topics from the current vehicle node."""
        service_name = f'/{self.control_vehicle_node}/get_parameters'
        self.get_logger().info(f"Requesting parameters from '{self.control_vehicle_node}'")

        client = self.create_client(GetParameters, service_name)
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                f"Parameter service of '{self.control_vehicle_node}' not available"
            )
            self.destroy_client(client)
            return

        request = GetParameters.Request()
        request.names = [
            'odom_topic',
            'vehicle_pose_topic',
            'battery_state_topic',
            'rtcm_frequency_topic',
            'nav_sat_diagnostics_topic',
            'nav_sat_fix_topic',
        ]

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            vals = future.result().values
            if len(vals) >= 6:
                self.odom_topic = vals[0].string_value or self.odom_topic
                self.vehicle_pose_topic = vals[1].string_value or self.vehicle_pose_topic
                self.battery_state_topic = vals[2].string_value or self.battery_state_topic
                self.rtcm_frequency_topic = vals[3].string_value or self.rtcm_frequency_topic
                self.nav_sat_diagnostics_topic = (
                    vals[4].string_value or self.nav_sat_diagnostics_topic
                )
                self.nav_sat_fix_topic = vals[5].string_value or self.nav_sat_fix_topic

                # Create subscribers
                self._create_subscribers()
                self.get_logger().info(f"Updated topics from '{self.control_vehicle_node}'")

        self.destroy_client(client)

    def _create_subscribers(self):
        """Create or recreate subscribers based on topic names."""
        if self.odom_topic:
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
                BatteryState, self.battery_state_topic, self.battery_state_callback, 10
            )

        if self.rtcm_frequency_topic:
            if self.rtcm_frequency_subscriber:
                self.destroy_subscription(self.rtcm_frequency_subscriber)
            self.rtcm_frequency_subscriber = self.create_subscription(
                Float32, self.rtcm_frequency_topic, self.rtcm_frequency_callback, 10
            )

        if self.nav_sat_diagnostics_topic:
            if self.nav_sat_diagnostics_subscriber:
                self.destroy_subscription(self.nav_sat_diagnostics_subscriber)
            self.nav_sat_diagnostics_subscriber = self.create_subscription(
                NavSatDiagnostics,
                self.nav_sat_diagnostics_topic,
                self.nav_sat_diagnostics_callback,
                10,
            )

        if self.nav_sat_fix_topic:
            if self.nav_sat_fix_subscriber:
                self.destroy_subscription(self.nav_sat_fix_subscriber)
            self.nav_sat_fix_subscriber = self.create_subscription(
                NavSatFix, self.nav_sat_fix_topic, self.gnss_fix_callback, 10
            )

    def odom_callback(self, msg):
        self.last_odom['pose'] = msg.pose.pose
        self.last_odom['twist'] = msg.twist.twist
        self.last_odom['stamp'] = self.get_clock().now()

    def vehicle_pose_callback(self, msg):
        self.last_vehicle_pose['pose'] = msg.pose
        self.last_vehicle_pose['stamp'] = self.get_clock().now()

    def battery_state_callback(self, msg):
        self.last_battery_state['msg'] = msg
        self.last_battery_state['stamp'] = self.get_clock().now()

    def rtcm_frequency_callback(self, msg):
        self.last_rtcm_frequency['frequency'] = msg.data
        self.last_rtcm_frequency['stamp'] = self.get_clock().now()

    def nav_sat_diagnostics_callback(self, msg):
        self.last_nav_sat_diagnostics['msg'] = msg
        self.last_nav_sat_diagnostics['stamp'] = self.get_clock().now()

    def gnss_fix_callback(self, msg):
        self.last_nav_sat_fix['msg'] = msg
        self.last_nav_sat_fix['stamp'] = self.get_clock().now()

    def emergency_stop_state_subscriber_callback(self, msg):
        self.last_emergency_stop_state['msg'] = msg
        self.last_emergency_stop_state['stamp'] = self.get_clock().now()

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
            # Just released all actuation keys - publish zero velocity
            self.twist_publisher.publish(twist)
        elif is_actuation_requested_now:
            # Process actuation keys
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

            self.twist_publisher.publish(twist)

        self.is_actuation_requested = is_actuation_requested_now
        self.current_twist = twist

    def set_emergency_stop(self, active=True):
        """Set or clear emergency stop."""
        if active:
            self.emergency_stop_target_state_msg.state = EmergencyStopState.ACTIVE
        else:
            self.emergency_stop_target_state_msg.state = EmergencyStopState.CLEAR

        self.emergency_stop_target_state_msg.stamp = self.get_clock().now().to_msg()
        self.emergency_stop_request_publisher.publish(self.emergency_stop_target_state_msg)

    def is_battery_low(self):
        """Check if battery is low."""
        return (
            self.last_battery_state['msg'] is not None
            and self.last_battery_state['msg'].state == BatteryState.LOW_VOLTAGE
        )


class VehicleNodeDialog(QDialog):
    """Dialog to prompt user for vehicle node name."""

    def __init__(self, node: Node, default_name='waywiser_car_node', parent=None):
        super().__init__(parent)
        self.node = node
        self.vehicle_node_name = None
        self.default_name = default_name

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
        QTimer.singleShot(100, self.fetch_nodes)

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
                break

    def on_node_double_clicked(self, item):
        """Handle double-click on a node in the list."""
        node_name = item.text()
        self.input_field.setText(node_name)
        self.input_field.setFocus()

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

        # Track if we've already played warning sound
        self.last_battery_warning = False

        # Show vehicle node dialog on startup
        QTimer.singleShot(100, self.show_vehicle_node_dialog)

    def setup_connections(self):
        """Connect UI signals to slots."""
        self.vehicle_node_button.clicked.connect(self.show_vehicle_node_dialog)
        self.usage_button.clicked.connect(self.show_usage_guide)

    def setup_audio(self):
        """Set up audio for low battery warning beep."""
        self.sound_effect = None
        self.temp_wav_file = None

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
            temp_dir = tempfile.gettempdir()
            self.temp_wav_file = os.path.join(temp_dir, 'twist_keyboard_beep.wav')

            # Write WAV file
            with wave.open(self.temp_wav_file, 'w') as wav_file:
                wav_file.setnchannels(1)  # Mono
                wav_file.setsampwidth(2)  # 2 bytes = 16 bits
                wav_file.setframerate(sample_rate)
                wav_file.writeframes(wave_data.tobytes())

            # Setup sound effect
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

    def show_vehicle_node_dialog(self):
        """Show dialog to select vehicle node."""
        dialog = VehicleNodeDialog(self.node, self.node.control_vehicle_node, self)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            vehicle_node = dialog.get_vehicle_node_name()
            if vehicle_node:
                self.node.control_vehicle_node = vehicle_node
                self.node.request_topics_from_vehicle_node()

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
        # Define colors
        self.gray_color = '#6b7280'
        self.green_color = '#34d399'
        self.yellow_color = '#facc15'
        self.red_color = '#ef4444'
        self.blue_color = '#60a5fa'

        # Update vehicle status
        self.vehicle_node_label.setText(self.node.control_vehicle_node)
        self.vehicle_node_label.setStyleSheet(f'color: {self.green_color}; font-weight: 700;')

        def get_time_ago_and_color(stamp):
            # Helper function for time formatting
            time_since = int((self.node.get_clock().now() - stamp).nanoseconds / 1e9)
            if time_since > 60:
                return (f'{time_since // 60}m ago', self.red_color)
            elif time_since > 1:
                return (f'{time_since}s ago', self.yellow_color)
            else:
                return ('now', self.green_color)

        # Emergency stop with color coding
        if self.node.last_emergency_stop_state['msg'] is not None:
            time_label, time_based_color = get_time_ago_and_color(
                self.node.last_emergency_stop_state['stamp']
            )
            self.estop_time_label.setText(f'Last updated: {time_label}')
            self.estop_time_label.setStyleSheet(f'color: {time_based_color}; font-size: 9pt;')

            if self.node.last_emergency_stop_state['msg'].state == EmergencyStopState.ACTIVE:
                self.estop_label.setText('[!] ACTIVE')
                self.estop_label.setStyleSheet(f'color: {self.red_color}; font-weight: 700;')
            elif self.node.last_emergency_stop_state['msg'].state == EmergencyStopState.CLEAR:
                self.estop_label.setText('CLEAR')
                self.estop_label.setStyleSheet(f'color: {self.green_color}; font-weight: 700;')
            else:
                self.estop_label.setText('[?] UNKNOWN')
                self.estop_label.setStyleSheet(f'color: {self.gray_color}; font-weight: 700;')

        # Update speed bars and labels
        max_linear = self.node.max_linear_speed
        max_angular = self.node.max_angular_speed

        # Linear speed - use dual-bar system for solid colors
        # Angular speed - centered control with left/right bars
        self.linear_config_label.setText(
            f'> Configured: {self.node.linear_speed:.2f} m/s (Max: {max_linear:.2f})'
        )
        self.angular_config_label.setText(
            f'> Configured: {self.node.angular_speed:.2f} rad/s (Max: {max_angular:.2f})'
        )
        configured_linear_percent = (
            int((self.node.linear_speed / max_linear) * 100) if max_linear > 0 else 0
        )
        configured_angular_percent = (
            int((self.node.angular_speed / max_angular) * 100) if max_angular > 0 else 0
        )

        # Background bar always shows configured level in gray
        self.linear_progress_bg.setValue(configured_linear_percent)
        self.angular_progress_left_bg.setValue(configured_angular_percent)
        self.angular_progress_right_bg.setValue(configured_angular_percent)

        # Foreground bar shows active level
        is_throttling = (
            Qt.Key.Key_W in self.node.keys_pressed or Qt.Key.Key_X in self.node.keys_pressed
        )
        is_turning_left = Qt.Key.Key_A in self.node.keys_pressed
        is_turning_right = Qt.Key.Key_D in self.node.keys_pressed

        if self.node.last_emergency_stop_state['msg'] is not None:
            if self.node.last_emergency_stop_state['msg'].state == EmergencyStopState.ACTIVE:
                self.linear_value_label.setText(f'{0:+.2f} m/s')
                self.angular_value_label.setText(f'{0:+.2f} rad/s')
                if is_throttling or is_turning_left or is_turning_right:
                    self.start_estop_blink()
            else:
                if is_throttling:
                    if self.linear_progress_fg.value() != configured_linear_percent:
                        self.linear_progress_fg.setValue(configured_linear_percent)
                elif self.linear_progress_fg.value() != 0:
                    self.linear_progress_fg.setValue(0)

                if is_turning_left and not is_turning_right:
                    if self.angular_progress_left_fg.value() != configured_angular_percent:
                        self.angular_progress_left_fg.setValue(configured_angular_percent)
                    if self.angular_progress_right_fg.value() != 0:
                        self.angular_progress_right_fg.setValue(0)
                elif is_turning_right and not is_turning_left:
                    if self.angular_progress_right_fg.value() != configured_angular_percent:
                        self.angular_progress_right_fg.setValue(configured_angular_percent)
                    if self.angular_progress_left_fg.value() != 0:
                        self.angular_progress_left_fg.setValue(0)
                else:
                    # No turning
                    if self.angular_progress_left_fg.value() != 0:
                        self.angular_progress_left_fg.setValue(0)
                    if self.angular_progress_right_fg.setValue(0) != 0:
                        self.angular_progress_right_fg.setValue(0)

                self.linear_value_label.setText(f'{self.node.current_twist.linear.x:+.2f} m/s')
                self.angular_value_label.setText(f'{self.node.current_twist.angular.z:+.2f} rad/s')

        # Battery voltage
        if self.node.last_battery_state['msg'] is not None:
            time_label, time_based_color = get_time_ago_and_color(
                self.node.last_battery_state['stamp']
            )
            self.battery_time_label.setText(f'Last updated: {time_label}')
            self.battery_time_label.setStyleSheet(f'color: {time_based_color}; font-size: 9pt;')

            voltage = self.node.last_battery_state['msg'].voltage
            if self.node.last_battery_state['msg'].state == BatteryState.LOW_VOLTAGE:
                self.battery_label.setText(f'[!] {voltage:.2f} V')
                self.battery_label.setStyleSheet(f'color: {self.red_color}; font-weight: 700;')
            else:
                self.battery_label.setText(f'{voltage:.2f} V')
                self.battery_label.setStyleSheet(f'color: {time_based_color}; font-weight: 700;')

        # RTCM frequency
        if self.node.last_rtcm_frequency['frequency'] is not None:
            rtcm_time_label, rtcm_time_based_color = get_time_ago_and_color(
                self.node.last_rtcm_frequency['stamp']
            )
            self.rtcm_time_label.setText(f'Last updated: {rtcm_time_label}')
            self.rtcm_time_label.setStyleSheet(f'color: {rtcm_time_based_color}; font-size: 9pt;')
            freq = self.node.last_rtcm_frequency['frequency']
            self.rtcm_label.setText(f'{freq:.2f} Hz')
            self.rtcm_label.setStyleSheet(f'color: {self.green_color}; font-weight: 700;')

        # Odometry
        if self.node.last_odom['pose'] is not None:
            time_label, time_based_color = get_time_ago_and_color(self.node.last_odom['stamp'])
            x = self.node.last_odom['pose'].position.x
            y = self.node.last_odom['pose'].position.y
            yaw_rad = euler_from_quaternion(
                [
                    self.node.last_odom['pose'].orientation.x,
                    self.node.last_odom['pose'].orientation.y,
                    self.node.last_odom['pose'].orientation.z,
                    self.node.last_odom['pose'].orientation.w,
                ]
            )[2]
            yaw_deg = yaw_rad * 180.0 / 3.14159265359

            self.odom_pos_label.setText(f'({x:.2f}, {y:.2f}) m')
            self.odom_yaw_label.setText(f'{yaw_deg:.1f}°')

            vx = self.node.last_odom['twist'].linear.x
            vy = self.node.last_odom['twist'].linear.y
            self.odom_vel_label.setText(f'({vx:.2f}, {vy:.2f}) m/s')

            self.odom_time_label.setText(time_label)

            self.odom_pos_label.setStyleSheet(f'color: {time_based_color}; font-weight: 700;')
            self.odom_yaw_label.setStyleSheet(f'color: {time_based_color}; font-weight: 700;')
            self.odom_vel_label.setStyleSheet(f'color: {time_based_color}; font-weight: 700;')
            self.odom_time_label.setStyleSheet(f'color: {time_based_color}; font-weight: 700;')

        # World pose
        if self.node.last_vehicle_pose['pose'] is not None:
            time_label, time_based_color = get_time_ago_and_color(
                self.node.last_vehicle_pose['stamp']
            )
            self.world_pose_time_label.setText(time_label)
            self.world_pose_time_label.setStyleSheet(
                f'color: {time_based_color}; font-weight: 700;'
            )

            x = self.node.last_vehicle_pose['pose'].position.x
            y = self.node.last_vehicle_pose['pose'].position.y
            yaw_rad = euler_from_quaternion(
                [
                    self.node.last_vehicle_pose['pose'].orientation.x,
                    self.node.last_vehicle_pose['pose'].orientation.y,
                    self.node.last_vehicle_pose['pose'].orientation.z,
                    self.node.last_vehicle_pose['pose'].orientation.w,
                ]
            )[2]
            yaw_deg = yaw_rad * 180.0 / 3.14159265359

            self.world_pose_pos_label.setText(f'({x:.2f}, {y:.2f}) m')
            self.world_pose_yaw_label.setText(f'{yaw_deg:.1f}°')

            self.world_pose_pos_label.setStyleSheet(
                f'color: {time_based_color}; font-weight: 700;'
            )
            self.world_pose_yaw_label.setStyleSheet(
                f'color: {time_based_color}; font-weight: 700;'
            )

        # GNSS Fix
        if self.node.last_nav_sat_fix['msg'] is not None:
            time_label, time_based_color = get_time_ago_and_color(
                self.node.last_nav_sat_fix['stamp']
            )
            self.gnss_time_label.setText(f'Last updated: {time_label}')
            self.gnss_time_label.setStyleSheet(f'color: {time_based_color}; font-weight: 700;')

            fix_type = self.node.last_nav_sat_fix['msg'].status.status
            fix_text = 'UNKNOWN'
            fix_color = self.gray_color

            if fix_type == NavSatStatus.STATUS_NO_FIX:
                fix_text = 'NO FIX'
                fix_color = self.red_color
            elif fix_type == NavSatStatus.STATUS_FIX:
                fix_text = '2D FIX'
                fix_color = self.blue_color
            elif fix_type == NavSatStatus.STATUS_GBAS_FIX:
                fix_text = '3D FIX'
                fix_color = self.yellow_color
            elif fix_type == 111:
                fix_text = 'DEAD RECKONING'
                fix_color = '#942478'
            elif fix_type == 114:
                fix_text = 'GNSS + DR'
                fix_color = self.green_color
            elif fix_type == 115:
                fix_text = 'TIME ONLY'
                fix_color = self.red_color

            self.gnss_fix_label.setText(fix_text)
            self.gnss_fix_label.setStyleSheet(f'color: {fix_color}; font-weight: 700;')

            lat = self.node.last_nav_sat_fix['msg'].latitude
            lon = self.node.last_nav_sat_fix['msg'].longitude
            alt = self.node.last_nav_sat_fix['msg'].altitude
            self.gnss_pos_label.setText(f'({lat:.6f}°, {lon:.6f}°, {alt:.1f}m)')
            self.gnss_pos_label.setStyleSheet(f'color: {time_based_color}; font-weight: 700;')

        # GNSS diagnostics
        if self.node.last_nav_sat_diagnostics['msg'] is not None:
            time_label, time_based_color = get_time_ago_and_color(
                self.node.last_nav_sat_diagnostics['stamp']
            )
            self.gnssdiag_time_label.setText(f'Last updated: {time_label}')
            self.gnssdiag_time_label.setStyleSheet(f'color: {time_based_color}; font-weight: 700;')

            msg = self.node.last_nav_sat_diagnostics['msg']
            self.gnss_accuracy_label.setText(
                f'({msg.horizontal_accuracy:.2f}m, {msg.vertical_accuracy:.2f}m, '
                f'{msg.heading_accuracy:.2f}°)'
            )
            self.gnss_accuracy_label.setStyleSheet(f'color: {self.green_color}; font-weight: 700;')
            self.gnss_last_rtcm_correction_label.setText(f'{msg.last_rtcm_correction}')
            self.gnss_last_rtcm_correction_label.setStyleSheet(
                f'color: {self.green_color}; font-weight: 700;'
            )
            self.gnss_num_satellites_label.setText(f'{msg.num_satellites}')
            self.gnss_num_satellites_label.setStyleSheet(
                f'color: {self.green_color}; font-weight: 700;'
            )

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


def main():
    # Install signal handler to close Qt app on Ctrl+C
    import signal

    rclpy.init()

    node = TwistKeyboard()
    app = QApplication(sys.argv)

    gui = TwistKeyboardUI(node)

    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
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
