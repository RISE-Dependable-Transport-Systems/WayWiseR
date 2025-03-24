#!/usr/bin/env python3

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import pygame
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from std_msgs.msg import Float32
from tf_transformations import euler_from_quaternion

from waywiser_twist_safety.msg import EmergencyStopState


class TwistKeyboard(Node):
    """Node that publishing twist messages using keypresses from the keyboard."""

    def __init__(self):
        super().__init__('twist_keyboard')
        self.twist_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.emergency_stop_request_publisher = self.create_publisher(
            EmergencyStopState, '/emergency_stop/target_state', 10
        )

        self.emergency_stop_state_subscriber = self.create_subscription(
            EmergencyStopState,
            '/emergency_stop/current_state',
            self.emergency_stop_state_subscriber_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=1,
            ),
        )
        self.emergency_stop_current_state = EmergencyStopState.UNKNOWN

        # Initialize emergency_stop_target_state_msg
        self.emergency_stop_target_state_msg = EmergencyStopState()
        self.emergency_stop_target_state_msg.sender_id = 'twist_keyboard'
        self.emergency_stop_target_state_msg.state = EmergencyStopState.ACTIVE

        # Set up keybindings
        self.forward_key = pygame.K_w
        self.backward_key = pygame.K_x
        self.left_key = pygame.K_a
        self.right_key = pygame.K_d
        self.stop_key = pygame.K_s
        self.increase_linear_speed_key = pygame.K_i
        self.decrease_linear_speed_key = pygame.K_k
        self.increase_angular_speed_key = pygame.K_o
        self.decrease_angular_speed_key = pygame.K_l
        self.emergency_stop_key = pygame.K_e  # set with Ctrl+e, clear with Ctrl+Shift+e

        self.actuation_keys = [
            self.forward_key,
            self.backward_key,
            self.left_key,
            self.right_key,
            self.stop_key,
        ]
        self.is_actuation_requested = False

        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('startup_linear_speed', 0.5)
        self.declare_parameter('startup_angular_speed', 1.0)
        self.declare_parameter('linear_speed_increment', 1.0)
        self.declare_parameter('angular_speed_increment', 1.0)
        self.declare_parameter('speed_control_rate', 10.0)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('odom_topic', '')
        self.declare_parameter('battery_voltage_topic', '')
        self.declare_parameter('nav_sat_fix_topic', '')
        self.declare_parameter('min_battery_voltage', 0.0)

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
        self.linear_speed_factor = 1.0
        self.angular_speed_factor = 1.0

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.battery_voltage_topic = (
            self.get_parameter('battery_voltage_topic').get_parameter_value().string_value
        )
        self.nav_sat_fix_topic = (
            self.get_parameter('nav_sat_fix_topic').get_parameter_value().string_value
        )
        self.min_battery_voltage = (
            self.get_parameter('min_battery_voltage').get_parameter_value().double_value
        )

        if self.min_battery_voltage < 0.0:
            self._logger.warn(
                "Param 'min_battery_voltage' is not set. "
                + 'Please set a value to get low battery warnings!'
            )

        if self.odom_topic != '':
            self.odom_subscriber = self.create_subscription(
                Odometry,
                self.odom_topic,
                self.odom_callback,
                QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    durability=QoSDurabilityPolicy.VOLATILE,
                    depth=1,
                ),
            )

        if self.battery_voltage_topic != '':
            self.battery_voltage_subscriber = self.create_subscription(
                Float32,
                self.battery_voltage_topic,
                self.battery_voltage_callback,
                QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    durability=QoSDurabilityPolicy.VOLATILE,
                    depth=1,
                ),
            )

        if self.nav_sat_fix_topic != '':
            self.gnss_fix_subscriber = self.create_subscription(
                NavSatFix,
                self.nav_sat_fix_topic,
                self.gnss_fix_callback,
                QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    durability=QoSDurabilityPolicy.VOLATILE,
                    depth=1,
                ),
            )

        # Create a timer to control speeds
        self.speed_control_rate = (
            self.get_parameter('speed_control_rate').get_parameter_value().double_value
        )
        self.speed_control_timer = self.create_timer(
            1.0 / self.speed_control_rate, self.process_speed_control_keys
        )

        # Create a timer to publish Twist messages
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_twist)

        # Start Pygame
        pygame.init()
        # Get available drivers for audio
        audio_drivers = pygame.mixer.get_init()

        if audio_drivers is None:
            print('No audio device found! Skipping low battery audio warnings...')
            self.beep_sound = None
        else:
            pygame.mixer.init()
            # Generate a beep sound for low battery warning
            sample_rate = 44100
            duration = 1.0  # seconds
            frequency = 440.0  # Hz (A4 note)
            # Create sound wave
            t = np.linspace(0, duration, int(sample_rate * duration), False)
            wave = 32767 * np.sin(2 * np.pi * frequency * t)
            wave = wave.astype(np.int16)
            # Convert to sound object
            self.beep_sound = pygame.sndarray.make_sound(np.column_stack((wave, wave)))

        self.screen = pygame.display.set_mode((640, 720), pygame.RESIZABLE)
        pygame.display.set_caption('Waywiser Twist Keyboard')

        # Define the font and its size
        self.pygame_font = pygame.font.SysFont('Arial', 16)

        # Define a flag to track if the quit event has occurred
        self.shutdown_requested = False

        # Set up Pygame event handlers
        pygame.event.set_allowed(None)  # Disable all events
        pygame.event.set_allowed(pygame.QUIT)  # Enable quit event

        self.current_pose = None
        self.current_twist = None
        self.battery_voltage = None
        self.gnss_fix = None

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.current_twist = msg.twist.twist

    def battery_voltage_callback(self, msg):
        self.battery_voltage = msg.data

    def gnss_fix_callback(self, msg):
        self.gnss_fix = msg

    def emergency_stop_state_subscriber_callback(self, emergency_stop_current_state):
        self.emergency_stop_current_state = emergency_stop_current_state.state

    def capture_pressed_keys(self):
        keys = pygame.key.get_pressed()

        # Check for quit event
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.shutdown_requested = True

        pygame.event.pump()

        if self.shutdown_requested:
            return None
        else:
            return keys

    def process_speed_control_keys(self):
        # Get the state of all keys
        keys = self.capture_pressed_keys()

        if keys:
            # Process emergency stop set/clear event
            emergency_stop_set_event_registered = self.process_emergency_stop_keys(keys)

            if not emergency_stop_set_event_registered:
                # Check if keys associated with increasing or decreasing linear speed are pressed
                if keys[self.increase_linear_speed_key]:
                    # Increase linear speed
                    self.linear_speed += self.linear_speed_increment * self.linear_speed_factor
                    self.linear_speed_factor *= 1.1
                    # Ensure linear speed does not exceed the maximum value
                    self.linear_speed = min(self.linear_speed, self.max_linear_speed)
                elif keys[self.decrease_linear_speed_key]:
                    # Decrease linear speed
                    self.linear_speed -= self.linear_speed_increment * self.linear_speed_factor
                    self.linear_speed_factor *= 1.1
                    # Ensure linear speed does not go to zero
                    self.linear_speed = max(self.linear_speed, self.linear_speed_increment)
                else:
                    self.linear_speed_factor = 1.0

                # Check if keys associated with increasing or decreasing angular speed are pressed
                if keys[self.increase_angular_speed_key]:
                    # Increase angular speed
                    self.angular_speed += self.angular_speed_increment * self.angular_speed_factor
                    self.angular_speed_factor *= 1.1
                    # Ensure angular speed does not exceed the maximum value
                    self.angular_speed = min(self.angular_speed, self.max_angular_speed)
                elif keys[self.decrease_angular_speed_key]:
                    # Decrease angular speed
                    self.angular_speed -= self.angular_speed_increment * self.angular_speed_factor
                    self.angular_speed_factor *= 1.1
                    # Ensure linear speed does not go to zero
                    self.angular_speed = max(self.angular_speed, self.angular_speed_increment)
                else:
                    self.angular_speed_factor = 1.0

    def publish_twist(self):
        # Prepare the Twist message
        twist = Twist()

        # Get the state of all keys
        keys = self.capture_pressed_keys()

        if keys:
            # Process emergency stop set/clear event
            emergency_stop_set_event_registered = self.process_emergency_stop_keys(keys)

            # Check if any of the actuation keys are pressed now
            is_actuation_requested_now = any(keys[i] for i in self.actuation_keys)

            if self.is_actuation_requested:
                if not is_actuation_requested_now:
                    # Publish zero velocity twist message
                    self.twist_publisher.publish(twist)

                elif not emergency_stop_set_event_registered:
                    # Process actuation keys
                    if not keys[self.stop_key]:
                        if keys[self.forward_key]:
                            twist.linear.x += self.linear_speed
                        if keys[self.backward_key]:
                            twist.linear.x += -self.linear_speed
                        if keys[self.left_key]:
                            if twist.linear.x > 0:
                                twist.angular.z += self.angular_speed
                            else:
                                twist.angular.z += -self.angular_speed
                        if keys[self.right_key]:
                            if twist.linear.x > 0:
                                twist.angular.z += -self.angular_speed
                            else:
                                twist.angular.z += self.angular_speed

                    # Publish the Twist message
                    self.twist_publisher.publish(twist)

            self.is_actuation_requested = is_actuation_requested_now

        # Display information in the terminal
        self.display_information(twist)

    def process_emergency_stop_keys(self, keys):
        emergency_stop_event_registered = keys[self.emergency_stop_key] and (
            keys[pygame.K_LCTRL] or keys[pygame.K_RCTRL]
        )

        # Check if emergency stop key is pressed
        if emergency_stop_event_registered:
            if keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]:
                emergency_stop_event_registered = False
                self.emergency_stop_target_state_msg.state = EmergencyStopState.CLEAR
            else:
                self.emergency_stop_target_state_msg.state = EmergencyStopState.ACTIVE

            self.emergency_stop_target_state_msg.stamp = self.get_clock().now().to_msg()
            self.emergency_stop_request_publisher.publish(self.emergency_stop_target_state_msg)

        return emergency_stop_event_registered

    def display_information(self, twist):
        # Fill the screen with black
        self.screen.fill(pygame.Color('black'))

        text = (
            '************************* Usage Guide **************************\n\n'
            'Publishing twist messages using keypresses from the keyboard.\n\n'
            'Moving around:\n'
            '        w                 (Forward)\n'
            '   a   s    d   (Left   Stop   Right)\n'
            '        x                 (Reverse)\n\n'
            'i / k : increase/decrease linear speed by 10%\n'
            'o / l : increase/decrease angular speed by 10%\n'
            'Ctrl + e : activate emergency stop\n'
            'Ctrl + Shift + e : clear emergency stop\n\n\n'
            '*********************** Twist Keyboard Status **********************\n\n'
            f'Configured speed (Vx, Vyaw): ({self.linear_speed:.2f}, {self.angular_speed:.2f})\n\n'  # noqa
            f'Publishing speed (Vx, Vyaw): ({twist.linear.x:.2f}, {twist.angular.z:.2f})\n\n\n'  # noqa
            '*************************** Vehicle Status *************************\n\n'
        )

        if self.emergency_stop_current_state == EmergencyStopState.ACTIVE:
            text = text + 'Emergency stop state: "ACTIVE"\n\n'
        elif self.emergency_stop_current_state == EmergencyStopState.CLEAR:
            text = text + 'Emergency stop state: "CLEAR"\n\n'
        else:
            text = text + 'Emergency stop state: "UNKNOWN"\n\n'

        if self.battery_voltage is not None:
            text = text + f'Battery voltage: {self.battery_voltage:.2f} V\n\n'
        else:
            text = text + 'Battery voltage: UNKNOWN\n\n'

        if self.current_pose is not None:
            yaw = euler_from_quaternion(
                [
                    self.current_pose.orientation.x,
                    self.current_pose.orientation.y,
                    self.current_pose.orientation.z,
                    self.current_pose.orientation.w,
                ]
            )[2]
            text = (
                text + f'Odom position (x, y, yaw): ({self.current_pose.position.x:.2f},'
                f'{self.current_pose.position.y:.2f}, {yaw:.2f})\n\n'
            )
        else:
            text = text + 'Odom position (x, y, yaw): UNKNOWN\n\n'

        if self.current_twist is not None:
            text = (
                text + f'Odom twist (Vx, Vy, Vyaw): ({self.current_twist.linear.x:.2f},'
                f'{self.current_twist.linear.y:.2f}, {self.current_twist.angular.z:.2f})\n\n'
            )
        else:
            text = text + 'Odom twist (Vx, Vy, Vyaw): UNKNOWN\n\n'

        if self.gnss_fix is not None:
            fix_type = self.gnss_fix.status.status
            if fix_type == NavSatStatus.STATUS_NO_FIX:
                fix_type = 'NO FIX'
            elif fix_type == NavSatStatus.STATUS_FIX:
                fix_type = '2D FIX'
            elif fix_type == NavSatStatus.STATUS_GBAS_FIX:
                fix_type = '3D FIX'
            elif fix_type == 111:  # Waywiser specific status
                fix_type = 'Dead reckoning only'
            elif fix_type == 114:  # Waywiser specific status
                fix_type = 'GNSS + dead reckoning'
            elif fix_type == 115:  # Waywiser specific status
                fix_type = 'Time only fix'
            else:
                fix_type = 'UNKNOWN'
            text = (
                text + f'GNSS (lat, lon, alt, fix-type): ({self.gnss_fix.latitude:.6f}, '
                f'{self.gnss_fix.longitude:.6f}, {self.gnss_fix.altitude:.2f}, {fix_type})\n\n'
            )
        else:
            text = text + 'GNSS (lat, lon, alt, fix-type): UNKNOWN\n\n'

        pos = self.blit_text(text, (20, 20))

        # Render red warning if low battery
        if self.battery_voltage is not None and self.battery_voltage < self.min_battery_voltage:
            self.blit_text(
                'WARNING: Low battery voltage!',
                pos,
                pygame.Color('red'),
            )
            if self.beep_sound is not None:
                self.beep_sound.play()

        # Update display
        pygame.display.update()

    def blit_text(self, text, pos, color=pygame.Color('white')):
        words = [
            word.split(' ') for word in text.splitlines()
        ]  # 2D array where each row is a list of words.
        space = self.pygame_font.size(' ')[0]  # The width of a space.
        max_width = self.screen.get_size()[0]
        x, y = pos
        for line in words:
            for word in line:
                word_surface = self.pygame_font.render(word, 0, color)
                word_width, word_height = word_surface.get_size()
                if x + word_width >= max_width:
                    x = pos[0]  # Reset the x.
                    y += word_height  # Start on new row.
                self.screen.blit(word_surface, (x, y))
                x += word_width + space
            x = pos[0]  # Reset the x.
            y += word_height  # Start on new row.
        return x, y


def main():
    rclpy.init()
    node = TwistKeyboard()

    try:
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
