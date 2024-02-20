#!/usr/bin/env python3

from geometry_msgs.msg import Twist
import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class TwistKeyboard(Node):
    """Node that publishing twist messages using keypresses from the keyboard."""

    def __init__(self):
        super().__init__('twist_keyboard')
        self.twist_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.emergency_stop_publisher = self.create_publisher(Bool, 'emergency_stop', 10)

        # Initialize emergency_stop_msg
        self.emergency_stop_msg = Bool()
        self.emergency_stop_msg.data = False

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

        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('startup_linear_speed', 0.5)
        self.declare_parameter('startup_angular_speed', 1.0)
        self.declare_parameter('speed_control_rate', 10.0)
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

        self.screen = pygame.display.set_mode((640, 480), pygame.RESIZABLE)
        pygame.display.set_caption('Waywiser Twist Keyboard')

        # Define the font and its size
        self.pygame_font = pygame.font.SysFont('Arial', 16)

        # Define a flag to track if the quit event has occurred
        self.shutdown_requested = False

        # Set up Pygame event handlers
        pygame.event.set_allowed(None)  # Disable all events
        pygame.event.set_allowed(pygame.QUIT)  # Enable quit event

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
            emergency_stop_event_registered = self.process_emergency_stop_keys(keys)

            if not emergency_stop_event_registered:
                # Check if the key associated with increasing linear speed is pressed
                if keys[self.increase_linear_speed_key]:
                    # Increase linear speed by 10%
                    self.linear_speed *= 1.1
                    # Ensure linear speed does not exceed the maximum value
                    self.linear_speed = min(self.linear_speed, self.max_linear_speed)

                # Check if the key associated with decreasing linear speed is pressed
                if keys[self.decrease_linear_speed_key]:
                    # Decrease linear speed by 10%
                    self.linear_speed /= 1.1

                # Check if the key associated with increasing angular speed is pressed
                if keys[self.increase_angular_speed_key]:
                    # Increase angular speed by 10%
                    self.angular_speed *= 1.1
                    # Ensure angular speed does not exceed the maximum value
                    self.angular_speed = min(self.angular_speed, self.max_angular_speed)

                # Check if the key associated with decreasing angular speed is pressed
                if keys[self.decrease_angular_speed_key]:
                    # Decrease angular speed by 10%
                    self.angular_speed /= 1.1

    def publish_twist(self):
        # Prepare the Twist message
        twist = Twist()

        # Get the state of all keys
        keys = self.capture_pressed_keys()

        if keys:
            # Process emergency stop set/clear event
            self.process_emergency_stop_keys(keys)

            if not self.emergency_stop_msg.data:
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

        # Display information in the terminal
        self.display_information(twist)

    def process_emergency_stop_keys(self, keys):
        emergency_stop_event_registered = keys[self.emergency_stop_key] and (
            keys[pygame.K_LCTRL] or keys[pygame.K_RCTRL]
        )

        # Check if emergency stop key is pressed
        if emergency_stop_event_registered:
            if keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]:
                if self.emergency_stop_msg.data:
                    # Clear emergency stop signal
                    self.emergency_stop_msg.data = False
                    self.get_logger().warn('Emergency stop CLEARED from keyboard.')
            else:
                if not self.emergency_stop_msg.data:
                    # Set emergency stop signal
                    self.emergency_stop_msg.data = True
                    self.get_logger().warn('Emergency stop ACTIVATED from keyboard.')
                    self.twist_publisher.publish(Twist())

            self.emergency_stop_publisher.publish(self.emergency_stop_msg)

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
            'Ctrl + Shift + e : clear emergency stop\n\n'
            '************************** Status *********************************\n\n'
            f'Configured speeds: Linear - {self.linear_speed:.2f}; Angular - {self.angular_speed:.2f}\n\n'  # noqa
            f'Publishing speeds: Linear - {twist.linear.x:.2f}; Angular - {twist.angular.z:.2f}\n\n'  # noqa
            f'Emergency stop: {"ACTIVATED" if self.emergency_stop_msg.data else "CLEARED"}\n\n'
            '*********************************************************************\n\n'
        )

        self.blit_text(text, (20, 20))

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
