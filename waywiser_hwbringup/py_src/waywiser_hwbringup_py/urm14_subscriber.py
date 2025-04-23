#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class URM14SubscriberNode(Node):
    def __init__(self):
        super().__init__('urm14_subscriber')

        # Declare parameters with default values
        self.declare_parameter('log_interval', 0.3)

        # Get parameters
        self.log_interval = self.get_parameter('log_interval').get_parameter_value().double_value

        # Create subscriber for distance measurements
        self.subscription = self.create_subscription(
            Float32,
            'ultrasonic_distance',
            self.distance_callback,
            qos_profile=10,
        )

        # Create timer for periodic logging
        self.timer = self.create_timer(self.log_interval, self.log_distance)
        self.last_distance = None
        self.last_timestamp = None

        self.get_logger().info(f'URM14 Subscriber Node has been initialized with logging interval {self.log_interval} s')

    def distance_callback(self, msg):
        """Callback for distance measurements."""
        self.last_distance = msg.data
        self.last_timestamp = self.get_clock().now()

    def log_distance(self):
        """Periodically log the latest distance measurement."""
        if self.last_distance is not None and self.last_timestamp is not None:
            self.get_logger().info(f'Latest distance measurement: {self.last_distance} mm')
        else:
            self.get_logger().warn('No distance measurements received yet')


def main(args=None):
    rclpy.init(args=args)

    # Create and run the node
    node = URM14SubscriberNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
