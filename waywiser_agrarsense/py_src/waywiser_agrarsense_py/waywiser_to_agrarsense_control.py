#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from waywiser_core.msg import CarControlCommand
from waywiser_py.waywiser_utils import RELIABLE_TRANSIENT_LOCAL_QOS


class WaywiserToAgrarsenseControl(Node):
    """ROS2 node that converts WayWiseR CarControlCommand messages to Agrarsense control msgs."""

    def __init__(self):
        super().__init__('waywiser_to_agrarsense_control')

        # Declare parameters
        self.declare_parameter('ego_vehicle_role_name', 'ego_vehicle')
        self.declare_parameter('steering_gain', 1.0)

        # Get parameters
        self.ego_vehicle_role_name = (
            self.get_parameter('ego_vehicle_role_name').get_parameter_value().string_value
        )
        self.steering_gain = self.get_parameter('steering_gain').get_parameter_value().double_value

        # Create subscribers
        self.waywiser_car_control_sub = self.create_subscription(
            CarControlCommand,
            'waywiser_control_cmd',
            self.waywiser_control_cmd_callback,
            10,
        )
        # Create publisher
        self.agrarsense_vehicle_control_pub = self.create_publisher(
            String,
            '/agrarsense/in/vehicles/{}/movement'.format(self.ego_vehicle_role_name),
            RELIABLE_TRANSIENT_LOCAL_QOS,
        )

    def waywiser_control_cmd_callback(self, msg):
        throttle = msg.throttle
        steering = self.steering_gain * msg.steering

        # Create and publish control message
        agrarsense_vehicle_control_msg = String()
        agrarsense_vehicle_control_msg.data = f'{throttle},{msg.brake},{steering}'

        self.agrarsense_vehicle_control_pub.publish(agrarsense_vehicle_control_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaywiserToAgrarsenseControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('User requested shutdown with SIGINT.')
    finally:
        # Cleanup on exit
        try:
            node.destroy_node()
        except Exception as e:
            print(f'Error during node destruction: {e}')
        # Only shutdown if the context is still valid
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
