#!/usr/bin/env python3


from carla_msgs.msg import CarlaEgoVehicleControl
import rclpy
from rclpy.node import Node

from waywiser_core.msg import CarControlCommand
from waywiser_py.waywiser_utils import RELIABLE_TRANSIENT_LOCAL_QOS


class WaywiserToCarlaControlNode(Node):
    """ROS2 node that converts WayWiseR CarControlCommand msgs to CarlaEgoVehicleControl msgs."""

    def __init__(self):
        super().__init__('waywiser_to_carla_control_node')

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
        self.carla_vehicle_control_pub = self.create_publisher(
            CarlaEgoVehicleControl,
            '/carla/{}/vehicle_control_cmd'.format(self.ego_vehicle_role_name),
            RELIABLE_TRANSIENT_LOCAL_QOS,
        )

    def waywiser_control_cmd_callback(self, msg):
        throttle = msg.throttle
        reverse = throttle < 0
        steering = self.steering_gain * msg.steering

        # Create and publish control message
        carla_vehicle_control_msg = CarlaEgoVehicleControl()
        carla_vehicle_control_msg.throttle = abs(throttle)
        carla_vehicle_control_msg.brake = float(msg.brake)
        carla_vehicle_control_msg.steer = float(steering)
        carla_vehicle_control_msg.reverse = reverse

        self.carla_vehicle_control_pub.publish(carla_vehicle_control_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaywiserToCarlaControlNode()

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
