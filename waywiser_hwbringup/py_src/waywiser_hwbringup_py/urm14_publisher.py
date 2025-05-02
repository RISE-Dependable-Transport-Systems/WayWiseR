#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from waywiser_hwbringup_py.urm14_sensor import initialize_master_bus
from waywiser_hwbringup_py.urm14_sensor import measure_distance


class URM14PublisherNode(Node):
    def __init__(self):
        super().__init__('urm14_publisher')

        # Declare parameters with default values
        self.declare_parameter('rs485_port', 'CH432T_PORT_1')
        self.declare_parameter('slave_address', '0x0c')
        self.declare_parameter('urm14_cycle_time', 0.2)
        self.declare_parameter('timeout', 0.3)
        self.declare_parameter('log_interval', 0.3)

        # Get parameters
        self.rs485_port = self.get_parameter('rs485_port').get_parameter_value().string_value
        slave_address_hex_string = (
            self.get_parameter('slave_address').get_parameter_value().string_value
        )
        self.slave_address = int(slave_address_hex_string, 16)
        self.urm14_cycle_time = (
            self.get_parameter('urm14_cycle_time').get_parameter_value().double_value
        )
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.log_interval = self.get_parameter('log_interval').get_parameter_value().double_value
        # Initialize Modbus RTU master
        self.master = initialize_master_bus(self.rs485_port, timeout=self.timeout)

        # Create publisher for distance measurements
        self.publisher = self.create_publisher(Float32, 'ultrasonic_distance', qos_profile=10)

        # Create timer for periodic measurements
        self.timer = self.create_timer(self.log_interval, self.publish_distance)

        self.get_logger().info(
            f'URM14 Publisher Node has been initialized with port {self.rs485_port} on slave address {hex(self.slave_address)}.'
        )

    def publish_distance(self):
        try:
            # Get distance measurement
            distance = measure_distance(self.master, self.slave_address, self.urm14_cycle_time)

            # Create and publish message
            msg = Float32()
            msg.data = distance
            self.publisher.publish(msg)

            # Log the measurement
            self.get_logger().debug(f'Published distance: {distance} mm')
        except Exception as e:
            self.get_logger().error(f'Error measuring distance: {str(e)}')


def main(args=None):
    rclpy.init(args=args)

    # Create and run the node
    node = URM14PublisherNode()

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
