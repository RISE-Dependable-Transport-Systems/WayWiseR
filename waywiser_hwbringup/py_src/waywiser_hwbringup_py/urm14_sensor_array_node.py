#!/usr/bin/env python3

import math
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from waywiser_hwbringup_py.urm14_sensor_array_manager import (
    cleanup_all_sensors,
    initialize_master_bus,
    measure_distance,
)


class URM1SensorArrayNode(Node):
    """
    ROS 2 node for managing an array of URM14 ultrasonic sensors.

    This node initializes the sensors specified via the 'sensor_names' parameter,
    sets up Modbus RTU masters for each sensor, and periodically measures distances.
    Each distance measurement is published to a ROS topic corresponding to the sensor.
    """

    def __init__(self):
        super().__init__('urm14_sensor_array_node')

        # Declare parameter for the array of ultrasonic sensors
        self.declare_parameter('sensor_names', [''])

        # Get the list of sensor names
        sensor_names = self.get_parameter('sensor_names').get_parameter_value().string_array_value

        if not sensor_names or (len(sensor_names) == 1 and sensor_names[0] == ''):  # type: ignore
            self.get_logger().error('No urm14 sensors specified in configuration!')
            return

        # Storage for sensor configurations and ROS components
        self.sensor_configs: Dict[str, Dict[str, Any]] = {}
        self.sensor_masters: Dict[str, Any] = {}
        self.distance_publishers: Dict[str, Any] = {}
        self.sensor_timers: Dict[str, Any] = {}

        # Initialize each sensor
        self.initialize_sensors(sensor_names)

        if not self.sensor_configs:
            self.get_logger().error('No urm14 sensors were successfully initialized!')
            return

    def initialize_sensors(self, sensor_names):
        """Initialize all specified sensors."""
        for sensor_name in sensor_names:
            try:
                # Declare parameters for this sensor
                self.declare_parameter(f'{sensor_name}.rs485_port', 'CH432T_PORT_1')
                self.declare_parameter(f'{sensor_name}.slave_address', '0x0c')
                self.declare_parameter(f'{sensor_name}.log_interval', 0.3)
                self.declare_parameter(f'{sensor_name}.cycle_time', 0.19)
                self.declare_parameter(f'{sensor_name}.timeout', 0.3)
                self.declare_parameter(f'{sensor_name}.topic', f'/ultrasonic/{sensor_name}')

                # Get sensor configuration
                config = {
                    'rs485_port': self.get_parameter(f'{sensor_name}.rs485_port')
                    .get_parameter_value()
                    .string_value,
                    'slave_address': int(
                        self.get_parameter(f'{sensor_name}.slave_address')
                        .get_parameter_value()
                        .string_value,
                        16,
                    ),
                    'log_interval': self.get_parameter(f'{sensor_name}.log_interval')
                    .get_parameter_value()
                    .double_value,
                    'cycle_time': self.get_parameter(f'{sensor_name}.cycle_time')
                    .get_parameter_value()
                    .double_value,
                    'timeout': self.get_parameter(f'{sensor_name}.timeout')
                    .get_parameter_value()
                    .double_value,
                    'topic': self.get_parameter(f'{sensor_name}.topic')
                    .get_parameter_value()
                    .string_value,
                }

                # Initialize Modbus RTU master for this sensor
                master = initialize_master_bus(config['rs485_port'], timeout=config['timeout'])
                if master is None:
                    self.get_logger().error(
                        f'Failed to initialize master for urm14 sensor {sensor_name}'
                        f'on port {config["rs485_port"]}'
                    )
                    continue

                # Create publisher for this sensor
                publisher = self.create_publisher(Float32, config['topic'], qos_profile=10)

                # Create timer for periodic measurements
                timer = self.create_timer(
                    config['log_interval'],
                    lambda sensor=sensor_name: self.publish_distance(sensor),
                )

                # Store everything
                self.sensor_configs[sensor_name] = config
                self.sensor_masters[sensor_name] = master
                self.distance_publishers[sensor_name] = publisher
                self.sensor_timers[sensor_name] = timer

                self.get_logger().info(
                    f'Initialized {sensor_name}: '
                    f'port={config["rs485_port"]}, '
                    f'slave={hex(config["slave_address"])}, '
                    f'topic={config["topic"]}'
                )

            except Exception as e:
                self.get_logger().error(
                    f'Failed to initialize urm14 sensor {sensor_name}: {str(e)}'
                )

    def publish_distance(self, sensor_name: str):
        """Publish distance measurement for a specific sensor."""
        if sensor_name not in self.sensor_configs:
            self.get_logger().error(f'Sensor {sensor_name} not found in configuration')
            return

        try:
            config = self.sensor_configs[sensor_name]
            master = self.sensor_masters[sensor_name]
            publisher = self.distance_publishers[sensor_name]

            # Get distance measurement
            distance = measure_distance(master, config['slave_address'], config['cycle_time'])

            # Create and publish message
            msg = Float32()
            msg.data = distance
            publisher.publish(msg)

            # Log the measurement in debug mode
            if not math.isnan(distance):
                self.get_logger().debug(f'{sensor_name}: Published distance {distance:.3f} m')
            else:
                self.get_logger().debug(f'{sensor_name}: Invalid distance measurement (NaN)')

        except Exception as e:
            self.get_logger().error(f'Error measuring distance for {sensor_name}: {str(e)}')

    def destroy_node(self):
        """Clean up resources when node is destroyed."""
        try:
            # Cancel all timers
            for timer in self.sensor_timers.values():
                if timer:
                    timer.cancel()

            # Clean up sensor connections
            cleanup_all_sensors()

            self.get_logger().info('Cleaned up all urm14 sensor resources')
        except Exception as e:
            self.get_logger().error(f'Error during cleanup: {str(e)}')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        # Create and run the node
        node = URM1SensorArrayNode()

        if node.sensor_configs:  # Only spin if we have sensors
            rclpy.spin(node)
        else:
            node.get_logger().error('No urm14 sensors initialized, shutting down')

    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Node stopped by user')
    except Exception as e:
        if node:
            node.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        # Cleanup
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
