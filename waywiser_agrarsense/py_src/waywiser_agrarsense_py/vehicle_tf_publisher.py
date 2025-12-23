#!/usr/bin/env python3
import math
import time

from geometry_msgs.msg import Transform, TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class VehicleTFPublisher(Node):
    """ROS2 node that publishes vehicle odometry and transforms."""

    def __init__(self):
        super().__init__('vehicle_tf_publisher')

        # Declare parameters
        self.declare_parameter('role_name', 'forwarder')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('yaw_offset', 0.0)

        # Get parameters
        self.role_name = self.get_parameter('role_name').get_parameter_value().string_value
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_link_frame = (
            self.get_parameter('base_link_frame').get_parameter_value().string_value
        )
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.yaw_offset = self.get_parameter('yaw_offset').get_parameter_value().double_value

        # Wait for sim time if needed
        use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        if use_sim_time:
            if rclpy.ok() and self.get_clock().now().nanoseconds == 0:
                self.get_logger().warn('Waiting for /clock to be published...')
            while rclpy.ok() and self.get_clock().now().nanoseconds == 0:
                time.sleep(1.0)
                rclpy.spin_once(self)
            self.get_logger().warn('Receiving /clock msgs now.')

        # Publisher for the odom topic
        self.odom_publisher = self.create_publisher(Odometry, self.odom_topic, 10)

        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscription to the global transform topic
        self.base_transform_subscription = self.create_subscription(
            Transform,
            f'/agrarsense/out/sensors/{self.role_name}/transform',
            self.global_base_frame_transform_callback,
            10,
        )

        # Create a timer to publish transforms at a fixed rate
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_transforms)

        # Initialize variables
        self.vehicle_transform = TransformStamped()
        self.vehicle_transform.header.frame_id = self.world_frame
        self.vehicle_transform.child_frame_id = self.role_name
        self.start_position = None
        self.start_rotation = None
        self.start_yaw = None
        self.prev_time = None
        self.prev_position = None
        self.prev_yaw = None
        self.clock_reset_threshold = 1.0
        self.standstill_velocity_threshold = 0.05

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.odom_frame
        self.odom_msg.child_frame_id = self.base_link_frame

    def global_base_frame_transform_callback(self, msg):
        # Convert translation from centimeters to meters
        msg.translation.x /= 100.0
        msg.translation.y /= 100.0
        msg.translation.z /= 100.0
        current_time = self.get_clock().now()

        # Check if the clock has reset by comparing time differences
        if self.prev_time is not None:
            dt = (current_time - self.prev_time).nanoseconds * 1e-9
            if dt < -self.clock_reset_threshold:
                self.get_logger().warn('Clock reset detected. Re-initializing state.')
                self.reset_state()

        # Initialize start position and orientation when vehicle stops
        if self.start_position is None:
            if self.prev_time is not None:
                dt = (current_time - self.prev_time).nanoseconds * 1e-9
                if dt > 0:
                    linear_velocity = (
                        (msg.translation.x - self.prev_position.x) ** 2
                        + (msg.translation.y - self.prev_position.y) ** 2
                        + (msg.translation.z - self.prev_position.z) ** 2
                    ) ** 0.5 / dt

                    if linear_velocity < self.standstill_velocity_threshold:
                        self.get_logger().info('Odometry position and orientation reference set.')
                        self.start_position = msg.translation
                        self.start_rotation = msg.rotation
                        self.start_yaw = euler_from_quaternion(
                            [
                                msg.rotation.x,
                                msg.rotation.y,
                                msg.rotation.z,
                                msg.rotation.w,
                            ]
                        )[2]
                        self.prev_yaw = 0

            self.prev_time = current_time
            self.prev_position = msg.translation
            return

        # Create Odometry message
        self.odom_msg.header.stamp = current_time.to_msg()

        # Compute relative position and orientation
        dx = msg.translation.x - self.start_position.x
        dy = -(msg.translation.y - self.start_position.y)
        relative_yaw = 0.0
        if self.start_yaw is not None:
            self.odom_msg.pose.pose.position.x = dx * math.cos(self.start_yaw) - dy * math.sin(
                self.start_yaw
            )
            self.odom_msg.pose.pose.position.y = dx * math.sin(self.start_yaw) + dy * math.cos(
                self.start_yaw
            )
            self.odom_msg.pose.pose.position.z = msg.translation.z - self.start_position.z

            # Compute relative yaw angle
            relative_yaw = -(
                euler_from_quaternion(
                    [msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w]
                )[2]
                - self.start_yaw
            )
        odom_quat = quaternion_from_euler(0, 0, relative_yaw)
        self.odom_msg.pose.pose.orientation.x = odom_quat[0]
        self.odom_msg.pose.pose.orientation.y = odom_quat[1]
        self.odom_msg.pose.pose.orientation.z = odom_quat[2]
        self.odom_msg.pose.pose.orientation.w = odom_quat[3]

        # Calculate linear and angular velocity
        if self.prev_time is not None:
            dt = (current_time - self.prev_time).nanoseconds * 1e-9

            if dt > 0:
                dx = msg.translation.x - self.prev_position.x
                dy = -(msg.translation.y - self.prev_position.y)
                d_yaw = relative_yaw - self.prev_yaw

                # Compute linear velocity
                linear_velocity = (dx**2 + dy**2) ** 0.5 / dt
                if linear_velocity < 0.01:
                    linear_velocity = 0.0
                # Compute angular velocity
                angular_velocity = d_yaw / dt

                # Populate the twist message
                self.odom_msg.twist.twist.linear.x = linear_velocity
                self.odom_msg.twist.twist.linear.y = 0.0  # Assuming no lateral movement (y-axis)
                self.odom_msg.twist.twist.linear.z = 0.0  # Assuming no vertical movement
                self.odom_msg.twist.twist.angular.x = 0.0  # Assuming no roll
                self.odom_msg.twist.twist.angular.y = 0.0  # Assuming no pitch
                self.odom_msg.twist.twist.angular.z = angular_velocity

        # Update previous values
        self.prev_time = current_time
        self.prev_position = msg.translation
        self.prev_yaw = relative_yaw

        # Publish the odom message
        self.odom_publisher.publish(self.odom_msg)

        # Create a World -> Vehicle transform
        self.vehicle_transform.transform = msg
        roll, pitch, yaw = euler_from_quaternion(
            [msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w]
        )
        quat = quaternion_from_euler(roll, pitch, yaw + self.yaw_offset)
        self.vehicle_transform.transform.rotation.x = quat[0]
        self.vehicle_transform.transform.rotation.y = quat[1]
        self.vehicle_transform.transform.rotation.z = quat[2]
        self.vehicle_transform.transform.rotation.w = quat[3]

    def reset_state(self):
        """Reset all state variables to handle clock reset."""
        self.start_position = None
        self.start_rotation = None
        self.start_yaw = None
        self.prev_time = None
        self.prev_position = None
        self.prev_yaw = None

    def publish_transforms(self):
        if self.start_position is None:
            return

        # Publish the map_to_vehicle transform
        self.vehicle_transform.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.vehicle_transform)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleTFPublisher()

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
