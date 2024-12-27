#!/usr/bin/env python3

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion
from tf_transformations import quaternion_from_euler


class VehicleTFPublisher(Node):
    def __init__(self):
        super().__init__('vehicle_tf_publisher')

        # Declare parameters
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('input_transform', 'input_transform')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_rate', 10.0)

        # Get parameters
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.input_transform = (
            self.get_parameter('input_transform').get_parameter_value().string_value
        )
        self.base_link_frame = (
            self.get_parameter('base_link_frame').get_parameter_value().string_value
        )
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # Publisher for the odom topic
        self.odom_publisher = self.create_publisher(Odometry, self.odom_topic, 10)

        # Transform broadcaster for map_to_odom and odom_to_base_link
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscription to the global transform topic
        self.subscription = self.create_subscription(
            Transform, self.input_transform, self.global_transform_callback, 10
        )

        self.start_position = None
        self.start_rotation = None
        self.start_rotation_euler = None
        self.prev_time = None
        self.prev_position = None
        self.prev_yaw = None
        self.clock_reset_threshold = 1.0
        self.standstill_velocity_threshold = 0.05

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.odom_frame
        self.odom_msg.child_frame_id = self.base_link_frame

        # Create a timer to publish transforms at a fixed rate
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_transforms)

    def global_transform_callback(self, msg):
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
                        self.start_rotation_euler = euler_from_quaternion(
                            [
                                msg.rotation.x,
                                msg.rotation.y,
                                msg.rotation.z,
                                msg.rotation.w,
                            ]
                        )
                        self.prev_yaw = 0

            self.prev_time = current_time
            self.prev_position = msg.translation
            return

        # Create Odometry message
        self.odom_msg.header.stamp = current_time.to_msg()

        # Compute relative position and orientation
        self.odom_msg.pose.pose.position.x = msg.translation.x - self.start_position.x
        self.odom_msg.pose.pose.position.y = -(msg.translation.y - self.start_position.y)
        self.odom_msg.pose.pose.position.z = msg.translation.z - self.start_position.z

        current_euler = euler_from_quaternion(
            [msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w]
        )

        # Compute relative yaw angle
        relative_yaw = -(current_euler[2] - self.start_rotation_euler[2])
        odom_quat = quaternion_from_euler(0, 0, relative_yaw)
        self.odom_msg.pose.pose.orientation.x = odom_quat[0]
        self.odom_msg.pose.pose.orientation.y = odom_quat[1]
        self.odom_msg.pose.pose.orientation.z = odom_quat[2]
        self.odom_msg.pose.pose.orientation.w = odom_quat[3]

        # Calculate linear and angular velocity
        if self.prev_time is not None:
            dt = (current_time - self.prev_time).nanoseconds * 1e-9

            if dt > 0:
                dx = self.odom_msg.pose.pose.position.x - self.prev_position.x
                dy = self.odom_msg.pose.pose.position.y - self.prev_position.y
                d_yaw = relative_yaw - self.prev_yaw

                # Compute linear velocity
                linear_velocity = (dx**2 + dy**2) ** 0.5 / dt
                # Compute angular velocity
                angular_velocity = d_yaw / dt

                # Populate the twist message
                self.odom_msg.twist.twist.linear.x = linear_velocity
                self.odom_msg.twist.twist.linear.y = 0.0  # Assuming no lateral movement
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

    def reset_state(self):
        """Reset all state variables to handle clock reset."""
        self.start_position = None
        self.start_rotation = None
        self.start_rotation_euler = None
        self.prev_time = None
        self.prev_position = None
        self.prev_yaw = None

    def publish_transforms(self):
        current_time = self.get_clock().now()

        # If start_position is not set, publish transform with default values (0)
        if self.start_position is None:
            start_position = Vector3()
            start_rotation = Quaternion()
        else:
            start_position = self.start_position
            start_rotation = self.start_rotation

        # Publish the map_to_odom transform
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = current_time.to_msg()
        map_to_odom.header.frame_id = 'map'
        map_to_odom.child_frame_id = self.odom_frame
        map_to_odom.transform.translation.x = start_position.x
        map_to_odom.transform.translation.y = start_position.y
        map_to_odom.transform.translation.z = start_position.z
        map_to_odom.transform.rotation = start_rotation

        # Broadcast the map_to_odom transform
        self.tf_broadcaster.sendTransform(map_to_odom)

        # Publish the odom_to_base_link transform
        odom_to_base_link = TransformStamped()
        odom_to_base_link.header.stamp = current_time.to_msg()
        odom_to_base_link.header.frame_id = self.odom_frame
        odom_to_base_link.child_frame_id = self.base_link_frame
        odom_to_base_link.transform.translation.x = self.odom_msg.pose.pose.position.x
        odom_to_base_link.transform.translation.y = self.odom_msg.pose.pose.position.y
        odom_to_base_link.transform.translation.z = self.odom_msg.pose.pose.position.z
        odom_to_base_link.transform.rotation = self.odom_msg.pose.pose.orientation

        # Broadcast the odom_to_base_link transform
        self.tf_broadcaster.sendTransform(odom_to_base_link)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
