#!/usr/bin/env python3

from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
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
        self.declare_parameter('start_position', '')
        self.declare_parameter('start_orientation', '')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('input_transform', 'input_transform')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')

        # Get parameters
        self.start_position = (
            self.get_parameter('start_position').get_parameter_value().string_value
        )
        if self.start_position == '':
            self.start_position = None
            self.start_orientation = None
            self.start_euler = None
        else:
            self.start_euler = euler_from_quaternion(
                [
                    self.start_orientation.x,
                    self.start_orientation.y,
                    self.start_orientation.z,
                    self.start_orientation.w,
                ]
            )

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.input_transform = (
            self.get_parameter('input_transform').get_parameter_value().string_value
        )
        self.base_link_frame = (
            self.get_parameter('base_link_frame').get_parameter_value().string_value
        )
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value

        # Publisher for the odom topic
        self.odom_publisher = self.create_publisher(Odometry, self.odom_topic, 10)

        # Transform broadcaster for map_to_odom and odom_to_base_link
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscription to the global transform topic
        self.subscription = self.create_subscription(
            Transform, self.input_transform, self.global_transform_callback, 10
        )

        # Variables for velocity calculation
        self.prev_time = None
        self.prev_position = None
        self.prev_yaw = None
        self.prev_twist = None

    def global_transform_callback(self, msg):
        # Convert translation from centimeters to meters
        msg.translation.x /= 100.0
        msg.translation.y /= 100.0
        msg.translation.z /= 100.0

        # Initialize start position and orientation only once
        if self.start_position is None:
            self.start_position = msg.translation
            self.start_orientation = msg.rotation

            # Convert orientations from quaternion to Euler
            self.start_euler = euler_from_quaternion(
                [
                    self.start_orientation.x,
                    self.start_orientation.y,
                    self.start_orientation.z,
                    self.start_orientation.w,
                ]
            )
            self.get_logger().info('Start position and orientation set.')

        # Compute the odom based on the start position
        odom_msg = Odometry()
        current_time = self.get_clock().now()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_link_frame

        # Relative position in the odom frame
        odom_msg.pose.pose.position.x = msg.translation.x - self.start_position.x
        odom_msg.pose.pose.position.y = -(msg.translation.y - self.start_position.y)
        odom_msg.pose.pose.position.z = msg.translation.z - self.start_position.z

        current_euler = euler_from_quaternion(
            [msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w]
        )

        # Compute relative yaw angle (only yaw for simplicity)
        relative_yaw = -(current_euler[2] - self.start_euler[2])
        odom_quat = quaternion_from_euler(0, 0, relative_yaw)
        odom_msg.pose.pose.orientation.x = odom_quat[0]
        odom_msg.pose.pose.orientation.y = odom_quat[1]
        odom_msg.pose.pose.orientation.z = odom_quat[2]
        odom_msg.pose.pose.orientation.w = odom_quat[3]

        # Calculate linear and angular velocity
        if self.prev_time is not None:
            dt = (current_time - self.prev_time).nanoseconds * 1e-9

            if dt > 0:
                dx = odom_msg.pose.pose.position.x - self.prev_position[0]
                dy = odom_msg.pose.pose.position.y - self.prev_position[1]
                d_yaw = relative_yaw - self.prev_yaw

                # Compute linear velocity
                linear_velocity = (dx**2 + dy**2) ** 0.5 / dt
                # Compute angular velocity
                angular_velocity = d_yaw / dt

                # Populate the twist message
                odom_msg.twist.twist.linear.x = linear_velocity
                odom_msg.twist.twist.linear.y = 0.0  # Assuming no lateral movement
                odom_msg.twist.twist.linear.z = 0.0  # Assuming no vertical movement
                odom_msg.twist.twist.angular.x = 0.0  # Assuming no roll
                odom_msg.twist.twist.angular.y = 0.0  # Assuming no pitch
                odom_msg.twist.twist.angular.z = angular_velocity
            else:
                odom_msg.twist.twist = self.prev_twist

        # Update previous values
        self.prev_time = current_time
        self.prev_position = (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y)
        self.prev_yaw = relative_yaw
        self.prev_twist = odom_msg.twist.twist

        # Publish the odom message
        self.odom_publisher.publish(odom_msg)

        # Publish the map_to_odom transform
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = current_time.to_msg()
        map_to_odom.header.frame_id = 'map'
        map_to_odom.child_frame_id = self.odom_frame
        map_to_odom.transform.translation.x = self.start_position.x
        map_to_odom.transform.translation.y = self.start_position.y
        map_to_odom.transform.translation.z = self.start_position.z

        # Set orientation directly for map_to_odom (start orientation)
        map_to_odom.transform.rotation = self.start_orientation

        # Broadcast the map_to_odom transform
        self.tf_broadcaster.sendTransform(map_to_odom)

        # Publish the odom_to_base_link transform
        odom_to_base_link = TransformStamped()
        odom_to_base_link.header.stamp = current_time.to_msg()
        odom_to_base_link.header.frame_id = self.odom_frame
        odom_to_base_link.child_frame_id = self.base_link_frame
        odom_to_base_link.transform.translation.x = odom_msg.pose.pose.position.x
        odom_to_base_link.transform.translation.y = odom_msg.pose.pose.position.y
        odom_to_base_link.transform.translation.z = odom_msg.pose.pose.position.z
        odom_to_base_link.transform.rotation = odom_msg.pose.pose.orientation

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
