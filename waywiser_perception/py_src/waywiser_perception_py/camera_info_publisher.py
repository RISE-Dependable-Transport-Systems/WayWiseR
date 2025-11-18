#!/usr/bin/env python3

import json
import math

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from tf2_ros import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler, quaternion_multiply


class CameraInfoPublisher(Node):
    """
    ROS 2 node that publishes camera intrinsic parameters and a static transform.

    This node initializes a CameraInfo message based on parameters such as width,
    height, and field of view. It publishes the CameraInfo on a specified topic
    and broadcasts a static transform from the base frame to the camera frame.

    The node also supports an optional 'spawn_point' parameter, which is a JSON
    string specifying the camera's initial position (x, y, z) in centimeters and
    orientation (roll, pitch, yaw) in degrees. The orientation is converted to a
    quaternion and combined with a fixed rotation from the camera optical frame to
    the ROS camera frame.
    """

    def __init__(self):
        super().__init__('camera_info_publisher')

        # Camera parameters
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fov', 90.0)
        self.declare_parameter('camera_frame', 'camera')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('topic_name', 'camera_info')
        self.declare_parameter('spawn_point', '')

        # Get camera parameters
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.fov = self.get_parameter('fov').get_parameter_value().double_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.spawn_point = self.get_parameter('spawn_point').get_parameter_value().string_value

        # Calculate focal length in pixels
        f_x = f_y = self.width / (2 * math.tan(math.radians(self.fov) / 2))
        c_x = self.width / 2.0
        c_y = self.height / 2.0

        # Initialize CameraInfo message
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.width = self.width
        self.camera_info_msg.height = self.height
        self.camera_info_msg.distortion_model = 'plumb_bob'
        self.camera_info_msg.header.frame_id = self.camera_frame

        # Intrinsic camera matrix K (3x3)
        self.camera_info_msg.k = [
            float(f_x),
            0.0,
            float(c_x),
            0.0,
            float(f_y),
            float(c_y),
            0.0,
            0.0,
            1.0,
        ]

        # Distortion coefficients (assuming zero distortion)
        self.camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Rectification matrix R (identity for monocular camera)
        self.camera_info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        # Projection matrix P (3x4)
        self.camera_info_msg.p = [
            float(f_x),
            0.0,
            float(c_x),
            0.0,
            0.0,
            float(f_y),
            float(c_y),
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]

        # Parse the JSON string
        try:
            spawn_point = json.loads(self.spawn_point)
            x = spawn_point['x'] / 100.0  # Convert cm to meters
            y = -spawn_point['y'] / 100.0
            z = spawn_point['z'] / 100.0
            roll = math.radians(spawn_point['roll'])
            pitch = -math.radians(spawn_point['pitch'])
            yaw = math.radians(spawn_point['yaw'])

            # Calculate quaternion for orientation
            qx_spawn, qy_spawn, qz_spawn, qw_spawn = quaternion_from_euler(roll, pitch, yaw)

            # Fixed rotation from camera optical frame to ROS camera frame
            # Rotation: -90° about X, then -90° about Z
            qx_opt, qy_opt, qz_opt, qw_opt = quaternion_from_euler(-math.pi / 2, 0.0, math.pi / 2)

            q_spawn = [qw_spawn, qx_spawn, qy_spawn, qz_spawn]
            q_opt_to_ros = [qw_opt, qx_opt, qy_opt, qz_opt]

            q_combined = quaternion_multiply(q_opt_to_ros, q_spawn)
            qw_final, qx_final, qy_final, qz_final = q_combined

            self.publish_static_transform(x, y, z, qx_final, qy_final, qz_final, qw_final)

        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse spawn_point JSON string.')

        # Publisher for the camera info topic
        self.publisher = self.create_publisher(CameraInfo, self.topic_name, 10)
        self.timer = self.create_timer(1.0, self.publish_camera_info)

    def publish_static_transform(self, x, y, z, qx, qy, qz, qw):
        static_broadcaster = StaticTransformBroadcaster(self)

        # Create a TransformStamped message
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.camera_frame

        # Set translation
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z

        # Set rotation
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        # Publish the static transform
        static_broadcaster.sendTransform(transform)

    def publish_camera_info(self):
        # Update the timestamp and publish
        self.camera_info_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.camera_info_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()

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
