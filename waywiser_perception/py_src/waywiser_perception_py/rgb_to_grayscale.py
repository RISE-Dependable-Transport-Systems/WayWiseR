#!/usr/bin/env python3

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class RGBToGrayScale(Node):
    def __init__(self):
        super().__init__('rgb_to_grayscale')

        # Declare parameters
        self.declare_parameter('far_plane', 1000.0)
        self.declare_parameter('frame_id_override', '')
        self.declare_parameter('rgb_topic', '/agrarsense/out/sensors/depthcamera')
        self.declare_parameter('depth_topic', 'depth_image')

        # Get the parameters
        self.far_plane = self.get_parameter('far_plane').get_parameter_value().double_value
        self.frame_id_override = self.get_parameter('frame_id_override').value
        self.rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Subscriptions and publishers
        self.rgb_subscription = self.create_subscription(
            Image, self.rgb_topic, self.rgb_callback, 10
        )
        self.raw_depth_publisher = self.create_publisher(Image, self.depth_topic + '_raw', 10)
        self.mono16_depth_publisher = self.create_publisher(
            Image, self.depth_topic + '_mono16', 10
        )

        self.get_logger().info(f'Node initialized with far_plane = {self.far_plane} meters.')

    def rgb_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            img_array = np.ndarray(
                buffer=self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'),
                dtype=np.uint8,
                shape=(msg.height, msg.width, 3),
            )

            # Compute depth from RGB values
            # r = img_array[:, :, 2].astype(np.float32)  # Red channel (least significant byte)
            # g = img_array[:, :, 1].astype(np.float32)  # Green channel
            # b = img_array[:, :, 0].astype(np.float32)  # Blue channel (most significant byte)
            # normalized = (r + g * 256.0 + b * 256.0 * 256.0) / (256.0 * 256.0 * 256.0 - 1.0)
            # depth_image_m = normalized * self.far_plane  # Convert to meters

            scales = (np.array([256.0**2, 256.0, 1.0]) / (256.0**3 - 1)) * self.far_plane
            depth_image_m = np.dot(img_array, scales).astype(np.float32)

            # self.get_logger().info(
            #     f'Min depth: {np.min(depth_image_m)}, Max depth: {np.max(depth_image_m)}'
            # )
            mono16_depth_image = np.clip(
                depth_image_m * 1000.0, 0.0, 256.0**2 - 1
            )  # Convert to mm
            mono16_depth_image = mono16_depth_image.astype(np.uint16)

            # Convert depth map to ROS Image message
            raw_depth_msg = self.bridge.cv2_to_imgmsg(depth_image_m, encoding='32FC1')
            raw_depth_msg.header = msg.header
            if self.frame_id_override:
                raw_depth_msg.header.frame_id = self.frame_id_override

            mono16_depth_msg = self.bridge.cv2_to_imgmsg(mono16_depth_image, encoding='16UC1')
            mono16_depth_msg.header = msg.header
            if self.frame_id_override:
                mono16_depth_msg.header.frame_id = self.frame_id_override

            # Publish the depth Image
            self.raw_depth_publisher.publish(raw_depth_msg)
            self.mono16_depth_publisher.publish(mono16_depth_msg)
        except Exception as e:
            self.get_logger().error(f'Error during depth conversion: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RGBToGrayScale()

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
