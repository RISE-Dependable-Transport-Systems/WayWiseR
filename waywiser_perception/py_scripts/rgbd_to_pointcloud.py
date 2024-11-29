#!/usr/bin/env python3

import struct

import cv2
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField


class RGBDToPointCloudNode(Node):
    def __init__(self):
        super().__init__('rgbd_to_pointcloud')

        # Parameters for topics
        self.declare_parameter('rgb_topic', '/rgb/image_raw')
        self.declare_parameter('depth_topic', '/depth_aligned/image_raw')
        self.declare_parameter('camera_info_topic', '/rgb/camera_info')
        self.declare_parameter('pointcloud_topic', '/depth/points')
        self.declare_parameter('max_depth_meters', 100.0)
        self.declare_parameter('optical_to_ros_transform', False)

        # Retrieve parameters
        self.rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.camera_info_topic = (
            self.get_parameter('camera_info_topic').get_parameter_value().string_value
        )
        self.pointcloud_topic = (
            self.get_parameter('pointcloud_topic').get_parameter_value().string_value
        )
        self.max_depth_meters = (
            self.get_parameter('max_depth_meters').get_parameter_value().double_value
        )
        self.optical_to_ros_transform = (
            self.get_parameter('optical_to_ros_transform').get_parameter_value().bool_value
        )

        # Bridge to convert ROS images to OpenCV
        self.cv_bridge = CvBridge()

        # Initialize subscribers and publisher
        self.create_subscription(Image, self.rgb_topic, self.rgb_callback, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)
        self.pointcloud_publisher = self.create_publisher(PointCloud2, self.pointcloud_topic, 10)

        # Cache for the RGB and depth images
        self.rgb_image = None
        self.depth_image = None
        self.camera_info = None

    def rgb_callback(self, msg):
        self.rgb_image = self.convert_to_rgb(msg)
        self.process_pointcloud()

    def convert_to_rgb(self, msg):
        try:
            # Directly handle different encodings
            if msg.encoding == 'rgb8':
                # If already rgb8, just convert as-is
                rgb_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            elif msg.encoding == 'bgr8':
                # Convert from BGR to RGB
                bgr_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)

            elif msg.encoding == 'bgra8':
                # Convert from BGRA to RGB
                bgra_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgra8')
                rgb_image = cv2.cvtColor(bgra_image, cv2.COLOR_BGRA2RGB)

            elif msg.encoding == 'mono8':
                # Convert from single-channel grayscale to RGB
                mono_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                rgb_image = cv2.cvtColor(mono_image, cv2.COLOR_GRAY2RGB)

            elif msg.encoding == 'mono16':
                # Convert from 16-bit grayscale to RGB, scaling if necessary
                mono16_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
                # Normalize 16-bit grayscale to 8-bit for visualization (optional)
                mono8_image = cv2.convertScaleAbs(mono16_image, alpha=255.0 / 65535.0)
                rgb_image = cv2.cvtColor(mono8_image, cv2.COLOR_GRAY2RGB)

            elif msg.encoding == 'rgba8':
                # Convert from RGBA to RGB
                rgba_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgba8')
                rgb_image = cv2.cvtColor(rgba_image, cv2.COLOR_RGBA2RGB)

            else:
                raise ValueError(f'Unsupported encoding: {msg.encoding}')

            return rgb_image  # The image is now in RGB format

        except CvBridgeError as e:
            print(f'Failed to convert image: {e}')
            return None

    def depth_callback(self, msg):
        depth_image = np.array(
            self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'), dtype=float
        )
        if '16UC1' in msg.encoding:
            depth_image = depth_image / 1000.0
        elif '32FC1' in msg.encoding:
            pass
        else:
            rclpy.logging.get_logger(self.get_name()).warn(
                f'Unsupported depth image encoding: {msg.encoding}'
            )
            return
        self.depth_image = depth_image
        self.process_pointcloud()

    def camera_info_callback(self, msg):
        self.camera_info = msg  # Cache the camera info

    def process_pointcloud(self):
        if self.rgb_image is None or self.depth_image is None or self.camera_info is None:
            return  # Wait until all data is available

        # Get camera intrinsic parameters
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        # Prepare point cloud data
        height, width = self.depth_image.shape
        points = []

        for v in range(height):
            for u in range(width):
                depth = self.depth_image[v, u]
                if (
                    (not np.isnan(depth))
                    and (not np.isinf(depth))
                    and depth < self.max_depth_meters
                ):
                    # Calculate 3D point in camera frame
                    x = (u - cx) * depth / fx
                    y = (v - cy) * depth / fy
                    z = depth

                    # Get RGB color
                    color = self.rgb_image[v, u]
                    r, g, b = color[0], color[1], color[2]

                    rgb = (r << 16) | (g << 8) | b

                    # Pack into a single float
                    rgb = struct.unpack('f', struct.pack('I', rgb))[0]

                    # Append the point (x, y, z, rgb)
                    if self.optical_to_ros_transform:
                        ros_x = z
                        ros_y = -x
                        ros_z = -y
                        points.append([ros_x, ros_y, ros_z, rgb])
                    else:
                        points.append([x, y, z, rgb])

        if len(points) > 0:
            # Create PointCloud2 message
            pointcloud_msg = self.create_pointcloud2_msg(points)
            self.pointcloud_publisher.publish(pointcloud_msg)

    def create_pointcloud2_msg(self, points):
        # Create a PointCloud2 message from list of points
        header = self.get_clock().now().to_msg()
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        pointcloud_data = []
        for point in points:
            pointcloud_data.append(struct.pack('ffff', *point))

        # Create and populate PointCloud2 message
        msg = PointCloud2()
        msg.header.stamp = header
        msg.header.frame_id = self.camera_info.header.frame_id
        msg.height = 1
        msg.width = len(points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * len(points)
        msg.is_dense = True
        msg.data = b''.join(pointcloud_data)

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = RGBDToPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
