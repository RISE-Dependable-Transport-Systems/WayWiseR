#!/usr/bin/env python3

from pathlib import Path

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Image
from ultralytics.engine.results import Results
from ultralytics.models.yolo.model import YOLO

from waywiser_perception.msg import Detection
from waywiser_perception.msg import DetectionArray


class Yolov8Node(Node):
    """Node that provides yolo object detection capabilities"""

    def __init__(self) -> None:
        super().__init__('yolov8_node')

        self.declare_parameter('model_file_path', '')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('prediction_verbose', True)
        self.declare_parameter('color_image_topic', 'color_image')
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('annotated_color_image_topic', 'yolov8_annotated_image')
        self.declare_parameter('camera_base_frame', 'camera_link')

        self.model_name = self.get_parameter('model_file_path').get_parameter_value().string_value
        self.model = YOLO(self.model_name)
        if Path(self.model_name).suffix == '.pt':
            self.model.fuse()
        self.confidence_threshold = (
            self.get_parameter('confidence_threshold').get_parameter_value().double_value
        )
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.publish_annotated_image = (
            self.get_parameter('publish_annotated_image').get_parameter_value().bool_value
        )
        self.prediction_verbose = (
            self.get_parameter('prediction_verbose').get_parameter_value().bool_value
        )

        self.cv_bridge = CvBridge()

        self.color_image_sub = self.create_subscription(
            Image,
            self.get_parameter('color_image_topic').get_parameter_value().string_value,
            self.color_image_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=1,
            ),
        )
        self.camera_base_frame = (
            self.get_parameter('camera_base_frame').get_parameter_value().string_value
        )
        self.detection_array_pub = self.create_publisher(DetectionArray, 'detection_array', 10)
        if self.publish_annotated_image:
            self.processed_image_pub = self.create_publisher(
                Image,
                self.get_parameter('annotated_color_image_topic')
                .get_parameter_value()
                .string_value,
                10,
            )

    def color_image_callback(self, msg: Image) -> None:
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg)

        results = self.model.predict(
            source=cv_image,
            verbose=self.prediction_verbose,
            stream=False,
            conf=self.confidence_threshold,
            device=self.device,
        )

        results: Results = results[0].cpu()

        if results.boxes:
            detection_array = DetectionArray()
            detection_array.header = msg.header
            detection_array.header.frame_id = self.camera_base_frame

            for box_data in results.boxes:
                detection = Detection()
                detection.class_id = int(box_data.cls)
                detection.confidence = float(box_data.conf)

                detection.bbox_2d.geometric_center_pose.position.x = float(box_data.xywh[0][0])
                detection.bbox_2d.geometric_center_pose.position.y = float(box_data.xywh[0][1])
                detection.bbox_2d.width = float(box_data.xywh[0][2])
                detection.bbox_2d.height = float(box_data.xywh[0][3])

                detection_array.detections.append(detection)

            self.detection_array_pub.publish(detection_array)

        if self.publish_annotated_image:
            annotated_image = results.plot(
                conf=True, boxes=True, labels=True, masks=True, probs=True
            )
            annotated_image_msg = self.cv_bridge.cv2_to_imgmsg(annotated_image, encoding='rgb8')
            annotated_image_msg.header = msg.header
            self.processed_image_pub.publish(annotated_image_msg)


def main():
    rclpy.init()
    node = Yolov8Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
