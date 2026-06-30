#!/usr/bin/env python3

from copy import deepcopy
from dataclasses import dataclass
import importlib
import logging
from pathlib import Path
from typing import Optional

from builtin_interfaces.msg import Duration
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Quaternion
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
import torch
from ultralytics.engine.results import Results
from ultralytics.models.yolo.model import YOLO
from ultralytics.trackers import BOTSORT, BYTETracker
from ultralytics.utils import IterableSimpleNamespace, LOGGER, YAML
from ultralytics.utils.checks import check_yaml
from ultralytics.utils.plotting import Annotator, Colors
from vision_msgs.msg import BoundingBox3D, Detection3D, Detection3DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray

# Suppress noisy ultralytics warnings (e.g., 'not enough matching points')
LOGGER.setLevel(logging.ERROR)


class YoloNode(Node):
    """Node that provides yolo object detection capabilities."""

    def __init__(self) -> None:
        super().__init__('yolo_node')

        self.declare_parameter('model_file_path', '')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('prediction_verbose', True)
        self.declare_parameter('color_image_topic', 'color_image')
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('annotated_color_image_topic', 'yolov8_annotated_image')
        self.declare_parameter('depth_image_topic', '')
        self.declare_parameter('depth_camerainfo_topic', '')
        self.declare_parameter('maximum_object_depth_size', 0.5)
        self.declare_parameter('publish_bbox_3d_markers', False)
        self.declare_parameter('rviz_3d_visualization_markers_topic', 'bbox_3d_markers')
        self.declare_parameter('use_tracker', False)
        self.declare_parameter('color_image_topic_as_stream', False)
        self.declare_parameter('tracker_config_filepath', '')
        self.declare_parameter('detections_topic', 'detections')

        self.model_name = self.get_parameter('model_file_path').get_parameter_value().string_value
        self.model_name = self.resolve_model_name(self.model_name)
        self.engine_fallback_attempted = False
        self.load_model(self.model_name)
        self.confidence_threshold = (
            self.get_parameter('confidence_threshold').get_parameter_value().double_value
        )
        self.device = self.get_parameter('device').get_parameter_value().string_value
        if self.device != 'cpu' and not torch.cuda.is_available():
            self.get_logger().warning(
                f"CUDA device '{self.device}' requested, but PyTorch cannot access CUDA. "
                "Falling back to device 'cpu'."
            )
            self.device = 'cpu'
        self.publish_annotated_image = (
            self.get_parameter('publish_annotated_image').get_parameter_value().bool_value
        )
        self.prediction_verbose = (
            self.get_parameter('prediction_verbose').get_parameter_value().bool_value
        )
        self.color_image_topic_as_stream = (
            self.get_parameter('color_image_topic_as_stream').get_parameter_value().bool_value
        )
        self.detections_topic = (
            self.get_parameter('detections_topic').get_parameter_value().string_value
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

        self.depth_image_topic = (
            self.get_parameter('depth_image_topic').get_parameter_value().string_value
        )
        if self.depth_image_topic != '':
            self.depth_image_sub = self.create_subscription(
                Image,
                self.depth_image_topic,
                self.depth_image_callback,
                QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    durability=QoSDurabilityPolicy.VOLATILE,
                    depth=1,
                ),
            )
            self.depth_image = None
            self.maximum_object_depth_size = (
                self.get_parameter('maximum_object_depth_size').get_parameter_value().double_value
            )

            self.depth_camerainfo_topic = (
                self.get_parameter('depth_camerainfo_topic').get_parameter_value().string_value
            )
            if self.depth_camerainfo_topic != '':
                self.depth_camerainfo_sub = self.create_subscription(
                    CameraInfo,
                    self.depth_camerainfo_topic,
                    self.depth_camerainfo_callback,
                    QoSProfile(
                        reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        history=QoSHistoryPolicy.KEEP_LAST,
                        durability=QoSDurabilityPolicy.VOLATILE,
                        depth=1,
                    ),
                )
                self.depth_camera_intrinsics: Optional[CameraIntrinsics] = None

        self.use_tracker = self.get_parameter('use_tracker').get_parameter_value().bool_value
        if self.use_tracker:
            tracker_config_filepath = check_yaml(
                (self.get_parameter('tracker_config_filepath').get_parameter_value().string_value)
            )
            tracker_config = IterableSimpleNamespace(**YAML.load(tracker_config_filepath))

            if tracker_config.tracker_type == 'bytetrack':
                self.tracker = BYTETracker(args=tracker_config, frame_rate=1)
            elif tracker_config.tracker_type == 'botsort':
                self.tracker = BOTSORT(args=tracker_config, frame_rate=1)
            else:
                raise AssertionError(
                    f"Only 'bytetrack' and 'botsort' are supported for now, but got '{tracker_config.tracker_type}'"  # noqa
                )

        self.plot_colors = Colors()

        self.publish_bbox_3d_markers = (
            self.get_parameter('publish_bbox_3d_markers').get_parameter_value().bool_value
        )
        if self.publish_bbox_3d_markers:
            if not self.use_tracker:
                raise AssertionError(
                    'publish_bbox_3d_markers parameter is set to True but use_tracker is set to False.'  # noqa
                    ' 3d_marker unique ID management without object tracking is not implemented yet.'  # noqa
                )

            self.rviz_3d_visualization_markers_pub = self.create_publisher(
                MarkerArray,
                self.get_parameter('rviz_3d_visualization_markers_topic')
                .get_parameter_value()
                .string_value,
                10,
            )

        self.detection_array_pub = self.create_publisher(
            Detection3DArray, self.detections_topic, 10
        )
        if self.publish_annotated_image:
            self.processed_image_pub = self.create_publisher(
                Image,
                self.get_parameter('annotated_color_image_topic')
                .get_parameter_value()
                .string_value,
                10,
            )

    def resolve_model_name(self, model_name: str) -> str:
        model_path = Path(model_name)
        if model_path.suffix != '.engine':
            return model_name

        try:
            importlib.import_module('tensorrt')
            return model_name
        except (ImportError, OSError) as exc:
            pytorch_model_path = model_path.with_suffix('.pt')
            if pytorch_model_path.exists():
                self.get_logger().warning(
                    f"Cannot load TensorRT for model '{model_path}': {exc}. "
                    f"Falling back to PyTorch model '{pytorch_model_path}'."
                )
                return str(pytorch_model_path)

            raise RuntimeError(
                f"Cannot load TensorRT for model '{model_path}', and fallback "
                f"'{pytorch_model_path}' does not exist: {exc}"
            ) from exc

    def load_model(self, model_name: str) -> None:
        self.model_name = model_name
        self.model = YOLO(model_name)
        if Path(model_name).suffix == '.pt':
            self.model.fuse()

    def run_inference(self, cv_image: np.ndarray) -> list[Results]:
        try:
            return list(self.run_inference_once(cv_image))
        except Exception as exc:
            model_path = Path(self.model_name)
            if model_path.suffix != '.engine' or self.engine_fallback_attempted:
                raise

            self.engine_fallback_attempted = True
            pytorch_model_path = model_path.with_suffix('.pt')
            if not pytorch_model_path.exists():
                raise RuntimeError(
                    f"TensorRT model '{model_path}' failed to load, and fallback "
                    f"'{pytorch_model_path}' does not exist: {exc}"
                ) from exc

            self.get_logger().error(
                f"TensorRT model '{model_path}' failed during inference: {exc}. "
                f"Falling back to PyTorch model '{pytorch_model_path}'. Re-export the "
                "TensorRT engine with the installed TensorRT version before using it again."
            )
            self.load_model(str(pytorch_model_path))
            return list(self.run_inference_once(cv_image))

    def run_inference_once(self, cv_image: np.ndarray):
        if self.use_tracker:
            return self.model.track(
                source=cv_image,
                verbose=self.prediction_verbose,
                stream=self.color_image_topic_as_stream,
                persist=True,
                conf=self.confidence_threshold,
                device=self.device,
            )

        return self.model.predict(
            source=cv_image,
            verbose=self.prediction_verbose,
            stream=self.color_image_topic_as_stream,
            conf=self.confidence_threshold,
            device=self.device,
        )

    def color_image_callback(self, msg: Image) -> None:
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
        detection_array = Detection3DArray()
        detection_array.header = msg.header

        depth_image = self.depth_image  # TODO: check the time diff between depth & color
        if depth_image is not None:
            object_mask = np.logical_and(
                ~np.isnan(depth_image),  # Check for NaN values
                np.isfinite(depth_image),  # Check for finite values
                depth_image != 0.0,  # Check for non-zero values
            )
            image_height, image_width = depth_image.shape[:2]
        else:
            object_mask = None

        results_list = self.run_inference(cv_image)
        results: Results = results_list[0].cpu()

        if results.boxes is not None:
            if self.publish_bbox_3d_markers:
                visualization_marker_array = MarkerArray()

            if self.publish_annotated_image:
                annotator = Annotator(
                    deepcopy(results.orig_img),
                    line_width=2,
                )

            for index, box_data in enumerate(results.boxes):
                detection = Detection3D()
                object_hypothesis = ObjectHypothesisWithPose()
                object_hypothesis.pose.pose.position.x = float(box_data.xywh[0][0])
                object_hypothesis.pose.pose.position.y = float(box_data.xywh[0][1])
                object_hypothesis.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                object_hypothesis.hypothesis.score = float(box_data.conf)
                object_hypothesis.hypothesis.class_id = f'{int(box_data.cls.item())}'

                detection.header = msg.header
                detection.bbox.center.position.x = float(box_data.xywh[0][0])
                detection.bbox.center.position.y = float(box_data.xywh[0][1])
                detection.bbox.center.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                detection.bbox.size.x = float(box_data.xywh[0][2])
                detection.bbox.size.y = float(box_data.xywh[0][3])

                if box_data.is_track:
                    detection.id = f'{int(box_data.id.item())}'

                if (
                    self.depth_image is not None
                    and self.depth_camera_intrinsics is not None
                    and object_mask is not None
                ):
                    object_mask_clone = object_mask.copy()
                    if results.masks:
                        seg_mask = results.masks[index].cpu().data.numpy().transpose(1, 2, 0)
                        seg_mask = cv2.resize(seg_mask, (image_width, image_height))
                        object_mask_clone = np.logical_and(
                            object_mask_clone, seg_mask.astype(bool)
                        )

                    bbox_3d = self.generate_3d_bbox(box_data, depth_image, object_mask_clone)
                    if bbox_3d is not None:
                        detection.bbox = bbox_3d
                        object_hypothesis.pose.pose = detection.bbox.center
                    else:
                        continue

                    if self.publish_bbox_3d_markers and box_data.id is not None:
                        visualization_marker_array.markers.append(
                            self.generate_rviz_3d_visualization_marker(
                                detection.bbox,
                                msg.header,
                                self.plot_colors(int(box_data.cls), True),
                                int(box_data.id),
                            )
                        )

                detection.results.append(object_hypothesis)
                detection_array.detections.append(detection)

                if self.publish_annotated_image:
                    if results.masks:
                        self.annotate_seg_bbox(
                            annotator,
                            box_data,
                            results.masks.xy[index],
                            results.names,
                        )
                    else:
                        self.annotate_bbox(
                            annotator,
                            box_data,
                            results.names,
                        )

            if self.publish_bbox_3d_markers:
                self.rviz_3d_visualization_markers_pub.publish(visualization_marker_array)

        self.detection_array_pub.publish(detection_array)

        if self.publish_annotated_image:
            if results.boxes:
                annotated_image = annotator.result()
                annotated_image_msg = self.cv_bridge.cv2_to_imgmsg(
                    annotated_image, encoding='rgb8'
                )
                annotated_image_msg.header = msg.header
            else:
                annotated_image_msg = msg

            self.processed_image_pub.publish(annotated_image_msg)

    def annotate_bbox(self, annotator, box_data, cls_names, txt_color=(255, 255, 255)):
        object_color = self.plot_colors(int(box_data.cls), True)
        box = box_data.xyxy.squeeze()
        p1, p2 = (int(box[0]), int(box[1])), (int(box[2]), int(box[3]))
        cv2.rectangle(
            annotator.im, p1, p2, object_color, thickness=annotator.lw, lineType=cv2.LINE_AA
        )

        self.annotate_bbox_label(annotator, box_data, object_color, cls_names)

    def annotate_seg_bbox(
        self, annotator, box_data, seg_mask_xy, cls_names, txt_color=(255, 255, 255)
    ):
        object_color = self.plot_colors(int(box_data.cls), True)
        cv2.polylines(
            annotator.im,
            [np.int32([seg_mask_xy])],
            isClosed=True,
            color=object_color,
            thickness=annotator.lw,
        )

        self.annotate_bbox_label(annotator, box_data, object_color, cls_names)

    def annotate_bbox_label(
        self, annotator, box_data, object_color, cls_names, txt_color=(255, 255, 255)
    ):
        label = f'{cls_names[int(box_data.cls)]}({float(box_data.conf[0].numpy()):.2f})'
        if box_data.is_track:
            label = label + f' ID:{int(box_data.id[0].numpy())}'

        box = box_data.xyxy.squeeze()
        p1, p2 = (int(box[0]), int(box[1])), (int(box[2]), int(box[3]))
        w, h = cv2.getTextSize(label, 0, fontScale=annotator.sf, thickness=annotator.tf)[
            0
        ]  # text width, height
        outside = p1[1] - h >= 3
        p2 = p1[0] + w, p1[1] - h - 3 if outside else p1[1] + h + 3
        cv2.rectangle(annotator.im, p1, p2, object_color, -1, cv2.LINE_AA)  # filled
        cv2.putText(
            annotator.im,
            label,
            (p1[0], p1[1] - 2 if outside else p1[1] + h + 2),
            0,
            annotator.sf,
            txt_color,
            thickness=annotator.tf,
            lineType=cv2.LINE_AA,
        )

    def depth_image_callback(self, msg: Image) -> None:
        depth_image = np.array(self.cv_bridge.imgmsg_to_cv2(msg), dtype=float)

        # Convert depth values to meters
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

    def depth_camerainfo_callback(self, msg: CameraInfo) -> None:
        # Extract camera intrinsic parameters from CameraInfo message
        fx = msg.k[0]  # Focal length in x direction
        fy = msg.k[4]  # Focal length in y direction
        cx = msg.k[2]  # Principal point (center) in x direction
        cy = msg.k[5]  # Principal point (center) in y direction

        # Create a CameraIntrinsics object
        self.depth_camera_intrinsics = CameraIntrinsics(fx=fx, fy=fy, cx=cx, cy=cy)

    def generate_3d_bbox(self, box_data, depth_image: np.ndarray, object_mask: np.ndarray):
        bbox_3d = BoundingBox3D()
        bbox_3d.center.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        bbox_2d_center_x = box_data.xywh[0][0].item()
        bbox_2d_center_y = box_data.xywh[0][1].item()
        width_2d = box_data.xywh[0][2].item()
        height_2d = box_data.xywh[0][3].item()

        # Get bounding box coordinates, ensuring they are within the image dimensions
        image_height, image_width = depth_image.shape[:2]
        bbox_x_min = np.clip(int(round(bbox_2d_center_x - width_2d / 2.0)), 0, image_width - 1)
        bbox_x_max = np.clip(int(round(bbox_2d_center_x + width_2d / 2.0)), 0, image_width - 1)
        bbox_y_min = np.clip(int(round(bbox_2d_center_y - height_2d / 2.0)), 0, image_height - 1)
        bbox_y_max = np.clip(int(round(bbox_2d_center_y + height_2d / 2.0)), 0, image_height - 1)

        # Create bbox mask and apply to object mask
        bbox_mask = np.zeros_like(depth_image, dtype=bool)
        bbox_mask[bbox_y_min : bbox_y_max + 1, bbox_x_min : bbox_x_max + 1] = True  # noqa
        object_mask = np.logical_and(
            object_mask,
            bbox_mask,
        )

        if np.any(object_mask):
            # Find centroid of the point cloud
            object_mask = np.logical_and(
                object_mask,
                (depth_image - np.min(depth_image[object_mask]))
                <= self.maximum_object_depth_size,  # Apply maximum object depth size
            )

            object_depth_values = depth_image[object_mask]
            object_2d_points = np.where(object_mask)
            object_3d_points = np.column_stack(
                (object_2d_points[1], object_2d_points[0], object_depth_values)
            )
            point_cloud_centroid = np.mean(object_3d_points, axis=0)

            # optical reference frame
            # get z-dim coordinate of geometric center from centroid
            optical_3d_geometric_center_z = point_cloud_centroid[2]

            # project bbox center to 3d
            optical_3d_geometric_center_x = (
                (bbox_2d_center_x - self.depth_camera_intrinsics.cx)
                * optical_3d_geometric_center_z
                / self.depth_camera_intrinsics.fx
            )
            optical_3d_geometric_center_y = (
                (bbox_2d_center_y - self.depth_camera_intrinsics.cy)
                * optical_3d_geometric_center_z
                / self.depth_camera_intrinsics.fy
            )

            bbox_3d.size.x = (
                width_2d * optical_3d_geometric_center_z / self.depth_camera_intrinsics.fx
            )
            bbox_3d.size.y = (
                height_2d * optical_3d_geometric_center_z / self.depth_camera_intrinsics.fy
            )

            # estimate depth size based on available point cloud
            closest_distance_to_object_from_camera = np.min(object_depth_values)
            bbox_3d.size.z = max(
                2 * (optical_3d_geometric_center_z - closest_distance_to_object_from_camera), 0.1
            )

            # camera base reference frame
            bbox_3d.center.position.x = optical_3d_geometric_center_z
            bbox_3d.center.position.y = -optical_3d_geometric_center_x
            bbox_3d.center.position.z = -optical_3d_geometric_center_y

            return bbox_3d
        else:
            return None

    def generate_rviz_3d_visualization_marker(
        self, bbox_3d: BoundingBox3D, header, color_rgb, object_id
    ):
        visualization_marker = Marker()

        visualization_marker.header = header
        visualization_marker.type = Marker.CUBE
        visualization_marker.id = object_id
        visualization_marker.lifetime = Duration(
            sec=1,
        )
        visualization_marker.frame_locked = True

        # Set the scale of the marker
        visualization_marker.scale.x = bbox_3d.size.z
        visualization_marker.scale.y = bbox_3d.size.x
        visualization_marker.scale.z = bbox_3d.size.y

        # Set the color and transparency of the marker
        visualization_marker.color.r = float(color_rgb[0]) / 255.0
        visualization_marker.color.g = float(color_rgb[1]) / 255.0
        visualization_marker.color.b = float(color_rgb[2]) / 255.0
        visualization_marker.color.a = 0.25

        return visualization_marker


@dataclass
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


def main(args=None):
    rclpy.init(args=args)
    yolo_node = YoloNode()
    executor = SingleThreadedExecutor()
    executor.add_node(yolo_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        yolo_node.get_logger().info('User requested shutdown with SIGINT.')
    finally:
        # Cleanup on exit
        try:
            yolo_node.destroy_node()
        except Exception as e:
            print(f'Error during node destruction: {e}')
        # Only shutdown if the context is still valid
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
