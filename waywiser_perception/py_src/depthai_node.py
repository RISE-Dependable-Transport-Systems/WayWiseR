#!/usr/bin/env python3

import os
import cv2
from cv_bridge import CvBridge
import math
import depthai as dai
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Quaternion

from waywiser_perception.msg import BoundingBox
from waywiser_perception.msg import Detection
from waywiser_perception.msg import DetectionArray

# --- Constants ---
COLOR_RESOLUTION = dai.ColorCameraProperties.SensorResolution.THE_1080_P
MONO_RESOLUTION = (
    dai.MonoCameraProperties.SensorResolution.THE_400_P
)  # 800p improves depth accuracy, but uses more resources
YOLO_SPATIAL_INPUT_SIZE = (416, 416)  # YOLOv3-tiny input size
NN_CONFIDENCE_THRESHOLD = 0.5  # Default confidence threshold for the NN
STEREO_CONFIDENCE_THRESHOLD = (
    250  # Confidence threshold for disparity calculation (0..255)
)

class DepthAI(Node):
    def __init__(self):
        super().__init__('depthai_node')

        # Parameters for topics
        self.declare_parameter('rgb_topic', '/rgb/image')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('model_path', '')
        self.declare_parameter('camera_base_frame', 'camera_frame')

        # Retrieve parameters
        self.rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        self.detections_topic = self.get_parameter('detections_topic').get_parameter_value().string_value
        self.model_path = os.path.expanduser(self.get_parameter('model_path').get_parameter_value().string_value)
        self.camera_base_frame = self.get_parameter('camera_base_frame').get_parameter_value().string_value

        # Publishers
        self.image_publisher = self.create_publisher(Image, self.rgb_topic, 10)
        self.detection_array_publisher = self.create_publisher(DetectionArray, self.detections_topic, 10)

        # Bridge to convert ROS images to OpenCV
        self.cv_bridge = CvBridge()

        self.pipeline = self.create_pipeline()

    def create_pipeline(self):
        self.get_logger().info('Creating DepthAI pipeline...')

        pipeline = dai.Pipeline()

        # --- Define Sources ---
        self.get_logger().info('Creating camera nodes (ColorCamera, MonoCamera)...')
        color = pipeline.create(dai.node.ColorCamera)
        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_right = pipeline.create(dai.node.MonoCamera)

        # --- Define Processing Nodes ---
        self.get_logger().info('Creating processing nodes (StereoDepth, ImageManip, NN)...')
        stereo = pipeline.create(dai.node.StereoDepth)
        manip = pipeline.create(dai.node.ImageManip)  # For resizing color frames for NN
        NN = pipeline.create(dai.node.YoloSpatialDetectionNetwork)

        # --- Define Outputs ---
        self.get_logger().info('Creating output nodes (XLinkOut)...')
        xout_RGB = pipeline.create(dai.node.XLinkOut)
        xout_RGB.setStreamName("frames")  # Output for NN passthrough frames

        xout_NN = pipeline.create(dai.node.XLinkOut)
        xout_NN.setStreamName("detections")  # Output for NN results

        # --- Configure Nodes ---

        # Color Camera Configuration
        self.get_logger().info(f'Configuring ColorCamera (Resolution: {COLOR_RESOLUTION})')
        color.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        color.setResolution(COLOR_RESOLUTION)
        color.setPreviewSize(1920, 1080)  # 16:9 aspect ratio to capture full FoV
        color.setInterleaved(
            False
        )  # Planar Format: Each color channel is stored separately. More efficient for some image processing tasks, as it allows easier access to individual color channels

        # Mono Camera Configuration
        self.get_logger().info(f'Configuring MonoCameras (Resolution: {MONO_RESOLUTION})')
        mono_left.setResolution(MONO_RESOLUTION)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_right.setResolution(MONO_RESOLUTION)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        # Stereo Camera Configuration
        self.get_logger().info(f'Configuring StereoDepth (Confidence: {STEREO_CONFIDENCE_THRESHOLD})')
        stereo.setDepthAlign(
            dai.CameraBoardSocket.RGB
        )  # Align depth map to the perspective of the color camera
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(False)  # Improves depth accuracy, but uses more resources
        stereo.setConfidenceThreshold(STEREO_CONFIDENCE_THRESHOLD)
        # stereo.setMedianFilter(dai.MedianFilter.KERNEL_7x7) # Add median filtering for smoother depth (might blur sharp depth changes), Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7

        # ImageManip Configuration
        self.get_logger().info(f'Configuring ImageManip (Resize: {YOLO_SPATIAL_INPUT_SIZE})')
        manip.initialConfig.setResizeThumbnail(
            YOLO_SPATIAL_INPUT_SIZE
        )  # Resize images while keeping the aspect ratio by letterboxing
        manip.inputImage.setBlocking(
            True
        )  # Less efficient, but guarantees that no frames are skipped

        # Neural Network (YoloSpatialDetectionNetwork) Configuration
        
        NN_path = self.model_path

        if not os.path.exists(NN_path):
            self.get_logger().info(f'Model file not found at path: {NN_path}')
            raise FileNotFoundError(f'Model file not found at path: {NN_path}')

        model_name = os.path.basename(NN_path)
        self.get_logger().info(f'Configuring YoloSpatialDetectionNetwork (Model: {model_name})')

        NN.setBlobPath(NN_path) # Set model path 

        # Configure NN specific parameters
        NN.setConfidenceThreshold(NN_CONFIDENCE_THRESHOLD)
        NN.input.setBlocking(
            False
        )  # Non-Blocking, allows more efficient and continuous data flow, useful in real-time processing

        # Spatial detection parameters
        NN.setBoundingBoxScaleFactor(0.1)  # Increasing scale factor reduces performance
        NN.setSpatialCalculationAlgorithm(
            dai.SpatialLocationCalculatorAlgorithm.MIN
        )  # Use the minimum value inside ROI (scaled bounding box) for calculation

        # YOLO specific parameters
        NN.setNumClasses(80)
        NN.setCoordinateSize(4)
        NN.setAnchors([10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319])
        NN.setAnchorMasks({"side26": [1, 2, 3], "side13": [3, 4, 5]})
        NN.setIouThreshold(0.5)

        # --- Link Nodes (transfer data from device to host via XLink) ---
        self.get_logger().info('Linking pipeline nodes...')

        # Color camera -> ImageManip -> NN
        color.preview.link(manip.inputImage)
        manip.out.link(NN.input)

        # Mono cameras -> StereoDepth -> NN
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)
        stereo.depth.link(NN.inputDepth)

        # NN outputs -> Host
        NN.passthrough.link(
            xout_RGB.input
        )  # Passthrough RGB frames to host# Passthrough frames are synchronised with NN outputs, ensuring zero discrepancy between what the NN sees and what is displayed
        NN.out.link(xout_NN.input)

        self.get_logger().info('Pipeline created successfully.')

        return pipeline

    def capture(self):
        # We are using context manager here that will dispose the device after we stop using it. This will also check USB and NETWORK interfaces for a device that is available and ready to accept connections
        with dai.Device() as device:
            self.get_logger().info(f'USB speed: {device.getUsbSpeed()}')    # HIGH = USB2, SUPER = USB3

            device.startPipeline(self.pipeline)

            queue_color = device.getOutputQueue(
                name="frames", maxSize=4, blocking=False
            )
            queue_NN = device.getOutputQueue(
                name="detections", maxSize=4, blocking=False
            )

            while True:
                in_frames = (
                    queue_color.get()
                )  # Blocking - Will wait until new data has arrived

                frame = None
                if in_frames is not None:
                    frame = in_frames.getCvFrame()

                if frame is None:
                    continue  # Skip to the next iteration if no frame

                image_msg = self.cv_bridge.cv2_to_imgmsg(
                    frame, encoding='rgb8'
                )

                self.image_publisher.publish(image_msg)
                
                in_NN = queue_NN.get()
                
                if in_NN is not None:

                    img_height = frame.shape[0]
                    img_width  = frame.shape[1]

                    detection_array = DetectionArray()
                    detection_array.header = image_msg.header
                    detection_array.header.frame_id = self.camera_base_frame
                    detection_array.header.stamp = self.get_clock().now().to_msg()

                    for depthai_detection in in_NN.detections:
                            
                        detection = Detection()
                        detection.class_id = depthai_detection.label
                        detection.confidence = depthai_detection.confidence

                        # Get bounding box coordinates
                        x_min = int(depthai_detection.xmin * img_width)
                        x_max = int(depthai_detection.xmax * img_width)
                        y_min = int(depthai_detection.ymin * img_height)
                        y_max = int(depthai_detection.ymax * img_height)

                        detection.bbox_2d.geometric_center_pose.position.x = float((x_min + x_max) / 2)
                        detection.bbox_2d.geometric_center_pose.position.y = float((y_min + y_max) / 2)
                        detection.bbox_2d.width = float(x_max - x_min)
                        detection.bbox_2d.height = float(y_max - y_min)
                        detection.bbox_2d.geometric_center_pose.orientation = Quaternion(
                            x=0.0, y=0.0, z=0.0, w=1.0
                        )

                        if hasattr(depthai_detection, 'spatialCoordinates') and depthai_detection.spatialCoordinates.z > 0: # Z=0 often means invalid depth
                            detection.bbox_3d.geometric_center_pose.position.x = depthai_detection.spatialCoordinates.x / 1000  # convert from [mm] to [m]   
                            detection.bbox_3d.geometric_center_pose.position.y = depthai_detection.spatialCoordinates.y / 1000
                            detection.bbox_3d.geometric_center_pose.position.z = depthai_detection.spatialCoordinates.z / 1000

                        # TODO add 3D dimensions using depth information

                        detection_array.detections.append(detection)
                    
                    self.detection_array_publisher.publish(detection_array)

def main(args=None):
    rclpy.init(args=args)
    node = DepthAI()
    try:
        #rclpy.spin(node)   # no subscribers in the node
        node.capture()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
