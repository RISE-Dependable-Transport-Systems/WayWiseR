#!/usr/bin/env python3

import math

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException
from vision_msgs.msg import Detection3DArray

from waywiser_py.waywiser_utils import RELIABLE_TRANSIENT_LOCAL_QOS
from waywiser_twist_safety.msg import EmergencyStopState


class CollisionMonitor(Node):
    """
    Node that monitors Detection3DArray messages and publishes EmergencyStopState if needed.

    Uses TF2 to transform detection points from camera frame to base_link.

    Publishes an EmergencyStopState message if the closest object detected is
    closer than a specified distance threshold.
    """

    def __init__(self):
        super().__init__('collision_monitor_node')

        # Parameters for topics
        self.declare_parameter('distance_threshold', 1.32)
        self.declare_parameter('detections_topic', '/oak/nn/spatial_detections')
        self.declare_parameter('emergency_stop_topic', '/emergency_stop')
        self.declare_parameter('class_ids_to_stop', [str()])  # Class IDs published as strings
        self.declare_parameter('tf_timeout_sec', 0.5)
        self.declare_parameter('reference_frame', 'base_link')

        # Retrieve parameters
        self.distance_threshold = (
            self.get_parameter('distance_threshold').get_parameter_value().double_value
        )
        self.detections_topic = (
            self.get_parameter('detections_topic').get_parameter_value().string_value
        )
        self.emergency_stop_topic = (
            self.get_parameter('emergency_stop_topic').get_parameter_value().string_value
        )
        self.class_ids_to_stop = set(
            self.get_parameter('class_ids_to_stop').get_parameter_value().string_array_value
        )
        self.reference_frame = (
            self.get_parameter('reference_frame').get_parameter_value().string_value
        )

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Convert timeout parameter to a Duration used by tf_buffer.transform
        self.tf_timeout = Duration(
            seconds=self.get_parameter('tf_timeout_sec').get_parameter_value().double_value
        )

        # Publishers
        self.emergency_stop_publisher = self.create_publisher(
            EmergencyStopState, self.emergency_stop_topic, RELIABLE_TRANSIENT_LOCAL_QOS
        )

        # Subscribers
        self.detection_array_subscriber = self.create_subscription(
            Detection3DArray, self.detections_topic, self.detection_array_callback, 10
        )

        # Initialise emergency_stop_target_state_msg
        self.emergency_stop_target_state_msg = EmergencyStopState()
        self.emergency_stop_target_state_msg.sender_id = 'collision_monitor'
        self.emergency_stop_target_state_msg.state = EmergencyStopState.ACTIVE

        self.get_logger().info(f'Subscribed to {self.detections_topic}')

    def get_best_result(self, detection):
        best = None
        best_score = float('-inf')
        for r in detection.results:
            try:
                _ = r.pose.pose.position  # Ensure pose exists
            except Exception:
                continue
            if r.hypothesis.score > best_score:
                best = r
                best_score = r.hypothesis.score
        return best

    def transform_point(self, point_stamped):
        try:
            return self.tf_buffer.transform(point_stamped, self.reference_frame, self.tf_timeout)
        except TransformException as e:
            self.get_logger().warning(f'TF transform failed: {e}')
            return None

    def create_point_stamped(self, position, detection, array_header):
        """Return PointStamped for position with detection.header if present, else array_header."""
        point_stamped = PointStamped()
        point_stamped.point = position

        if hasattr(detection, 'header') and detection.header.frame_id:
            point_stamped.header.frame_id = detection.header.frame_id
            point_stamped.header.stamp = detection.header.stamp
        else:
            point_stamped.header.frame_id = array_header.frame_id
            point_stamped.header.stamp = array_header.stamp

        return point_stamped

    def detection_array_callback(self, msg):  # Process incoming Detection3DArray messages
        closest_detection = None
        closest_best = (
            None  # Result with highest confidence score result for the closest detection
        )
        min_distance_sq = float(
            'inf'
        )  # Use squared distance for comparison to avoid repeated sqrt calculations

        for detection in msg.detections:
            if not detection.results:
                continue

            # Find the best result in this detection that has a pose
            best = self.get_best_result(detection)
            if best is None:
                continue

            class_id = best.hypothesis.class_id  # string
            if self.class_ids_to_stop and class_id not in self.class_ids_to_stop:
                continue

            # Create a stamped point for the object's position in the original frame
            point_stamped = self.create_point_stamped(
                best.pose.pose.position, detection, msg.header
            )  # best.pose.pose.position is a Point with x,y,z

            # Transform the point into the target frame
            transformed = self.transform_point(point_stamped)
            if transformed is None:
                continue  # Skip detection if transform fails

            # Calculate squared distance in the target frame
            p = transformed.point
            current_distance_sq = p.x**2 + p.y**2 + p.z**2

            # If detection is closer than the current minimum, update
            if current_distance_sq < min_distance_sq:
                min_distance_sq = current_distance_sq
                closest_detection = detection
                closest_best = best

        # Process only the closest object (already transformed distance used)
        if closest_detection is not None and closest_best is not None:
            distance = math.sqrt(min_distance_sq)  # Actual distance in meters
            if distance < self.distance_threshold:
                self.emergency_stop_target_state_msg.reason = (
                    f'Object class {closest_best.hypothesis.class_id} detected at '
                    f'{distance:.2f} m with score {closest_best.hypothesis.score:.2f}'
                )
                self.emergency_stop_target_state_msg.stamp = (
                    msg.header.stamp
                )  # Forwarding time stamp of detections
                self.emergency_stop_publisher.publish(self.emergency_stop_target_state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionMonitor()

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
