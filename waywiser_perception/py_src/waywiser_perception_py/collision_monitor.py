#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from waywiser_py.waywiser_utils import RELIABLE_TRANSIENT_LOCAL_QOS

from waywiser_perception.msg import DetectionArray
from waywiser_twist_safety.msg import EmergencyStopState


class CollisionMonitor(Node):
    """
    Node that monitors DetectionArray messages and publishes EmergencyStopState if needed.

    Publishes an EmergencyStopState message if the closest object detected is
    closer than a specified distance threshold.
    """

    def __init__(self):
        super().__init__('collision_monitor_node')

        # Parameters for topics
        self.declare_parameter('distance_threshold', 1.32)
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('emergency_stop_topic', '/emergency_stop')
        self.declare_parameter('class_ids_to_stop', [int()])

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
        self.class_ids_to_stop = (
            self.get_parameter('class_ids_to_stop').get_parameter_value().integer_array_value
        )

        # Publishers
        self.emergency_stop_publisher = self.create_publisher(
            EmergencyStopState, self.emergency_stop_topic, RELIABLE_TRANSIENT_LOCAL_QOS
        )

        # Subscribers
        self.detection_array_subscriber = self.create_subscription(
            DetectionArray, self.detections_topic, self.detection_array_callback, 10
        )

        # Initialize emergency_stop_target_state_msg
        self.emergency_stop_target_state_msg = EmergencyStopState()
        self.emergency_stop_target_state_msg.sender_id = 'collision_monitor'
        self.emergency_stop_target_state_msg.state = EmergencyStopState.ACTIVE

        self.get_logger().info(f'Subscribed to {self.detections_topic}')

    def detection_array_callback(self, msg):  # Process incoming DetectionArray messages
        closest_detection = None
        min_distance_sq = float(
            'inf'
        )  # Use squared distance for comparison to avoid repeated sqrt calculations

        for detection in msg.detections:
            class_id = detection.class_id
            if (len(self.class_ids_to_stop) > 0 and class_id in self.class_ids_to_stop) or len(
                self.class_ids_to_stop
            ) == 0:
                bbox_3d = detection.bbox_3d

                # Calculate squared distance
                current_distance_sq = (
                    bbox_3d.geometric_center_pose.position.x**2
                    + bbox_3d.geometric_center_pose.position.y**2
                    + bbox_3d.geometric_center_pose.position.z**2
                )

                # If this detection is closer than the current minimum, update
                if current_distance_sq < min_distance_sq:
                    min_distance_sq = current_distance_sq
                    closest_detection = detection

        # Process only the closest object
        if closest_detection is not None:
            distance = math.sqrt(min_distance_sq)  # Calculate the actual distance in meters

            if distance < self.distance_threshold:
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
