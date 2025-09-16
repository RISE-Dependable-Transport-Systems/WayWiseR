#!/usr/bin/env python3
import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor

from std_msgs.msg import Float32
from depthai_ros_msgs.msg import SpatialDetectionArray


# Use a SensorData-style QoS so we play nice with DepthAI topics
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class HumanDistanceNode(Node):
    def __init__(self) -> None:
        super().__init__("human_distance_node")

        # --- Parameters (documented with descriptors) ---
        self.declare_parameter(
            "detections_topic",
            "/oak/nn/spatial_detections",
            descriptor=ParameterDescriptor(
                description="Topic to subscribe for SpatialDetectionArray from DepthAI"
            ),
        )
        self.declare_parameter(
            "person_ids",
            [0],
            descriptor=ParameterDescriptor(
                description="List of class IDs to treat as person (e.g., COCO: [0])"
            ),
        )
        self.declare_parameter(
            "publish_nan_when_absent",
            True,
            descriptor=ParameterDescriptor(
                description="If true, publish NaN when no person is detected"
            ),
        )

        detections_topic = (
            self.get_parameter("detections_topic").get_parameter_value().string_value
        )
        self.person_class_ids: List[int] = list(
            self.get_parameter("person_ids").get_parameter_value().integer_array_value
        )
        self.publish_nan_when_absent: bool = (
            self.get_parameter("publish_nan_when_absent")
            .get_parameter_value()
            .bool_value
        )

        # --- ROS I/O ---
        self.sub = self.create_subscription(
            SpatialDetectionArray, detections_topic, self._on_detections, SENSOR_QOS
        )
        self.pub = self.create_publisher(Float32, "human_distance", SENSOR_QOS)

        self.get_logger().info(
            f"Listening on {detections_topic}; publishing /human_distance "
            f"(person_ids={self.person_class_ids}, publish_nan_when_absent={self.publish_nan_when_absent})"
        )

    def _on_detections(self, msg: SpatialDetectionArray) -> None:
        closest_dist = float("inf")

        for detection in msg.detections:
            if not detection.results:
                continue
            class_id = detection.results[0].id  # highest score first
            if class_id not in self.person_class_ids:
                continue

            x, y, z = detection.position.x, detection.position.y, detection.position.z
            distance = math.sqrt(x**2 + y**2 + z**2)
            if distance < closest_dist:
                closest_dist = distance

        if math.isfinite(closest_dist):
            self.pub.publish(Float32(data=float(closest_dist)))
        elif self.publish_nan_when_absent:
            # expose "no person" as NaN if requested (handy for downstream filters)
            self.pub.publish(Float32(data=float("nan")))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = HumanDistanceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Node stopped by user")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
