#!/usr/bin/env python3

import numpy as np
import rclpy
import tf2_ros
from octomap import OcTree, point3d
from octomap_msgs.msg import Octomap
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from tf2_ros import TransformException


class OcclusionAnalysis(Node):
    """
    Node that analyses occlusions in the OctoMap.

    Computes occlusion volume (within a radius) by ray-casting from the TF2 reference frame and
    counting unknown voxels that are blocked by occupied voxels.

    Reports the occluded volume in cubic meters via logging.
    """

    def __init__(self):
        super().__init__('occlusion_analysis')

        # Parameters
        self.declare_parameter('reference_frame', 'base_link')  # origin for analysis
        self.declare_parameter('octomap_frame_fallback', 'map')
        self.declare_parameter('octomap_topic', '/octomap_full')
        self.declare_parameter('search_radius', 5.0)  # radius in meters
        self.declare_parameter('tf_timeout_sec', 0.5)
        self.declare_parameter('analysis_rate', 1.0)  # Hz
        self.declare_parameter('angular_resolution', 5.0)  # degrees

        # Get string parameters with type safety
        self.reference_frame = self.get_string_parameter('reference_frame', 'base_link')
        self.octomap_frame_fallback = self.get_string_parameter('octomap_frame_fallback', 'map')
        self.octomap_topic = self.get_string_parameter('octomap_topic', '/octomap_full')

        # Get numerical parameters with type safety
        self.search_radius = self.get_float_parameter('search_radius', 5.0)
        self.tf_timeout_sec = self.get_float_parameter('tf_timeout_sec', 0.5)
        self.analysis_rate = self.get_float_parameter('analysis_rate', 1.0)
        self.angular_resolution = np.radians(self.get_float_parameter('angular_resolution', 5.0))

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Octomap
        self.octree = None
        self.octomap_frame = None
        self.map_received = False

        # Subscribers
        qos = QoSProfile(
            depth=1,  # optimal since newer octomaps completely replace older ones
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # ensure publisher keeps the last octomap
        )
        self.octomap_subscriber = self.create_subscription(
            Octomap, self.octomap_topic, self.octomap_callback, qos
        )

        # Timer for periodic analysis
        self.timer = self.create_timer(1.0 / self.analysis_rate, self.calculate_occlusion_volume)

        self.get_logger().info(
            f'OcclusionAnalysis node initialised (radius={self.search_radius}m, '
            f'reference_frame={self.reference_frame})'
        )

    def get_string_parameter(self, name, default_value):
        """Get parameter value with type safety"""
        param = self.get_parameter(name)
        value = param.value

        if value is None:
            self.get_logger().warning(
                f'Parameter "{name}" is None, using default value: {default_value}'
            )
            return str(default_value)
        else:
            try:
                return str(value)
            except (ValueError, TypeError) as e:
                self.get_logger().warning(
                    f'Invalid {name} parameter value: {e}, using default value {default_value}'
                )
                return str(default_value)

    def get_float_parameter(self, name, default_value):
        """Get parameter value with type safety"""
        param = self.get_parameter(name)
        value = param.value

        if value is None:
            self.get_logger().warning(
                f'Parameter "{name}" is None, using default value: {default_value}'
            )
            return float(default_value)
        else:
            try:
                return float(value)
            except (ValueError, TypeError) as e:
                self.get_logger().warning(
                    f'Invalid {name} parameter value: {e}, using default value {default_value}'
                )
                return float(default_value)

    def octomap_callback(self, msg: Octomap):
        """Convert ROS OctoMap message to octomap OcTree."""
        try:
            self.octree = OcTree(msg.resolution)  # create empty octree with given resolution
            self.octree.readBinaryData(
                msg.data
            )  # deserialise the octomap structure from the message's binary data

            if hasattr(msg, 'header') and msg.header.frame_id:
                self.octomap_frame = msg.header.frame_id
            else:
                self.octomap_frame = self.octomap_frame_fallback
                self.get_logger().warning(
                    f'Octomap frame_id unavailable, using fallback: {self.octomap_frame_fallback}.'
                    f' If this is incorrect, set the "octomap_frame_fallback" parameter.'
                )

            self.map_received = True  # signal that the map is ready for processing

            self.get_logger().info('Octomap received and parsed')
        except Exception as e:
            self.get_logger().error(f'Failed to parse octomap: {e}')

    def calculate_occlusion_volume(self):
        """Calculate occlusion volume within radius using spherical ray casting"""
        if not self.map_received or self.octree is None:
            return

        try:
            # ensure ray casting occurs in the correct coordinate system
            origin = self.get_origin_in_map_frame()

            # generate spherical directions for ray casting
            directions = self.generate_spherical_directions()

            # analyse rays and count occluded unknown voxels
            occluded_count, total_count = self.analyse_rays(origin, directions)

            # convert voxel count to actual volume
            voxel_volume = self.octree.getResolution() ** 3
            occluded_volume = occluded_count * voxel_volume

            self.get_logger().info(
                f'Occluded unknown voxels: {occluded_count}/{total_count}, '
                f'Volume: {occluded_volume:.6f} m³, within {self.search_radius}m '
                f'from the reference frame: {self.reference_frame}'
            )

        except TransformException as e:
            self.get_logger().error(f'TF transform error: {e}')
        except Exception as e:
            self.get_logger().error(f'Calculation error: {e}')

    def get_origin_in_map_frame(self):
        """Get transform from reference frame to octomap frame"""

        octomap_frame = getattr(self, 'octomap_frame', self.octomap_frame_fallback)

        if (
            octomap_frame == self.octomap_frame_fallback
            and hasattr(self, 'octomap_frame')
            and self.octomap_frame is None
        ):
            self.get_logger().warning(
                f'Using octomap frame fallback: {self.octomap_frame_fallback}. '
            )

        transform = self.tf_buffer.lookup_transform(
            octomap_frame,
            self.reference_frame,
            Time(),
            timeout=Duration(seconds=self.tf_timeout_sec),
        )

        base_pos = transform.transform.translation
        return point3d(base_pos.x, base_pos.y, base_pos.z)

    def generate_spherical_directions(self):
        """Generate uniform spherical directions for ray casting"""
        directions = []

        # spherical coordinates
        phi_values = np.arange(0, np.pi, self.angular_resolution)
        theta_values = np.arange(0, 2 * np.pi, self.angular_resolution)

        for phi in phi_values:
            for theta in theta_values:
                # convert spherical to Cartesian direction (xyz)
                direction = point3d(
                    np.sin(phi) * np.cos(theta), np.sin(phi) * np.sin(theta), np.cos(phi)
                )
                directions.append(direction)

        return directions

    def analyse_rays(self, origin, directions):
        """Cast rays and count occluded vs total unknown voxels"""
        occluded_unknown_count = 0
        total_unknown_count = 0

        for direction in directions:
            ray_end = point3d()
            # create a uniform distribution of rays covering all directions from the origin point
            hit_occupied = self.octree.castRay(
                origin,
                direction,
                ray_end,
                ignoreUnknownCells=False,  # ensures rays stop at unknown space, not only occupied
                maxRange=self.search_radius,
            )

            # check if ray ended at unknown space within radius
            ray_distance = (ray_end - origin).norm()
            if ray_distance < self.search_radius:
                # check if the endpoint is unknown space
                end_node = self.octree.search(ray_end)  # returns None for unknown voxels
                if end_node is None:
                    total_unknown_count += 1

                    # check if unknown voxel is occluded
                    if hit_occupied:  # ray hit occupied space before reaching the unknown voxel
                        occluded_unknown_count += 1

        return occluded_unknown_count, total_unknown_count


def main(args=None):
    rclpy.init(args=args)
    node = OcclusionAnalysis()

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
