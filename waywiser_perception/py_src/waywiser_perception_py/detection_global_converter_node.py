#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PointStamped, Pose, PoseArray
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

import tf2_ros
from tf2_ros import TransformException

# DepthAI spatial detection message - optionally present in the workspace
try:
    from depthai_ros_msgs.msg import SpatialDetectionArray
except Exception:
    SpatialDetectionArray = None  # node will check at runtime and error if not present

import numpy as np
from threading import Lock


# ----------------------------
# Quaternion utility functions
# ----------------------------
def quat_conjugate(q):
    # Conjugate of quaternion q = [-v, w]
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quat_mul(q1, q2):
    # Multiplication of two quaternions (x,y,z,w) 
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return np.array([x, y, z, w])


def rotate_vector_by_quaternion(vec, quat):
    # Rotate a 3D vector by a quaternion: v' = q * (v_quat) * q_conj
    q_vec = np.array([vec[0], vec[1], vec[2], 0.0])
    q = np.array([quat[0], quat[1], quat[2], quat[3]])
    q_conj = quat_conjugate(q)
    res = quat_mul(quat_mul(q, q_vec), q_conj)
    return res[:3]


# ----------------------------
# Main Node
# ----------------------------
class DetectionGlobalConverter(Node): 

    """
    Node that:
    1. Subscribes to camera-relative spatial detections.
    2. Listens to IMU topic (default: /imu/data) to get quaternion orientation (TODO camera configuration: i_enable_rotation: true and i_rot_mode: "ROTATION_VECTOR").
    3. Uses TF2 to transform detection points from the camera frame into the IMU frame.
    4. Rotates those points by the IMU quaternion to align with gravity (world frame).
    5. Publishes a geometry_msgs/PoseArray on 'detections_topic_global' with the transformed positions.

    TODO World -> GPS conversion is currently omitted. Publishes instead the gravity-aligned world frame.
        - Use /nav_sat_fix position after converting lat/lon to a local cartesian frame like ENU, with the coordinatetransforms.h infrastructure from WayWise. Alternatively, use the robot_localization sensor fusion package to combine IMU and GNSS data.
    """
    def __init__(self):
        super().__init__('detection_global_converter')

        # Declare arameters
        self.declare_parameter('detections_topic', 'detections_topic_drone')
        self.declare_parameter('global_topic', 'detections_topic_global')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('imu_frame', 'imu_link') # expected TF frame name for the IMU
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('tf_timeout_sec', 0.5)   # TF lookup timeout in seconds

        # Read parameters into local variables
        det_topic = self.get_parameter('detections_topic').get_parameter_value().string_value
        global_topic = self.get_parameter('global_topic').get_parameter_value().string_value
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value

        # TF2, used to transform detection points from the camera's frame to the IMU frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # IMU 
        self.latest_imu = None
        self._imu_lock = Lock() # Protected by a lock for thread-safety
        self.create_subscription(Imu, imu_topic, self.imu_callback, 10)

        # Detections subscriber (expects depthai_ros_msgs/SpatialDetectionArray)
        if SpatialDetectionArray is None:
            raise RuntimeError("depthai_ros_msgs.SpatialDetectionArray is not importable. Install the dependency or adapt the node.")
        self.create_subscription(
            SpatialDetectionArray,
            det_topic,
            self.detections_callback,
            10
        )

        # Publisher for transformed detections
        self.publisher = self.create_publisher(PoseArray, global_topic, 10)

        # Convert configured timeout parameter to a Duration used by tf_buffer.transform.
        self.tf_timeout = Duration(seconds=self.get_parameter('tf_timeout_sec').get_parameter_value().double_value)

        self.get_logger().info(f"Subscribed: {det_topic}, IMU: {imu_topic}, Publishing: {global_topic}")

    # ----------------------------
    # Callbacks
    # ----------------------------
    def imu_callback(self, msg: Imu):
        with self._imu_lock:
            self.latest_imu = msg
        # Runtime sanity check: if quaternion norm is ~0, likely rotation vectors are not enabled
        q = msg.orientation
        quat_norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if quat_norm < 1e-3:
            self.get_logger().warning_once("Received IMU orientation appears unset (quaternion norm ~0). Ensure camera rotation output is enabled.")

    def detections_callback(self, msg):
        """
        Main processing callback for incoming detections.
        Steps:
         1. Ensure IMU data exists.
         2. Determine the frame where detection points are expressed.
         3. For each detection:
            a. Extract spatial coordinates.
            b. Build a PointStamped in the detection frame.
            c. Use TF2 to transform that point into the IMU frame.
            d. Rotate the transformed point by the IMU quaternion to align with gravity.
            e. Collect as a Pose for publishing.
         4. Publish a PoseArray with all transformed detections.
        """

        # Expect msg.detections -> list of SpatialDetection, each with spatial_coordinates (x,y,z)
        if not hasattr(msg, 'detections'):
            self.get_logger().warning("Incoming detections message has no 'detections' field. Ignoring.")
            return

        # Use the latest IMU orientation
        with self._imu_lock:
            imu = self.latest_imu
        if imu is None:
            self.get_logger().warning_once("No IMU message received yet; cannot convert to world frame.")
            return

        # Determine source frame for points. Prefer the top-level header.frame_id
        source_frame = msg.header.frame_id if hasattr(msg, 'header') and msg.header.frame_id else None
        # Fallback to detection header
        if not source_frame and len(msg.detections) > 0 and hasattr(msg.detections[0], 'header'):
            source_frame = msg.detections[0].header.frame_id

        if not source_frame:
             # If the detection message lacks a frame, assume a typical camera optical frame name
            source_frame = 'camera_optical_frame'

        poses = []
        # Process each detection entry
        for det in msg.detections:
            # Attempt to extract spatial coordinates in a few common formats:
            # - det.spatial_coordinates with x,y,z attributes
            # - det.spatial_coordinates.data (array-like)
            # - det.pose.position (fallback)
            xyz = None
            if hasattr(det, 'spatial_coordinates'):
                sc = det.spatial_coordinates
                # fields maybe .x,.y,.z or array
                if hasattr(sc, 'x') and hasattr(sc, 'y') and hasattr(sc, 'z'):
                    xyz = (float(sc.x), float(sc.y), float(sc.z))
                elif hasattr(sc, 'data'):
                    arr = list(sc.data)
                    if len(arr) >= 3:
                        xyz = (float(arr[0]), float(arr[1]), float(arr[2]))
            # fallback: some messages might carry a pose field
            if xyz is None and hasattr(det, 'pose') and hasattr(det.pose, 'position'):
                p = det.pose.position
                xyz = (float(p.x), float(p.y), float(p.z))

            if xyz is None:
                self.get_logger().debug("Could not parse detection spatial coordinates; skipping one detection.")
                continue

            # Build a PointStamped containing the detection point in the source (camera) frame
            p_stamped = PointStamped()
            p_hdr = Header()
            p_hdr.stamp = msg.header.stamp if hasattr(msg, 'header') else self.get_clock().now().to_msg()
            p_hdr.frame_id = source_frame
            p_stamped.header = p_hdr
            p_stamped.point.x, p_stamped.point.y, p_stamped.point.z = xyz

            # Transform point into IMU frame via TF, resolves the camera -> camera_base -> imu_frame chain
            imu_frame = self.get_parameter('imu_frame').get_parameter_value().string_value
            try:
                # request transform camera_frame -> imu_frame
                transformed_point = self.tf_buffer.transform(
                    p_stamped, imu_frame, timeout=self.tf_timeout
                )
            except TransformException as ex:
                self.get_logger().warning_once(f"TF transform failed from {source_frame} to {imu_frame}: {ex}")
                # If TF fails, attempt to proceed best-effort by assuming point is already in imu_frame
                transformed_point = p_stamped
                transformed_point.header.frame_id = imu_frame

            # Rotate the transformed point using the IMU quaternion into a gravity-aligned "world" frame
            q = imu.orientation
            # quat is [x,y,z,w] (vector part then scalar), vec is [x,y,z]
            quat = [q.x, q.y, q.z, q.w]
            vec = np.array([transformed_point.point.x, transformed_point.point.y, transformed_point.point.z])
             # Rotate vector into world alignment
            world_vec = rotate_vector_by_quaternion(vec, quat)

            # Build a Pose from the rotated position
            pose = Pose()
            pose.position.x = float(world_vec[0])
            pose.position.y = float(world_vec[1])
            pose.position.z = float(world_vec[2])
            # orientation left as identity (no rotation encoded for the detected point itself)
            pose.orientation.w = 1.0
            poses.append(pose)

        # Publish poses as PoseArray
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = self.get_parameter('world_frame').get_parameter_value().string_value
        pa.poses = poses

        self.publisher.publish(pa)

# ----------------------------
# Entry point
# ----------------------------
def main(args=None):
    rclpy.init(args=args)
    node = DetectionGlobalConverter()

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