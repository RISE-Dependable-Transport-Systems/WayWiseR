#!/usr/bin/env python3

from copy import deepcopy
import json
import math
import os
import time
from typing import Union
import xml.etree.ElementTree as ET

from ament_index_python import get_package_share_directory
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
import numpy as np
import pymap3d as pm
import rclpy
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.node import Node
from shapely import affinity
from shapely.geometry import box as shapely_box
from std_msgs.msg import Bool
from std_msgs.msg import String
from tf2_ros import Buffer
from tf2_ros import ConnectivityException
from tf2_ros import ExtrapolationException
from tf2_ros import LookupException
from tf2_ros import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf_transformations
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from waywiser_py.waywiser_utils import are_poses_equal
from waywiser_py.waywiser_utils import cleanup_subprocesses
from waywiser_py.waywiser_utils import create_subprocess
from waywiser_py.waywiser_utils import get_full_file_path
from waywiser_py.waywiser_utils import RELIABLE_TRANSIENT_LOCAL_QOS
from waywiser_py.waywiser_utils import send_email

from waywiser_core.msg import MissionState
from waywiser_core.msg import PathWithTwists
from waywiser_test_runner.msg import SetupState
from waywiser_test_runner.msg import TestState
from waywiser_twist_safety.msg import EmergencyStopState

PACKAGE_NAME = 'waywiser_test_runner'


class TrackTestRunner(Node):
    """TrackTestRunner is a ROS2 node that manages the orchestration of track tests."""

    def __init__(self):
        super().__init__('track_test_runner_node')

        # Declare parameters
        self.declare_parameter('test_configurations_json_path', '')
        self.declare_parameter('rosbag_output_dir', '')
        self.declare_parameter('use_rosbag_recording', False)
        self.declare_parameter('static_tf_publishers', [''])
        self.declare_parameter('mission_status_topic', '')
        self.declare_parameter('autopilot_state_control_topic', '/autopilot_state_control')
        self.declare_parameter('preplanned_route_filepath', '')
        self.declare_parameter('vehicle_start_position_is_test_start_point', True)
        self.declare_parameter('has_trailer', False)
        self.declare_parameter('vehicle_pose_topic', '')
        self.declare_parameter('trailer_pose_topic', '')
        self.declare_parameter('track_test_runner_timer_rate', 1.0)
        self.declare_parameter('test_start_point', '')
        self.declare_parameter('emergency_stop_topic', '/emergency_stop/current_state')
        self.declare_parameter('end_goal_alignment_type', 0)
        self.declare_parameter('end_goal_alignment_threshold', 0.1)
        self.declare_parameter('trailer_wheelbase', 1.0)
        self.declare_parameter('length', 1.0)
        self.declare_parameter('width', 0.2)
        self.declare_parameter('trailer_length', 2.0)
        self.declare_parameter('trailer_width', 0.2)
        self.declare_parameter('rear_axle_frame', 'rear_axle_link')
        self.declare_parameter('rear_end_frame', 'rear_end_link')
        self.declare_parameter('hitch_frame', 'fifth_wheel_link')
        self.declare_parameter('trailer_rear_axle_frame', 'rear_axle_link')
        self.declare_parameter('trailer_rear_end_frame', 'rear_end_link')
        self.declare_parameter('trailer_hitch_frame', 'fifth_wheel_link')
        self.declare_parameter('continue_test_after_estop_clear', False)
        self.declare_parameter('enu_ref_topic', '/enu_refernce')
        self.declare_parameter('enuref', [0.0, 0.0, 0.0])
        self.declare_parameter('orchestrate_test_setup', True)
        self.declare_parameter('setup_request_topic', '/setup_request')
        self.declare_parameter('setup_status_topic', '/setup_status')
        self.declare_parameter('staging_area_margin', 0.1)
        self.declare_parameter('test_timeout', 150.0)
        self.declare_parameter('trace_topic', '/trace')
        self.declare_parameter('topics_to_record', [''])
        self.declare_parameter('test_cooldown_time', 0.0)
        self.declare_parameter('test_config_start_index', 1)
        self.declare_parameter('halt_test_runner_on_timeout', False)
        self.declare_parameter('notify_via_mail_on_timeout', False)
        self.declare_parameter('notify_via_mail_on_completion', False)
        self.declare_parameter('delay_to_stop_test_with_emergency_stop', 0.0)
        self.declare_parameter('clear_emergency_stop_on_exec_start', True)

        # Get parameters
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.vehicle_length = self.get_parameter('length').get_parameter_value().double_value
        self.vehicle_width = self.get_parameter('width').get_parameter_value().double_value
        self.rear_axle_frame = (
            self.get_parameter('rear_axle_frame').get_parameter_value().string_value
        )
        self.rear_end_frame = (
            self.get_parameter('rear_end_frame').get_parameter_value().string_value
        )
        self.hitch_frame = self.get_parameter('hitch_frame').get_parameter_value().string_value
        self.has_trailer = self.get_parameter('has_trailer').get_parameter_value().bool_value
        if self.has_trailer:
            self.trailer_length = (
                self.get_parameter('trailer_length').get_parameter_value().double_value
            )
            self.trailer_width = (
                self.get_parameter('trailer_width').get_parameter_value().double_value
            )
            self.trailer_rear_axle_frame = (
                self.get_parameter('trailer_rear_axle_frame').get_parameter_value().string_value
            )
            self.trailer_rear_end_frame = (
                self.get_parameter('trailer_rear_end_frame').get_parameter_value().string_value
            )
            self.trailer_hitch_frame = (
                self.get_parameter('trailer_hitch_frame').get_parameter_value().string_value
            )

        self.test_configurations_json_path = get_full_file_path(
            self.get_parameter('test_configurations_json_path').get_parameter_value().string_value,
            os.path.join(get_package_share_directory(PACKAGE_NAME), 'config'),
        )
        self.rosbag_output_dir = (
            self.get_parameter('rosbag_output_dir').get_parameter_value().string_value
        )
        self.use_rosbag_recording = (
            self.get_parameter('use_rosbag_recording').get_parameter_value().bool_value
        )
        self.emergency_stop_topic = (
            self.get_parameter('emergency_stop_topic').get_parameter_value().string_value
        )
        self.clear_emergency_stop_on_exec_start = (
            self.get_parameter('clear_emergency_stop_on_exec_start')
            .get_parameter_value()
            .bool_value
        )

        static_tf_publishers_ = (
            self.get_parameter('static_tf_publishers').get_parameter_value().string_array_value
        )
        static_tf_publishers_ = [x for x in static_tf_publishers_ if x]
        self.static_tf_publishers = []
        for tf_publisher_ in static_tf_publishers_:
            self.declare_parameter(f'{tf_publisher_}.frame_id', '')
            self.declare_parameter(f'{tf_publisher_}.parent_frame_id', '')
            self.declare_parameter(f'{tf_publisher_}.transform_to_parent', '')
            tf_publisher_info = {}
            tf_publisher_info['frame_id'] = (
                self.get_parameter(f'{tf_publisher_}.frame_id').get_parameter_value().string_value
            )
            tf_publisher_info['parent_frame_id'] = (
                self.get_parameter(f'{tf_publisher_}.parent_frame_id')
                .get_parameter_value()
                .string_value
            )
            tf_publisher_info['transform_to_parent'] = (
                self.get_parameter(f'{tf_publisher_}.transform_to_parent')
                .get_parameter_value()
                .string_value
            )
            if '' not in list(tf_publisher_info.values()):
                self.static_tf_publishers.append(tf_publisher_info)

        if len(self.static_tf_publishers) > 0:
            self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.mission_status_topic = (
            self.get_parameter('mission_status_topic').get_parameter_value().string_value
        )
        self.autopilot_state_control_topic = (
            self.get_parameter('autopilot_state_control_topic').get_parameter_value().string_value
        )
        self.preplanned_route_filepath = get_full_file_path(
            self.get_parameter('preplanned_route_filepath').get_parameter_value().string_value
        )
        self.delay_to_stop_test_with_emergency_stop = (
            self.get_parameter('delay_to_stop_test_with_emergency_stop')
            .get_parameter_value()
            .double_value
        )
        self.get_logger().info(f'Preplanned route file path: {self.preplanned_route_filepath}')

        topics_to_record = (
            self.get_parameter('topics_to_record').get_parameter_value().string_array_value
        )
        self.topics_to_record = []
        for topic in topics_to_record:
            if topic != '':
                self.topics_to_record.append(topic)

        default_test_start_point = (
            self.get_parameter('test_start_point').get_parameter_value().string_value
        )
        if default_test_start_point == '':
            self.default_test_start_point = None
        else:
            self.default_test_start_point = PoseStamped()
            self.default_test_start_point.header.frame_id = 'map'
            default_test_start_point_split = default_test_start_point.split(',')
            if len(default_test_start_point_split) == 6:
                x, y, z, roll, pitch, yaw = map(float, default_test_start_point.split(','))
                orientation = tf_transformations.quaternion_from_euler(
                    math.radians(roll), math.radians(pitch), math.radians(yaw)
                )
                self.default_test_start_point.pose.orientation.x = orientation[0]
                self.default_test_start_point.pose.orientation.y = orientation[1]
                self.default_test_start_point.pose.orientation.z = orientation[2]
                self.default_test_start_point.pose.orientation.w = orientation[3]
            elif len(default_test_start_point_split) == 7:
                x, y, z, orientation_x, orientation_y, orientation_z, orientation_w = map(
                    float, default_test_start_point.split(',')
                )
                self.default_test_start_point.pose.orientation.x = orientation_x
                self.default_test_start_point.pose.orientation.y = orientation_y
                self.default_test_start_point.pose.orientation.z = orientation_z
                self.default_test_start_point.pose.orientation.w = orientation_w
            else:
                self.get_logger().warn(
                    f'Invalid default test start point: {default_test_start_point}'
                )
                x, y, z = 0.0, 0.0, 0.0
            self.default_test_start_point.pose.position.x = x
            self.default_test_start_point.pose.position.y = y
            self.default_test_start_point.pose.position.z = z

        self.vehicle_start_position_is_test_start_point = (
            self.get_parameter('vehicle_start_position_is_test_start_point')
            .get_parameter_value()
            .bool_value
        )
        self.vehicle_pose_topic = (
            self.get_parameter('vehicle_pose_topic').get_parameter_value().string_value
        )
        self.trailer_pose_topic = (
            self.get_parameter('trailer_pose_topic').get_parameter_value().string_value
        )
        self.track_test_runner_timer_rate = (
            self.get_parameter('track_test_runner_timer_rate').get_parameter_value().double_value
        )
        self.end_goal_alignment_type = (
            self.get_parameter('end_goal_alignment_type').get_parameter_value().integer_value
        )
        self.end_goal_alignment_threshold = (
            self.get_parameter('end_goal_alignment_threshold').get_parameter_value().double_value
        )
        self.trailer_wheelbase = (
            self.get_parameter('trailer_wheelbase').get_parameter_value().double_value
        )
        self.continue_test_after_estop_clear = (
            self.get_parameter('continue_test_after_estop_clear').get_parameter_value().bool_value
        )
        self.enu_ref_topic = self.get_parameter('enu_ref_topic').get_parameter_value().string_value
        self.enuref = self.get_parameter('enuref').get_parameter_value().double_array_value
        self.orchestrate_test_setup = (
            self.get_parameter('orchestrate_test_setup').get_parameter_value().bool_value
        )
        self.setup_request_topic = (
            self.get_parameter('setup_request_topic').get_parameter_value().string_value
        )
        self.setup_status_topic = (
            self.get_parameter('setup_status_topic').get_parameter_value().string_value
        )
        self.staging_area_margin = (
            self.get_parameter('staging_area_margin').get_parameter_value().double_value
        )
        self.test_timeout = self.get_parameter('test_timeout').get_parameter_value().double_value
        self.trace_topic = self.get_parameter('trace_topic').get_parameter_value().string_value
        self.test_cooldown_time = (
            self.get_parameter('test_cooldown_time').get_parameter_value().double_value
        )
        self.test_config_start_index = (
            self.get_parameter('test_config_start_index').get_parameter_value().integer_value
        )
        self.halt_test_runner_on_timeout = (
            self.get_parameter('halt_test_runner_on_timeout').get_parameter_value().bool_value
        )
        self.notify_via_mail_on_timeout = (
            self.get_parameter('notify_via_mail_on_timeout').get_parameter_value().bool_value
        )
        self.notify_via_mail_on_completion = (
            self.get_parameter('notify_via_mail_on_completion')
            .get_parameter_value()
            .bool_value
        )

        # Wait for clock to be published if using simulation time
        if self.use_sim_time:
            # if rclpy.ok() and self.get_clock().now().nanoseconds == 0:
            #     self.get_logger().warn('Waiting for sim clock to start...')

            # while rclpy.ok() and self.get_clock().now().nanoseconds == 0:
            #     time.sleep(1.0)
            #     rclpy.spin_once(self)

            # self.get_logger().info('Sim clock started.')

            self.current_sim_time = None

        # Create subscribers
        self.vehicle_pose_subscriber = self.create_subscription(
            PoseStamped,
            self.vehicle_pose_topic,
            self.vehicle_pose_callback,
            10,
        )
        if self.has_trailer:
            self.trailer_pose_subscriber = self.create_subscription(
                PoseStamped,
                self.trailer_pose_topic,
                self.trailer_pose_callback,
                10,
            )
        if self.mission_status_topic:
            self.mission_status_subscriber = self.create_subscription(
                MissionState,
                self.mission_status_topic,
                self.mission_status_callback,
                RELIABLE_TRANSIENT_LOCAL_QOS,
            )
        self.emergency_stop_state_subscriber = self.create_subscription(
            EmergencyStopState,
            self.emergency_stop_topic,
            self.emergency_stop_state_subscriber_callback,
            10,
        )
        self.enu_ref_subscriber = self.create_subscription(
            Vector3, self.enu_ref_topic, self.enu_ref_callback, RELIABLE_TRANSIENT_LOCAL_QOS
        )
        self.setup_status_subscriber = self.create_subscription(
            SetupState,
            self.setup_status_topic,
            self.setup_status_subscriber_callback,
            RELIABLE_TRANSIENT_LOCAL_QOS,
        )
        self.trace_subscriber = self.create_subscription(
            PoseStamped, self.trace_topic, self.trace_subscriber_callback, 10
        )

        # Create publishers
        self.autopilot_state_control_publisher = self.create_publisher(
            Bool, self.autopilot_state_control_topic, RELIABLE_TRANSIENT_LOCAL_QOS
        )
        self.waywiser_path_publisher = self.create_publisher(
            PathWithTwists, '/waywiser_path', RELIABLE_TRANSIENT_LOCAL_QOS
        )
        self.goal_pose_publisher = self.create_publisher(
            PoseStamped, '/goal_pose', RELIABLE_TRANSIENT_LOCAL_QOS
        )
        self.track_test_marker_publisher = self.create_publisher(
            MarkerArray, '/track_test_marker', RELIABLE_TRANSIENT_LOCAL_QOS
        )
        if not self.orchestrate_test_setup:
            self.setup_request_publisher = self.create_publisher(
                String, self.setup_request_topic, RELIABLE_TRANSIENT_LOCAL_QOS
            )
        self.trace_marker_publisher = self.create_publisher(Marker, '/trace_marker', 10)
        self.emergency_stop_publisher = self.create_publisher(
            EmergencyStopState, '/emergency_stop/target_state', RELIABLE_TRANSIENT_LOCAL_QOS
        )

        # Create timers
        self.wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.track_test_runner_wall_timer = self.create_timer(
            1.0 / self.track_test_runner_timer_rate,
            self.track_test_runner_wall_timer_callback,
            clock=self.wall_clock,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize Attributes for controlling orchestrator actions
        self.stop_test_wall_timer = None
        self.artculated_vehicle_length = self.vehicle_length
        self.artculated_vehicle_width = self.vehicle_width
        self.subprocesses = {}
        self.current_config_index = self.test_config_start_index - 1
        self.current_iter_idx = -1
        self.staging_area_pose = None
        self.test_route = None
        self.reverse_test_route = None
        self.test_state_name_lookup = {
            value: name for name, value in TestState.__dict__.items() if isinstance(value, int)
        }
        self.vehicle_pose = None
        self.trailer_pose = None
        self.emergency_stop_current_state = EmergencyStopState.UNKNOWN
        self.vehicle_offset_parameters_initialized = False
        self.transform_warning_logged = False
        self.update_test_state(TestState.WAITING_FOR_VEHICLE_INIT)
        self.test_configurations = self.parse_test_configs()
        self.test_start_wall_time = None
        self.trace_marker_id = 0
        self.staging_area_marker_id = 10
        self.is_test_runner_alive = True

    def trace_subscriber_callback(self, msg):
        match self.test_state:
            case (
                TestState.EXECUTION_INIT
                | TestState.EXECUTION_ONGOING
                | TestState.EXECUTION_COMPLETED
            ):
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.wall_clock.now().to_msg()
                marker.ns = 'trace'
                marker.id = self.trace_marker_id
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = msg.pose.position.x
                marker.pose.position.y = msg.pose.position.y
                marker.pose.position.z = msg.pose.position.z
                marker.pose.orientation.x = msg.pose.orientation.x
                marker.pose.orientation.y = msg.pose.orientation.y
                marker.pose.orientation.z = msg.pose.orientation.z
                marker.pose.orientation.w = msg.pose.orientation.w
                marker.scale.x = self.vehicle_width / 5.0
                marker.scale.y = self.vehicle_width / 5.0
                marker.scale.z = self.vehicle_width / 5.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                self.trace_marker_publisher.publish(marker)
                self.trace_marker_id += 1
            case _:
                pass

    def setup_status_subscriber_callback(self, msg):
        if self.test_state in (
            TestState.CONFIG_LOADED,
            TestState.SETUP_INIT,
            TestState.SETUP_ONGOING,
        ):
            match msg.state:
                case SetupState.SETUP_INIT:
                    self.update_test_state(TestState.SETUP_INIT)
                case SetupState.SETUP_ONGOING:
                    self.update_test_state(TestState.SETUP_ONGOING)
                case SetupState.SETUP_COMPLETED:
                    self.update_test_state(TestState.SETUP_COMPLETED)
                    self.get_logger().info('Test setup completed.')
                    self.vehicle_pose = None
                    self.trailer_pose = None
                case _:
                    # self.get_logger().warn(f'Received unexpected test setup state: {msg.state}')
                    pass

    def update_test_state(self, state: TestState):
        self.test_state = state
        self.get_logger().info(f'Test state: {self.test_state_name_lookup[state]}.')

    def enu_ref_callback(self, enu_ref_msg: Vector3):
        self.enuref = [enu_ref_msg.x, enu_ref_msg.y, enu_ref_msg.z]

    def emergency_stop_state_subscriber_callback(self, msg):
        if self.emergency_stop_current_state != msg.state:
            self.emergency_stop_current_state = msg.state
            if self.emergency_stop_current_state == EmergencyStopState.ACTIVE:
                if (
                    self.test_state == TestState.WAITING_FOR_VEHICLE_INIT
                    or self.test_state == TestState.IDLE
                    or self.test_state == TestState.CONFIG_LOADED
                ):
                    self.get_logger().info('Emergency stop active. Clear it to proceed.')
                if self.test_state == TestState.SETUP_ONGOING:
                    self.update_test_state(TestState.SETUP_STOPPED)
                    self.get_logger().info('Emergency stop active. Halting test setup.')
                elif self.test_state == TestState.EXECUTION_ONGOING:
                    if self.delay_to_stop_test_with_emergency_stop > 0.0:
                        if self.stop_test_wall_timer is None:
                            self.stop_test_wall_timer = self.create_timer(
                                self.delay_to_stop_test_with_emergency_stop,
                                self.end_current_test,
                                clock=self.wall_clock,
                            )
                            self.get_logger().info(
                                'Emergency stop active. Test will be stopped in '
                                f'{self.delay_to_stop_test_with_emergency_stop} s.'
                            )
                    else:
                        self.update_test_state(TestState.EXECUTION_STOPPED)
                        self.get_logger().info('Emergency stop active. Halting test execution.')
            elif self.continue_test_after_estop_clear:
                if self.test_state == TestState.SETUP_STOPPED:
                    self.update_test_state(TestState.CONFIG_LOADED)
                    self.get_logger().info('Emergency stop clear. Resuming test setup.')
                elif self.test_state == TestState.EXECUTION_STOPPED:
                    self.update_test_state(TestState.SETUP_COMPLETED)
                    self.get_logger().info('Emergency stop clear. Resuming test execution.')

    def track_test_runner_wall_timer_callback(self):
        if self.use_sim_time:
            current_sim_time = self.get_clock().now()
            if self.current_sim_time is not None and current_sim_time < self.current_sim_time:
                self.get_logger().warn('Simulation clock is reset. Clearing TF buffer.')
                self.tf_buffer.clear()
                self.vehicle_pose = None
                self.trailer_pose = None
            self.current_sim_time = current_sim_time

        if not self.vehicle_offset_parameters_initialized:
            if not self.load_vehicle_offset_parameters():
                return

        test_total_wall_time = None
        if self.test_start_wall_time is not None:
            test_total_wall_time = self.wall_clock.now() - self.test_start_wall_time
            test_total_wall_time = test_total_wall_time.nanoseconds / 1e9
            if test_total_wall_time > self.test_timeout:
                self.get_logger().warn(
                    f'Test timeout reached. Total time: {test_total_wall_time} s.'
                )
                self.end_current_test()
                if self.halt_test_runner_on_timeout:
                    self.get_logger().info('Halting test runner.')
                    self.track_test_runner_wall_timer.cancel()
                    if self.notify_via_mail_on_timeout:
                        send_email(
                            subject='Waywiser Test Runner Timeout',
                            body=f'Test instance {self.current_config_index + 1}-{self.current_iter_idx + 1} timed out after {test_total_wall_time} s.',
                        )
                    self.is_test_runner_alive = False
                    return

        match self.test_state:
            case TestState.IDLE:
                self.initialize_next_test()
            case TestState.TEST_INIT:
                self.initalize_test_configuration()
            case TestState.CONFIG_LOADED:
                self.setup_test()
            case TestState.SETUP_COMPLETED:
                self.execute_test()
            case TestState.EXECUTION_COMPLETED:
                self.get_logger().info(f'Total test time: {test_total_wall_time} s.')
                self.end_current_test()
            case _:
                pass

    def load_vehicle_offset_parameters(self):
        try:
            log_msgs = []
            transform = self.tf_buffer.lookup_transform(
                self.rear_axle_frame, self.rear_end_frame, rclpy.time.Time()
            )
            self.vehicle_rear_axle_to_rear_end_offset_x = transform.transform.translation.x
            log_msgs.append(
                f'Vehicle rear end offset: {self.vehicle_rear_axle_to_rear_end_offset_x}'
            )

            if self.has_trailer:
                transform = self.tf_buffer.lookup_transform(
                    self.rear_axle_frame, self.hitch_frame, rclpy.time.Time()
                )
                self.vehicle_rear_axle_to_hitch_offset_x = transform.transform.translation.x
                log_msgs.append(
                    f'Vehicle hitch offset: {self.vehicle_rear_axle_to_hitch_offset_x}'
                )

                transform = self.tf_buffer.lookup_transform(
                    self.trailer_rear_axle_frame, self.trailer_rear_end_frame, rclpy.time.Time()
                )
                self.trailer_rear_axle_to_rear_end_offset_x = transform.transform.translation.x
                log_msgs.append(
                    f'Trailer rear end offset: {self.trailer_rear_axle_to_rear_end_offset_x}'
                )

                transform = self.tf_buffer.lookup_transform(
                    self.trailer_rear_axle_frame, self.trailer_hitch_frame, rclpy.time.Time()
                )
                self.trailer_rear_axle_to_hitch_offset_x = transform.transform.translation.x
                log_msgs.append(
                    f'Trailer hitch offset: {self.trailer_rear_axle_to_hitch_offset_x}'
                )

            if self.transform_warning_logged:
                self.get_logger().info('Vehicle offset parameters are initialized now.')
                for log_msg in log_msgs:
                    self.get_logger().info(log_msg)
                self.transform_warning_logged = False
            self.vehicle_offset_parameters_initialized = True

            if self.has_trailer:
                self.artculated_vehicle_length += (
                    self.vehicle_rear_axle_to_rear_end_offset_x
                    - self.vehicle_rear_axle_to_hitch_offset_x
                    + self.trailer_rear_axle_to_hitch_offset_x
                    - self.trailer_rear_axle_to_rear_end_offset_x
                )
                self.artculated_vehicle_width = max(
                    self.artculated_vehicle_width, self.trailer_width
                )

            return True
        except (LookupException, ConnectivityException, ExtrapolationException):
            if not self.transform_warning_logged:
                self.get_logger().warn('Vehicle transforms are not available yet.')
                self.transform_warning_logged = True
            return False

    def vehicle_pose_callback(self, msg):
        """Callback for the vehicle pose subscriber."""
        self.vehicle_pose = msg

    def trailer_pose_callback(self, msg):
        """Callback for the trailer pose subscriber."""
        self.trailer_pose = msg

    def parse_test_configs(self):
        """Parse the raw scenario configurations from the YAML parameter."""
        parsed_configs = []

        # Load scenario_configurations_json_file
        if self.test_configurations_json_path:
            try:
                with open(self.test_configurations_json_path, 'r', encoding='utf-8') as f:
                    track_test_configurations = json.load(f).get('track_test_configurations', [])
                    # Extract spawn points for vehicle.* types
                    for track_test_config in track_test_configurations:
                        test_start_point = self.default_test_start_point
                        if (
                            'test_start_point' in track_test_config
                            and track_test_config['test_start_point'] != ''
                        ):
                            test_start_point = track_test_config['test_start_point']
                            x, y, z, roll, pitch, yaw = map(float, test_start_point.split(','))
                            test_start_point = PoseStamped()
                            test_start_point.header.frame_id = 'map'
                            test_start_point.pose.position.x = x
                            test_start_point.pose.position.y = y
                            test_start_point.pose.position.z = z
                            test_start_point.pose.orientation = (
                                tf_transformations.quaternion_from_euler(
                                    math.radians(roll), math.radians(pitch), math.radians(yaw)
                                )
                            )

                        topics_to_record = self.topics_to_record
                        if 'topics_to_record' in track_test_config:
                            topics_to_record = track_test_config.get('topics_to_record', [])

                        setup_conditions = track_test_config.get('setup_conditions', '')

                        parsed_config = {
                            'iterations': track_test_config.get('iterations', 1),
                            'preplanned_route_filepath': track_test_config.get(
                                'preplanned_route_filepath', self.preplanned_route_filepath
                            ),
                            'vehicle_start_position_is_test_start_point': track_test_config.get(
                                'vehicle_start_position_is_test_start_point',
                                self.vehicle_start_position_is_test_start_point,
                            ),
                            'test_start_point': test_start_point,
                            'setup_conditions': setup_conditions,
                            'topics_to_record': topics_to_record,
                        }
                        parsed_configs.append(parsed_config)
            except Exception as e:
                self.get_logger().error(f'Failed to load objects JSON file: {e}')

        return parsed_configs

    def mission_status_callback(self, msg):
        """Callback for the mission status subscriber."""
        if msg.state in (MissionState.IDLE, MissionState.FOLLOW_ROUTE_FINISHED):
            if self.test_state == TestState.WAITING_FOR_VEHICLE_INIT:
                self.get_logger().info('Vehicle is initialized.')
                self.update_test_state(TestState.IDLE)
            elif self.test_state == TestState.SETUP_ONGOING:
                if self.orchestrate_test_setup:
                    self.update_test_state(TestState.SETUP_COMPLETED)
                    self.get_logger().info('Test setup completed.')
                    self.vehicle_pose = None
                    self.trailer_pose = None
            elif self.test_state == TestState.EXECUTION_ONGOING:
                self.update_test_state(TestState.EXECUTION_COMPLETED)
                self.get_logger().info('Test execution completed.')
        elif msg.state in (
            MissionState.FOLLOW_ROUTE_INIT,
            MissionState.FOLLOW_ROUTE_GOTO_BEGIN,
            MissionState.FOLLOW_ROUTE_FOLLOWING,
            MissionState.FOLLOW_ROUTE_APPROACHING_END_GOAL,
        ):
            if self.test_state == TestState.SETUP_INIT:
                if self.orchestrate_test_setup:
                    self.update_test_state(TestState.SETUP_ONGOING)
                    self.get_logger().info('Test setup ongoing.')
            elif self.test_state == TestState.EXECUTION_INIT:
                self.update_test_state(TestState.EXECUTION_ONGOING)
                self.get_logger().info('Test execution ongoing.')
        elif msg.state == MissionState.WAITING_FOR_VEHICLE_INIT:
            if self.test_state != TestState.SETUP_ONGOING:
                self.update_test_state(TestState.WAITING_FOR_VEHICLE_INIT)
                self.get_logger().info('Vehicle is initializing.')

    def setup_test(self):
        """Start the test."""
        if self.orchestrate_test_setup:
            if self.reverse_test_route is not None:
                if self.vehicle_pose is None or (self.has_trailer and self.trailer_pose is None):
                    self.get_logger().info('No vehicle pose received yet.')
                    return

                vehicle_pose: Union[PoseStamped, None] = self.vehicle_pose
                if self.has_trailer and self.reverse_test_route.twists[0].linear.x < 0.0:
                    vehicle_pose: Union[PoseStamped, None] = self.trailer_pose
                if are_poses_equal(
                    vehicle_pose, self.staging_area_pose, tol=self.end_goal_alignment_threshold
                ):
                    self.update_test_state(TestState.SETUP_COMPLETED)
                    self.get_logger().info('Test setup completed.')
                    return
                else:
                    self.waywiser_path_publisher.publish(self.reverse_test_route)
            else:
                self.goal_pose_publisher.publish(
                    self.staging_area_pose
                )  # nav2 will do the path planning

            msg = Bool()
            msg.data = True
            self.autopilot_state_control_publisher.publish(msg)
            self.update_test_state(TestState.SETUP_INIT)
            self.get_logger().info('Test setup is initialized.')
            self.test_start_wall_time = self.wall_clock.now()
        else:
            setup_request_msg = String()
            setup_request_msg.data = json.dumps(
                self.get_current_test_configuration().get('setup_conditions', '')
            )
            self.setup_request_publisher.publish(setup_request_msg)
            self.get_logger().info(f'Test setup request: {setup_request_msg.data} published.')
            self.test_start_wall_time = self.wall_clock.now()

    def create_bounding_box_marker(
        self,
        bounding_box_center_pose: PoseStamped,
        marker_id: int,
        color: tuple,
        length: float,
        width: float,
        thickness: float = 0.1,
    ) -> Marker:
        # --- Extract pose values ---
        quat = [
            bounding_box_center_pose.pose.orientation.x,
            bounding_box_center_pose.pose.orientation.y,
            bounding_box_center_pose.pose.orientation.z,
            bounding_box_center_pose.pose.orientation.w,
        ]

        cx = bounding_box_center_pose.pose.position.x
        cy = bounding_box_center_pose.pose.position.y
        cz = bounding_box_center_pose.pose.position.z

        # --- Create a rectangle polygon in 2D using shapely ---
        rect = shapely_box(-length / 2, -width / 2, length / 2, width / 2)

        # Convert quaternion to yaw (rotation in Z)
        _, _, yaw = tf_transformations.euler_from_quaternion(quat)

        # Rotate and translate rectangle
        rect = affinity.rotate(rect, np.degrees(yaw), origin=(0, 0))
        rect = affinity.translate(rect, xoff=cx, yoff=cy)

        # --- Convert exterior coordinates into ROS Line List points ---
        coords = list(rect.exterior.coords)
        line_points = []
        for i in range(len(coords) - 1):  # skip last point (duplicate)
            p1 = Point(x=coords[i][0], y=coords[i][1], z=cz)
            p2 = Point(x=coords[i + 1][0], y=coords[i + 1][1], z=cz)
            line_points.append(p1)
            line_points.append(p2)

        # --- Create Marker ---
        marker = Marker()
        marker.header.frame_id = bounding_box_center_pose.header.frame_id
        marker.header.stamp = self.wall_clock.now().to_msg()
        marker.ns = 'bounding_box'
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose = Pose()  # identity, points are already in world coords
        marker.scale.x = thickness
        marker.points = line_points
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        return marker

    def publish_staging_area(
        self,
        color=(1.0, 1.0, 0.0, 1.0),  # Yellow
        marker_id=0,
    ):
        if self.staging_area_pose is None:
            return
        bounding_box_marker = self.create_bounding_box_marker(
            bounding_box_center_pose=self.staging_area_pose,
            marker_id=marker_id,
            color=color,
            length=self.artculated_vehicle_length + 2 * self.staging_area_margin,
            width=self.artculated_vehicle_width + 2 * self.staging_area_margin,
            thickness=self.artculated_vehicle_width / 20.0,
        )

        marker_array = MarkerArray()
        marker_array.markers.append(bounding_box_marker)
        self.track_test_marker_publisher.publish(marker_array)
        self.get_logger().info(f'Published bounding box marker with ID {marker_id}.')

    def publish_simple_vehicle_footprint(
        self,
        reference_pose=None,
        margin_across_length=0.0,
        margin_across_width=0.0,
        yaw_offset=0.0,
        color=(1.0, 1.0, 0.0, 1.0),  # Yellow
        marker_id=0,
        footprint_length=1.0,
        footprint_width=1.0,
        reference_pose_to_rear_end_offset_x=0.0,
    ):
        """
        Compute composite bounding dimensions for the articulated vehicle,
        then publish a hollow (wireframe) bounding box marker.
        """
        if reference_pose is None:
            return

        marker_array = MarkerArray()
        reference_pose_to_composite_bounding_box_center_offset_x = 0.0

        q = [
            reference_pose.pose.orientation.x,
            reference_pose.pose.orientation.y,
            reference_pose.pose.orientation.z,
            reference_pose.pose.orientation.w,
        ]
        bounding_box_yaw = tf_transformations.euler_from_quaternion(q)[2] + yaw_offset

        reference_pose_to_composite_bounding_box_center_offset_x = (
            footprint_length / 2.0 + reference_pose_to_rear_end_offset_x
        )

        bounding_box_center_pose = deepcopy(reference_pose)
        bounding_box_center_pose.pose.position.x += (
            reference_pose_to_composite_bounding_box_center_offset_x * math.cos(bounding_box_yaw)
        )
        bounding_box_center_pose.pose.position.y += (
            reference_pose_to_composite_bounding_box_center_offset_x * math.sin(bounding_box_yaw)
        )

        bounding_box_marker = self.create_bounding_box_marker(
            bounding_box_center_pose=bounding_box_center_pose,
            marker_id=marker_id,
            color=color,
            length=footprint_length + 2 * margin_across_length,
            width=footprint_width + 2 * margin_across_width,
            thickness=footprint_width / 20.0,
        )

        marker_array.markers.append(bounding_box_marker)
        self.track_test_marker_publisher.publish(marker_array)

    def _visualize_path(self, waywiser_path: PathWithTwists, xml_filepath: str):
        """Visualize the path in a non-blocking matplotlib window."""

        # Extract coordinates from the path
        x_coords = [pose.pose.position.x for pose in waywiser_path.path.poses]
        y_coords = [pose.pose.position.y for pose in waywiser_path.path.poses]
        speeds = [twist.linear.x for twist in waywiser_path.twists]

        # Create figure and axis
        fig, ax = plt.subplots(figsize=(10, 8))

        # Plot the path
        ax.plot(x_coords, y_coords, 'b-', linewidth=2, label='Path')
        ax.plot(x_coords, y_coords, 'bo', markersize=3, alpha=0.7)

        # Mark start and end points
        if len(x_coords) > 0:
            ax.plot(x_coords[0], y_coords[0], 'go', markersize=10, label='Start')
            ax.plot(x_coords[-1], y_coords[-1], 'ro', markersize=10, label='End')

            # Annotate start and end
            ax.annotate(
                'Start',
                (x_coords[0], y_coords[0]),
                xytext=(10, 10),
                textcoords='offset points',
                color='green',
                fontweight='bold',
                fontsize=12,
            )
            ax.annotate(
                'End',
                (x_coords[-1], y_coords[-1]),
                xytext=(10, 10),
                textcoords='offset points',
                color='red',
                fontweight='bold',
                fontsize=12,
            )

        # Add arrows to show direction (every 10th point)
        arrow_frequency = max(1, len(x_coords) // 10)
        for i in range(0, len(x_coords) - 1, arrow_frequency):
            ax.annotate(
                '',
                xy=(x_coords[i + 1], y_coords[i + 1]),
                xytext=(x_coords[i], y_coords[i]),
                arrowprops=dict(arrowstyle='->', color='blue', lw=1.5),
            )

        # Set title and labels
        filename = os.path.basename(xml_filepath)
        ax.set_title(f'Route Visualization: {filename}\n{len(x_coords)} waypoints')
        ax.set_xlabel('X coordinate (m)')
        ax.set_ylabel('Y coordinate (m)')
        ax.grid(True, linestyle='--', alpha=0.7)
        ax.axis('equal')  # Equal aspect ratio
        ax.legend()

        # Add speed info if available
        if speeds:
            avg_speed = sum(speeds) / len(speeds)
            ax.text(
                0.02,
                0.98,
                f'Avg speed: {avg_speed:.2f} m/s',
                transform=ax.transAxes,
                verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
            )

        # Make layout tight
        plt.tight_layout()

        # Show the plot in non-blocking mode
        plt.show(block=False)
        plt.draw()

        # Optional: pause briefly to ensure the window appears
        plt.pause(0.5)

    def get_path_with_twists_from_xml(
        self,
        xml_filepath: str,
        refernse_pose_to_align_start_point=None,
        refernse_pose_to_align_end_point=None,
        show_path=False,
    ) -> PathWithTwists:
        if not os.path.exists(xml_filepath):
            self.get_logger().info(f'Could not open file: {xml_filepath}')
            return None

        waywiser_path = PathWithTwists()
        waywiser_path.path = Path()
        waywiser_path.path.header.frame_id = 'map'
        waywiser_path.path.header.stamp = self.get_clock().now().to_msg()
        waywiser_path.path.poses = []
        waywiser_path.twists = []

        try:
            tree = ET.parse(xml_filepath)
            root = tree.getroot()

            if root.tag == 'routes':
                use_vehilce_enuref_to_import_points = True
                imported_enu_ref = {'latitude': 0.0, 'longitude': 0.0, 'height': 0.0}

                # Read the enuref values if available.
                for enuref in root.findall('enuref'):
                    lat = enuref.find('Latitude')
                    lon = enuref.find('Longitude')
                    hgt = enuref.find('Height')

                    if lat is not None:
                        imported_enu_ref['latitude'] = float(lat.text)
                        use_vehilce_enuref_to_import_points = False
                    if lon is not None:
                        imported_enu_ref['longitude'] = float(lon.text)
                    if hgt is not None:
                        imported_enu_ref['height'] = float(hgt.text)

                if use_vehilce_enuref_to_import_points:
                    imported_enu_ref['latitude'] = self.enuref[0]
                    imported_enu_ref['longitude'] = self.enuref[1]
                    imported_enu_ref['height'] = self.enuref[2]

                # Process each route and point to create PoseStamped messages.
                for route in root.findall('route'):
                    for point in route.findall('point'):
                        pose_stamped = PoseStamped()
                        twist = Twist()
                        # Set header stamp and frame id.
                        pose_stamped.header.stamp = self.get_clock().now().to_msg()
                        pose_stamped.header.frame_id = 'map'

                        # Read x, y, z values.
                        x_elem = point.find('x')
                        y_elem = point.find('y')
                        z_elem = point.find('z')
                        speed = point.find('speed')

                        x_val = float(x_elem.text) if x_elem is not None else 0.0
                        y_val = float(y_elem.text) if y_elem is not None else 0.0
                        z_val = float(z_elem.text) if z_elem is not None else 0.0
                        speed_val = float(speed.text) if speed is not None else 0.0

                        # print(f'Before: x_val: {x_val}, y_val: {y_val}, z_val: {z_val}')

                        importedAbsPoint_lat, importedAbsPoint_lon, importedAbsPoint_h = (
                            pm.enu2geodetic(
                                x_val,
                                y_val,
                                z_val,
                                imported_enu_ref['latitude'],
                                imported_enu_ref['longitude'],
                                imported_enu_ref['height'],
                            )
                        )
                        x_val, y_val, z_val = pm.geodetic2enu(
                            importedAbsPoint_lat,
                            importedAbsPoint_lon,
                            importedAbsPoint_h,
                            self.enuref[0],
                            self.enuref[1],
                            self.enuref[2],
                        )
                        # print(
                        #     f'After: x_val: {x_val}, y_val: {y_val}, z_val: {z_val}, speed: {speed_val}'
                        # )

                        # Set the computed position in the PoseStamped.
                        pose_stamped.pose.position.x = x_val
                        pose_stamped.pose.position.y = y_val
                        pose_stamped.pose.position.z = z_val
                        twist.linear.x = speed_val

                        waywiser_path.path.poses.append(pose_stamped)
                        waywiser_path.twists.append(twist)

                num_poses = len(waywiser_path.path.poses)

                # Alignment: shift and rotate the route if requested.
                if refernse_pose_to_align_start_point is not None:
                    # Align so that the first point becomes the current vehicle pose.
                    original_start = waywiser_path.path.poses[0].pose.position
                    reference_pose_to_align: PoseStamped = refernse_pose_to_align_start_point[
                        'forward'
                    ]
                    if (
                        self.has_trailer
                        and waywiser_path.twists[0].linear.x < 0.0
                        and refernse_pose_to_align_start_point.get('reverse', None) is not None
                    ):
                        reference_pose_to_align: PoseStamped = refernse_pose_to_align_start_point[
                            'reverse'
                        ]

                    translation_x = reference_pose_to_align.pose.position.x - original_start.x
                    translation_y = reference_pose_to_align.pose.position.y - original_start.y

                    # Get the vehicle’s current yaw.
                    reference_yaw = tf_transformations.euler_from_quaternion(
                        [
                            reference_pose_to_align.pose.orientation.x,
                            reference_pose_to_align.pose.orientation.y,
                            reference_pose_to_align.pose.orientation.z,
                            reference_pose_to_align.pose.orientation.w,
                        ]
                    )[2]
                    # Compute desired yaw from the first two points.
                    if num_poses >= 2:
                        first_point = waywiser_path.path.poses[0].pose.position
                        second_point = waywiser_path.path.poses[1].pose.position
                        desired_yaw = math.atan2(
                            second_point.y - first_point.y, second_point.x - first_point.x
                        )
                    else:
                        desired_yaw = reference_yaw

                    if waywiser_path.twists[0].linear.x < 0.0:
                        desired_yaw += math.pi

                    # Rotation offset so that the vehicle’s yaw aligns with the route direction.
                    rotation_offset = reference_yaw - desired_yaw

                    # Transform all poses relative to the original start.
                    for pose in waywiser_path.path.poses:
                        new_x = pose.pose.position.x * math.cos(
                            rotation_offset
                        ) - pose.pose.position.y * math.sin(rotation_offset)
                        new_y = pose.pose.position.x * math.sin(
                            rotation_offset
                        ) + pose.pose.position.y * math.cos(rotation_offset)

                        pose.pose.position.x = new_x + translation_x
                        pose.pose.position.y = new_y + translation_y

                elif refernse_pose_to_align_end_point is not None:
                    # Align so that the last point becomes the current vehicle pose.
                    original_end = waywiser_path.path.poses[-1].pose.position
                    reference_pose_to_align: PoseStamped = refernse_pose_to_align_end_point[
                        'forward'
                    ]
                    if (
                        self.has_trailer
                        and waywiser_path.twists[-1].linear.x < 0.0
                        and refernse_pose_to_align_end_point.get('reverse', None) is not None
                    ):
                        reference_pose_to_align: PoseStamped = refernse_pose_to_align_end_point[
                            'reverse'
                        ]

                    reference_yaw = tf_transformations.euler_from_quaternion(
                        [
                            reference_pose_to_align.pose.orientation.x,
                            reference_pose_to_align.pose.orientation.y,
                            reference_pose_to_align.pose.orientation.z,
                            reference_pose_to_align.pose.orientation.w,
                        ]
                    )[2]
                    # Compute desired yaw from the last two points.
                    if num_poses >= 2:
                        last_point = waywiser_path.path.poses[-1].pose.position
                        second_last_point = waywiser_path.path.poses[-2].pose.position
                        desired_yaw = math.atan2(
                            last_point.y - second_last_point.y, last_point.x - second_last_point.x
                        )
                    else:
                        desired_yaw = reference_yaw

                    if waywiser_path.twists[-1].linear.x < 0.0:
                        desired_yaw += math.pi

                    rotation_offset = reference_yaw - desired_yaw

                    # Transform all poses relative to the original end.
                    for pose in waywiser_path.path.poses:
                        dx = pose.pose.position.x - original_end.x
                        dy = pose.pose.position.y - original_end.y
                        dz = pose.pose.position.z - original_end.z

                        new_x = (
                            reference_pose_to_align.pose.position.x
                            + dx * math.cos(rotation_offset)
                            - dy * math.sin(rotation_offset)
                        )
                        new_y = (
                            reference_pose_to_align.pose.position.y
                            + dx * math.sin(rotation_offset)
                            + dy * math.cos(rotation_offset)
                        )
                        new_z = reference_pose_to_align.pose.position.z + dz

                        pose.pose.position.x = new_x
                        pose.pose.position.y = new_y
                        pose.pose.position.z = new_z

                if num_poses > 2:
                    for i in range(1, num_poses - 1):
                        prev_position = waywiser_path.path.poses[i - 1].pose.position
                        next_position = waywiser_path.path.poses[i + 1].pose.position

                        # Calculate the vector from the previous point to the next point.
                        dx = next_position.x - prev_position.x
                        dy = next_position.y - prev_position.y

                        # Compute the yaw (heading) from this vector.
                        yaw = math.atan2(dy, dx)
                        if waywiser_path.twists[i].linear.x > 0.0:
                            yaw += math.pi

                        # Create a quaternion from the yaw angle.
                        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
                        waywiser_path.path.poses[i].pose.orientation.x = q[0]
                        waywiser_path.path.poses[i].pose.orientation.y = q[1]
                        waywiser_path.path.poses[i].pose.orientation.z = q[2]
                        waywiser_path.path.poses[i].pose.orientation.w = q[3]
                    waywiser_path.path.poses[0].pose.orientation = waywiser_path.path.poses[
                        1
                    ].pose.orientation
                    waywiser_path.path.poses[
                        num_poses - 1
                    ].pose.orientation = waywiser_path.path.poses[num_poses - 2].pose.orientation

                if show_path and num_poses > 0:
                    self._visualize_path(waywiser_path, xml_filepath)

                self.get_logger().info(f'Imported {num_poses} waypoints.')

        except Exception as e:
            self.get_logger().error(f'Error reading XML file {xml_filepath}: {e}')

        return waywiser_path

    def compute_reverse_route(self):
        # Reverse the route and invert twist directions
        self.reverse_test_route: PathWithTwists = deepcopy(self.test_route)
        self.reverse_test_route.path.poses.reverse()
        self.reverse_test_route.twists.reverse()
        for twist in self.reverse_test_route.twists:
            twist.linear.x = -twist.linear.x

        num_poses = len(self.reverse_test_route.path.poses)
        if num_poses > 2:
            for i in range(1, num_poses - 1):
                prev_position = self.reverse_test_route.path.poses[i - 1].pose.position
                next_position = self.reverse_test_route.path.poses[i + 1].pose.position

                # Calculate the vector from the previous point to the next point.
                dx = next_position.x - prev_position.x
                dy = next_position.y - prev_position.y

                # Compute the yaw (heading) from this vector.
                yaw = math.atan2(dy, dx)
                if self.reverse_test_route.twists[i].linear.x < 0.0:
                    yaw += math.pi

                # Create a quaternion from the yaw angle.
                q = tf_transformations.quaternion_from_euler(0, 0, yaw)
                self.reverse_test_route.path.poses[i].pose.orientation.x = q[0]
                self.reverse_test_route.path.poses[i].pose.orientation.y = q[1]
                self.reverse_test_route.path.poses[i].pose.orientation.z = q[2]
                self.reverse_test_route.path.poses[i].pose.orientation.w = q[3]
            self.reverse_test_route.path.poses[
                0
            ].pose.orientation = self.reverse_test_route.path.poses[1].pose.orientation
            self.reverse_test_route.path.poses[
                num_poses - 1
            ].pose.orientation = self.reverse_test_route.path.poses[num_poses - 2].pose.orientation

        # If the vehicle has a trailer, extend the path with one extra point.
        if self.has_trailer:
            if len(self.reverse_test_route.path.poses) >= 2:
                # Get the last two points in the reversed path.
                last_pose = self.reverse_test_route.path.poses[-1]
                second_last_pose = self.reverse_test_route.path.poses[-2]

                # Calculate the vector from the second last to the last point.
                dx = last_pose.pose.position.x - second_last_pose.pose.position.x
                dy = last_pose.pose.position.y - second_last_pose.pose.position.y
                segment_length = math.sqrt(dx**2 + dy**2)

                if segment_length > 0:
                    # Compute the normalized direction.
                    dx_norm = dx / segment_length
                    dy_norm = dy / segment_length

                    # Create a new PoseStamped for the extension.
                    new_pose = PoseStamped()
                    new_pose.header.stamp = self.get_clock().now().to_msg()
                    new_pose.header.frame_id = 'map'
                    new_pose.pose.position.x = (
                        last_pose.pose.position.x + self.trailer_wheelbase * dx_norm
                    )
                    new_pose.pose.position.y = (
                        last_pose.pose.position.y + self.trailer_wheelbase * dy_norm
                    )
                    new_pose.pose.position.z = last_pose.pose.position.z
                    new_pose.pose.orientation = last_pose.pose.orientation

                    # Append the new pose.
                    self.reverse_test_route.path.poses.append(new_pose)

                    # Also extend the twist list with a copy of the last twist.
                    new_twist = deepcopy(self.reverse_test_route.twists[-1])
                    self.reverse_test_route.twists.append(new_twist)
                else:
                    self.get_logger().warn('Cannot extend path: the last segment length is zero.')
            else:
                self.get_logger().warn(
                    'Not enough points in the path to extend with trailer offset.'
                )

    def start_rosbag_recording(self, topics_to_record):
        """Start the rosbag_recording node."""
        command = ['ros2', 'bag', 'record']
        if self.use_sim_time:
            command += ['--use-sim-time']

        if self.rosbag_output_dir:
            output_dir = self.rosbag_output_dir + '/' + str(int(time.time()))
            if not os.path.exists(self.rosbag_output_dir):
                os.makedirs(self.rosbag_output_dir)
            command += ['--output', output_dir]
        command += topics_to_record

        subprocess_name = 'ros_bag_recorder'
        self.subprocesses[subprocess_name] = create_subprocess(self, command, subprocess_name)

    def get_current_test_configuration(self):
        return self.test_configurations[self.current_config_index]

    def end_current_test(self):
        if self.stop_test_wall_timer is not None:
            self.stop_test_wall_timer.cancel()
            self.stop_test_wall_timer = None

        self.get_logger().info(
            f'Ending test with index [{self.current_config_index + 1}-{self.current_iter_idx + 1}].'
        )
        cleanup_subprocesses(self.subprocesses)

        self.test_start_wall_time = None
        self.update_test_state(TestState.IDLE)
        time.sleep(self.test_cooldown_time)

    def initialize_next_test(self):
        """Load the next test configuration and start the test."""
        self.current_iter_idx += 1
        if self.current_iter_idx >= self.get_current_test_configuration().get('iterations', 1):
            self.current_config_index += 1
            self.current_iter_idx = 0

            self.staging_area_pose = None
            self.test_route = None
            self.reverse_test_route = None
            self.trace_marker_id = 0
            self.get_logger().info('Resetting test configuration.')

        if self.current_config_index >= len(self.test_configurations):
            self.get_logger().info('All tests are completed.')
            self.track_test_runner_wall_timer.cancel()
            # self.destroy_node()
            if self.notify_via_mail_on_completion:
                send_email(
                    subject='Waywiser Test Runner Completion',
                    body='All test cases are completed.',
                )
            return

        self.get_logger().info(
            f'Starting test with index [{self.current_config_index + 1}-{self.current_iter_idx + 1}].'
        )
        self.publish_static_tfs()
        self.update_test_state(TestState.TEST_INIT)

    def initalize_test_configuration(self):
        test_config = self.get_current_test_configuration()

        reference_pose_to_align_start_point = None
        if self.current_iter_idx == 0:
            self.staging_area_pose = test_config['test_start_point']

            if (test_config['vehicle_start_position_is_test_start_point']) and (
                self.vehicle_pose is None or (self.has_trailer and self.trailer_pose is None)
            ):
                self.get_logger().info('No vehicle pose received yet.')
                return

            if self.staging_area_pose is not None:
                reference_pose_to_align_start_point = (
                    self.get_vehicle_reference_poses_from_combined_center_pose(
                        self.staging_area_pose
                    )
                )
            elif test_config['vehicle_start_position_is_test_start_point']:
                reference_pose_to_align_start_point = {
                    'forward': self.vehicle_pose,
                    'reverse': self.trailer_pose,
                }
                self.staging_area_pose = self.get_combined_center_pose(self.vehicle_pose)

        if test_config['preplanned_route_filepath'] != '' and self.test_route is None:
            self.test_route: PathWithTwists = self.get_path_with_twists_from_xml(
                test_config['preplanned_route_filepath'],
                refernse_pose_to_align_start_point=reference_pose_to_align_start_point,
            )

            if self.test_route is None:
                return

            self.compute_reverse_route()

        if self.staging_area_pose is None:
            self.get_logger().warn('No test start point is provided. Ending test execution.')
            self.end_current_test()
            return
        else:  # publish staging area
            self.publish_staging_area(marker_id=self.staging_area_marker_id)

        self.update_test_state(TestState.CONFIG_LOADED)

        test_start_point_yaw = math.degrees(
            tf_transformations.euler_from_quaternion(
                [
                    self.staging_area_pose.pose.orientation.x,
                    self.staging_area_pose.pose.orientation.y,
                    self.staging_area_pose.pose.orientation.z,
                    self.staging_area_pose.pose.orientation.w,
                ]
            )[2]
        )
        test_start_point_str = f'{self.staging_area_pose.pose.position.x:0.2f}, {self.staging_area_pose.pose.position.y:0.2f}, {self.staging_area_pose.pose.position.z:0.2f}, 0, 0, {test_start_point_yaw:0.2f}'
        self.get_logger().info(f'Test start point location: {test_start_point_str}')

    def get_vehicle_reference_poses_from_combined_center_pose(self, combined_center_pose):
        reference_poses = {}

        q = [
            combined_center_pose.pose.orientation.x,
            combined_center_pose.pose.orientation.y,
            combined_center_pose.pose.orientation.z,
            combined_center_pose.pose.orientation.w,
        ]
        combined_center_yaw = tf_transformations.euler_from_quaternion(q)[2]

        if not self.has_trailer:
            center_pose_to_vehicle_rear_axle_offset_x = -(
                self.vehicle_length / 2.0 + self.vehicle_rear_axle_to_rear_end_offset_x
            )
        else:
            center_pose_to_vehicle_rear_axle_offset_x = self.artculated_vehicle_length / 2.0 - (
                self.vehicle_length + self.vehicle_rear_axle_to_rear_end_offset_x
            )

            trailer_rear_axle_to_vehicle_rear_axle_offset_x = (
                self.trailer_rear_axle_to_hitch_offset_x - self.vehicle_rear_axle_to_hitch_offset_x
            )
            center_pose_to_trailer_rear_axle_offset_x = (
                center_pose_to_vehicle_rear_axle_offset_x
                - trailer_rear_axle_to_vehicle_rear_axle_offset_x
            )

            reference_poses['reverse'] = deepcopy(combined_center_pose)
            reference_poses['reverse'].pose.position.x += (
                center_pose_to_trailer_rear_axle_offset_x * math.cos(combined_center_yaw)
            )
            reference_poses['reverse'].pose.position.y += (
                center_pose_to_trailer_rear_axle_offset_x * math.sin(combined_center_yaw)
            )

        reference_poses['forward'] = deepcopy(combined_center_pose)
        reference_poses['forward'].pose.position.x += (
            center_pose_to_vehicle_rear_axle_offset_x * math.cos(combined_center_yaw)
        )
        reference_poses['forward'].pose.position.y += (
            center_pose_to_vehicle_rear_axle_offset_x * math.sin(combined_center_yaw)
        )
        return reference_poses

    def get_combined_center_pose(self, vehicle_pose):
        q = [
            vehicle_pose.pose.orientation.x,
            vehicle_pose.pose.orientation.y,
            vehicle_pose.pose.orientation.z,
            vehicle_pose.pose.orientation.w,
        ]
        vehicle_yaw = tf_transformations.euler_from_quaternion(q)[2]

        if not self.has_trailer:
            vehicle_pose_to_combined_center_offset_x = (
                self.vehicle_length / 2.0 + self.vehicle_rear_axle_to_rear_end_offset_x
            )
        else:
            vehicle_pose_to_combined_center_offset_x = self.artculated_vehicle_length / 2.0 - (
                self.vehicle_length + self.vehicle_rear_axle_to_rear_end_offset_x
            )

        combined_center_pose = deepcopy(vehicle_pose)
        combined_center_pose.pose.position.x -= (
            vehicle_pose_to_combined_center_offset_x * math.cos(vehicle_yaw)
        )
        combined_center_pose.pose.position.y -= (
            vehicle_pose_to_combined_center_offset_x * math.sin(vehicle_yaw)
        )
        return combined_center_pose

    def execute_test(self):
        if self.vehicle_pose is None or (self.has_trailer and self.trailer_pose is None):
            return

        if self.clear_emergency_stop_on_exec_start:
            emergency_stop_target_state_msg = EmergencyStopState()
            emergency_stop_target_state_msg.sender_id = 'track_test_runner'
            emergency_stop_target_state_msg.state = EmergencyStopState.CLEAR
            self.emergency_stop_publisher.publish(emergency_stop_target_state_msg)

        test_config = self.get_current_test_configuration()
        self.publish_staging_area(marker_id=self.staging_area_marker_id)
        # publish vehicle footprint
        self.publish_simple_vehicle_footprint(
            reference_pose=self.vehicle_pose,
            color=(0.0, 0.0, 1.0, 0.25),  # Blue
            marker_id=0,
            footprint_length=self.vehicle_length,
            footprint_width=self.vehicle_width,
            reference_pose_to_rear_end_offset_x=self.vehicle_rear_axle_to_rear_end_offset_x,
        )
        if self.has_trailer:
            self.publish_simple_vehicle_footprint(
                reference_pose=self.trailer_pose,
                color=(1.0, 1.0, 1.0, 0.25),  # White
                marker_id=1,
                footprint_length=self.trailer_length,
                footprint_width=self.trailer_width,
                reference_pose_to_rear_end_offset_x=self.trailer_rear_axle_to_rear_end_offset_x,
            )

        # start rosbag recording
        topics_to_record = test_config['topics_to_record']
        if self.use_rosbag_recording and len(topics_to_record) > 0:
            self.start_rosbag_recording(topics_to_record)
            time.sleep(1.0)

        # publish the route to the vehicle (assuming that vehicle is at start point)
        self.waywiser_path_publisher.publish(self.test_route)
        time.sleep(1.0)

        msg = Bool()
        msg.data = True
        self.autopilot_state_control_publisher.publish(msg)

        self.update_test_state(TestState.EXECUTION_INIT)
        self.get_logger().info('Test execution is initialized.')

    def publish_static_tfs(self):
        for static_tf_publisher_info in self.static_tf_publishers:
            x, y, z, roll, pitch, yaw = map(
                float, static_tf_publisher_info['transform_to_parent'].split(',')
            )
            quaternion = tf_transformations.quaternion_from_euler(
                math.radians(roll), math.radians(pitch), math.radians(yaw)
            )

            # Populate the TransformStamped message
            transform = TransformStamped()
            transform.header.frame_id = static_tf_publisher_info['parent_frame_id']
            transform.child_frame_id = static_tf_publisher_info['frame_id']
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z
            transform.transform.rotation.x = quaternion[0]
            transform.transform.rotation.y = quaternion[1]
            transform.transform.rotation.z = quaternion[2]
            transform.transform.rotation.w = quaternion[3]

            # Broadcast the static transform
            self.static_tf_broadcaster.sendTransform(transform)

    def destroy_node(self):
        """Override to ensure the subprocesses are terminated on shutdown."""
        try:
            print('Shutting down track_test_runner node.')
            cleanup_subprocesses(self.subprocesses)
        except Exception as e:
            print(f'Error during node destruction: {e}')
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    # Initialize and run the node
    track_test_runner_node = TrackTestRunner()

    try:
        while track_test_runner_node.is_test_runner_alive:
            rclpy.spin_once(track_test_runner_node)
    except KeyboardInterrupt:
        track_test_runner_node.get_logger().info('User requested shutdown with SIGINT.')
    finally:
        # Cleanup on exit
        try:
            track_test_runner_node.destroy_node()
        except Exception as e:
            print(f'Error during node destruction: {e}')
        # Only shutdown if the context is still valid
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
