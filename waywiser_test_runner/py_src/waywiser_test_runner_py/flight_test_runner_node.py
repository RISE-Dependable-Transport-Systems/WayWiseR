#!/usr/bin/env python3

import json
import math
import os
import signal
import time
from typing import Any

from ament_index_python import get_package_share_directory
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.signals import SignalHandlerOptions
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from waywiser_core.msg import HeartbeatRxState, PathWithTwists, QuadcopterState
from waywiser_py.waywiser_utils import FileUtils, RELIABLE_TRANSIENT_LOCAL_QOS
from waywiser_test_runner.msg import SetupState
from waywiser_test_runner_py.test_runner_base import TestRunnerBase

PACKAGE_NAME = 'waywiser_test_runner'


class FlightTestRunnerNode(TestRunnerBase):

    def __init__(self):
        super().__init__('flight_test_runner')

        self.package_share_dir = get_package_share_directory(PACKAGE_NAME)

        self.declare_parameter('test_configurations_json_path', '')
        self.declare_parameter('test_config_start_index', 1)
        self.declare_parameter('test_config_end_index', 0)
        self.declare_parameter('quadcopter_state_topic', 'quadcopter_state')
        self.declare_parameter('heartbeat_rx_state_topic', 'control_tower_heartbeat_rx_state')
        self.declare_parameter('control_vehicle_node_fqn', 'waywiser_drone_node')
        self.declare_parameter('autopilot_state_control_topic', 'autopilot_state_control')
        self.declare_parameter('route_publish_topic', 'waywiser_path')
        self.declare_parameter('preplanned_route_filepath', '')
        self.declare_parameter('publish_route_after_ready', True)
        self.declare_parameter('use_route_start_as_staging_area', False)
        self.declare_parameter('route_start_staging_height', 0.3)
        self.declare_parameter('route_start_staging_yaw_from_route', False)
        self.declare_parameter('enuref', [0.0, 0.0, 0.0])
        self.declare_parameter('arm_request_retry_period', 1.0)
        self.declare_parameter('ready_state_hold_sec', 3.0)
        self.declare_parameter('trigger_state_hold_sec', 0.5)
        self.declare_parameter('state_stamp_tolerance_sec', 0.1)
        self.declare_parameter('vehicle_parameter_service_timeout_sec', 10.0)
        self.declare_common_test_runner_parameters()

        configurations_json_path = (
            self.get_parameter('test_configurations_json_path').get_parameter_value().string_value
        )
        self.test_configurations_json_path = FileUtils.get_full_file_path(
            configurations_json_path, os.path.join(self.package_share_dir, 'config')
        )
        self.test_config_start_index = max(
            1, self.get_parameter('test_config_start_index').get_parameter_value().integer_value
        )
        self.test_config_end_index = max(
            0, self.get_parameter('test_config_end_index').get_parameter_value().integer_value
        )
        self.initialize_common_test_runner()
        self.configured_staging_area_pose = list(self.staging_area_pose)
        self.quadcopter_state_topic = (
            self.get_parameter('quadcopter_state_topic').get_parameter_value().string_value
        )
        self.heartbeat_rx_state_topic = (
            self.get_parameter('heartbeat_rx_state_topic').get_parameter_value().string_value
        )
        self.control_vehicle_node_fqn = (
            self.get_parameter('control_vehicle_node_fqn').get_parameter_value().string_value
        )
        self.autopilot_state_control_topic = (
            self.get_parameter('autopilot_state_control_topic').get_parameter_value().string_value
        )
        self.route_publish_topic = (
            self.get_parameter('route_publish_topic').get_parameter_value().string_value
        )
        self.preplanned_route_filepath = FileUtils.get_full_file_path(
            self.get_parameter('preplanned_route_filepath').get_parameter_value().string_value
        )
        self.publish_route_after_ready = (
            self.get_parameter('publish_route_after_ready').get_parameter_value().bool_value
        )
        self.use_route_start_as_staging_area = (
            self.get_parameter('use_route_start_as_staging_area').get_parameter_value().bool_value
        )
        self.route_start_staging_height = (
            self.get_parameter('route_start_staging_height').get_parameter_value().double_value
        )
        self.route_start_staging_yaw_from_route = (
            self.get_parameter('route_start_staging_yaw_from_route')
            .get_parameter_value()
            .bool_value
        )
        self.enuref = list(self.get_parameter('enuref').get_parameter_value().double_array_value)
        self.arm_request_retry_period = (
            self.get_parameter('arm_request_retry_period').get_parameter_value().double_value
        )
        self.ready_state_hold_sec = (
            self.get_parameter('ready_state_hold_sec').get_parameter_value().double_value
        )
        self.trigger_state_hold_sec = (
            self.get_parameter('trigger_state_hold_sec').get_parameter_value().double_value
        )
        self.state_stamp_tolerance_sec = (
            self.get_parameter('state_stamp_tolerance_sec').get_parameter_value().double_value
        )
        self.vehicle_parameter_service_timeout_sec = (
            self.get_parameter('vehicle_parameter_service_timeout_sec')
            .get_parameter_value()
            .double_value
        )
        self.quadcopter_state_code_to_name = {
            value: name
            for name, value in QuadcopterState.__dict__.items()
            if name.isupper() and isinstance(value, int)
        }
        self.quadcopter_state_name_to_code = {
            name: value for value, name in self.quadcopter_state_code_to_name.items()
        }
        self.heartbeat_state_code_to_name = {
            value: name
            for name, value in HeartbeatRxState.__dict__.items()
            if name.isupper() and isinstance(value, int)
        }

        self.activation_trigger_conditions: dict[str, dict[str, Any]] = {}
        self.activation_trigger_subscriptions: dict[tuple[str, str], object] = {}
        self.current_quadcopter_state_code: int | None = None
        self.current_quadcopter_state_stamp_s: float | None = None
        self.current_quadcopter_state_received_at: float | None = None
        self.current_quadcopter_state_changed_at: float | None = None
        self.current_heartbeat_rx_state: int | None = None
        self.state_epoch_started_at: float | None = None
        self.current_profile_index = -1
        self.current_profile: dict[str, Any] | None = None
        self.pending_parameter_request: dict[str, Any] | None = None
        self.phase = 'idle'
        self.phase_started_at: float | None = None
        self.profile_started_at: float | None = None
        self.last_arm_request_time = 0.0
        self.last_auto_climb_request_time = 0.0
        self.arm_requested = False
        self.auto_climb_requested = False
        self.route_sent = False
        self.rosbag_started = False
        self.completed = False
        self.route_msg = None
        self.active_perturbation_channels: dict[str, float] = {}
        self.pending_perturbation_channels: dict[str, float] = {}
        self.completed_perturbation_channels: set[str] = set()

        self.autopilot_state_control_publisher = self.create_publisher(
            Bool,
            self.autopilot_state_control_topic,
            RELIABLE_TRANSIENT_LOCAL_QOS,
        )
        self.route_publisher = self.create_publisher(
            PathWithTwists, self.route_publish_topic, RELIABLE_TRANSIENT_LOCAL_QOS
        )
        self.create_subscription(
            QuadcopterState,
            self.quadcopter_state_topic,
            self.quadcopter_state_callback,
            RELIABLE_TRANSIENT_LOCAL_QOS,
        )
        self.create_subscription(
            HeartbeatRxState,
            self.heartbeat_rx_state_topic,
            self.heartbeat_rx_state_callback,
            RELIABLE_TRANSIENT_LOCAL_QOS,
        )
        self.create_timer(1.0 / self.runner_timer_rate, self.runner_timer_callback)

        self.test_profiles = self.parse_test_profiles()
        self.configure_test_profile_index_bounds(len(self.test_profiles))
        self.publish_perturbation_active(False, force=True)

        if not self.test_profiles:
            self.completed = True
            self.get_logger().warn('No flight test profiles were loaded.')
        else:
            self.get_logger().info(f'Loaded {len(self.test_profiles)} flight test run(s).')

    def configure_test_profile_index_bounds(self, profile_count: int):
        # Count unique configs (not expanded runs)
        config_count = 0
        if self.test_profiles:
            config_count = self.test_profiles[-1].get('config_index', 1)

        requested_end_index = self.test_config_end_index
        if requested_end_index > 0 and requested_end_index < self.test_config_start_index:
            message = (
                'test_config_end_index must be greater than or equal to '
                f'test_config_start_index ({requested_end_index} < '
                f'{self.test_config_start_index}).'
            )
            self.get_logger().fatal(message)
            raise ValueError(message)

        if requested_end_index <= 0:
            self.test_config_end_index = config_count
        elif requested_end_index > config_count:
            self.get_logger().warn(
                f'test_config_end_index {requested_end_index} is greater than the loaded '
                f'config count {config_count}; using {config_count}.'
            )
            self.test_config_end_index = config_count

        # Set current_profile_index to the last profile of the config before start
        # so that start_next_profile advances into the first desired config
        self.current_profile_index = -1
        for i, p in enumerate(self.test_profiles):
            if p.get('config_index', 1) >= self.test_config_start_index:
                self.current_profile_index = i - 1
                break
        if self.current_profile_index < 0:
            # All configs are before start_index; nothing to run
            self.current_profile_index = len(self.test_profiles)

    def quadcopter_state_callback(self, msg: QuadcopterState):
        now = self.experiment_now()
        stamp_s = self.stamp_to_sec(msg.stamp)
        if self.state_epoch_started_at is not None and stamp_s is not None:
            if stamp_s + self.state_stamp_tolerance_sec < self.state_epoch_started_at:
                return

        if self.current_quadcopter_state_code != msg.state_code:
            self.current_quadcopter_state_code = msg.state_code
            self.current_quadcopter_state_changed_at = now
            state_name = self.quadcopter_state_code_to_name.get(
                msg.state_code, str(msg.state_code)
            )
            self.get_logger().info(f'Quadcopter state: {state_name}.')
        elif self.current_quadcopter_state_changed_at is None:
            self.current_quadcopter_state_changed_at = now

        self.current_quadcopter_state_stamp_s = stamp_s
        self.current_quadcopter_state_received_at = now

    def heartbeat_rx_state_callback(self, msg: HeartbeatRxState):
        if self.current_heartbeat_rx_state != msg.state:
            self.current_heartbeat_rx_state = msg.state
            state_name = self.heartbeat_state_code_to_name.get(msg.state, str(msg.state))
            self.get_logger().info(f'Heartbeat RX state: {state_name} age={msg.age_s:0.3f} s.')

    def runner_timer_callback(self):
        if self.completed:
            return

        if self.current_profile is None:
            self.start_next_profile()
            return

        wall_now = self.runner_now()
        experiment_now = self.experiment_now()
        self.poll_pending_parameter_request(wall_now)
        if (
            self.profile_started_at is not None
            and (experiment_now - self.profile_started_at) > self.test_timeout
        ):
            self.handle_timeout()
            return

        if self.phase == 'waiting_for_mission_trigger':
            if self.activation_triggers_are_stable(
                self.current_profile['mission_activation_triggers'],
                experiment_now,
                self.trigger_state_hold_sec,
            ):
                self.publish_route_if_configured()
                self.phase = 'waiting_for_perturbation_trigger'
                self.phase_started_at = experiment_now
                self.get_logger().info(
                    'Mission trigger observed. Waiting for topic perturbation trigger.'
                )
                return

            if self.activation_triggers_are_matched(
                self.current_profile['mission_activation_triggers']
            ):
                return

            self.handle_auto_takeoff(wall_now)
            return

        if self.phase == 'waiting_for_perturbation_trigger':
            if self.start_ready_perturbation_channels(experiment_now):
                self.phase = 'perturbation'
                self.phase_started_at = experiment_now
            return

        if self.phase == 'perturbation':
            self.start_ready_perturbation_channels(experiment_now)
            if self.stop_finished_perturbation_channels(experiment_now):
                self.phase = 'cooldown'
                self.phase_started_at = experiment_now
                self.get_logger().info('Flight test topic perturbation window completed.')
            return

        if self.phase == 'cooldown' and self.phase_started_at is not None:
            if (experiment_now - self.phase_started_at) >= self.test_cooldown_time:
                self.start_next_profile()

        if self.phase == 'setup' and self.current_setup_state == SetupState.SETUP_COMPLETED:
            self.begin_profile_execution()

    def parse_test_profiles(self) -> list[dict[str, Any]]:
        raw_configurations = self.load_json_list(
            self.test_configurations_json_path,
            ('flight_test_configurations',),
            'flight test configurations',
        )

        parsed_profiles = []
        for (
            raw_index,
            raw_configuration,
            raw_profile,
            setup_conditions,
            iteration,
            iterations,
        ) in self.expand_test_configuration_runs(raw_configurations):
            config_index = raw_index  # 1-based config index from expand
            if not isinstance(raw_configuration.get('test_profile'), dict):
                self.get_logger().error(
                    f"Skipping configuration '{raw_configuration.get('name', raw_index)}': "
                    "missing required object 'test_profile'"
                )
                continue

            mission_config = raw_profile.get('mission', {})
            perturbation_config = raw_profile.get('topic_perturbation', {})

            if not isinstance(mission_config, dict):
                mission_config = {}
            if not isinstance(perturbation_config, dict):
                perturbation_config = {}
            setup_conditions_config = (
                setup_conditions if isinstance(setup_conditions, dict) else {}
            )

            missing_fields = []
            if 'activation_trigger' not in mission_config:
                missing_fields.append('mission.activation_trigger')
            if not perturbation_config:
                missing_fields.append('topic_perturbation')
            if missing_fields:
                self.get_logger().error(
                    f"Skipping profile '{raw_configuration.get('name', raw_index)}': missing "
                    f'required fields {missing_fields}'
                )
                continue

            mission_activation_triggers = self.parse_activation_trigger_config(
                mission_config['activation_trigger'],
                raw_configuration.get('name', raw_index),
                'mission.activation_trigger',
            )
            if not mission_activation_triggers:
                self.get_logger().error(
                    f"Skipping profile '{raw_configuration.get('name', raw_index)}': "
                    'mission.activation_trigger contains no valid trigger conditions'
                )
                continue

            perturbation_channels = self.parse_topic_perturbation_config(
                perturbation_config,
                raw_configuration.get('name', raw_index),
                self.parse_activation_trigger_config,
            )
            if not perturbation_channels:
                continue

            route_filepath = FileUtils.get_full_file_path(
                mission_config.get(
                    'preplanned_route_filepath',
                    setup_conditions_config.get(
                        'preplanned_route_filepath', self.preplanned_route_filepath
                    ),
                )
            )
            topics_to_record = raw_profile.get('topics_to_record', self.default_topics_to_record)
            topics_to_record = [topic for topic in topics_to_record if topic]

            parsed_profiles.append(
                {
                    'name': raw_configuration.get(
                        'name', raw_profile.get('name', f'flight_test_{raw_index}')
                    ),
                    'config_index': config_index,
                    'iteration': iteration,
                    'iterations': iterations,
                    'mission_activation_triggers': mission_activation_triggers,
                    'route_filepath': route_filepath,
                    'perturbation_channels': perturbation_channels,
                    'topics_to_record': topics_to_record,
                    'setup': setup_conditions,
                }
            )

        return parsed_profiles

    def configure_current_profile_route(self):
        route_filepath = (
            self.current_profile.get('route_filepath', self.preplanned_route_filepath)
            if self.current_profile
            else self.preplanned_route_filepath
        )
        self.route_msg = self.load_waywise_route(route_filepath, self.enuref)
        self.configure_route_start_staging_area()

    def configure_route_start_staging_area(self):
        if not self.use_route_start_as_staging_area:
            self.staging_area_pose = list(self.configured_staging_area_pose)
            return
        if self.route_msg is None or not self.route_msg.path.poses:
            self.get_logger().warn(
                'Route-start staging was requested, but no preplanned route was loaded.'
            )
            return

        start_position = self.route_msg.path.poses[0].pose.position
        yaw = 0.0
        if self.route_start_staging_yaw_from_route and len(self.route_msg.path.poses) > 1:
            next_position = self.route_msg.path.poses[1].pose.position
            yaw = math.atan2(
                next_position.y - start_position.y,
                next_position.x - start_position.x,
            )

        self.staging_area_pose = [
            float(start_position.x),
            float(start_position.y),
            float(self.route_start_staging_height),
            0.0,
            0.0,
            float(yaw),
        ]
        self.get_logger().info(
            'Using route start as staging pose: '
            f'{self.staging_area_pose[0]:0.3f}, {self.staging_area_pose[1]:0.3f}, '
            f'{self.staging_area_pose[2]:0.3f}, 0.000, 0.000, {self.staging_area_pose[5]:0.3f}'
        )

    def publish_route_if_configured(self):
        if not self.publish_route_after_ready or self.route_sent:
            return

        if self.route_msg is None:
            self.get_logger().warn('Route publishing requested, but no route was loaded.')
            self.route_sent = True
            return

        now_msg = self.get_clock().now().to_msg()
        route_msg = self.route_msg
        route_msg.path.header.stamp = now_msg
        for pose in route_msg.path.poses:
            pose.header.stamp = now_msg
        self.route_publisher.publish(route_msg)

        self.publish_autopilot_state(True)

        self.route_sent = True
        self.get_logger().info(
            f'Published route on {self.route_publish_topic} and enabled autopilot.'
        )

    def publish_autopilot_state(self, enabled: bool):
        autopilot_msg = Bool()
        autopilot_msg.data = enabled
        self.autopilot_state_control_publisher.publish(autopilot_msg)

    def parse_trigger_state_codes(self, raw_states: Any) -> set[int]:
        if not isinstance(raw_states, (list, tuple)):
            raw_states = [raw_states]

        parsed_state_codes = set()
        for raw_state in raw_states:
            if isinstance(raw_state, int):
                parsed_state_codes.add(raw_state)
                continue

            if isinstance(raw_state, str):
                normalized_state = raw_state.strip().upper().replace(' ', '_')
                state_code = self.quadcopter_state_name_to_code.get(normalized_state)
                if state_code is None:
                    state_code = getattr(QuadcopterState, normalized_state, None)
                if isinstance(state_code, int):
                    parsed_state_codes.add(state_code)

        return parsed_state_codes

    def parse_activation_trigger_config(
        self, raw_trigger: Any, profile_name: str, field_name: str
    ) -> list[str]:
        trigger_specs = raw_trigger
        if not isinstance(raw_trigger, (list, tuple)):
            trigger_specs = [raw_trigger]

        if all(not isinstance(item, dict) for item in trigger_specs):
            trigger_specs = [
                {
                    'topic': self.quadcopter_state_topic,
                    'message_type': 'waywiser_core/msg/QuadcopterState',
                    'field': 'state_code',
                    'value': list(trigger_specs),
                }
            ]

        trigger_keys = []
        for trigger_index, trigger_spec in enumerate(trigger_specs, start=1):
            trigger_key = self.register_activation_trigger_condition(
                trigger_spec,
                profile_name,
                f'{field_name}[{trigger_index}]' if len(trigger_specs) > 1 else field_name,
            )
            if trigger_key:
                trigger_keys.append(trigger_key)

        return trigger_keys

    def register_activation_trigger_condition(
        self, trigger_spec: dict[str, Any], profile_name: str, field_name: str
    ) -> str | None:
        if not isinstance(trigger_spec, dict):
            self.get_logger().error(
                f"Skipping profile '{profile_name}': {field_name} must be a string, "
                'list of strings, or trigger object'
            )
            return None

        topic = str(trigger_spec.get('topic', '')).strip()
        message_type_name = str(trigger_spec.get('message_type', '')).strip()
        field_path = str(trigger_spec.get('field', 'data')).strip() or 'data'
        if 'value' in trigger_spec:
            raw_values = trigger_spec['value']
        else:
            raw_values = trigger_spec.get('values')

        missing_fields = []
        if not topic:
            missing_fields.append('topic')
        if not message_type_name:
            missing_fields.append('message_type')
        if raw_values is None:
            missing_fields.append('value')
        if missing_fields:
            self.get_logger().error(
                f"Skipping profile '{profile_name}': {field_name} missing required "
                f'field(s) {missing_fields}'
            )
            return None

        try:
            message_type = get_message(message_type_name)
        except (AttributeError, ModuleNotFoundError, ValueError) as exc:
            self.get_logger().error(
                f"Skipping profile '{profile_name}': {field_name} could not load "
                f"message type '{message_type_name}': {exc}"
            )
            return None

        expected_values = self.normalize_trigger_expected_values(raw_values, message_type)
        if not expected_values:
            self.get_logger().error(
                f"Skipping profile '{profile_name}': {field_name} contains no trigger values"
            )
            return None

        key = json.dumps(
            {
                'topic': topic,
                'message_type': message_type_name,
                'field': field_path,
                'values': expected_values,
            },
            sort_keys=True,
            default=str,
        )
        if key not in self.activation_trigger_conditions:
            self.activation_trigger_conditions[key] = {
                'topic': topic,
                'message_type_name': message_type_name,
                'message_type': message_type,
                'field': field_path,
                'expected_values': expected_values,
                'matched': False,
                'changed_at': None,
                'stamp_s': None,
                'received_at': None,
                'description': f'{topic} {field_path} in {expected_values}',
            }

        subscription_key = (topic, message_type_name)
        if subscription_key not in self.activation_trigger_subscriptions:
            self.activation_trigger_subscriptions[subscription_key] = self.create_subscription(
                message_type,
                topic,
                lambda msg, msg_topic=topic, msg_type=message_type_name: (
                    self.activation_trigger_callback(msg_topic, msg_type, msg)
                ),
                10,
            )
            self.get_logger().info(
                f'Listening for activation triggers on {topic} ({message_type_name}).'
            )

        return key

    @staticmethod
    def normalize_trigger_expected_values(raw_values: Any, message_type) -> list[Any]:
        if not isinstance(raw_values, (list, tuple, set)):
            raw_values = [raw_values]

        expected_values = []
        for raw_value in raw_values:
            expected_value = raw_value
            if isinstance(raw_value, str):
                normalized_name = raw_value.strip().upper().replace(' ', '_')
                constant_value = getattr(message_type, normalized_name, None)
                expected_value = constant_value if constant_value is not None else raw_value
            if expected_value not in expected_values:
                expected_values.append(expected_value)

        return expected_values

    def activation_trigger_callback(self, topic: str, message_type_name: str, msg):
        now = self.experiment_now()
        stamp_s = self.extract_message_stamp_s(msg)
        for condition in self.activation_trigger_conditions.values():
            if condition['topic'] != topic or condition['message_type_name'] != message_type_name:
                continue

            try:
                field_value = self.extract_message_field(msg, condition['field'])
            except AttributeError:
                self.get_logger().warn(
                    f"Activation trigger field '{condition['field']}' is not present on "
                    f'{message_type_name}.'
                )
                continue

            matched = any(
                self.trigger_values_equal(field_value, expected_value)
                for expected_value in condition['expected_values']
            )
            if condition['matched'] != matched:
                condition['matched'] = matched
                condition['changed_at'] = now
                self.get_logger().info(
                    f'Activation trigger {condition["description"]} is '
                    f'{"matched" if matched else "not matched"}.'
                )
            elif condition['changed_at'] is None:
                condition['changed_at'] = now

            condition['stamp_s'] = stamp_s
            condition['received_at'] = now

    @staticmethod
    def extract_message_field(msg, field_path: str) -> Any:
        value = msg
        for field_name in field_path.split('.'):
            value = getattr(value, field_name)
        return value

    def extract_message_stamp_s(self, msg) -> float | None:
        stamp = getattr(msg, 'stamp', None)
        if stamp is None:
            header = getattr(msg, 'header', None)
            stamp = getattr(header, 'stamp', None) if header is not None else None
        return self.stamp_to_sec(stamp) if stamp is not None else None

    @staticmethod
    def trigger_values_equal(actual_value: Any, expected_value: Any) -> bool:
        if isinstance(actual_value, bool) or isinstance(expected_value, bool):
            if isinstance(actual_value, str):
                actual_value = actual_value.strip().lower() in ('true', '1', 'yes', 'on')
            if isinstance(expected_value, str):
                expected_value = expected_value.strip().lower() in ('true', '1', 'yes', 'on')
            return bool(actual_value) == bool(expected_value)
        if isinstance(actual_value, (int, float)) and isinstance(expected_value, (int, float)):
            return actual_value == expected_value
        return str(actual_value) == str(expected_value)

    def start_next_profile(self, advance: bool = True):
        self.clear_pending_parameter_request()
        self.stop_rosbag_recording()
        self.publish_perturbation_active(False, force=True)
        self.publish_autopilot_state(False)
        self.reset_topic_perturbation_nodes()
        if advance:
            self.current_profile_index += 1

        if self.current_profile_index >= len(self.test_profiles):
            self.current_profile = None
            self.completed = True
            self.phase = 'completed'
            self.get_logger().info('All flight test profiles completed.')
            self.publish_autopilot_state(False)
            return

        current_config_index = self.test_profiles[self.current_profile_index].get(
            'config_index', 1
        )
        if current_config_index > self.test_config_end_index:
            self.current_profile = None
            self.completed = True
            self.phase = 'completed'
            self.get_logger().info('All flight test profiles completed.')
            self.publish_autopilot_state(False)
            return

        self.current_profile = self.test_profiles[self.current_profile_index]
        self.configure_current_profile_route()
        self.profile_started_at = None
        self.phase_started_at = self.experiment_now()
        self.phase = 'setup' if self.orchestrate_test_setup else 'waiting_for_mission_trigger'
        self.arm_requested = False
        self.auto_climb_requested = False
        self.route_sent = False
        self.rosbag_started = False
        self.last_arm_request_time = 0.0
        self.last_auto_climb_request_time = 0.0
        self.active_perturbation_channels = {}
        self.pending_perturbation_channels = {}
        self.completed_perturbation_channels = set()

        self.get_logger().info(
            'Starting flight test config '
            f'[{current_config_index}/{self.test_config_end_index}] '
            f'{self.current_profile["name"]} iteration '
            f'{self.current_profile["iteration"]}/{self.current_profile["iterations"]}.'
        )
        if self.orchestrate_test_setup:
            self.current_setup_state = None
            self.publish_setup_request(self.setup_overrides_for_current_profile())
        else:
            self.begin_profile_execution()

    def setup_overrides_for_current_profile(self) -> dict:
        setup_overrides = self.current_profile.get('setup', {}) if self.current_profile else {}
        if not isinstance(setup_overrides, dict):
            setup_overrides = {}
        else:
            setup_overrides = dict(setup_overrides)
        setup_overrides.pop('preplanned_route_filepath', None)

        if self.use_route_start_as_staging_area and self.route_start_staging_yaw_from_route:
            setup_overrides['skip_managed_px4_set_pose_after_model_reset'] = False

        return setup_overrides

    def begin_profile_execution(self):
        self.profile_started_at = self.experiment_now()
        self.phase_started_at = self.experiment_now()
        self.phase = 'waiting_for_mission_trigger'
        self.state_epoch_started_at = self.clock_now_s()
        self.current_quadcopter_state_code = None
        self.current_quadcopter_state_stamp_s = None
        self.current_quadcopter_state_received_at = None
        self.current_quadcopter_state_changed_at = None
        self.current_heartbeat_rx_state = None
        self.reset_activation_trigger_conditions()
        self.active_perturbation_channels = {}
        self.pending_perturbation_channels = {}
        self.completed_perturbation_channels = set()
        for perturbation_channel in self.current_profile['perturbation_channels']:
            self.publish_perturbation_profile(
                perturbation_channel['profile'],
                perturbation_channel['channel'],
            )

    def start_rosbag_recording_if_needed(self):
        if self.rosbag_started:
            return
        self.start_rosbag_recording(self.current_profile['topics_to_record'])
        self.rosbag_started = True

    def start_ready_perturbation_channels(self, now: float) -> bool:
        started_any = False
        for perturbation_channel in self.current_profile['perturbation_channels']:
            channel = perturbation_channel['channel']
            if channel in self.active_perturbation_channels:
                continue
            if channel in self.completed_perturbation_channels:
                continue
            pending_start_time = self.pending_perturbation_channels.get(channel)
            if pending_start_time is not None:
                if now < pending_start_time:
                    continue
                self.pending_perturbation_channels.pop(channel, None)
                self.start_rosbag_recording_if_needed()
                if self.publish_perturbation_active(True, channel):
                    self.active_perturbation_channels[channel] = now
                    started_any = True
                    self.get_logger().info(
                        f'Flight test perturbation enabled for channel {channel} for '
                        f'{perturbation_channel["duration_s"]:0.1f} s.'
                    )
                continue

            if not self.activation_triggers_are_stable(
                perturbation_channel['activation_triggers'],
                now,
                self.trigger_state_hold_sec,
            ):
                continue

            start_delay_s = perturbation_channel.get('start_delay_s', 0.0)
            if start_delay_s > 0.0:
                self.start_rosbag_recording_if_needed()
                self.pending_perturbation_channels[channel] = now + start_delay_s
                self.get_logger().info(
                    f'Topic perturbation for channel {channel} will start in '
                    f'{start_delay_s:0.1f} s.'
                )
                continue

            self.start_rosbag_recording_if_needed()
            if self.publish_perturbation_active(True, channel):
                self.active_perturbation_channels[channel] = now
                started_any = True
                self.get_logger().info(
                    f'Flight test perturbation enabled for channel {channel} for '
                    f'{perturbation_channel["duration_s"]:0.1f} s.'
                )

        return started_any

    def stop_finished_perturbation_channels(self, now: float) -> bool:
        for perturbation_channel in self.current_profile['perturbation_channels']:
            channel = perturbation_channel['channel']
            started_at = self.active_perturbation_channels.get(channel)
            if started_at is None:
                continue
            if (now - started_at) < perturbation_channel['duration_s']:
                continue

            self.publish_perturbation_active(False, channel)
            self.active_perturbation_channels.pop(channel, None)
            self.completed_perturbation_channels.add(channel)
            self.get_logger().info(f'Flight test perturbation completed for channel {channel}.')

        return len(self.completed_perturbation_channels) >= len(
            self.current_profile['perturbation_channels']
        )

    def handle_timeout(self):
        profile_name = self.current_profile['name'] if self.current_profile else 'unknown'
        self.get_logger().error(
            f'Flight test profile {profile_name} timed out after {self.test_timeout:0.1f} s.'
        )
        self.clear_pending_parameter_request()
        self.stop_rosbag_recording(delete_recording=True)
        self.publish_perturbation_active(False, force=True)
        self.active_perturbation_channels = {}
        self.pending_perturbation_channels = {}
        self.completed_perturbation_channels = set()

        if self.halt_test_runner_on_timeout:
            self.completed = True
            self.phase = 'completed'
            self.get_logger().error('Halting flight test runner on timeout.')
            return

        self.get_logger().warn(
            f'Retrying flight test profile {profile_name} after deleting the timed-out bag.'
        )
        self.start_next_profile(advance=False)

    def start_rosbag_recording(self, topics_to_record: list[str]):
        profile_name = (
            self.slugify(self.current_profile['name'], 'flight_test')
            if self.current_profile
            else 'run'
        )
        bag_name = (
            f'{time.strftime("%Y%m%d_%H%M%S")}_'
            f'{self.current_profile_index + 1:02d}_{profile_name}_'
            f'iter_{self.current_profile["iteration"]:02d}'
        )
        super().start_rosbag_recording(
            topics_to_record, bag_name, process_label='flight_test_rosbag'
        )

    def handle_auto_takeoff(self, now: float):
        if self.current_quadcopter_state_code == QuadcopterState.READY_TO_ARM:
            if not self.quadcopter_state_is_stable(
                {QuadcopterState.READY_TO_ARM}, now, self.ready_state_hold_sec
            ):
                return
            if (now - self.last_arm_request_time) >= self.arm_request_retry_period:
                self.request_remote_node_bool_service(
                    self.control_vehicle_node_fqn, 'arm', True
                )
                self.arm_requested = True
                self.last_arm_request_time = now
            return

        if self.current_quadcopter_state_code == QuadcopterState.ARMED:
            if not self.auto_climb_requested and (
                self.pending_parameter_request is None
                and (now - self.last_auto_climb_request_time) >= self.arm_request_retry_period
            ):
                self.request_remote_node_bool_service(
                    self.control_vehicle_node_fqn, 'auto_climb', True
                )
                self.last_auto_climb_request_time = now
            return

        if self.current_quadcopter_state_code in (
            QuadcopterState.CLIMBING,
            QuadcopterState.IN_FLIGHT,
            QuadcopterState.HOVERING,
        ):
            if self.parameter_request_effect_observed():
                self.get_logger().info(
                    'Observed climb after requesting auto_climb; '
                    'treating the parameter request as applied.'
                )
                self.clear_pending_parameter_request()
            self.auto_climb_requested = True

    @staticmethod
    def stamp_to_sec(stamp) -> float | None:
        stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        return stamp_s if stamp_s > 0.0 else None

    def quadcopter_state_is_stable(
        self, state_codes: set[int], now: float, hold_sec: float
    ) -> bool:
        if self.current_quadcopter_state_code not in state_codes:
            return False
        if self.current_quadcopter_state_changed_at is None:
            return False
        if (
            self.state_epoch_started_at is not None
            and self.current_quadcopter_state_stamp_s is not None
            and self.current_quadcopter_state_stamp_s + self.state_stamp_tolerance_sec
            < self.state_epoch_started_at
        ):
            return False
        return (now - self.current_quadcopter_state_changed_at) >= max(0.0, hold_sec)

    def activation_triggers_are_stable(
        self, trigger_keys: list[str], now: float, hold_sec: float
    ) -> bool:
        for trigger_key in trigger_keys:
            condition = self.activation_trigger_conditions.get(trigger_key)
            if condition is None or not condition['matched']:
                continue
            if condition['changed_at'] is None:
                continue
            if (
                self.state_epoch_started_at is not None
                and condition['stamp_s'] is not None
                and condition['stamp_s'] + self.state_stamp_tolerance_sec
                < self.state_epoch_started_at
            ):
                continue
            if (now - condition['changed_at']) >= max(0.0, hold_sec):
                return True

        return False

    def activation_triggers_are_matched(self, trigger_keys: list[str]) -> bool:
        return any(
            condition is not None and condition['matched']
            for condition in (
                self.activation_trigger_conditions.get(trigger_key) for trigger_key in trigger_keys
            )
        )

    def reset_activation_trigger_conditions(self):
        for condition in self.activation_trigger_conditions.values():
            condition['matched'] = False
            condition['changed_at'] = None
            condition['stamp_s'] = None
            condition['received_at'] = None

    def request_remote_node_parameter(self, node_fqn: str, name: str, value: Any) -> bool:
        resolved_node_fqn = self.resolve_node_fqn(node_fqn)
        service_name = f'{resolved_node_fqn}/set_parameters'.replace('//', '/')
        client = self.create_client(SetParameters, service_name)
        if not client.wait_for_service(timeout_sec=self.vehicle_parameter_service_timeout_sec):
            self.get_logger().warn(f"Parameter service of '{resolved_node_fqn}' not available")
            self.destroy_client(client)
            return False

        request = SetParameters.Request()
        parameter = ParameterMsg()
        parameter.name = name

        if isinstance(value, bool):
            parameter.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
        elif isinstance(value, int):
            parameter.value = ParameterValue(
                type=ParameterType.PARAMETER_INTEGER, integer_value=value
            )
        elif isinstance(value, float):
            parameter.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=value
            )
        elif isinstance(value, str):
            parameter.value = ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value=value
            )
        else:
            self.get_logger().error(f'Unsupported parameter type for {name}: {type(value)}')
            self.destroy_client(client)
            return False

        request.parameters.append(parameter)
        self.pending_parameter_request = {
            'client': client,
            'future': client.call_async(request),
            'node_fqn': resolved_node_fqn,
            'name': name,
            'value': value,
            'started_at': self.runner_now(),
        }
        self.get_logger().info(f'Requested {name} on {resolved_node_fqn}.')
        return True

    def request_remote_node_bool_service(self, node_fqn: str, name: str, value: bool) -> bool:
        resolved_node_fqn = node_fqn if node_fqn.startswith('/') else f'/{node_fqn}'
        service_name = f'{resolved_node_fqn}/{name}'
        client = self.create_client(SetBool, service_name)

        if not client.service_is_ready():
            self.get_logger().warn(f'Service {service_name} is not ready.')
            self.destroy_client(client)
            return False

        request = SetBool.Request()
        request.data = value
        self.pending_parameter_request = {
            'client': client,
            'future': client.call_async(request),
            'node_fqn': resolved_node_fqn,
            'name': name,
            'value': value,
            'started_at': self.runner_now(),
            'kind': 'bool_service',
        }
        self.get_logger().info(f'Requested {name} on {resolved_node_fqn}.')
        return True

    def poll_pending_parameter_request(self, now: float):
        if self.pending_parameter_request is None:
            return

        request = self.pending_parameter_request
        future = request['future']
        if future.done():
            try:
                result = future.result()
            except Exception as exc:
                self.clear_pending_parameter_request()
                self.get_logger().warn(
                    f'Setting {request["name"]} on {request["node_fqn"]} failed: {exc}'
                )
                return

            self.clear_pending_parameter_request()
            if result is None:
                self.get_logger().warn(
                    f'Setting {request["name"]} on {request["node_fqn"]} returned no result.'
                )
                return

            if request.get('kind') == 'bool_service':
                if not result.success:
                    self.get_logger().warn(
                        f'Requesting {request["name"]} on {request["node_fqn"]} was rejected: '
                        f'{result.message}'
                    )
                    return

                self.get_logger().info(f'Requested {request["name"]} on {request["node_fqn"]}.')
                if request['name'] == 'auto_climb' and request['value'] is True:
                    self.auto_climb_requested = True
                return

            if not all(item.successful for item in result.results):
                self.get_logger().warn(
                    f'Setting {request["name"]} on {request["node_fqn"]} was rejected.'
                )
                return

            self.get_logger().info(f'Set {request["name"]} on {request["node_fqn"]}.')
            if request['name'] == 'auto_climb' and request['value'] is True:
                self.auto_climb_requested = True
            return

        if self.parameter_request_effect_observed():
            self.get_logger().info(
                'Observed climb before the auto_climb parameter service responded.'
            )
            self.clear_pending_parameter_request()
            self.auto_climb_requested = True
            return

        if (now - request['started_at']) >= self.vehicle_parameter_service_timeout_sec:
            self.clear_pending_parameter_request()
            self.get_logger().warn(
                f'Setting {request["name"]} on {request["node_fqn"]} timed out after '
                f'{self.vehicle_parameter_service_timeout_sec:0.1f} s.'
            )

    def parameter_request_effect_observed(self) -> bool:
        if self.pending_parameter_request is None:
            return False

        request = self.pending_parameter_request
        if request['name'] != 'auto_climb' or request['value'] is not True:
            return False

        return self.current_quadcopter_state_code in (
            QuadcopterState.CLIMBING,
            QuadcopterState.IN_FLIGHT,
            QuadcopterState.HOVERING,
        )

    def clear_pending_parameter_request(self):
        if self.pending_parameter_request is None:
            return

        client = self.pending_parameter_request['client']
        self.pending_parameter_request = None
        self.destroy_client(client)

    def destroy_node(self):
        self.clear_pending_parameter_request()
        if rclpy.ok():
            self.publish_perturbation_active(False, force=True)
        self.stop_rosbag_recording()
        super().destroy_node()

    @staticmethod
    def runner_now() -> float:
        return time.monotonic()

    def experiment_now(self) -> float:
        clock_now = self.clock_now_s()
        return clock_now if clock_now is not None else self.runner_now()

    def clock_now_s(self) -> float | None:
        try:
            ns = self.get_clock().now().nanoseconds
        except Exception:
            return None
        if self.use_sim_time:
            return float(ns) / 1e9
        return float(ns) / 1e9 if ns and ns > 0 else None


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = FlightTestRunnerNode()

    # Convert SIGTERM → SystemExit so the finally block always runs on
    # launch-system shutdown (default Python SIGTERM kills without finally).
    def _sigterm_handler(*_):
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, _sigterm_handler)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('User requested shutdown with SIGINT.')
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
