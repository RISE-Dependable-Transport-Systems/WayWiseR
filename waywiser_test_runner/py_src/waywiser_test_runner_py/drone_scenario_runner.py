#!/usr/bin/env python3

import json
import os
import re
import time
from typing import Any

from ament_index_python import get_package_share_directory
from geometry_msgs.msg import Twist
import rclpy
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_msgs.msg import Bool, String

from waywiser_core.msg import QuadcopterState
from waywiser_py.waywiser_utils import FileUtils, ProcessUtils, RELIABLE_TRANSIENT_LOCAL_QOS

PACKAGE_NAME = 'waywiser_test_runner'


class DroneScenarioRunner(Node):
    def __init__(self):
        super().__init__('drone_scenario_runner')

        self.package_share_dir = get_package_share_directory(PACKAGE_NAME)

        self.declare_parameter('scenario_configurations_json_path', '')
        self.declare_parameter('rosbag_output_dir', 'drone_scenario_3b_data')
        self.declare_parameter('use_rosbag_recording', True)
        self.declare_parameter('quadcopter_state_topic', 'quadcopter_state')
        self.declare_parameter('control_vehicle_node_fqn', 'waywiser_drone_node')
        self.declare_parameter('arm_command_topic', 'arm_command')
        self.declare_parameter('nominal_command_topic', 'teleop_mux_vel')
        self.declare_parameter('attack_control_topic', 'scenario_3b/attack_enabled')
        self.declare_parameter('attack_profile_topic', 'scenario_3b/attack_profile')
        self.declare_parameter('auto_takeoff_enabled', True)
        self.declare_parameter('default_trigger_states', ['HOVERING'])
        self.declare_parameter('default_trigger_delay_sec', 2.0)
        self.declare_parameter('default_attack_duration_sec', 8.0)
        self.declare_parameter('default_nominal_linear', [0.2, 0.0, 0.0])
        self.declare_parameter('default_nominal_angular', [0.0, 0.0, 0.0])
        self.declare_parameter('default_nominal_command_publish_rate_hz', 10.0)
        self.declare_parameter('arm_request_retry_period', 1.0)
        self.declare_parameter('vehicle_parameter_service_timeout_sec', 10.0)
        self.declare_parameter('runner_timer_rate', 5.0)
        self.declare_parameter('test_timeout', 180.0)
        self.declare_parameter('test_cooldown_time', 5.0)
        self.declare_parameter('halt_test_runner_on_timeout', False)
        self.declare_parameter('topics_to_record', [''])

        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.scenario_configurations_json_path = FileUtils.get_full_file_path(
            self.get_parameter('scenario_configurations_json_path')
            .get_parameter_value()
            .string_value,
            os.path.join(self.package_share_dir, 'config'),
        )
        self.rosbag_output_dir = FileUtils.get_full_file_path(
            self.get_parameter('rosbag_output_dir').get_parameter_value().string_value,
            os.getcwd(),
        )
        self.use_rosbag_recording = (
            self.get_parameter('use_rosbag_recording').get_parameter_value().bool_value
        )
        self.quadcopter_state_topic = (
            self.get_parameter('quadcopter_state_topic').get_parameter_value().string_value
        )
        self.control_vehicle_node_fqn = (
            self.get_parameter('control_vehicle_node_fqn').get_parameter_value().string_value
        )
        self.arm_command_topic = (
            self.get_parameter('arm_command_topic').get_parameter_value().string_value
        )
        self.nominal_command_topic = (
            self.get_parameter('nominal_command_topic').get_parameter_value().string_value
        )
        self.attack_control_topic = (
            self.get_parameter('attack_control_topic').get_parameter_value().string_value
        )
        self.attack_profile_topic = (
            self.get_parameter('attack_profile_topic').get_parameter_value().string_value
        )
        self.auto_takeoff_enabled = (
            self.get_parameter('auto_takeoff_enabled').get_parameter_value().bool_value
        )
        self.default_trigger_states = list(
            self.get_parameter('default_trigger_states').get_parameter_value().string_array_value
        )
        self.default_trigger_delay_sec = (
            self.get_parameter('default_trigger_delay_sec').get_parameter_value().double_value
        )
        self.default_attack_duration_sec = (
            self.get_parameter('default_attack_duration_sec').get_parameter_value().double_value
        )
        self.default_nominal_linear = list(
            self.get_parameter('default_nominal_linear').get_parameter_value().double_array_value
        )
        self.default_nominal_angular = list(
            self.get_parameter('default_nominal_angular').get_parameter_value().double_array_value
        )
        self.default_nominal_command_publish_rate_hz = (
            self.get_parameter('default_nominal_command_publish_rate_hz')
            .get_parameter_value()
            .double_value
        )
        self.arm_request_retry_period = (
            self.get_parameter('arm_request_retry_period').get_parameter_value().double_value
        )
        self.vehicle_parameter_service_timeout_sec = (
            self.get_parameter('vehicle_parameter_service_timeout_sec')
            .get_parameter_value()
            .double_value
        )
        self.runner_timer_rate = (
            self.get_parameter('runner_timer_rate').get_parameter_value().double_value
        )
        self.test_timeout = self.get_parameter('test_timeout').get_parameter_value().double_value
        self.test_cooldown_time = (
            self.get_parameter('test_cooldown_time').get_parameter_value().double_value
        )
        self.halt_test_runner_on_timeout = (
            self.get_parameter('halt_test_runner_on_timeout').get_parameter_value().bool_value
        )
        topics_to_record = (
            self.get_parameter('topics_to_record').get_parameter_value().string_array_value
        )
        self.default_topics_to_record = [topic for topic in topics_to_record if topic]

        self.state_code_to_name = {
            value: name
            for name, value in QuadcopterState.__dict__.items()
            if name.isupper() and isinstance(value, int)
        }
        self.state_name_to_code = {
            name: value for value, name in self.state_code_to_name.items()
        }

        self.current_quadcopter_state_code: int | None = None
        self.current_config_index = -1
        self.current_config: dict[str, Any] | None = None
        self.pending_parameter_request: dict[str, Any] | None = None
        self.phase = 'idle'
        self.phase_started_at: float | None = None
        self.config_started_at: float | None = None
        self.last_arm_request_time = 0.0
        self.last_auto_lift_off_request_time = 0.0
        self.arm_requested = False
        self.auto_lift_off_requested = False
        self.subprocesses: dict[str, Any] = {}
        self.completed = False
        self.nominal_command = Twist()
        self.nominal_command_publish_rate_hz = self.default_nominal_command_publish_rate_hz
        self.nominal_command_active = False
        self.nominal_command_timer = None

        self.arm_command_publisher = self.create_publisher(Bool, self.arm_command_topic, 10)
        self.nominal_command_publisher = self.create_publisher(Twist, self.nominal_command_topic, 10)
        self.attack_control_publisher = self.create_publisher(
            Bool, self.attack_control_topic, RELIABLE_TRANSIENT_LOCAL_QOS
        )
        self.attack_profile_publisher = self.create_publisher(
            String, self.attack_profile_topic, RELIABLE_TRANSIENT_LOCAL_QOS
        )
        self.create_subscription(
            QuadcopterState,
            self.quadcopter_state_topic,
            self.quadcopter_state_callback,
            RELIABLE_TRANSIENT_LOCAL_QOS,
        )
        self.create_timer(1.0 / self.runner_timer_rate, self.runner_timer_callback)

        self.scenario_configurations = self.parse_scenario_configurations()
        self.publish_attack_enabled(False, force=True)

        if not self.scenario_configurations:
            self.completed = True
            self.get_logger().warn('No drone Scenario 3b configurations were loaded.')
        else:
            self.get_logger().info(
                f'Loaded {len(self.scenario_configurations)} Scenario 3b configuration iteration(s).'
            )

    def quadcopter_state_callback(self, msg: QuadcopterState):
        if self.current_quadcopter_state_code != msg.state_code:
            self.current_quadcopter_state_code = msg.state_code
            self.get_logger().info(
                'Quadcopter state: '
                f'{self.state_code_to_name.get(msg.state_code, str(msg.state_code))} '
                f'({msg.state_str}).'
            )

    def runner_timer_callback(self):
        if self.completed:
            return

        if self.current_config is None:
            self.start_next_configuration()
            return

        now = time.monotonic()
        self.poll_pending_parameter_request(now)
        if self.config_started_at is not None and (now - self.config_started_at) > self.test_timeout:
            self.handle_timeout()
            return

        if self.phase == 'waiting_for_ready':
            if self.current_quadcopter_state_code in self.current_config['trigger_state_codes']:
                self.start_nominal_command_stream()
                self.phase = 'delay'
                self.phase_started_at = now
                self.get_logger().info(
                    'Ready state observed. '
                    f'Attack will start in {self.current_config["trigger_delay_sec"]:0.1f} s.'
                )
                return

            if self.current_config['auto_takeoff_enabled']:
                self.handle_auto_takeoff(now)
            return

        if self.phase == 'delay' and self.phase_started_at is not None:
            if (now - self.phase_started_at) >= self.current_config['trigger_delay_sec']:
                self.publish_attack_enabled(True)
                self.phase = 'attack'
                self.phase_started_at = now
                self.get_logger().info(
                    f'Attack enabled for {self.current_config["duration_sec"]:0.1f} s.'
                )
            return

        if self.phase == 'attack' and self.phase_started_at is not None:
            if (now - self.phase_started_at) >= self.current_config['duration_sec']:
                self.publish_attack_enabled(False)
                self.phase = 'cooldown'
                self.phase_started_at = now
                self.get_logger().info('Attack window completed.')
            return

        if self.phase == 'cooldown' and self.phase_started_at is not None:
            if (now - self.phase_started_at) >= self.test_cooldown_time:
                self.start_next_configuration()

    def parse_scenario_configurations(self) -> list[dict[str, Any]]:
        if not self.scenario_configurations_json_path:
            self.get_logger().warn('No scenario configuration JSON path was provided.')
            return []

        if not os.path.exists(self.scenario_configurations_json_path):
            self.get_logger().error(
                f'Scenario configuration file not found: {self.scenario_configurations_json_path}'
            )
            return []

        try:
            with open(self.scenario_configurations_json_path, 'r', encoding='utf-8') as config_file:
                raw_configurations = json.load(config_file).get('drone_scenario_configurations', [])
        except Exception as exc:
            self.get_logger().error(
                f'Failed to load scenario configurations from '
                f'{self.scenario_configurations_json_path}: {exc}'
            )
            return []

        parsed_configurations = []
        for raw_index, raw_configuration in enumerate(raw_configurations, start=1):
            iterations = max(int(raw_configuration.get('iterations', 1)), 1)
            trigger_config = raw_configuration.get('trigger', {})
            attack_config = raw_configuration.get('attack', {})
            automation_config = raw_configuration.get('automation', {})
            nominal_command_config = raw_configuration.get('nominal_command', {})

            if not isinstance(trigger_config, dict):
                trigger_config = {}
            if not isinstance(attack_config, dict):
                attack_config = {}
            if not isinstance(automation_config, dict):
                automation_config = {}
            if not isinstance(nominal_command_config, dict):
                nominal_command_config = {}

            attack_profile = raw_configuration.get('attack_profile')
            if attack_profile is None:
                attack_profile = attack_config.get('profile', attack_config)
            if not isinstance(attack_profile, dict):
                attack_profile = {'mode': 'passthrough'}

            trigger_states = trigger_config.get('states', self.default_trigger_states)
            trigger_state_codes = self.parse_trigger_state_codes(trigger_states)
            trigger_delay_sec = float(
                trigger_config.get('delay_sec', self.default_trigger_delay_sec)
            )
            duration_sec = float(
                attack_config.get(
                    'duration_sec',
                    raw_configuration.get('duration_sec', self.default_attack_duration_sec),
                )
            )
            auto_takeoff_enabled = bool(
                automation_config.get('auto_takeoff', self.auto_takeoff_enabled)
            )
            nominal_command_publish_rate_hz = float(
                raw_configuration.get(
                    'nominal_command_publish_rate_hz',
                    self.default_nominal_command_publish_rate_hz,
                )
            )
            topics_to_record = raw_configuration.get(
                'topics_to_record', self.default_topics_to_record
            )
            topics_to_record = [topic for topic in topics_to_record if topic]

            for iteration_index in range(iterations):
                parsed_configurations.append(
                    {
                        'name': raw_configuration.get('name', f'drone_scenario_{raw_index}'),
                        'iteration': iteration_index + 1,
                        'iterations': iterations,
                        'trigger_state_codes': trigger_state_codes,
                        'trigger_delay_sec': trigger_delay_sec,
                        'duration_sec': duration_sec,
                        'auto_takeoff_enabled': auto_takeoff_enabled,
                        'nominal_command': self.build_nominal_command(nominal_command_config),
                        'nominal_command_publish_rate_hz': nominal_command_publish_rate_hz,
                        'attack_profile': attack_profile,
                        'topics_to_record': topics_to_record,
                    }
                )

        return parsed_configurations

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
                state_code = self.state_name_to_code.get(normalized_state)
                if state_code is None:
                    state_code = getattr(QuadcopterState, normalized_state, None)
                if isinstance(state_code, int):
                    parsed_state_codes.add(state_code)

        if not parsed_state_codes:
            parsed_state_codes.add(QuadcopterState.HOVERING)

        return parsed_state_codes

    def start_next_configuration(self):
        self.clear_pending_parameter_request()
        self.stop_nominal_command_stream()
        self.stop_rosbag_recording()
        self.publish_attack_enabled(False, force=True)
        self.current_config_index += 1

        if self.current_config_index >= len(self.scenario_configurations):
            self.current_config = None
            self.completed = True
            self.phase = 'completed'
            self.get_logger().info('All Scenario 3b configurations completed.')
            return

        self.current_config = self.scenario_configurations[self.current_config_index]
        self.config_started_at = time.monotonic()
        self.phase_started_at = self.config_started_at
        self.phase = 'waiting_for_ready'
        self.arm_requested = False
        self.auto_lift_off_requested = False
        self.last_arm_request_time = 0.0
        self.last_auto_lift_off_request_time = 0.0
        self.nominal_command = self.current_config['nominal_command']
        self.nominal_command_publish_rate_hz = self.current_config[
            'nominal_command_publish_rate_hz'
        ]

        self.publish_attack_profile(self.current_config['attack_profile'])
        self.start_rosbag_recording(self.current_config['topics_to_record'])

        self.get_logger().info(
            'Starting Scenario 3b configuration '
            f'[{self.current_config_index + 1}/{len(self.scenario_configurations)}] '
            f"{self.current_config['name']} iteration "
            f"{self.current_config['iteration']}/{self.current_config['iterations']}."
        )

    def publish_attack_profile(self, attack_profile: dict[str, Any]):
        profile_message = String()
        profile_message.data = json.dumps(attack_profile)
        self.attack_profile_publisher.publish(profile_message)
        self.get_logger().info(f'Published attack profile: {profile_message.data}')

    def publish_attack_enabled(self, enabled: bool, force: bool = False):
        if not force and self.phase == 'attack' and enabled:
            return

        enabled_message = Bool()
        enabled_message.data = enabled
        self.attack_control_publisher.publish(enabled_message)

    def handle_timeout(self):
        scenario_name = self.current_config['name'] if self.current_config else 'unknown'
        self.get_logger().error(
            f'Scenario {scenario_name} timed out after {self.test_timeout:0.1f} s.'
        )
        self.clear_pending_parameter_request()
        self.stop_nominal_command_stream()
        self.stop_rosbag_recording()
        self.publish_attack_enabled(False, force=True)

        if self.halt_test_runner_on_timeout:
            self.completed = True
            self.phase = 'completed'
            self.get_logger().error('Halting Scenario 3b runner on timeout.')
            return

        self.start_next_configuration()

    def start_rosbag_recording(self, topics_to_record: list[str]):
        if not self.use_rosbag_recording or not topics_to_record:
            return

        resolved_topics = [self.resolve_topic_name(topic) for topic in topics_to_record]
        os.makedirs(self.rosbag_output_dir, exist_ok=True)
        scenario_name = self.slugify(self.current_config['name']) if self.current_config else 'scenario'
        bag_name = (
            f'{time.strftime("%Y%m%d_%H%M%S")}_'
            f'{self.current_config_index + 1:02d}_{scenario_name}_'
            f'iter_{self.current_config["iteration"]:02d}'
        )
        bag_path = os.path.join(self.rosbag_output_dir, bag_name)
        command = ['ros2', 'bag', 'record', '-o', bag_path]
        if self.use_sim_time:
            command.append('--use-sim-time')
        command.extend(resolved_topics)

        self.subprocesses['rosbag'] = ProcessUtils.create_subprocess(
            self, command, 'scenario_3b_rosbag'
        )

    def stop_rosbag_recording(self):
        ProcessUtils.cleanup_subprocesses(self.subprocesses)

    def handle_auto_takeoff(self, now: float):
        if self.current_quadcopter_state_code == QuadcopterState.READY_TO_ARM:
            if (now - self.last_arm_request_time) >= self.arm_request_retry_period:
                self.publish_arm_request(True)
                self.arm_requested = True
                self.last_arm_request_time = now
            return

        if self.current_quadcopter_state_code == QuadcopterState.ARMED:
            if not self.auto_lift_off_requested and (
                self.pending_parameter_request is None
                and (now - self.last_auto_lift_off_request_time) >= self.arm_request_retry_period
            ):
                self.request_remote_node_parameter(
                    self.control_vehicle_node_fqn, 'auto_lift_off', True
                )
                self.last_auto_lift_off_request_time = now
            return

        if self.current_quadcopter_state_code in (
            QuadcopterState.LIFTING_OFF,
            QuadcopterState.AUTO_LIFTING_OFF,
            QuadcopterState.IN_FLIGHT,
            QuadcopterState.HOVERING,
        ):
            if self.parameter_request_effect_observed():
                self.get_logger().info(
                    'Observed lift-off after requesting auto_lift_off; '
                    'treating the parameter request as applied.'
                )
                self.clear_pending_parameter_request()
            self.auto_lift_off_requested = True

    def publish_arm_request(self, arm: bool):
        arm_message = Bool()
        arm_message.data = arm
        self.arm_command_publisher.publish(arm_message)
        if arm:
            self.get_logger().info('Published arm command request.')
        else:
            self.get_logger().info('Published disarm command request.')

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
            'started_at': time.monotonic(),
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
                    f"Setting {request['name']} on {request['node_fqn']} failed: {exc}"
                )
                return

            self.clear_pending_parameter_request()
            if result is None:
                self.get_logger().warn(
                    f"Setting {request['name']} on {request['node_fqn']} returned no result."
                )
                return

            if not all(item.successful for item in result.results):
                self.get_logger().warn(
                    f"Setting {request['name']} on {request['node_fqn']} was rejected."
                )
                return

            self.get_logger().info(f"Set {request['name']} on {request['node_fqn']}.")
            if request['name'] == 'auto_lift_off' and request['value'] is True:
                self.auto_lift_off_requested = True
            return

        if self.parameter_request_effect_observed():
            self.get_logger().info(
                'Observed lift-off before the auto_lift_off parameter service responded.'
            )
            self.clear_pending_parameter_request()
            self.auto_lift_off_requested = True
            return

        if (now - request['started_at']) >= self.vehicle_parameter_service_timeout_sec:
            self.clear_pending_parameter_request()
            self.get_logger().warn(
                f"Setting {request['name']} on {request['node_fqn']} timed out after "
                f'{self.vehicle_parameter_service_timeout_sec:0.1f} s.'
            )

    def parameter_request_effect_observed(self) -> bool:
        if self.pending_parameter_request is None:
            return False

        request = self.pending_parameter_request
        if request['name'] != 'auto_lift_off' or request['value'] is not True:
            return False

        return self.current_quadcopter_state_code in (
            QuadcopterState.LIFTING_OFF,
            QuadcopterState.AUTO_LIFTING_OFF,
            QuadcopterState.IN_FLIGHT,
            QuadcopterState.HOVERING,
        )

    def clear_pending_parameter_request(self):
        if self.pending_parameter_request is None:
            return

        client = self.pending_parameter_request['client']
        self.pending_parameter_request = None
        self.destroy_client(client)

    def build_nominal_command(self, nominal_command_config: dict[str, Any]) -> Twist:
        linear = self.vector_from_value(
            nominal_command_config.get('linear'), self.default_nominal_linear
        )
        angular = self.vector_from_value(
            nominal_command_config.get('angular'), self.default_nominal_angular
        )

        if 'linear_x' in nominal_command_config:
            linear[0] = float(nominal_command_config['linear_x'])
        if 'linear_y' in nominal_command_config:
            linear[1] = float(nominal_command_config['linear_y'])
        if 'linear_z' in nominal_command_config:
            linear[2] = float(nominal_command_config['linear_z'])
        if 'angular_x' in nominal_command_config:
            angular[0] = float(nominal_command_config['angular_x'])
        if 'angular_y' in nominal_command_config:
            angular[1] = float(nominal_command_config['angular_y'])
        if 'angular_z' in nominal_command_config:
            angular[2] = float(nominal_command_config['angular_z'])

        command = Twist()
        command.linear.x, command.linear.y, command.linear.z = linear
        command.angular.x, command.angular.y, command.angular.z = angular
        return command

    @staticmethod
    def vector_from_value(value: Any, default: list[float]) -> list[float]:
        if value is None:
            return list(default)
        if isinstance(value, (int, float)):
            return [float(value)] * 3
        if isinstance(value, (list, tuple)):
            if len(value) == 1:
                return [float(value[0])] * 3
            if len(value) >= 3:
                return [float(value[0]), float(value[1]), float(value[2])]
        return list(default)

    def start_nominal_command_stream(self):
        if self.nominal_command_active:
            return

        self.nominal_command_active = True
        if self.nominal_command_timer is not None:
            self.nominal_command_timer.cancel()
            self.destroy_timer(self.nominal_command_timer)
            self.nominal_command_timer = None

        if self.nominal_command_publish_rate_hz > 0.0:
            self.nominal_command_timer = self.create_timer(
                1.0 / self.nominal_command_publish_rate_hz, self.publish_nominal_command
            )

        self.publish_nominal_command()
        self.get_logger().info(
            'Started nominal command stream on '
            f'{self.resolve_topic_name(self.nominal_command_topic)} '
            f'at {self.nominal_command_publish_rate_hz:0.1f} Hz.'
        )

    def stop_nominal_command_stream(self, publish_zero: bool = True):
        self.nominal_command_active = False
        if self.nominal_command_timer is not None:
            self.nominal_command_timer.cancel()
            self.destroy_timer(self.nominal_command_timer)
            self.nominal_command_timer = None

        if publish_zero:
            self.nominal_command_publisher.publish(Twist())

    def publish_nominal_command(self):
        if not self.nominal_command_active:
            return

        self.nominal_command_publisher.publish(self.nominal_command)

    def resolve_topic_name(self, topic_name: str) -> str:
        if topic_name.startswith('/'):
            return topic_name

        namespace = self.get_namespace().strip('/')
        if namespace == '':
            return '/' + topic_name.lstrip('/')

        return f'/{namespace}/{topic_name.lstrip("/")}'

    def resolve_node_fqn(self, node_fqn: str) -> str:
        if node_fqn.startswith('/'):
            return node_fqn.rstrip('/')

        namespace = self.get_namespace().rstrip('/')
        if namespace == '':
            return '/' + node_fqn.strip('/')

        return f'{namespace}/{node_fqn.strip("/")}'

    @staticmethod
    def slugify(value: str) -> str:
        slug = re.sub(r'[^a-zA-Z0-9]+', '_', value).strip('_').lower()
        return slug or 'scenario'

    def destroy_node(self):
        self.clear_pending_parameter_request()
        self.stop_nominal_command_stream()
        self.publish_attack_enabled(False, force=True)
        self.stop_rosbag_recording()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DroneScenarioRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('User requested shutdown with SIGINT.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()