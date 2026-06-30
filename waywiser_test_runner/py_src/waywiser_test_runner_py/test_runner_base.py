#!/usr/bin/env python3

import json
import math
import os
import re
import shutil
import sys
import time
from typing import Any
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
import pymap3d as pm
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import tf_transformations

from waywiser_core.msg import PathWithTwists
from waywiser_py.waywiser_utils import FileUtils, ProcessUtils, RELIABLE_TRANSIENT_LOCAL_QOS
from waywiser_test_runner.msg import SetupState


class TestRunnerBase(Node):
    """Shared mechanics for WayWiseR test runner nodes."""

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.subprocesses: dict[str, object] = {}
        self._companion_subprocesses: dict[str, object] = {}
        self.current_setup_state: int | None = None
        self.awaiting_setup_status_for_request = False
        self.setup_request_started = False
        self.current_rosbag_path: str | None = None

    def declare_common_test_runner_parameters(self, defaults: dict | None = None):
        defaults = defaults or {}
        parameter_defaults = {
            'rosbag_output_dir': '',
            'use_rosbag_recording': False,
            'orchestrate_test_setup': True,
            'setup_request_topic': '/setup_request',
            'setup_status_topic': '/setup_status',
            'setup_request_json': '{}',
            'runner_timer_rate': 5.0,
            'test_timeout': 360.0,
            'test_cooldown_time': 5.0,
            'halt_test_runner_on_timeout': False,
            'topics_to_record': [''],
            'staging_entity_name': '',
            'staging_area_pose': '',
            'respawn_vehicle_at_staging_area': False,
        }
        for name, default in parameter_defaults.items():
            self.declare_parameter(name, defaults.get(name, default))

    def initialize_common_test_runner(self):
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.rosbag_output_dir = FileUtils.get_full_file_path(
            self.get_parameter('rosbag_output_dir').get_parameter_value().string_value,
            os.getcwd(),
        )
        self.use_rosbag_recording = (
            self.get_parameter('use_rosbag_recording').get_parameter_value().bool_value
        )
        self.orchestrate_test_setup = (
            self.get_parameter('orchestrate_test_setup').get_parameter_value().bool_value
        )
        self.setup_request_topic = (
            self.get_parameter('setup_request_topic').get_parameter_value().string_value
        )
        self.setup_status_topic = (
            self.get_parameter('setup_status_topic').get_parameter_value().string_value
        )
        self.setup_request_json = (
            self.get_parameter('setup_request_json').get_parameter_value().string_value
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
        self.staging_entity_name = (
            self.get_parameter('staging_entity_name').get_parameter_value().string_value.strip()
        )
        self.staging_area_pose = self.parse_pose_values(
            self.get_parameter('staging_area_pose').get_parameter_value().string_value
        )
        self.respawn_vehicle_at_staging_area = (
            self.get_parameter('respawn_vehicle_at_staging_area').get_parameter_value().bool_value
        )

        self.setup_request_publisher = self.create_publisher(
            String, self.setup_request_topic, RELIABLE_TRANSIENT_LOCAL_QOS
        )
        self.create_subscription(
            SetupState,
            self.setup_status_topic,
            self.setup_status_callback,
            RELIABLE_TRANSIENT_LOCAL_QOS,
        )

        self.topic_perturbation_channels: list[str] = []
        self.perturbation_control_publishers = {}
        self.perturbation_profile_publishers = {}
        self._reset_publishers = {}
        self._start_topic_perturbation_nodes()

    def _start_topic_perturbation_nodes(self):
        if not self.has_parameter('topic_perturbation.channels'):
            self.declare_parameter('topic_perturbation.channels', [''])
        channels = [
            c
            for c in self.get_parameter('topic_perturbation.channels')
            .get_parameter_value()
            .string_array_value
            if c
        ]
        if not channels:
            return
        self.topic_perturbation_channels = channels

        namespace = self.get_namespace().rstrip('/')
        perturbation_executable = os.path.join(
            get_package_prefix('waywiser_test_runner'),
            'lib',
            'waywiser_test_runner',
            'topic_perturbation_node.py',
        )
        channel_param_defs = [
            ('message_type', 'string_value', 'std_msgs/msg/Bool'),
            ('input_topic', 'string_value', ''),
            ('output_topic', 'string_value', ''),
            ('control_topic', 'string_value', ''),
            ('profile_topic', 'string_value', ''),
            ('default_profile_json', 'string_value', '{"mode": "passthrough"}'),
            ('start_active', 'bool_value', False),
            ('default_publish_rate_hz', 'double_value', 0.0),
            ('delay_poll_rate_hz', 'double_value', 50.0),
        ]

        for channel_index, channel in enumerate(channels):
            channel_node_suffix = (
                re.sub(r'[^a-zA-Z0-9_]', '_', channel).strip('_') or f'channel_{channel_index + 1}'
            )
            perturbation_node_name = 'topic_perturbation_node'
            if len(channels) > 1:
                perturbation_node_name = f'{perturbation_node_name}_{channel_node_suffix}'

            params = {}
            for param_name, getter, default in channel_param_defs:
                full_name = f'topic_perturbation.{channel}.{param_name}'
                if not self.has_parameter(full_name):
                    self.declare_parameter(full_name, default)
                params[param_name] = getattr(
                    self.get_parameter(full_name).get_parameter_value(), getter
                )

            control_topic = params['control_topic'] or (
                f'topic_perturbation/{channel}/perturbation_active'
            )
            profile_topic = params['profile_topic'] or (
                f'topic_perturbation/{channel}/perturbation_profile'
            )
            self.perturbation_control_publishers[channel] = self.create_publisher(
                Bool, control_topic, RELIABLE_TRANSIENT_LOCAL_QOS
            )
            self.perturbation_profile_publishers[channel] = self.create_publisher(
                String, profile_topic, RELIABLE_TRANSIENT_LOCAL_QOS
            )

            # JSON value must be single-quoted so the ROS2 CLI YAML parser treats it as a string.
            profile_json_arg = f"default_profile_json:='{params['default_profile_json']}'"

            reset_topic = f'topic_perturbation/{channel}/reset'

            command = [
                sys.executable,
                perturbation_executable,
                '--ros-args',
                '-r',
                f'__name:={perturbation_node_name}',
                '-p',
                f'message_type:={params["message_type"]}',
                '-p',
                f'input_topic:={params["input_topic"]}',
                '-p',
                f'output_topic:={params["output_topic"]}',
                '-p',
                f'perturbation_control_topic:={control_topic}',
                '-p',
                f'perturbation_profile_topic:={profile_topic}',
                '-p',
                f'reset_topic:={reset_topic}',
                '-p',
                profile_json_arg,
                '-p',
                f'start_active:={str(params["start_active"]).lower()}',
                '-p',
                f'default_publish_rate_hz:={params["default_publish_rate_hz"]}',
                '-p',
                f'delay_poll_rate_hz:={params["delay_poll_rate_hz"]}',
                '-p',
                f'use_sim_time:={str(self.use_sim_time).lower()}',
            ]
            if namespace:
                command += ['-r', f'__ns:={namespace}']

            self._companion_subprocesses[perturbation_node_name] = ProcessUtils.create_subprocess(
                self, command, perturbation_node_name
            )

    def publish_perturbation_active(
        self, enabled: bool, channel: str | None = None, force: bool = False
    ) -> bool:
        if not rclpy.ok():
            return False
        channels = self._resolve_perturbation_channels(channel, allow_all=True)
        if not channels:
            return False
        enabled_message = Bool()
        enabled_message.data = enabled
        for channel_name in channels:
            self.perturbation_control_publishers[channel_name].publish(enabled_message)
        return True

    def publish_perturbation_profile(
        self, perturbation_profile: dict, channel: str | None = None
    ) -> bool:
        channels = self._resolve_perturbation_channels(channel, allow_all=False)
        if not channels:
            return False
        profile_message = String()
        profile_message.data = json.dumps(perturbation_profile)
        channel_name = channels[0]
        self.perturbation_profile_publishers[channel_name].publish(profile_message)
        self.get_logger().info(
            f'Published perturbation profile for channel {channel_name}: {profile_message.data}'
        )
        return True

    def _resolve_perturbation_channels(self, channel: str | None, allow_all: bool) -> list[str]:
        if channel:
            if channel in self.perturbation_control_publishers:
                return [channel]
            self.get_logger().error(
                f"Unknown topic perturbation channel '{channel}'. Available channels: "
                f'{self.topic_perturbation_channels}'
            )
            return []

        if allow_all:
            return list(self.topic_perturbation_channels)
        if len(self.topic_perturbation_channels) == 1:
            return list(self.topic_perturbation_channels)

        self.get_logger().error(
            'A perturbation channel must be specified when multiple topic perturbation '
            f'channels are configured: {self.topic_perturbation_channels}'
        )
        return []

    def reset_topic_perturbation_nodes(self):
        """
        Reset perturbation node state for a fresh iteration.

        Sends a reset command to each perturbation node via a latched topic,
        then re-publishes the default profile so the node reconfigures itself.
        """
        for channel in self.topic_perturbation_channels:
            reset_topic = f'topic_perturbation/{channel}/reset'
            if reset_topic not in self._reset_publishers:
                self._reset_publishers[reset_topic] = self.create_publisher(
                    Bool, reset_topic, RELIABLE_TRANSIENT_LOCAL_QOS
                )
            reset_msg = Bool()
            reset_msg.data = True
            self._reset_publishers[reset_topic].publish(reset_msg)

    def destroy_node(self):
        ProcessUtils.cleanup_subprocesses(self._companion_subprocesses)
        super().destroy_node()

    def _now(self) -> float:
        try:
            ns = self.get_clock().now().nanoseconds
            if ns and ns != 0:
                return float(ns) / 1e9
        except Exception:
            pass
        return time.monotonic()

    def setup_status_callback(self, msg: SetupState):
        if self.awaiting_setup_status_for_request:
            if msg.state == SetupState.SETUP_COMPLETED and not self.setup_request_started:
                return
            if msg.state in (SetupState.SETUP_INIT, SetupState.SETUP_ONGOING):
                self.setup_request_started = True
            if msg.state == SetupState.SETUP_COMPLETED and self.setup_request_started:
                self.awaiting_setup_status_for_request = False

        if self.current_setup_state != msg.state:
            self.current_setup_state = msg.state
            state_name = {
                value: name
                for name, value in SetupState.__dict__.items()
                if name.isupper() and isinstance(value, int)
            }.get(msg.state, str(msg.state))
            self.get_logger().info(f'Setup state: {state_name}.')

    def build_setup_request(self, overrides: dict | str | None = None) -> dict:
        setup_request = {}
        try:
            setup_request = json.loads(self.setup_request_json) if self.setup_request_json else {}
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Invalid setup_request_json: {exc}. Using empty request.')

        if isinstance(overrides, str) and overrides.strip():
            try:
                overrides = json.loads(overrides)
            except json.JSONDecodeError as exc:
                self.get_logger().warn(f'Invalid setup request override: {exc}. Ignoring it.')
                overrides = None
        if isinstance(overrides, dict):
            setup_request.update(overrides)

        staging = setup_request.pop('staging_area', None)
        if not isinstance(staging, dict):
            staging = {
                'entity_name': self.staging_entity_name,
                'pose': self.staging_area_pose,
                'respawn': self.respawn_vehicle_at_staging_area,
            }
        staging_entity_name = str(staging.get('entity_name', '')).strip()
        staging_pose = self.parse_pose_values(staging.get('pose'))
        if staging_entity_name and staging_pose:
            entity_pose = {
                'name': staging_entity_name,
                'pose': staging_pose,
            }
            entity_poses = setup_request.get('entity_poses', [])
            if not isinstance(entity_poses, list):
                entity_poses = []
            entity_poses = [
                item
                for item in entity_poses
                if not isinstance(item, dict) or item.get('name') != staging_entity_name
            ]
            setup_request['entity_poses'] = [*entity_poses, entity_pose]
            if bool(staging.get('respawn', False)):
                respawn_entities = setup_request.get('respawn_entities', [])
                if isinstance(respawn_entities, str):
                    respawn_entities = [respawn_entities]
                setup_request['respawn_entities'] = list(
                    dict.fromkeys([*respawn_entities, staging_entity_name])
                )

        return setup_request

    def publish_setup_request(self, overrides: dict | str | None = None):
        setup_request = self.build_setup_request(overrides)

        setup_request_msg = String()
        setup_request_msg.data = json.dumps(setup_request)
        self.awaiting_setup_status_for_request = True
        self.setup_request_started = False
        self.current_setup_state = None
        self.setup_request_publisher.publish(setup_request_msg)
        self.get_logger().info(f'Published setup request: {setup_request_msg.data}')

    @staticmethod
    def parse_pose_values(value: Any) -> list[float]:
        if value is None or value == '':
            return []
        if isinstance(value, str):
            raw_values = value.replace(',', ' ').split()
        else:
            raw_values = list(value)
        try:
            pose = [float(item) for item in raw_values]
        except (TypeError, ValueError):
            return []
        return pose if len(pose) in (6, 7) else []

    def load_json_list(
        self, json_path: str, keys: tuple[str, ...], description: str
    ) -> list[dict]:
        if not json_path:
            self.get_logger().warn(f'No {description} JSON path was provided.')
            return []
        if not os.path.exists(json_path):
            self.get_logger().error(f'{description.capitalize()} file not found: {json_path}')
            return []
        try:
            with open(json_path, 'r', encoding='utf-8') as config_file:
                data = json.load(config_file)
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().error(f'Failed to load {description} from {json_path}: {exc}')
            return []
        for key in keys:
            entries = data.get(key)
            if isinstance(entries, list):
                return entries
        self.get_logger().error(
            f'{description.capitalize()} file has none of the expected keys: {list(keys)}'
        )
        return []

    @staticmethod
    def normalize_test_configuration(configuration: dict) -> tuple[dict, Any, int]:
        if not isinstance(configuration, dict):
            return {}, {}, 1

        test_profile = configuration.get('test_profile')
        if not isinstance(test_profile, dict):
            test_profile = configuration

        setup_conditions = configuration.get('setup_conditions', configuration.get('setup', {}))
        iterations = configuration.get('iterations', test_profile.get('iterations', 1))
        try:
            iterations = max(int(iterations), 1)
        except (TypeError, ValueError):
            iterations = 1

        return test_profile, setup_conditions, iterations

    def expand_test_configuration_runs(self, configurations: list[dict]):
        for configuration_index, configuration in enumerate(configurations, start=1):
            test_profile, setup_conditions, iterations = self.normalize_test_configuration(
                configuration
            )
            for iteration_index in range(iterations):
                yield (
                    configuration_index,
                    configuration,
                    test_profile,
                    setup_conditions,
                    iteration_index + 1,
                    iterations,
                )

    def parse_topic_perturbation_config(
        self, perturbation_config: dict[str, Any], profile_name: str, parse_trigger_states
    ) -> list[dict[str, Any]]:
        perturbation_channels = []
        for channel_name, channel_config in perturbation_config.items():
            channel = str(channel_name).strip()
            if channel not in self.topic_perturbation_channels:
                self.get_logger().error(
                    f"Skipping profile '{profile_name}': unknown topic perturbation channel "
                    f"'{channel}'. Available channels: {self.topic_perturbation_channels}"
                )
                return []
            if not isinstance(channel_config, dict):
                self.get_logger().error(
                    f"Skipping profile '{profile_name}': topic_perturbation."
                    f'{channel} must be an object'
                )
                return []

            missing_fields = []
            if 'activation_trigger' not in channel_config:
                missing_fields.append(f'topic_perturbation.{channel}.activation_trigger')
            if 'duration_s' not in channel_config:
                missing_fields.append(f'topic_perturbation.{channel}.duration_s')
            if missing_fields:
                self.get_logger().error(
                    f"Skipping profile '{profile_name}': missing required fields {missing_fields}"
                )
                return []

            activation_triggers = parse_trigger_states(
                channel_config['activation_trigger'],
                profile_name,
                f'topic_perturbation.{channel}.activation_trigger',
            )
            if not activation_triggers:
                self.get_logger().error(
                    f"Skipping profile '{profile_name}': "
                    f'topic_perturbation.{channel}.activation_trigger contains '
                    'no valid trigger conditions'
                )
                return []

            try:
                duration_s = float(channel_config['duration_s'])
            except (TypeError, ValueError):
                self.get_logger().error(
                    f"Skipping profile '{profile_name}': "
                    f'topic_perturbation.{channel}.duration_s must be numeric'
                )
                return []
            if duration_s <= 0.0:
                self.get_logger().error(
                    f"Skipping profile '{profile_name}': "
                    f'topic_perturbation.{channel}.duration_s must be positive'
                )
                return []

            start_delay_s = channel_config.get('start_delay_s', 0.0)
            try:
                start_delay_s = float(start_delay_s)
            except (TypeError, ValueError):
                self.get_logger().error(
                    f"Skipping profile '{profile_name}': "
                    f'topic_perturbation.{channel}.start_delay_s must be numeric'
                )
                return []
            if start_delay_s < 0.0:
                self.get_logger().error(
                    f"Skipping profile '{profile_name}': "
                    f'topic_perturbation.{channel}.start_delay_s must be non-negative'
                )
                return []

            perturbation_profile = dict(channel_config)
            perturbation_profile.pop('activation_trigger', None)
            perturbation_profile.pop('duration_s', None)
            perturbation_channels.append(
                {
                    'channel': channel,
                    'duration_s': duration_s,
                    'start_delay_s': start_delay_s,
                    'activation_triggers': activation_triggers,
                    'profile': perturbation_profile,
                }
            )

        return perturbation_channels

    def load_waywise_route(
        self, route_path: str, target_enuref: list[float]
    ) -> PathWithTwists | None:
        if not route_path:
            return None
        if not os.path.exists(route_path):
            self.get_logger().warn(f'Preplanned route file not found: {route_path}')
            return None

        try:
            root = ET.parse(route_path).getroot()
        except (ET.ParseError, OSError) as exc:
            self.get_logger().error(f'Could not read route XML: {exc}')
            return None

        route_msg = PathWithTwists()
        route_msg.path = Path()
        route_msg.path.header.frame_id = 'map'
        imported_enuref = list(target_enuref)
        enuref_element = root.find('enuref') if root.tag == 'routes' else None
        if enuref_element is not None:
            for index, name in enumerate(('Latitude', 'Longitude', 'Height')):
                element = enuref_element.find(name)
                if element is not None and element.text:
                    imported_enuref[index] = float(element.text)

        route_elements = root.findall('route') if root.tag == 'routes' else [root]
        for route in route_elements:
            for point in route.findall('point'):
                values = {}
                for name in ('x', 'y', 'z', 'speed'):
                    element = point.find(name)
                    values[name] = (
                        float(element.text) if element is not None and element.text else 0.0
                    )
                latitude, longitude, height = pm.enu2geodetic(
                    values['x'], values['y'], values['z'], *imported_enuref
                )
                x, y, z = pm.geodetic2enu(latitude, longitude, height, *target_enuref)
                pose = PoseStamped()
                pose.header.frame_id = route_msg.path.header.frame_id
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = z
                pose.pose.orientation.w = 1.0
                twist = Twist()
                twist.linear.x = values['speed']
                route_msg.path.poses.append(pose)
                route_msg.twists.append(twist)

        self.populate_route_orientations(route_msg)
        self.get_logger().info(
            f'Loaded route {route_path} with {len(route_msg.path.poses)} waypoint(s).'
        )
        return route_msg

    @staticmethod
    def populate_route_orientations(route_msg: PathWithTwists):
        poses = route_msg.path.poses
        if len(poses) < 2:
            return
        for index, pose in enumerate(poses):
            start = poses[index if index < len(poses) - 1 else index - 1].pose.position
            end = poses[index + 1 if index < len(poses) - 1 else index].pose.position
            yaw = math.atan2(end.y - start.y, end.x - start.x)
            quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

    def start_rosbag_recording(
        self,
        topics_to_record: list[str],
        bag_label: str,
        process_label: str = 'test_runner_rosbag',
    ):
        if not self.use_rosbag_recording or not topics_to_record:
            return

        resolved_topics = [self.resolve_topic_name(topic) for topic in topics_to_record]
        os.makedirs(self.rosbag_output_dir, exist_ok=True)
        bag_path = os.path.join(self.rosbag_output_dir, bag_label)
        self.current_rosbag_path = bag_path
        command = ['ros2', 'bag', 'record', '-o', bag_path]
        if self.use_sim_time:
            command.append('--use-sim-time')
        command.extend(resolved_topics)

        self.subprocesses['rosbag'] = ProcessUtils.create_subprocess(self, command, process_label)

    def stop_rosbag_recording(self, delete_recording: bool = False):
        bag_path = self.current_rosbag_path
        ProcessUtils.cleanup_subprocesses(self.subprocesses)
        self.current_rosbag_path = None

        if not delete_recording or not bag_path:
            return

        if not os.path.exists(bag_path):
            return

        try:
            shutil.rmtree(bag_path)
            self.get_logger().info(f'Deleted incomplete rosbag: {bag_path}')
        except OSError as exc:
            self.get_logger().warn(f'Failed to delete incomplete rosbag {bag_path}: {exc}')

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
    def slugify(value: str, fallback: str = 'test_run') -> str:
        slug = re.sub(r'[^a-zA-Z0-9]+', '_', value).strip('_').lower()
        return slug or fallback
