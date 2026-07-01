#!/usr/bin/env python3

import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

from ament_index_python import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from waywiser_py.waywiser_utils import RELIABLE_TRANSIENT_LOCAL_QOS
from waywiser_test_runner.msg import SetupState


class GazeboOrchestratorNode(Node):
    """Minimal Gazebo reset orchestrator for repeatable validation runs."""

    def __init__(self):
        super().__init__('gazebo_orchestrator_node')

        self.declare_parameter('world_name', 'bounded_world')
        self.declare_parameter('setup_request_topic', '/setup_request')
        self.declare_parameter('setup_status_topic', '/setup_status')
        self.declare_parameter('reset_all', False)
        self.declare_parameter('service_timeout_ms', 5000)
        self.declare_parameter('settle_time_sec', 4.0)
        self.declare_parameter('pause_gazebo_during_setup', True)
        self.declare_parameter('manage_px4_process', False)
        self.declare_parameter('start_px4_on_startup', True)
        self.declare_parameter('restart_px4_on_setup', True)
        self.declare_parameter('respawn_managed_px4_entity_on_setup', False)
        self.declare_parameter('skip_managed_px4_set_pose_after_model_reset', True)
        self.declare_parameter('clear_actuators_on_setup', True)
        self.declare_parameter(
            'actuator_zero_topics',
            ['/model/{model_name}/command/motor_speed', '/{model_name}/command/motor_speed'],
        )
        self.declare_parameter('actuator_zero_count', 4)
        self.declare_parameter('px4_start_delay_sec', 0.0)
        self.declare_parameter('px4_shutdown_timeout_sec', 5.0)
        self.declare_parameter('px4_command_json', '[]')
        self.declare_parameter('px4_working_directory', '')
        self.declare_parameter('px4_environment_json', '{}')
        self.declare_parameter('spawn_config_file', '')
        self.declare_parameter('spawn_on_startup', False)
        self.declare_parameter('spawn_on_setup', False)
        self.declare_parameter('spawn_start_delay_sec', 0.0)
        self.declare_parameter('spawn_interval_sec', 1.0)
        self.declare_parameter('spawn_backend', 'gz_service')
        self.declare_parameter('start_gazebo_bridge', True)
        self.declare_parameter('gz_service_suppress_output', False)

        self.world_name = self.get_parameter('world_name').get_parameter_value().string_value
        self.setup_request_topic = (
            self.get_parameter('setup_request_topic').get_parameter_value().string_value
        )
        self.setup_status_topic = (
            self.get_parameter('setup_status_topic').get_parameter_value().string_value
        )
        self.reset_all = self.get_parameter('reset_all').get_parameter_value().bool_value
        self.service_timeout_ms = (
            self.get_parameter('service_timeout_ms').get_parameter_value().integer_value
        )
        self.settle_time_sec = (
            self.get_parameter('settle_time_sec').get_parameter_value().double_value
        )
        self.pause_gazebo_during_setup = (
            self.get_parameter('pause_gazebo_during_setup').get_parameter_value().bool_value
        )
        self.manage_px4_process = (
            self.get_parameter('manage_px4_process').get_parameter_value().bool_value
        )
        self.start_px4_on_startup = (
            self.get_parameter('start_px4_on_startup').get_parameter_value().bool_value
        )
        self.restart_px4_on_setup = (
            self.get_parameter('restart_px4_on_setup').get_parameter_value().bool_value
        )
        self.respawn_managed_px4_entity_on_setup = (
            self.get_parameter('respawn_managed_px4_entity_on_setup')
            .get_parameter_value()
            .bool_value
        )
        self.skip_managed_px4_set_pose_after_model_reset = (
            self.get_parameter('skip_managed_px4_set_pose_after_model_reset')
            .get_parameter_value()
            .bool_value
        )
        self.clear_actuators_on_setup = (
            self.get_parameter('clear_actuators_on_setup').get_parameter_value().bool_value
        )
        self.actuator_zero_topics = list(
            self.get_parameter('actuator_zero_topics').get_parameter_value().string_array_value
        )
        self.actuator_zero_count = (
            self.get_parameter('actuator_zero_count').get_parameter_value().integer_value
        )
        self.px4_start_delay_sec = (
            self.get_parameter('px4_start_delay_sec').get_parameter_value().double_value
        )
        self.px4_shutdown_timeout_sec = (
            self.get_parameter('px4_shutdown_timeout_sec').get_parameter_value().double_value
        )
        self.px4_command = self.parse_json_parameter('px4_command_json', [])
        self.px4_working_directory = (
            self.get_parameter('px4_working_directory').get_parameter_value().string_value
        )
        self.px4_environment = self.parse_json_parameter('px4_environment_json', {})
        self.spawn_config_file = (
            self.get_parameter('spawn_config_file').get_parameter_value().string_value
        )
        self.spawn_on_startup = (
            self.get_parameter('spawn_on_startup').get_parameter_value().bool_value
        )
        self.spawn_on_setup = self.get_parameter('spawn_on_setup').get_parameter_value().bool_value
        self.spawn_start_delay_sec = (
            self.get_parameter('spawn_start_delay_sec').get_parameter_value().double_value
        )
        self.spawn_interval_sec = (
            self.get_parameter('spawn_interval_sec').get_parameter_value().double_value
        )
        self.spawn_backend = self.get_parameter('spawn_backend').get_parameter_value().string_value
        self.start_gazebo_bridge = (
            self.get_parameter('start_gazebo_bridge').get_parameter_value().bool_value
        )
        self.gz_service_suppress_output = (
            self.get_parameter('gz_service_suppress_output').get_parameter_value().bool_value
        )

        self.setup_status = SetupState()
        self.setup_status.state = SetupState.IDLE
        self.pending_request = None
        self.setup_started_at = None
        self.paused_setup_world_name = None
        self.px4_process = None
        self.px4_process_group_id = None
        self.px4_start_timer = None
        self.spawn_start_timer = None
        self.managed_spawn_processes = []
        self.entities_spawned_at_requested_pose = set()

        self.create_subscription(
            String,
            self.setup_request_topic,
            self.setup_request_callback,
            RELIABLE_TRANSIENT_LOCAL_QOS,
        )
        self.setup_status_publisher = self.create_publisher(
            SetupState, self.setup_status_topic, RELIABLE_TRANSIENT_LOCAL_QOS
        )
        self.create_timer(0.5, self.timer_callback)

        if self.manage_px4_process and self.start_px4_on_startup:
            self.px4_start_timer = self.create_timer(
                max(0.01, self.px4_start_delay_sec), self.start_px4_once
            )
        if self.spawn_on_startup and self.spawn_config_file:
            self.spawn_start_timer = self.create_timer(
                max(0.01, self.spawn_start_delay_sec), self.spawn_once
            )

    def setup_request_callback(self, msg: String):
        if self.setup_status.state not in (SetupState.IDLE, SetupState.SETUP_COMPLETED):
            self.get_logger().warn('Ignoring Gazebo setup request while setup is already active.')
            return

        try:
            self.pending_request = json.loads(msg.data) if msg.data else {}
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Invalid Gazebo setup request JSON: {exc}. Using defaults.')
            self.pending_request = {}

        self.get_logger().info(f'Processing Gazebo setup request: {self.pending_request}')
        self.setup_status.state = SetupState.SETUP_INIT
        self.setup_started_at = time.monotonic()

    def timer_callback(self):
        if self.setup_status.state == SetupState.IDLE:
            return

        self.setup_status_publisher.publish(self.setup_status)

        if self.setup_status.state == SetupState.SETUP_INIT:
            self.setup_status.state = SetupState.SETUP_ONGOING
            restart_px4 = self.should_restart_px4()
            request = self.pending_request if isinstance(self.pending_request, dict) else {}
            world_name = str(request.get('world_name', self.world_name)).strip() or self.world_name
            paused_for_setup = (
                self.pause_gazebo_during_setup
                and not self.request_reset_all()
                and (self.should_reset_gazebo() or bool(self.requested_entity_poses()))
            )
            if restart_px4:
                self.cancel_px4_start_timer()
                self.stop_px4()
            self.clear_requested_entity_actuators()
            if paused_for_setup:
                self.set_gazebo_paused(world_name, True)
                self.paused_setup_world_name = world_name
            if self.should_reset_gazebo():
                self.reset_gazebo()
                self.clear_requested_entity_actuators()
            for entity_name in self.requested_respawn_entities():
                if self.request_reset_all():
                    self.get_logger().info(
                        f'Full Gazebo reset requested; model {entity_name} will be respawned.'
                    )
                    continue
                if (
                    self.is_managed_px4_entity(entity_name)
                    and not self.should_respawn_managed_px4_entity()
                ):
                    self.get_logger().info(
                        f'Moving PX4 model {entity_name} instead of deleting it. '
                        'Gazebo can exit when a PX4-managed model is removed at runtime.'
                    )
                    continue
                self.remove_entity(entity_name)
            if self.should_spawn_on_setup():
                self.stop_managed_spawn_processes()
                self.spawn_models()
            self.apply_requested_entity_poses()
            if restart_px4:
                self.start_px4()
            self.setup_started_at = time.monotonic()
            return

        if self.setup_status.state == SetupState.SETUP_ONGOING:
            if (time.monotonic() - self.setup_started_at) >= self.settle_time_sec:
                self.unpause_setup_world()
                self.setup_status.state = SetupState.SETUP_COMPLETED
                self.get_logger().info('Gazebo setup completed.')
            return

        if self.setup_status.state == SetupState.SETUP_COMPLETED:
            self.unpause_setup_world()
            self.setup_status.state = SetupState.IDLE
            self.pending_request = None

    def reset_gazebo(self):
        request = self.pending_request if isinstance(self.pending_request, dict) else {}
        world_name = str(request.get('world_name', self.world_name)).strip() or self.world_name
        reset_all = self.request_reset_all()
        timeout_ms = int(request.get('service_timeout_ms', self.service_timeout_ms))

        if reset_all:
            reset_request = 'reset: {all: true}'
        else:
            reset_request = 'reset: {model_only: true}'

        command = [
            'gz',
            'service',
            '-s',
            f'/world/{world_name}/control',
            '--reqtype',
            'gz.msgs.WorldControl',
            '--reptype',
            'gz.msgs.Boolean',
            '--timeout',
            str(timeout_ms),
            '--req',
            reset_request,
        ]
        self.get_logger().info(f'Resetting Gazebo world {world_name} with {reset_request}.')
        try:
            result = subprocess.run(
                command,
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=max(1.0, timeout_ms / 1000.0 + 1.0),
            )
        except subprocess.TimeoutExpired:
            self.get_logger().error(f'Gazebo reset timed out for world {world_name}.')
            return
        except OSError as exc:
            self.get_logger().error(f'Could not run Gazebo reset command: {exc}')
            return

        if result.returncode != 0:
            self.get_logger().error(
                f'Gazebo reset failed with code {result.returncode}: {result.stderr.strip()}'
            )
            return

        output = result.stdout.strip()
        if output:
            self.get_logger().info(f'Gazebo reset response: {output}')

    def set_gazebo_paused(self, world_name: str, paused: bool):
        request = self.pending_request if isinstance(self.pending_request, dict) else {}
        timeout_ms = int(request.get('service_timeout_ms', self.service_timeout_ms))
        control_request = f'pause: {str(paused).lower()}'
        action = 'Pausing' if paused else 'Unpausing'
        command = [
            'gz',
            'service',
            '-s',
            f'/world/{world_name}/control',
            '--reqtype',
            'gz.msgs.WorldControl',
            '--reptype',
            'gz.msgs.Boolean',
            '--timeout',
            str(timeout_ms),
            '--req',
            control_request,
        ]
        self.get_logger().info(f'{action} Gazebo world {world_name}.')
        try:
            result = subprocess.run(
                command,
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=max(1.0, timeout_ms / 1000.0 + 1.0),
            )
        except subprocess.TimeoutExpired:
            self.get_logger().error(f'Gazebo pause control timed out for world {world_name}.')
            return
        except OSError as exc:
            self.get_logger().error(f'Could not run Gazebo pause control command: {exc}')
            return

        if result.returncode != 0:
            self.get_logger().error(
                f'Gazebo pause control failed with code {result.returncode}: '
                f'{result.stderr.strip()}'
            )
            return

        output = result.stdout.strip()
        if output:
            self.get_logger().info(f'Gazebo pause control response: {output}')

    def unpause_setup_world(self):
        if not self.paused_setup_world_name:
            return
        world_name = self.paused_setup_world_name
        self.paused_setup_world_name = None
        self.set_gazebo_paused(world_name, False)

    def should_restart_px4(self) -> bool:
        request = self.pending_request if isinstance(self.pending_request, dict) else {}
        return self.get_request_bool(
            request, 'restart_px4', self.manage_px4_process and self.restart_px4_on_setup
        )

    def should_reset_gazebo(self) -> bool:
        request = self.pending_request if isinstance(self.pending_request, dict) else {}
        return self.get_request_bool(request, 'reset_gazebo', True)

    def request_reset_all(self) -> bool:
        request = self.pending_request if isinstance(self.pending_request, dict) else {}
        return self.get_request_bool(request, 'reset_all', self.reset_all)

    def should_spawn_on_setup(self) -> bool:
        request = self.pending_request if isinstance(self.pending_request, dict) else {}
        respawn_entities = self.requested_respawn_entities()
        if respawn_entities and self.request_reset_all():
            return True
        respawn_spawn_required = any(
            not self.is_managed_px4_entity(name) or self.should_respawn_managed_px4_entity()
            for name in respawn_entities
        )
        if respawn_entities:
            return respawn_spawn_required
        return self.get_request_bool(request, 'spawn_models', self.spawn_on_setup)

    def requested_respawn_entities(self) -> list[str]:
        request = self.pending_request if isinstance(self.pending_request, dict) else {}
        entities = request.get('respawn_entities', [])
        if isinstance(entities, str):
            entities = [entities]
        return [str(name).strip() for name in entities if str(name).strip()]

    def requested_entity_poses(self) -> list[dict]:
        request = self.pending_request if isinstance(self.pending_request, dict) else {}
        entity_poses = request.get('entity_poses', [])
        return entity_poses if isinstance(entity_poses, list) else []

    def requested_entity_pose_for_name(self, name: str) -> list[str]:
        if not name:
            return []
        for entity_pose in self.requested_entity_poses():
            if not isinstance(entity_pose, dict):
                continue
            if str(entity_pose.get('name', '')).strip() != name:
                continue
            pose = self.pose_values(entity_pose.get('pose'))
            if len(pose) >= 6:
                return pose[:6]
        return []

    def effective_spawn_model_name(self, model: dict) -> str:
        name = str(model.get('name', '')).strip()
        if name:
            return name
        px4_model_name = self.managed_px4_model_name()
        if px4_model_name and self.requested_entity_pose_for_name(px4_model_name):
            return px4_model_name
        return ''

    def should_respawn_managed_px4_entity(self) -> bool:
        request = self.pending_request if isinstance(self.pending_request, dict) else {}
        return self.get_request_bool(
            request,
            'respawn_managed_px4_entity',
            self.respawn_managed_px4_entity_on_setup,
        )

    def should_clear_actuators_on_setup(self) -> bool:
        request = self.pending_request if isinstance(self.pending_request, dict) else {}
        return self.get_request_bool(request, 'clear_actuators', self.clear_actuators_on_setup)

    def should_skip_managed_px4_set_pose_after_model_reset(self, entity_name: str) -> bool:
        request = self.pending_request if isinstance(self.pending_request, dict) else {}
        skip_set_pose = self.get_request_bool(
            request,
            'skip_managed_px4_set_pose_after_model_reset',
            self.skip_managed_px4_set_pose_after_model_reset,
        )
        if not skip_set_pose:
            return False
        if self.request_reset_all() or not self.should_reset_gazebo():
            return False
        if not self.is_managed_px4_entity(entity_name):
            return False
        return not self.should_respawn_managed_px4_entity()

    def is_managed_px4_entity(self, entity_name: str) -> bool:
        px4_model_name = self.managed_px4_model_name()
        return bool(px4_model_name) and entity_name == px4_model_name

    def managed_px4_model_name(self) -> str:
        if not self.manage_px4_process or not isinstance(self.px4_environment, dict):
            return ''
        return str(self.px4_environment.get('PX4_GZ_MODEL_NAME', '')).strip()

    def apply_requested_entity_poses(self):
        request = self.pending_request if isinstance(self.pending_request, dict) else {}
        entity_poses = self.requested_entity_poses()
        world_name = str(request.get('world_name', self.world_name)).strip() or self.world_name
        for entity_pose in entity_poses:
            if not isinstance(entity_pose, dict):
                continue
            name = str(entity_pose.get('name', '')).strip()
            pose = self.pose_values(entity_pose.get('pose'))
            if not name or len(pose) < 6:
                self.get_logger().warn(f'Ignoring invalid setup entity pose: {entity_pose}')
                continue
            if name in self.entities_spawned_at_requested_pose:
                self.get_logger().info(
                    f'Gazebo model {name} was spawned at the requested staging pose.'
                )
                continue
            if self.should_skip_managed_px4_set_pose_after_model_reset(name):
                self.get_logger().info(
                    f'Skipping set_pose for PX4 model {name} after model-only reset. '
                    'The model reset restores the configured spawn pose and clears model dynamics.'
                )
                continue
            self.set_entity_pose(world_name, name, pose[:6])

    def clear_requested_entity_actuators(self):
        if not self.should_clear_actuators_on_setup():
            return

        entity_names = set(self.requested_respawn_entities())
        for entity_pose in self.requested_entity_poses():
            if isinstance(entity_pose, dict):
                name = str(entity_pose.get('name', '')).strip()
                if name:
                    entity_names.add(name)

        px4_model_name = self.managed_px4_model_name()
        if px4_model_name:
            entity_names.add(px4_model_name)

        for entity_name in sorted(entity_names):
            self.publish_zero_actuators(entity_name)

    def publish_zero_actuators(self, model_name: str):
        zero_count = max(0, int(self.actuator_zero_count))
        if zero_count == 0:
            return

        message = ' '.join('velocity: 0.0' for _ in range(zero_count))
        for topic_template in self.actuator_zero_topics:
            topic = str(topic_template).format(model_name=model_name)
            self.get_logger().info(f'Publishing zero actuator command on {topic}.')
            self.run_process(
                [
                    'gz',
                    'topic',
                    '-t',
                    topic,
                    '-m',
                    'gz.msgs.Actuators',
                    '-p',
                    message,
                ],
                wait=True,
                suppress_output=self.gz_service_suppress_output,
            )

    def remove_entity(self, name: str):
        request = self.pending_request if isinstance(self.pending_request, dict) else {}
        world_name = str(request.get('world_name', self.world_name)).strip() or self.world_name
        timeout_ms = int(request.get('service_timeout_ms', self.service_timeout_ms))
        command = [
            'gz',
            'service',
            '-s',
            f'/world/{world_name}/remove',
            '--reqtype',
            'gz.msgs.Entity',
            '--reptype',
            'gz.msgs.Boolean',
            '--timeout',
            str(timeout_ms),
            '--req',
            f'name: "{name}", type: MODEL',
        ]
        self.get_logger().info(f'Removing Gazebo model {name} for respawn.')
        self.run_process(command, wait=True, suppress_output=self.gz_service_suppress_output)

    def set_entity_pose(self, world_name: str, name: str, pose):
        quaternion = self.quaternion_from_rpy(float(pose[3]), float(pose[4]), float(pose[5]))
        request = (
            f'name: "{name}", position {{ x: {pose[0]} y: {pose[1]} z: {pose[2]} }}, '
            'orientation { '
            f'x: {quaternion[0]} y: {quaternion[1]} z: {quaternion[2]} w: {quaternion[3]} '
            '}'
        )
        self.get_logger().info(f'Moving Gazebo model {name} to staging pose {pose}.')
        self.run_process(
            [
                'gz',
                'service',
                '-s',
                f'/world/{world_name}/set_pose',
                '--reqtype',
                'gz.msgs.Pose',
                '--reptype',
                'gz.msgs.Boolean',
                '--timeout',
                str(self.service_timeout_ms),
                '--req',
                request,
            ],
            wait=True,
            suppress_output=self.gz_service_suppress_output,
        )

    def spawn_once(self):
        if self.spawn_start_timer is not None:
            self.destroy_timer(self.spawn_start_timer)
            self.spawn_start_timer = None
        self.spawn_models(apply_start_delay=False)

    def spawn_models(self, apply_start_delay=True):
        request = self.pending_request if isinstance(self.pending_request, dict) else {}
        config_file = str(request.get('spawn_config_file', self.spawn_config_file)).strip()
        if not config_file:
            return
        self.entities_spawned_at_requested_pose = set()

        world_name = str(request.get('world_name', self.world_name)).strip() or self.world_name
        start_delay = float(request.get('spawn_start_delay_sec', self.spawn_start_delay_sec))
        interval = float(request.get('spawn_interval_sec', self.spawn_interval_sec))
        backend = str(request.get('spawn_backend', self.spawn_backend)).strip() or 'gz_service'
        start_bridge = self.get_request_bool(
            request, 'start_gazebo_bridge', self.start_gazebo_bridge
        )
        suppress_output = self.get_request_bool(
            request, 'gz_service_suppress_output', self.gz_service_suppress_output
        )

        config_path = Path(config_file).expanduser().resolve()
        if not config_path.is_file():
            self.get_logger().error(f'Gazebo spawn config file not found: {config_path}')
            return

        try:
            with config_path.open('r', encoding='utf-8') as config_stream:
                config = json.load(config_stream)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid Gazebo spawn config JSON: {exc}')
            return

        config_dir = config_path.parent
        model_index = 0
        if apply_start_delay and start_delay > 0.0:
            time.sleep(start_delay)

        for model in config.get('sdf_models', []):
            self.spawn_sdf_model(
                model, config_dir, world_name, backend, start_bridge, suppress_output
            )
            model_index += 1
            self.sleep_between_spawns(interval, model_index)

        for model in config.get('topic_models', []):
            self.spawn_topic_model(model, world_name, start_bridge, config_dir)
            model_index += 1
            self.sleep_between_spawns(interval, model_index)

        for transform in config.get('static_transforms', []):
            self.start_static_transform(transform)

    @staticmethod
    def sleep_between_spawns(interval, model_index):
        if interval > 0.0 and model_index > 0:
            time.sleep(interval)

    def spawn_sdf_model(
        self, model, config_dir, world_name, backend, start_bridge, suppress_output
    ):
        path = model.get('path')
        if not path:
            return
        sdf_path = self.resolve_resource_path(path, config_dir)
        try:
            sdf_path = self.create_sdf_with_static_override(sdf_path, model.get('static'))
        except (OSError, ET.ParseError, RuntimeError, ValueError) as exc:
            self.get_logger().error(f'Could not prepare SDF spawn file {sdf_path}: {exc}')
            return

        name = self.effective_spawn_model_name(model)
        spawn_pose = self.requested_entity_pose_for_name(name)
        if spawn_pose and name:
            self.entities_spawned_at_requested_pose.add(name)
        else:
            spawn_pose = self.resolve_spawn_pose(sdf_path, model.get('pose'))
        if backend == 'ros_gz_sim':
            self.run_ros_gz_create(sdf_path, world_name, spawn_pose, name)
        else:
            self.run_gz_service_spawn(sdf_path, world_name, spawn_pose, name, suppress_output)

        bridge_config = model.get('bridge_config')
        if start_bridge and bridge_config:
            self.start_bridge(self.resolve_resource_path(bridge_config, config_dir), world_name)

    def spawn_topic_model(self, model, world_name, start_bridge, config_dir):
        topic = model.get('topic')
        if not topic:
            return
        arguments = ['-topic', str(topic), '-world', world_name]
        name = self.effective_spawn_model_name(model)
        pose = self.requested_entity_pose_for_name(name)
        if pose and name:
            self.entities_spawned_at_requested_pose.add(name)
        else:
            pose = self.pose_values(model.get('pose'))
        if name:
            arguments.extend(['-name', str(name)])
        if len(pose) >= 3:
            arguments.extend(['-x', pose[0], '-y', pose[1], '-z', pose[2]])
        if len(pose) >= 6:
            arguments.extend(['-R', pose[3], '-P', pose[4], '-Y', pose[5]])

        self.run_process(['ros2', 'run', 'ros_gz_sim', 'create', *arguments], wait=True)

        bridge_config = model.get('bridge_config')
        if start_bridge and bridge_config:
            self.start_bridge(self.resolve_resource_path(bridge_config, config_dir), world_name)

    def run_ros_gz_create(self, sdf_path, world_name, spawn_pose, name):
        arguments = [
            '-file',
            str(sdf_path),
            '-world',
            world_name,
            '-x',
            spawn_pose[0],
            '-y',
            spawn_pose[1],
            '-z',
            spawn_pose[2],
            '-R',
            spawn_pose[3],
            '-P',
            spawn_pose[4],
            '-Y',
            spawn_pose[5],
        ]
        if name:
            arguments.extend(['-name', str(name)])
        self.run_process(['ros2', 'run', 'ros_gz_sim', 'create', *arguments], wait=True)

    def run_gz_service_spawn(self, sdf_path, world_name, spawn_pose, name, suppress_output):
        pose = [float(value) for value in spawn_pose[:6]]
        quaternion = self.quaternion_from_rpy(pose[3], pose[4], pose[5])
        request_parts = [
            f'sdf_filename: "{Path(sdf_path).resolve()}"',
            'allow_renaming: false',
            (
                'pose { '
                f'position {{ x: {pose[0]} y: {pose[1]} z: {pose[2]} }} '
                'orientation { '
                f'x: {quaternion[0]} y: {quaternion[1]} z: {quaternion[2]} '
                f'w: {quaternion[3]} '
                '} '
                '}'
            ),
        ]
        if name:
            request_parts.insert(0, f'name: "{name}"')

        command = [
            'gz',
            'service',
            '-s',
            f'/world/{world_name}/create',
            '--reqtype',
            'gz.msgs.EntityFactory',
            '--reptype',
            'gz.msgs.Boolean',
            '--timeout',
            str(self.service_timeout_ms),
            '--req',
            ', '.join(request_parts),
        ]
        self.run_process(command, wait=True, suppress_output=suppress_output)

    def start_bridge(self, bridge_config_path, world_name):
        runtime_config = self.create_runtime_bridge_config(bridge_config_path, world_name)
        command = [
            'ros2',
            'run',
            'ros_gz_bridge',
            'parameter_bridge',
            '--ros-args',
            '-p',
            f'config_file:={runtime_config}',
            '-p',
            'use_sim_time:=true',
        ]
        self.run_process(command, wait=False)

    def start_static_transform(self, transform):
        frame_id = transform.get('frame_id')
        child_frame_id = transform.get('child_frame_id')
        if not frame_id or not child_frame_id:
            return
        pose = self.pose_values(transform.get('pose', [0, 0, 0, 0, 0, 0]))
        pose += ['0'] * (6 - len(pose))
        command = [
            'ros2',
            'run',
            'tf2_ros',
            'static_transform_publisher',
            '--x',
            pose[0],
            '--y',
            pose[1],
            '--z',
            pose[2],
            '--roll',
            pose[3],
            '--pitch',
            pose[4],
            '--yaw',
            pose[5],
            '--frame-id',
            str(frame_id),
            '--child-frame-id',
            str(child_frame_id),
            '--ros-args',
            '-p',
            'use_sim_time:=true',
        ]
        self.run_process(command, wait=False)

    def run_process(self, command, wait=False, suppress_output=False):
        self.get_logger().info(f'Running Gazebo orchestration command: {command}')
        stdout = subprocess.DEVNULL if suppress_output else None
        stderr = subprocess.DEVNULL if suppress_output else None
        timeout_sec = max(1.0, float(self.service_timeout_ms) / 1000.0 + 1.0)
        try:
            if wait:
                result = subprocess.run(
                    command,
                    check=False,
                    stdout=stdout,
                    stderr=stderr,
                    timeout=timeout_sec,
                )
                if result.returncode != 0:
                    self.get_logger().warn(
                        f'Gazebo orchestration command exited with code {result.returncode}: '
                        f'{command}'
                    )
            else:
                process = subprocess.Popen(
                    command, stdout=stdout, stderr=stderr, start_new_session=True
                )
                self.managed_spawn_processes.append(process)
        except subprocess.TimeoutExpired:
            self.get_logger().warn(
                f'Gazebo orchestration command timed out after {timeout_sec:0.1f} s: {command}'
            )
        except OSError as exc:
            self.get_logger().error(f'Could not run Gazebo orchestration command: {exc}')

    def stop_managed_spawn_processes(self):
        for process in self.managed_spawn_processes:
            self.terminate_process_tree(process, 'Gazebo orchestration subprocess', 2.0)
        self.managed_spawn_processes = []

    def resolve_spawn_pose(self, sdf_path, pose_override):
        pose = self.pose_values(pose_override)
        if pose:
            pose += ['0'] * (6 - len(pose))
            return pose[:6]

        try:
            tree = ET.parse(sdf_path)
            root = tree.getroot()
            actor_tag = root.find('actor')
            if actor_tag is not None and actor_tag.find('pose') is not None:
                values = self.pose_values(actor_tag.find('pose').text)
                values += ['0'] * (6 - len(values))
                return values[:6]
            model_tag = root.find('model')
            if model_tag is not None and model_tag.find('pose') is not None:
                values = self.pose_values(model_tag.find('pose').text)
                values += ['0'] * (6 - len(values))
                return values[:6]
        except (ET.ParseError, FileNotFoundError, AttributeError):
            pass

        return ['0', '0', '0', '0', '0', '0']

    @staticmethod
    def pose_values(pose):
        if pose is None:
            return []
        if isinstance(pose, str):
            values = pose.split()
        elif isinstance(pose, list):
            values = [str(value) for value in pose]
        else:
            values = []
        return values

    @staticmethod
    def quaternion_from_rpy(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def resolve_resource_path(self, path, base_dir):
        path = str(path)
        if path.startswith('package://'):
            package_path = path[len('package://') :]
            package_name, _, relative_path = package_path.partition('/')
            if not package_name or not relative_path:
                raise RuntimeError(f'Invalid package resource URI: {path}')
            return str(Path(get_package_share_directory(package_name)) / relative_path)

        candidate = Path(path).expanduser()
        if candidate.is_absolute() or candidate.exists():
            return str(candidate)
        return str(base_dir / candidate)

    def create_sdf_with_static_override(self, sdf_path, static):
        static = self.normalize_optional_bool(static)
        if static is None:
            return sdf_path

        tree = ET.parse(sdf_path)
        root = tree.getroot()
        model_tag = root.find('model')
        if model_tag is None:
            raise RuntimeError(f'Cannot set static on SDF without a <model> tag: {sdf_path}')

        static_tag = model_tag.find('static')
        if static_tag is None:
            static_tag = ET.Element('static')
            model_tag.insert(0, static_tag)
        static_tag.text = static

        temp_sdf = tempfile.NamedTemporaryFile(
            mode='wb',
            prefix='waywiser_orchestrator_spawn_',
            suffix='.sdf',
            delete=False,
        )
        with temp_sdf:
            tree.write(temp_sdf, encoding='utf-8', xml_declaration=True)

        return temp_sdf.name

    @staticmethod
    def normalize_optional_bool(value):
        if value is None:
            return None
        if isinstance(value, bool):
            return 'true' if value else 'false'
        if isinstance(value, str):
            value = value.strip().lower()
            if value in ('true', '1', 'yes', 'on'):
                return 'true'
            if value in ('false', '0', 'no', 'off'):
                return 'false'
        raise ValueError(f'static must be a boolean value, got: {value}')

    @staticmethod
    def create_runtime_bridge_config(bridge_config_path, world_name):
        import yaml

        with open(bridge_config_path, 'r', encoding='utf-8') as bridge_file:
            config = yaml.safe_load(bridge_file) or []

        if not isinstance(config, list):
            raise RuntimeError(f'Gazebo bridge config must be a list: {bridge_config_path}')

        for entry in config:
            gz_topic_name = entry.get('gz_topic_name')
            if isinstance(gz_topic_name, str):
                entry['gz_topic_name'] = gz_topic_name.format(world_name=world_name)

        temp_config = tempfile.NamedTemporaryFile(
            mode='w',
            prefix='waywiser_orchestrator_bridge_',
            suffix='.yaml',
            delete=False,
        )
        with temp_config:
            yaml.safe_dump(config, temp_config)

        return temp_config.name

    def start_px4_once(self):
        if self.px4_start_timer is not None:
            self.destroy_timer(self.px4_start_timer)
            self.px4_start_timer = None
        if self.setup_status.state not in (SetupState.IDLE, SetupState.SETUP_COMPLETED):
            self.get_logger().info('Skipping delayed PX4 startup because setup is active.')
            return
        self.start_px4()

    def cancel_px4_start_timer(self):
        if self.px4_start_timer is None:
            return
        self.get_logger().info('Cancelling pending delayed PX4 startup for setup reset.')
        self.destroy_timer(self.px4_start_timer)
        self.px4_start_timer = None

    def start_px4(self):
        if not self.manage_px4_process:
            return
        if self.px4_process is not None and self.px4_process.poll() is None:
            self.get_logger().info('PX4 process is already running.')
            return
        if not isinstance(self.px4_command, list) or not self.px4_command:
            self.get_logger().warn(
                'PX4 process management is enabled but px4_command_json is empty.'
            )
            return

        env = None
        if isinstance(self.px4_environment, dict):
            env = {
                **os.environ,
                **{str(k): str(v) for k, v in self.px4_environment.items()},
            }

        cwd = self.px4_working_directory or None
        self.get_logger().info(f'Starting PX4 process: {self.px4_command}')
        try:
            self.px4_process = subprocess.Popen(
                [str(item) for item in self.px4_command],
                cwd=cwd,
                env=env,
                start_new_session=True,
            )
            self.px4_process_group_id = os.getpgid(self.px4_process.pid)
        except OSError as exc:
            self.get_logger().error(f'Could not start PX4 process: {exc}')
            self.px4_process = None
            self.px4_process_group_id = None

    def stop_px4(self):
        if self.px4_process is None:
            if self.px4_process_group_id is not None:
                self.cleanup_process_group(self.px4_process_group_id, 'PX4 process')
                self.px4_process_group_id = None
            return
        if self.px4_process.poll() is not None:
            self.px4_process = None
            if self.px4_process_group_id is not None:
                self.cleanup_process_group(self.px4_process_group_id, 'PX4 process')
                self.px4_process_group_id = None
            return

        self.terminate_process_tree(
            self.px4_process,
            'PX4 process',
            max(0.1, self.px4_shutdown_timeout_sec),
            self.px4_process_group_id,
        )
        self.px4_process = None
        self.px4_process_group_id = None

    def destroy_node(self):
        self.unpause_setup_world()
        self.stop_managed_spawn_processes()
        self.stop_px4()
        super().destroy_node()

    def terminate_process_tree(
        self, process, label: str, timeout_sec: float, process_group_id: int | None = None
    ):
        if process is None or process.poll() is not None:
            if process_group_id is not None:
                self.cleanup_process_group(process_group_id, label)
            return

        self.get_logger().info(f'Stopping {label}.')
        if process_group_id is None:
            try:
                process_group_id = os.getpgid(process.pid)
            except ProcessLookupError:
                return
            except OSError as exc:
                self.get_logger().warn(f'Could not get {label} process group: {exc}.')
                process_group_id = None

        try:
            if process_group_id is not None:
                os.killpg(process_group_id, signal.SIGTERM)
            else:
                process.terminate()
        except ProcessLookupError:
            return
        except OSError as exc:
            self.get_logger().warn(
                f'Could not send SIGTERM to {label} process group: {exc}. '
                'Trying direct process termination.'
            )
            process.terminate()

        try:
            process.wait(timeout=timeout_sec)
            if process_group_id is not None:
                self.cleanup_process_group(process_group_id, label)
            return
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f'{label} did not exit after SIGTERM; sending SIGKILL.')

        try:
            if process_group_id is not None:
                os.killpg(process_group_id, signal.SIGKILL)
            else:
                process.kill()
        except ProcessLookupError:
            return
        except OSError as exc:
            self.get_logger().warn(
                f'Could not send SIGKILL to {label} process group: {exc}. '
                'Trying direct process kill.'
            )
            process.kill()

        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            self.get_logger().error(f'{label} still did not exit after SIGKILL.')

    def cleanup_process_group(self, process_group_id: int, label: str):
        try:
            os.killpg(process_group_id, signal.SIGTERM)
        except ProcessLookupError:
            return
        except OSError:
            return

        time.sleep(0.2)
        try:
            os.killpg(process_group_id, signal.SIGKILL)
        except ProcessLookupError:
            return
        except OSError as exc:
            self.get_logger().warn(f'Could not finish cleanup for {label} process group: {exc}.')

    @staticmethod
    def get_request_bool(request: dict, name: str, default: bool) -> bool:
        value = request.get(name, default)
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def parse_json_parameter(self, parameter_name, default):
        raw_value = self.get_parameter(parameter_name).get_parameter_value().string_value
        if not raw_value:
            return default
        try:
            return json.loads(raw_value)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(
                f'Invalid JSON in parameter {parameter_name}: {exc}. Using default.'
            )
            return default


def main(args=None):
    rclpy.init(args=args)
    node = GazeboOrchestratorNode()
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
