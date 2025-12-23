#!/usr/bin/env python3

from enum import auto, Enum
import json
import os
import re
import subprocess
import time

from ament_index_python import get_package_share_directory
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rosgraph_msgs.msg import Clock as ClockMsg
from std_msgs.msg import String

from waywiser_py.waywiser_utils import (
    create_subprocess,
    get_full_file_path,
    RELIABLE_TRANSIENT_LOCAL_QOS,
    terminate_subprocess,
)
from waywiser_test_runner.msg import SetupState

PACKAGE_NAME = 'waywiser_agrarsense'
WAYWISER_AGRARSENSE_TEMPDIR = os.path.expanduser('~/.waywiser/agrarsense')
os.makedirs(WAYWISER_AGRARSENSE_TEMPDIR, exist_ok=True)


class AgrarsenseSimulatorState(Enum):
    UNKNOWN = auto()
    INITIALIZING = auto()
    INITIALIZED = auto()
    CONFIGURING = auto()
    CONFIGURED = auto()


class AgrarsenseOrchestrator(Node):
    """ROS2 node that orchestrates the Agrarsense simulator and ROS bridge."""

    def __init__(self):
        super().__init__('agrarsense_orchestrator_node')

        # Declare parameters
        self.declare_parameter('agrarsense_script_path', '')
        self.declare_parameter('ego_vehicle_role_name', '')
        self.declare_parameter('ros_host_ip', '127.0.0.1')
        self.declare_parameter('ros_port', 9090)
        self.declare_parameter('map_name', 'playground')
        self.declare_parameter('sim_startup_time', 2.0)
        self.declare_parameter('render_off_screen', True)
        self.declare_parameter('reset_agrarsense_after_exec', False)
        self.declare_parameter('objects_json_path', '')
        self.declare_parameter('agrarsense_orchestrator_timer_rate', 1.0)

        self.declare_parameter('agrarsense_ros_bridge.script_path', '')
        self.declare_parameter(
            'agrarsense_ros_bridge.docker_container_name', 'agrarsense_ros_bridge'
        )

        self.declare_parameter('agrarsense_in_command_topic', '/agrarsense/in/commands')
        self.declare_parameter('command_publish_delay', 2.0)
        self.declare_parameter('workersthreadpool', 16)
        self.declare_parameter('quality_level', 'Balanced')

        self.declare_parameter('spectator_transform', '')
        self.declare_parameter('object_ids', [''])
        self.declare_parameter('setup_request_topic', '/setup_request')
        self.declare_parameter('setup_status_topic', '/setup_status')
        self.declare_parameter('weather_json_path', '')
        self.declare_parameter('sim_clock_timeout', 10.0)

        # Get parameters
        self.object_ids = self.get_parameter('object_ids').get_parameter_value().string_array_value
        self.spawn_point = {}
        for object_id in self.object_ids:
            self.declare_parameter(f'spawn_point.{object_id}', '')
            self.spawn_point[object_id] = (
                self.get_parameter(f'spawn_point.{object_id}').get_parameter_value().string_value
            )

        self.agrarsense_script_path = get_full_file_path(
            self.get_parameter('agrarsense_script_path').get_parameter_value().string_value
        )
        self.ego_vehicle_role_name = (
            self.get_parameter('ego_vehicle_role_name').get_parameter_value().string_value
        )
        self.ros_host_ip = self.get_parameter('ros_host_ip').get_parameter_value().string_value
        self.ros_port = self.get_parameter('ros_port').get_parameter_value().integer_value
        self.map_name = self.get_parameter('map_name').get_parameter_value().string_value
        self.sim_startup_time = (
            self.get_parameter('sim_startup_time').get_parameter_value().double_value
        )
        self.render_off_screen = (
            self.get_parameter('render_off_screen').get_parameter_value().bool_value
        )
        self.reset_agrarsense_after_exec = (
            self.get_parameter('reset_agrarsense_after_exec').get_parameter_value().bool_value
        )
        self.objects_json_path = (
            self.get_parameter('objects_json_path').get_parameter_value().string_value
        )
        self.agrarsense_orchestrator_timer_rate = (
            self.get_parameter('agrarsense_orchestrator_timer_rate')
            .get_parameter_value()
            .double_value
        )

        self.agrarsense_ros_bridge_params = {
            'script_path': get_full_file_path(
                self.get_parameter('agrarsense_ros_bridge.script_path')
                .get_parameter_value()
                .string_value,
                os.path.join(get_package_share_directory(PACKAGE_NAME), 'ros_bridge'),
            ),
            'docker_container_name': self.get_parameter(
                'agrarsense_ros_bridge.docker_container_name'
            )
            .get_parameter_value()
            .string_value,
        }

        self.agrarsense_in_command_topic = (
            self.get_parameter('agrarsense_in_command_topic').get_parameter_value().string_value
        )
        self.command_publish_delay = (
            self.get_parameter('command_publish_delay').get_parameter_value().double_value
        )
        self.workersthreadpool = (
            self.get_parameter('workersthreadpool').get_parameter_value().integer_value
        )
        self.quality_level = self.get_parameter('quality_level').get_parameter_value().string_value
        self.spectator_transform = (
            self.get_parameter('spectator_transform').get_parameter_value().string_value
        )
        self.setup_request_topic = (
            self.get_parameter('setup_request_topic').get_parameter_value().string_value
        )
        self.setup_status_topic = (
            self.get_parameter('setup_status_topic').get_parameter_value().string_value
        )
        self.weather_json_path = (
            self.get_parameter('weather_json_path').get_parameter_value().string_value
        )
        self.weather_json_path = get_full_file_path(
            self.weather_json_path,
            os.path.join(get_package_share_directory(PACKAGE_NAME), 'config'),
        )
        self.sim_clock_timeout = (
            self.get_parameter('sim_clock_timeout').get_parameter_value().double_value
        )

        # Create subscribers
        self.out_info_group = MutuallyExclusiveCallbackGroup()
        self.setup_request_subscriber = self.create_subscription(
            String,
            self.setup_request_topic,
            self.setup_request_callback,
            RELIABLE_TRANSIENT_LOCAL_QOS,
        )
        self.agrarsense_out_info_subscriber = self.create_subscription(
            String,
            '/agrarsense/out/info',
            self.agrarsense_out_info_callback,
            10,
            callback_group=self.out_info_group,
        )
        self.sim_clock_subscriber = self.create_subscription(
            ClockMsg, '/clock', self.sim_clock_callback, 10
        )
        # Create publishers
        self.setup_status_publisher = self.create_publisher(
            SetupState, self.setup_status_topic, RELIABLE_TRANSIENT_LOCAL_QOS
        )
        self.agrarsense_in_command_publisher = self.create_publisher(
            String, self.agrarsense_in_command_topic, RELIABLE_TRANSIENT_LOCAL_QOS
        )

        # Create timers
        self.wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.agrarsense_orchestrator_timer = self.create_timer(
            1.0 / self.agrarsense_orchestrator_timer_rate,
            self.agrarsense_orchestrator_timer_callback,
            clock=self.wall_clock,
        )

        # Initialize Attributes
        self.setup_state_name_lookup = {
            value: name for name, value in SetupState.__dict__.items() if isinstance(value, int)
        }
        self.first_clock_received_time = None
        self.consecutive_stable_clocks_at_startup = 0
        self.simulator_subprocess = None
        self.ros_bridge_subprocess = None
        self.setup_status = SetupState()
        self.setup_status.state = SetupState.IDLE
        self.simulation_config_to_process = {}
        self.configured_map_name = None
        self.configured_objects_json_path = None
        self.configured_ego_vehicle_role_name = None
        self.previous_sim_time = None
        self.spawn_objects_pending = set()
        self.spawn_objects_completed_future = None
        self.last_command_publish_time = None
        self.sim_state = AgrarsenseSimulatorState.UNKNOWN
        self.sim_clock_timeout_timer = None

        self.start_agrarsense_ros_bridge()

    def sim_clock_callback(self, msg):
        current_sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

        # Check if we have a previous time for comparison
        if self.previous_sim_time is not None and self.first_clock_received_time is None:
            time_diff = abs(current_sim_time - self.previous_sim_time)

            # Check if time difference is within threshold
            if time_diff <= 0.1:
                self.consecutive_stable_clocks_at_startup += 1
            else:
                # Reset counter if time difference exceeds threshold
                self.consecutive_stable_clocks_at_startup = 0

            # Set first clock received time after 3 consecutive stable messages
            if self.consecutive_stable_clocks_at_startup >= 3:
                self.first_clock_received_time = current_sim_time
                self.get_logger().warn(
                    f'Received first stable clock message at time {current_sim_time}.'
                )

        if self.first_clock_received_time is not None:
            # Handle clock reset detection
            if current_sim_time < self.previous_sim_time:  # Detect clock reset
                self.first_clock_received_time = None  # Reset first clock time
                self.consecutive_stable_clocks_at_startup = 0  # Reset counter
                self.get_logger().warn('Detected clock reset!')
                return

            # Check if we've passed the simulator startup time
            if (
                self.sim_state == AgrarsenseSimulatorState.INITIALIZING
                and current_sim_time - self.first_clock_received_time >= self.sim_startup_time
            ):
                # self.sim_started_future.set_result(True)
                self.update_sim_state(AgrarsenseSimulatorState.INITIALIZED)
                if self.sim_clock_timeout_timer is not None:
                    self.sim_clock_timeout_timer.cancel()
                    self.sim_clock_timeout_timer = None

        # Update the previous time
        self.previous_sim_time = current_sim_time

    def agrarsense_orchestrator_timer_callback(self):
        match self.sim_state:
            case AgrarsenseSimulatorState.UNKNOWN:
                self.restart_simulation()
            case AgrarsenseSimulatorState.INITIALIZED:
                self.configure_simulation()
            case AgrarsenseSimulatorState.CONFIGURING:
                if self.spawn_objects_pending:
                    self.get_logger().info(
                        f'Spawning objects pending: {self.spawn_objects_pending}'
                    )
            case _:
                pass

        match self.setup_status.state:
            case SetupState.SETUP_INIT:
                self.setup_status_publisher.publish(self.setup_status)
                self.setup_status.state = SetupState.SETUP_ONGOING
                self.setup_status_publisher.publish(self.setup_status)
                self.restart_simulation()
            case SetupState.SETUP_ONGOING:
                if self.sim_state == AgrarsenseSimulatorState.CONFIGURED:
                    self.setup_status.state = SetupState.SETUP_COMPLETED
            case SetupState.SETUP_COMPLETED:
                self.setup_status_publisher.publish(self.setup_status)
                self.setup_status.state = SetupState.IDLE
                self.simulation_config_to_process = {}
            case _:
                pass

    def update_sim_state(self, state: AgrarsenseSimulatorState):
        self.sim_state = state
        self.get_logger().info(f'Simulator state: {state.name}.')

    def end_simulation(self):
        if (
            self.reset_agrarsense_after_exec
            and self.simulator_subprocess is not None
            or self.sim_state == AgrarsenseSimulatorState.INITIALIZING
        ):
            terminate_subprocess(self.simulator_subprocess)
            self.simulator_subprocess = None
            self.sim_state = AgrarsenseSimulatorState.UNKNOWN
        elif self.sim_state in [
            AgrarsenseSimulatorState.INITIALIZED,
            AgrarsenseSimulatorState.CONFIGURING,
            AgrarsenseSimulatorState.CONFIGURED,
        ]:
            self.publish_command('quit')

    def restart_simulation(self):
        if self.sim_clock_timeout_timer is not None:
            self.sim_clock_timeout_timer.cancel()
            self.sim_clock_timeout_timer = None
            self.get_logger().warn('Simulation clock timeout reached. Restarting simulation.')

        self.end_simulation()

        if self.reset_agrarsense_after_exec or self.simulator_subprocess is None:
            self.initialize_simulation()
        elif self.sim_state in [
            AgrarsenseSimulatorState.CONFIGURING,
            AgrarsenseSimulatorState.CONFIGURED,
        ]:
            self.sim_state = AgrarsenseSimulatorState.INITIALIZED

    def setup_request_callback(self, msg):
        if (
            self.setup_status.state == SetupState.IDLE
            or self.setup_status.state == SetupState.SETUP_COMPLETED
        ):
            self.simulation_config_to_process = json.loads(msg.data)
            if 'objects_json_path' in self.simulation_config_to_process:
                self.simulation_config_to_process['objects_json_path'] = get_full_file_path(
                    self.simulation_config_to_process['objects_json_path'],
                    os.path.join(get_package_share_directory(PACKAGE_NAME), 'config'),
                )
            self.get_logger().info(
                f'Processing setup request: {self.simulation_config_to_process}'
            )
            self.setup_status.state = SetupState.SETUP_INIT

    def agrarsense_out_info_callback(self, msg):
        message_data = msg.data

        if self.spawn_objects_pending:
            match = re.search(r'Spawned Sensor:.*?\n.*?ID: ([^\s]+)', message_data)
            if match:
                spawned_id = match.group(1)

                for obj_id in list(self.spawn_objects_pending):
                    if obj_id == spawned_id:
                        self.spawn_objects_pending.remove(obj_id)

                if not self.spawn_objects_pending:
                    self.get_logger().info('All objects spawned successfully.')
                    self.update_sim_state(AgrarsenseSimulatorState.CONFIGURED)

    def start_agrarsense_ros_bridge(self):
        """Start the agrarsense ros bridge."""
        command = [self.agrarsense_ros_bridge_params['script_path']]
        command.append('--container-name')
        command.append(f'{self.agrarsense_ros_bridge_params["docker_container_name"]}')

        subprocess_name = 'agrarsense_ros_bridge'
        self.ros_bridge_subprocess = create_subprocess(self, command, subprocess_name)
        time.sleep(5.0)  # Wait for the bridge to start

    def initialize_simulation(self):
        """Start the Agrarsense simulator."""
        start_simulator_command = [self.agrarsense_script_path]
        # start_simulator_command.append(f'--{self.map_name}')
        # self.configured_map_name = self.map_name
        if self.workersthreadpool > 0:
            start_simulator_command.append(f'--workersthreadpool={self.workersthreadpool}')
        if self.render_off_screen:
            start_simulator_command.append('--no-spectator-rendering')
        if self.ros_host_ip != '':
            start_simulator_command.append(f'--ros-host-ip={self.ros_host_ip}')
        if self.ros_port > 0:
            start_simulator_command.append(f'--ros-port={self.ros_port}')
        if self.quality_level != '':
            start_simulator_command.append(f'--quality-level={self.quality_level}')

        subprocess_name = 'simulator'
        self.simulator_subprocess = create_subprocess(
            self, start_simulator_command, subprocess_name
        )
        self.get_logger().warn(
            'Waiting for the first clock message.'
        )  # will wait for the first clock message to spawn objects
        self.update_sim_state(AgrarsenseSimulatorState.INITIALIZING)
        if self.sim_clock_timeout_timer is None:
            self.sim_clock_timeout_timer = self.create_timer(
                self.sim_clock_timeout,
                self.restart_simulation,
                clock=self.wall_clock,
            )

    def publish_command(self, command, silent=False):
        """Publish a command to the simulator."""
        if self.command_publish_delay > 0.0 and self.last_command_publish_time is not None:
            elapsed_time = time.time() - self.last_command_publish_time
            remaining_delay = self.command_publish_delay - elapsed_time

            # Only wait if the remaining delay is positive
            if remaining_delay > 0:
                time.sleep(remaining_delay)

        command_msg = String()
        command_msg.data = command
        self.agrarsense_in_command_publisher.publish(command_msg)

        if self.command_publish_delay > 0.0:
            self.last_command_publish_time = time.time()

        if not silent:
            self.get_logger().info(f'Published command: {command}')

    def configure_simulation(self, _=None):
        self.update_sim_state(AgrarsenseSimulatorState.CONFIGURING)
        # configure map
        map_name = self.map_name
        if 'map_name' in self.simulation_config_to_process:
            map_name = self.simulation_config_to_process['map_name']
        if map_name != self.configured_map_name:
            self.publish_command(f'loadmap {map_name}')
        self.configured_map_name = map_name

        # configure weather
        weather_json_path = self.weather_json_path
        if 'weather_json_path' in self.simulation_config_to_process:
            weather_json_path = self.simulation_config_to_process['weather_json_path']
            weather_json_path = get_full_file_path(
                weather_json_path,
                os.path.join(get_package_share_directory(PACKAGE_NAME), 'config'),
            )

        if weather_json_path != '':
            self.publish_command(f'SpawnObjects {weather_json_path}')

        # spawn objects
        objects_json_path = self.objects_json_path
        if 'objects_json_path' in self.simulation_config_to_process:
            objects_json_path = self.simulation_config_to_process['objects_json_path']
        objects_json_path = get_full_file_path(
            objects_json_path, os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')
        )
        if objects_json_path == '':
            self.get_logger().warn(
                f'Invalid objects_json_path: {objects_json_path}. Skipping spawn_objects.'
            )
            return

        spawn_objects_config = {}
        spawn_objects_config['objects_json_path'] = objects_json_path

        spawn_point = self.spawn_point
        if 'spawn_point' in self.simulation_config_to_process:
            spawn_point = self.simulation_config_to_process['spawn_point']
        spawn_objects_config['spawn_point'] = spawn_point

        ego_vehicle_role_name = self.ego_vehicle_role_name
        if 'ego_vehicle_role_name' in self.simulation_config_to_process:
            ego_vehicle_role_name = self.simulation_config_to_process['ego_vehicle_role_name']
        spawn_objects_config['ego_vehicle_role_name'] = ego_vehicle_role_name

        self.spawn_objects_pending.clear()
        objects_json_path = spawn_objects_config.get('objects_json_path')
        if objects_json_path:
            update_objects_json_path = False
            try:
                with open(objects_json_path, 'r', encoding='utf-8') as file:
                    file_data = json.load(file)

                    for obj in file_data.get('objects', []):
                        obj_info_id = obj['id']
                        if obj.get('type', '') in ['vehicle', 'Walker']:
                            obj_info_id = obj_info_id + '/transform'
                        self.spawn_objects_pending.add(obj_info_id)

                    if not spawn_objects_config.get('spawn_point', {}):
                        self.publish_command(f'SpawnObjects {objects_json_path}')
                        self.get_logger().info(
                            f'Spawning objects pending: {self.spawn_objects_pending}'
                        )
                    else:
                        update_objects_json_path = True
            except FileNotFoundError:
                self.get_logger().error(f'Object file not found: {objects_json_path}')
            except json.JSONDecodeError:
                self.get_logger().error(f'Error decoding JSON in file: {objects_json_path}')

            if update_objects_json_path:
                for obj in file_data.get('objects', []):
                    obj_id = obj.get('id')
                    spawn_point_override = spawn_objects_config.get('spawn_point', {}).get(obj_id)

                    if spawn_point_override:
                        # Split the string into components and map to spawnPoint keys
                        x, y, z, roll, pitch, yaw = map(float, spawn_point_override.split(','))
                        obj['spawnPoint'].update(
                            {'x': x, 'y': y, 'z': z, 'roll': roll, 'pitch': pitch, 'yaw': yaw}
                        )

                updated_json_path = os.path.join(
                    WAYWISER_AGRARSENSE_TEMPDIR,
                    f'{os.path.splitext(os.path.basename(objects_json_path))[0]}.json',
                )
                try:
                    with open(updated_json_path, 'w', encoding='utf-8') as file:
                        json.dump(file_data, file, indent=4)
                    # self.get_logger().info(f'Updated objects JSON saved to {updated_json_path}')
                    self.publish_command(f'SpawnObjects {updated_json_path}')
                    self.get_logger().info(
                        f'Spawning objects pending: {self.spawn_objects_pending}'
                    )
                except Exception as e:
                    self.get_logger().error(f'Failed to write updated JSON: {e}')

            self.configured_objects_json_path = objects_json_path
            self.configured_ego_vehicle_role_name = ego_vehicle_role_name

            # TODO: clear spawn_objects_pending based on feedback from simulator
            self.spawn_objects_pending.clear()
            self.update_sim_state(AgrarsenseSimulatorState.CONFIGURED)
        else:
            self.get_logger().info('Configured simulator.')
            self.update_sim_state(AgrarsenseSimulatorState.CONFIGURED)

    def destroy_node(self):
        """Override to ensure the simulator process is terminated on shutdown."""
        docker_container_name = self.agrarsense_ros_bridge_params['docker_container_name']
        try:
            print('Shutting down carla_orchestrator node.')
            terminate_subprocess(self.simulator_subprocess)
            terminate_subprocess(self.ros_bridge_subprocess)

            subprocess.run(['docker', 'rm', '-f', docker_container_name], check=True)
            print(f'Docker container {docker_container_name} removed.')
        except subprocess.CalledProcessError as e:
            print(f'Error removing Docker container {docker_container_name}: {e}')
        except Exception as e:
            print(f'Error during node destruction: {e}')
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    # Initialize and run the AgrarsenseOrchestrator node
    agrarsense_orchestrator_node = AgrarsenseOrchestrator()
    executor = MultiThreadedExecutor(num_threads=16)
    executor.add_node(agrarsense_orchestrator_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        agrarsense_orchestrator_node.get_logger().info('User requested shutdown with SIGINT.')
    finally:
        # Cleanup on exit
        try:
            agrarsense_orchestrator_node.destroy_node()
        except Exception as e:
            print(f'Error during node destruction: {e}')
        # Only shutdown if the context is still valid
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
