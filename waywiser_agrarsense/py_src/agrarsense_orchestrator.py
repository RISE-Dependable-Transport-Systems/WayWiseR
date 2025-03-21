#!/usr/bin/env python3

import json
import os
import re
import signal
import subprocess
import time

import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import LivelinessPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool
from std_msgs.msg import String


class AgrarsenseOrchestrator(Node):
    """ROS2 node that orchestrates the Agrarsense simulator and ROS bridge."""

    def __init__(self):
        super().__init__('agrarsense_orchestrator_node')

        # Declare parameters
        self.declare_parameter('sim_script_path', '')
        self.declare_parameter('ros_bridge_script_path', '')
        self.declare_parameter('sim_in_command_topic', '/agrarsense/in/commands')
        self.declare_parameter('docker_container_name', 'agrarsense_ros_bridge')
        self.declare_parameter('ego_vehicle_identifier', '')
        self.declare_parameter('sim_startup_time', 2.0)
        self.declare_parameter('map_name', 'playground')
        self.declare_parameter('objects_json_path', '')
        self.declare_parameter('command_publish_delay', 2.0)
        self.declare_parameter('ros_async', True)
        self.declare_parameter('ros_host_ip', '127.0.0.1')
        self.declare_parameter('workersthreadpool', 16)
        self.declare_parameter('spectator_rendering', True)
        self.declare_parameter('quality_level', 'Balanced')
        self.declare_parameter('spectator_transform', '')
        self.declare_parameter('sim_configurations_json_path', '')
        self.declare_parameter('rosbag_output_dir', '')
        self.declare_parameter('use_rosbag_recording', False)

        # Get parameters
        self.sim_script_path = (
            self.get_parameter('sim_script_path').get_parameter_value().string_value
        )
        self.ros_bridge_script_path = (
            self.get_parameter('ros_bridge_script_path').get_parameter_value().string_value
        )
        self.sim_in_command_topic = (
            self.get_parameter('sim_in_command_topic').get_parameter_value().string_value
        )
        self.docker_container_name = (
            self.get_parameter('docker_container_name').get_parameter_value().string_value
        )
        self.ego_vehicle_identifier = (
            self.get_parameter('ego_vehicle_identifier').get_parameter_value().string_value
        )
        self.sim_startup_time = (
            self.get_parameter('sim_startup_time').get_parameter_value().double_value
        )
        self.map_name = self.get_parameter('map_name').get_parameter_value().string_value
        self.objects_json_path = (
            self.get_parameter('objects_json_path').get_parameter_value().string_value
        )
        self.command_publish_delay = (
            self.get_parameter('command_publish_delay').get_parameter_value().double_value
        )
        self.ros_async = self.get_parameter('ros_async').get_parameter_value().bool_value
        self.ros_host_ip = self.get_parameter('ros_host_ip').get_parameter_value().string_value
        self.workersthreadpool = (
            self.get_parameter('workersthreadpool').get_parameter_value().integer_value
        )
        self.spectator_rendering = (
            self.get_parameter('spectator_rendering').get_parameter_value().bool_value
        )
        self.quality_level = self.get_parameter('quality_level').get_parameter_value().string_value
        self.spectator_transform = (
            self.get_parameter('spectator_transform').get_parameter_value().string_value
        )
        self.sim_configurations_json_path = (
            self.get_parameter('sim_configurations_json_path').get_parameter_value().string_value
        )
        self.rosbag_output_dir = (
            self.get_parameter('rosbag_output_dir').get_parameter_value().string_value
        )
        self.use_rosbag_recording = (
            self.get_parameter('use_rosbag_recording').get_parameter_value().bool_value
        )

        # Parse the simulation configurations
        self.sim_configurations = self.parse_sim_configs()
        self.current_config_index = 0
        self.current_iter_index = 0
        self.subprocesses = {}

        self.waywiser_tempdir = os.path.expanduser('~/.waywiser/agrarsense')
        os.makedirs(self.waywiser_tempdir, exist_ok=True)

        # Create subscribers
        self.clock_subscriber = self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        self.end_simulation_subscriber = self.create_subscription(
            Bool, '/agrarsense/end_simulation', self.end_simulation_callback, 10
        )

        self.info_subscriber = self.create_subscription(
            String,
            '/agrarsense/out/info',
            self.info_callback,
            10,  # QoS profile depth
        )

        # Create publishers
        self.simulation_ready_publisher = self.create_publisher(
            Bool, '/carla/simulation_ready', 10
        )
        command_publisher_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            lifespan=rclpy.duration.Duration(seconds=0),
            deadline=rclpy.duration.Duration(seconds=0),
            liveliness=LivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=rclpy.duration.Duration(seconds=0),
        )
        self.command_publisher = self.create_publisher(
            String, self.sim_in_command_topic, command_publisher_qos_profile
        )

        # Flags for controlling simulator actions
        self.sim_paused = False
        self.previous_sim_time = None
        self.spawn_objects_started = False
        self.spawn_objects_pending = set()

        self.start_agrarsense_ros_bridge()

        self.start_next_simulation()

    def info_callback(self, msg):
        message_data = msg.data
        # self.get_logger().info(f'Agrarsense info: {msg.data}')

        if self.spawn_objects_pending:
            match = re.search(r'Spawned Sensor:.*?\n.*?ID: ([^\s]+)', message_data)
            if match:
                spawned_id = match.group(1)

                for obj_id in list(self.spawn_objects_pending):
                    if obj_id == spawned_id:
                        self.spawn_objects_pending.remove(obj_id)
                        # self.get_logger().info(f"Object '{obj_id}' spawn confirmed.")

                if not self.spawn_objects_pending:
                    self.get_logger().info('All objects spawned successfully.')
                    self.start_sim_execution()

    def parse_sim_configs(self):
        """Parse the raw simulation configurations from the YAML parameter."""
        parsed_configs = []

        # Load sim_configurations_json_file
        if self.sim_configurations_json_path:
            try:
                with open(self.sim_configurations_json_path, 'r') as f:
                    sim_configurations = json.load(f).get('sim_configurations', [])
                    # Extract spawn points for vehicle.* types
                    for sim_config in sim_configurations:
                        parsed_config = {
                            'iterations': sim_config.get('iterations', 1),
                            'spawn_point': sim_config.get('spawn_point', {}),
                            'objects_json_path': sim_config.get(
                                'objects_json_path', self.objects_json_path
                            ),
                            'topics_to_record': sim_config.get('topics_to_record', []),
                            'ego_vehicle_identifier': sim_config.get(
                                'ego_vehicle_identifier', self.ego_vehicle_identifier
                            ),
                        }
                        parsed_configs.append(parsed_config)
            except Exception as e:
                self.get_logger().error(f'Failed to load objects JSON file: {e}')

        return parsed_configs

    def create_subprocess(
        self,
        command,
        subprocess_name,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        text=False,
    ):
        subprocess_ = subprocess.Popen(
            command,
            start_new_session=True,
            stdout=stdout,
            stderr=stderr,
            text=text,
        )
        self.get_logger().info(f'Started {subprocess_name} with PID [{subprocess_.pid}].')
        return subprocess_

    def start_simulator(self):
        """Start the Agrarsense simulator."""
        start_simulator_command = [self.sim_script_path]
        start_simulator_command.append(f'--{self.map_name}')
        if self.workersthreadpool > 0:
            start_simulator_command.append(f'--workersthreadpool={self.workersthreadpool}')
        if not self.spectator_rendering:
            start_simulator_command.append('--no-spectator-rendering')
        if not self.ros_async:
            start_simulator_command.append('--no-ros-async')
        if self.ros_host_ip != '':
            start_simulator_command.append(f'--ros-host-ip={self.ros_host_ip}')
        if self.quality_level != '':
            start_simulator_command.append(f'--quality-level={self.quality_level}')

        subprocess_name = 'simulator'
        self.subprocesses[subprocess_name] = self.create_subprocess(
            start_simulator_command, subprocess_name
        )
        self.get_logger().info(
            'Waiting for the first clock message.'
        )  # will wait for the first clock message to spawn objects

    def start_agrarsense_ros_bridge(self):
        """Start the agrarsense ros bridge."""
        command = [self.ros_bridge_script_path]
        command.append('--container-name')
        command.append(f'{self.docker_container_name}')

        subprocess_name = 'agrarsense_ros_bridge'
        self.agrarsense_ros_bridge = self.create_subprocess(command, subprocess_name)
        time.sleep(3.0)  # Wait for the bridge to start

    def start_rosbag_recording(self, topics_to_record):
        """Start the rosbag_recording node."""
        command = ['ros2', 'bag', 'record', '--use-sim-time']
        if self.rosbag_output_dir:
            output_dir = self.rosbag_output_dir + '/' + str(int(time.time()))
            if not os.path.exists(self.rosbag_output_dir):
                os.makedirs(self.rosbag_output_dir)
            command += ['--output', output_dir]
        command += topics_to_record

        subprocess_name = 'ros_bag_recorder'
        self.subprocesses[subprocess_name] = self.create_subprocess(command, subprocess_name)

    def publish_command(self, command):
        """Publish a command to the simulator."""
        command_msg = String()
        command_msg.data = command
        self.command_publisher.publish(command_msg)
        self.get_logger().info(f'Published command: {command}')
        time.sleep(self.command_publish_delay)

    def spawn_objects(self, sim_config):
        self.spawn_objects_pending.clear()
        update_objects_json_path = False
        if self.objects_json_path:
            try:
                with open(self.objects_json_path, 'r') as file:
                    file_data = json.load(file)

                    for obj in file_data.get('objects', []):
                        obj_info_id = obj['id']
                        if obj.get('type', '') == 'vehicle':
                            obj_info_id = obj_info_id + '/transform'
                        self.spawn_objects_pending.add(obj_info_id)

                    if not sim_config.get('spawn_point', {}):
                        self.publish_command(f'SpawnObjects {self.objects_json_path}')
                    else:
                        update_objects_json_path = True

            except FileNotFoundError:
                self.get_logger().error(f'Object file not found: {self.objects_json_path}')
            except json.JSONDecodeError:
                self.get_logger().error(f'Error decoding JSON in file: {self.objects_json_path}')

            if update_objects_json_path:
                for obj in file_data.get('objects', []):
                    obj_id = obj.get('id')
                    spawn_point_override = sim_config.get('spawn_point', {}).get(obj_id)

                    if spawn_point_override:
                        # Split the string into components and map to spawnPoint keys
                        x, y, z, roll, pitch, yaw = map(float, spawn_point_override.split(','))
                        obj['spawnPoint'].update(
                            {'x': x, 'y': y, 'z': z, 'roll': roll, 'pitch': pitch, 'yaw': yaw}
                        )

                updated_json_path = os.path.join(
                    self.waywiser_tempdir,
                    f'{os.path.splitext(os.path.basename(self.objects_json_path))[0]}_'
                    f'{self.current_config_index}.json',
                )
                try:
                    with open(updated_json_path, 'w') as file:
                        json.dump(file_data, file, indent=4)
                    self.get_logger().info(f'Updated objects JSON saved to {updated_json_path}')

                    self.publish_command(f'SpawnObjects {updated_json_path}')
                except Exception as e:
                    self.get_logger().error(f'Failed to write updated JSON: {e}')

    def clock_callback(self, msg):
        current_sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

        if self.previous_sim_time is None:
            self.get_logger().info('Received first clock message.')
            if current_sim_time < self.sim_startup_time:
                self.get_logger().info('Waiting for simulator to startup before spawning objects.')
        elif current_sim_time < self.previous_sim_time:  # Detect clock reset
            self.spawn_objects_started = False
            self.get_logger().info('Detected clock reset!')

        # Check if we should spawn the objects
        if (not self.spawn_objects_started) and (current_sim_time >= self.sim_startup_time):
            self.spawn_objects_started = True

            self.spawn_objects(self.sim_configurations[self.current_config_index])

        # Update the previous time
        self.previous_sim_time = current_sim_time

    def end_simulation_callback(self, msg):
        if msg.data:
            self.get_logger().info(
                f'Ending simulation with index [{self.current_config_index}-'
                f'{self.current_iter_index}].'
            )
            self.cleanup_subprocesses()
            self.current_iter_index += 1
            if (
                self.current_iter_index
                >= self.sim_configurations[self.current_config_index]['iterations']
            ):
                self.current_config_index += 1
                self.current_iter_index = 0

            self.spawn_objects_started = False
            self.start_next_simulation()

    def start_next_simulation(self):
        """Load the next simulation configuration and starts the simulation."""
        if self.current_config_index >= len(self.sim_configurations):
            self.get_logger().info('All simulations completed.')
            self.destroy_node()
            return

        self.get_logger().info(
            f'Executing simulation with index [{self.current_config_index}-'
            f'{self.current_iter_index}].'
        )

        # Setup the simulator
        self.start_simulator()

    def start_sim_execution(self):
        sim_config = self.sim_configurations[self.current_config_index]

        if self.spectator_transform != '':
            self.publish_command(f'TeleportSpectator {self.spectator_transform}')

        # start rosbag recording
        topics_to_record = sim_config['topics_to_record']
        if self.use_rosbag_recording and len(topics_to_record) > 0:
            self.start_rosbag_recording(topics_to_record)

        # publish that simulation is ready
        msg = Bool()
        msg.data = True
        self.simulation_ready_publisher.publish(msg)
        self.get_logger().info('Published that simultion is ready for execution.')

    def pause_simulation(self):
        """Command to pause the simulation."""
        if not self.sim_paused:
            self.publish_command('pause_simulation')
            self.sim_paused = True
            self.get_logger().info('Simulation paused.')

    def unpause_simulation(self):
        """Command to unpause the simulation."""
        if self.sim_paused:
            self.publish_command('unpause_simulation')
            self.sim_paused = False
            self.get_logger().info('Simulation unpaused.')

    def end_simulation(self):
        """Command to end the simulation and terminate the simulator process."""
        self.publish_command('quit')
        self.get_logger().info('Ending simulation and shutting down simulator.')

    def destroy_node(self):
        """Override to ensure the simulator process is terminated on shutdown."""
        try:
            if self.context.ok():
                self.get_logger().info('Shutting down AgrarsenseOrchestrator node.')
            self.cleanup_subprocesses()
            self.terminate_process(self.agrarsense_ros_bridge)

            subprocess.run(['docker', 'rm', '-f', self.docker_container_name], check=True)
            self.get_logger().info(f'Docker container {self.docker_container_name} removed.')
        except subprocess.CalledProcessError as e:
            self.get_logger().warn(
                f'Error removing Docker container {self.docker_container_name}: {e}'
            )
        except Exception as e:
            self.get_logger().warn(f'Error during node destruction: {e}')
        finally:
            super().destroy_node()

    def terminate_process(self, process):
        """Force terminate a process and its children using psutil."""
        try:
            process_ps = psutil.Process(process.pid)
            child_ps = process_ps.children(recursive=True)
            log_string = f'Terminating process with PID [{process.pid}]'
            if len(child_ps) > 0:
                log_string += f' and its child processes {[child.pid for child in child_ps]}'
            self.get_logger().info(log_string)

            # Send SIGTERM to the entire process group
            pgid = os.getpgid(process.pid)  # Get the process group ID
            os.killpg(pgid, signal.SIGTERM)  # Send SIGTERM to the process group

            # Wait for the main process to terminate
            process_ps.wait(timeout=10)

            # Forcefully kill remaining processes if still running
            if process_ps.is_running():
                process_ps.kill()
            for child in child_ps:
                if child.is_running():
                    child.kill()
        except psutil.NoSuchProcess:
            self.get_logger().info(
                f'Process with PID [{process.pid}] does not exist or already terminated.'
            )
        except psutil.TimeoutExpired:
            self.get_logger().warn(
                f'Timed out waiting for process [{process.pid}] to terminate. Forcing kill.'
            )
            process_ps.kill()
        except Exception as e:
            self.get_logger().warn(f'Failed to terminate process {process.pid}: {e}')

    def cleanup_subprocesses(self):
        """Cleanup subprocesses."""
        for id_ in list(self.subprocesses.keys()):
            try:
                process = self.subprocesses.pop(id_)
                self.terminate_process(process)
            except Exception as e:
                self.get_logger().warn(f'Error cleaning up subprocess {id}: {e}')


def main(args=None):
    rclpy.init(args=args)

    # Initialize and run the AgrarsenseOrchestrator node
    agrarsense_orchestrator_node = AgrarsenseOrchestrator()

    try:
        rclpy.spin(agrarsense_orchestrator_node)
    except KeyboardInterrupt:
        agrarsense_orchestrator_node.get_logger().info('User requested shutdown with SIGINT.')
    finally:
        # Cleanup on exit
        try:
            agrarsense_orchestrator_node.destroy_node()
        except Exception as e:
            agrarsense_orchestrator_node.get_logger().warn(f'Error during node destruction: {e}')
        # Only shutdown if the context is still valid
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
