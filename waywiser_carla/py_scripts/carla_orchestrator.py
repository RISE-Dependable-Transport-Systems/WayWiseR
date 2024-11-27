#!/usr/bin/env python3

import json
import math
import os
import signal
import subprocess
import time

import carla
from geometry_msgs.msg import TransformStamped
import psutil
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf_transformations


class CarlaOrchestrator(Node):
    def __init__(self):
        super().__init__('carla_orchestrator_node')

        # Declare parameters
        self.declare_parameter('carla_script_path', '')
        self.declare_parameter('ego_vehicle_role_name', '')
        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 2000)
        self.declare_parameter('timeout', 2)
        self.declare_parameter('town', 'Town10HD_Opt')
        self.declare_parameter('sim_startup_time', 2.0)
        self.declare_parameter('render_off_screen', False)
        self.declare_parameter('reset_carla_after_exec', False)
        self.declare_parameter('weather', 'ClearNoon')
        self.declare_parameter('objects_json_path', '')
        self.declare_parameter('sim_configurations_json_path', '')
        self.declare_parameter('carla_ros_bridge.passive', True)
        self.declare_parameter('carla_ros_bridge.synchronous_mode', True)
        self.declare_parameter(
            'carla_ros_bridge.synchronous_mode_wait_for_vehicle_control_command', False
        )
        self.declare_parameter('carla_ros_bridge.fixed_delta_seconds', 0.01)
        self.declare_parameter('carla_ros_bridge.register_all_sensors', True)
        self.declare_parameter('rosbag_output_dir', '')
        self.declare_parameter('use_rosbag_recording', False)
        self.declare_parameter('static_tf_publishers', [''])

        # Get parameters
        self.carla_script_path = (
            self.get_parameter('carla_script_path').get_parameter_value().string_value
        )
        self.ego_vehicle_role_name = (
            self.get_parameter('ego_vehicle_role_name').get_parameter_value().string_value
        )
        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().integer_value
        self.town = self.get_parameter('town').get_parameter_value().string_value
        self.sim_startup_time = (
            self.get_parameter('sim_startup_time').get_parameter_value().double_value
        )
        self.render_off_screen = (
            self.get_parameter('render_off_screen').get_parameter_value().bool_value
        )
        self.reset_carla_after_exec = (
            self.get_parameter('reset_carla_after_exec').get_parameter_value().bool_value
        )
        self.weather = self.get_parameter('weather').get_parameter_value().string_value
        self.objects_json_path = (
            self.get_parameter('objects_json_path').get_parameter_value().string_value
        )
        self.sim_configurations_json_path = (
            self.get_parameter('sim_configurations_json_path').get_parameter_value().string_value
        )
        self.carla_ros_bridge_params = {
            'passive': self.get_parameter('carla_ros_bridge.passive')
            .get_parameter_value()
            .bool_value,
            'synchronous_mode': self.get_parameter('carla_ros_bridge.synchronous_mode')
            .get_parameter_value()
            .bool_value,
            'synchronous_mode_wait_for_vehicle_control_command': self.get_parameter(
                'carla_ros_bridge.synchronous_mode_wait_for_vehicle_control_command'
            )
            .get_parameter_value()
            .bool_value,
            'fixed_delta_seconds': self.get_parameter('carla_ros_bridge.fixed_delta_seconds')
            .get_parameter_value()
            .double_value,
            'register_all_sensors': self.get_parameter('carla_ros_bridge.register_all_sensors')
            .get_parameter_value()
            .bool_value,
        }
        self.carla_ros_bridge_params['ego_vehicle_role_name'] = self.ego_vehicle_role_name
        self.carla_ros_bridge_params['host'] = self.host
        self.carla_ros_bridge_params['timeout'] = self.timeout
        self.carla_ros_bridge_params['town'] = self.town

        self.rosbag_output_dir = (
            self.get_parameter('rosbag_output_dir').get_parameter_value().string_value
        )
        self.use_rosbag_recording = (
            self.get_parameter('use_rosbag_recording').get_parameter_value().bool_value
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

        # Parse the simulation configurations
        self.sim_configurations = self.parse_sim_configs()
        self.current_config_index = 0
        self.current_iter_index = 0
        self.subprocesses = {}

        # Create subscribers
        self.clock_subscriber = self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        self.end_simulation_subscriber = self.create_subscription(
            Bool, '/carla/end_simulation', self.end_simulation_callback, 10
        )
        # Create publishers
        self.simulation_ready_publisher = self.create_publisher(
            Bool, '/carla/simulation_ready', 10
        )

        # Flags for controlling simulator actions
        self.received_first_clock = False

        self.start_next_simulation()

    def parse_sim_configs(self):
        """Parse the raw simulation configurations from the YAML parameter."""
        weather_presets = {
            'ClearNoon': carla.WeatherParameters.ClearNoon,
            'WetCloudySunset': carla.WeatherParameters.WetCloudySunset,
        }
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
                            'weather': weather_presets.get(
                                sim_config.get('weather', self.weather),
                                carla.WeatherParameters.ClearNoon,
                            ),
                            'spawn_point': sim_config.get('spawn_point', {}),
                            'objects_json_path': sim_config.get(
                                'objects_json_path', self.objects_json_path
                            ),
                            'topics_to_record': sim_config.get('topics_to_record', []),
                            'ego_vehicle_role_name': sim_config.get(
                                'ego_vehicle_role_name', self.ego_vehicle_role_name
                            ),
                        }
                        parsed_configs.append(parsed_config)
            except Exception as e:
                self.get_logger().error(f'Failed to load objects JSON file: {e}')

        return parsed_configs

    def create_subprocess(
        self,
        start_simulator_command,
        subprocess_name,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        text=False,
    ):
        self.subprocesses[subprocess_name] = subprocess.Popen(
            start_simulator_command,
            start_new_session=True,
            stdout=stdout,
            stderr=stderr,
            text=text,
        )
        self.get_logger().info(
            f'Started {subprocess_name} with PID [{self.subprocesses[subprocess_name].pid}].'
        )

    def setup_simulator(self, weather):
        """Setup the CARLA simulator."""
        if self.carla_script_path:
            start_simulator_command = [self.carla_script_path]
            if self.town != '':
                start_simulator_command.append(f'/Game/Carla/Maps/{self.town}')
            start_simulator_command.append(f'-carla-port={self.port}')
            if self.render_off_screen:
                start_simulator_command.append('-RenderOffScreen')
            start_simulator_command.append('-vulkan')

            self.create_subprocess(start_simulator_command, 'simulator')
            time.sleep(self.sim_startup_time)

        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(self.timeout)
        carla_world = self.client.get_world()
        carla_map = carla_world.get_map()
        carla_map_name = carla_map.name.split('/')[-1]
        if self.town != carla_map_name:
            if 'simulator' in self.subprocesses:
                raise ValueError(
                    f'Could not start carla with town {self.town}. "{carla_map_name}" town is loaded.'
                )
            else:
                self.client.load_world(self.town)
                time.sleep(self.sim_startup_time)
                self.get_logger().info(f'Loaded map: {self.town}')
        else:
            self.get_logger().info(f'Simulator is ready with map: {carla_map_name}')

        # Set the weather in CARLA
        carla_world.set_weather(weather)

    def start_carla_ros_bridge(self):
        """Start the carla_ros_bridge node."""

        command = ['ros2', 'run', 'carla_ros_bridge', 'bridge', '--ros-args']
        for param, value in self.carla_ros_bridge_params.items():
            command.extend(['-p', f'{param}:={value}'])

        self.create_subprocess(command, 'carla_ros_bridge')

    def start_rosbag_recording(self, topics_to_record):
        """Start the rosbag_recording node."""
        command = ['ros2', 'bag', 'record', '--use-sim-time']
        if self.rosbag_output_dir:
            output_dir = self.rosbag_output_dir + '/' + str(int(time.time()))
            if not os.path.exists(self.rosbag_output_dir):
                os.makedirs(self.rosbag_output_dir)
            command += ['--output', output_dir]
        command += topics_to_record

        self.create_subprocess(command, 'ros_bag_recorder')

    def spawn_objects(self, sim_config, timeout=10):
        """Start the carla_spawn_objects node."""

        spawn_objects_params = {}
        spawn_objects_params['use_sim_time'] = True
        spawn_objects_params['objects_definition_file'] = sim_config['objects_json_path']
        for id_, value in sim_config['spawn_point'].items():
            spawn_objects_params['spawn_point_' + id_] = value

        command = [
            'ros2',
            'run',
            'carla_spawn_objects',
            'carla_spawn_objects',
            '--ros-args',
        ]
        for param, value in spawn_objects_params.items():
            command.extend(['-p', f'{param}:={value}'])

        self.create_subprocess(
            ['unbuffer'] + command,
            'carla_spawn_objects',
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

        # Tail the stdout and wait until all objects are spawned
        start_time = time.time()
        while True:
            output = self.subprocesses['carla_spawn_objects'].stdout.readline().strip()

            if output:
                # self.get_logger().info(output)
                if 'All objects spawned' in output:
                    self.get_logger().info('All objects are spawned.')
                    break
            elif self.subprocesses['carla_spawn_objects'].poll() is not None:
                # Process has ended, exit the loop
                self.get_logger().warn(
                    'carla_spawn_objects process ended before all objects are spawned'
                )
                break

            # Check for timeout
            if time.time() - start_time > timeout:
                self.get_logger().warn(
                    "Timeout reached while waiting for 'All objects spawned.' message."
                )
                break

            time.sleep(0.1)
        self.subprocesses['carla_spawn_objects'].stdout.close()

        if sim_config['ego_vehicle_role_name']:
            objects_json_path = sim_config['objects_json_path']
            try:
                with open(objects_json_path, 'r') as f:
                    objects = json.load(f).get('objects', [])
                    # Extract spawn points for vehicle.* types
                    for object_ in objects:
                        if sim_config['ego_vehicle_role_name'] in object_['id']:
                            sensors = object_['sensors']
                            for sensor in sensors:
                                if 'actor.pseudo.control' in sensor['type']:
                                    self.set_initial_pose(
                                        sim_config['ego_vehicle_role_name'], sensor['id']
                                    )
                                    self.get_logger().info(
                                        f"Initialized vehicle control for {sim_config['ego_vehicle_role_name']} with control id {sensor['id']}."
                                    )
                                    break
            except Exception as e:
                self.get_logger().error(f'Failed to load objects JSON file: {e}')

    def set_initial_pose(self, role_name, control_id):
        """Start the carla_set_initial_pose node."""

        command = [
            'ros2',
            'run',
            'carla_spawn_objects',
            'set_initial_pose',
            '--ros-args',
            '-p',
            f'role_name:={role_name}',
            '-p',
            f'control_id:={control_id}',
            '-p',
            'use_sim_time:=True',
        ]

        self.create_subprocess(command, 'carla_set_initial_pose')

    def clock_callback(self, msg):
        if not self.received_first_clock:
            self.received_first_clock = True
            self.execute_simulation()

    def end_simulation_callback(self, msg):
        if msg.data:
            self.get_logger().info(
                f'Ending simulation with index [{self.current_config_index}-{self.current_iter_index}].'
            )
            self.cleanup_subprocesses()
            self.current_iter_index += 1
            if (
                self.current_iter_index
                >= self.sim_configurations[self.current_config_index]['iterations']
            ):
                self.current_config_index += 1
                self.current_iter_index = 0

            self.received_first_clock = False
            self.start_next_simulation()

    def start_next_simulation(self):
        """Loads the next simulation configuration and starts the simulation."""
        if self.current_config_index >= len(self.sim_configurations):
            self.get_logger().info('All simulations completed.')
            self.destroy_node()
            return

        self.get_logger().info(
            f'Starting simulation with index [{self.current_config_index}-{self.current_iter_index}].'
        )
        sim_config = self.sim_configurations[self.current_config_index]

        # Setup the CARLA simulator and ros bridge
        self.setup_simulator(sim_config['weather'])
        self.start_carla_ros_bridge()

    def execute_simulation(self):
        sim_config = self.sim_configurations[self.current_config_index]
        self.get_logger().info(
            f'Starting simulation {self.current_config_index} with map {self.town}'
        )

        # spawn objects
        self.spawn_objects(sim_config)

        self.publish_static_tfs()

        # start rosbag recording
        topics_to_record = sim_config['topics_to_record']
        if self.use_rosbag_recording and len(topics_to_record) > 0:
            self.start_rosbag_recording(topics_to_record)

        # publish that simulation is ready
        msg = Bool()
        msg.data = True
        self.simulation_ready_publisher.publish(msg)
        self.get_logger().info('Published that simultion is ready for execution.')

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

    def terminate_process(self, process):
        """Forcefully terminate a process and its children using psutil."""
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
        for id in list(self.subprocesses.keys()):
            try:
                process = self.subprocesses.pop(id)
                self.terminate_process(process)
            except Exception as e:
                self.get_logger().warn(f'Error cleaning up subprocess {id}: {e}')

    def destroy_node(self):
        """Override to ensure the subprocesses are terminated on shutdown."""
        try:
            if self.context.ok():
                self.get_logger().info('Shutting down carla orchestrator node.')
            self.cleanup_subprocesses()
        except Exception as e:
            self.get_logger().warn(f'Error during node destruction: {e}')
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    # Initialize and run the CarlaOrchestrator node
    sim_orchestrator_node = CarlaOrchestrator()

    try:
        rclpy.spin(sim_orchestrator_node)
    except KeyboardInterrupt:
        sim_orchestrator_node.get_logger().info('User requested shutdown with SIGINT.')
    finally:
        # Cleanup on exit
        try:
            sim_orchestrator_node.destroy_node()
        except Exception as e:
            sim_orchestrator_node.get_logger().warn(f'Error during node destruction: {e}')
        # Only shutdown if the context is still valid
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
