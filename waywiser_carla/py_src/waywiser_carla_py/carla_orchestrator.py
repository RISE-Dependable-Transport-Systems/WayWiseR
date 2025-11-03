#!/usr/bin/env python3
import json
import math
import os
import subprocess
import time

from ament_index_python import get_package_share_directory
import carla
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf_transformations
from waywiser_py.waywiser_utils import cleanup_subprocesses
from waywiser_py.waywiser_utils import create_subprocess
from waywiser_py.waywiser_utils import get_full_file_path
from waywiser_py.waywiser_utils import RELIABLE_TRANSIENT_LOCAL_QOS
from waywiser_py.waywiser_utils import terminate_subprocess

from waywiser_test_runner.msg import SetupState

WEATHER_PRESETS = {
    'ClearNoon': carla.WeatherParameters.ClearNoon,
    'WetCloudySunset': carla.WeatherParameters.WetCloudySunset,
}
PACKAGE_NAME = 'waywiser_carla'


class CarlaOrchestrator(Node):
    """CarlaOrchestrator is a ROS2 node that manages the orchestration of the CARLA simulator."""

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
        self.declare_parameter('relativemousemode', False)
        self.declare_parameter('weather', 'ClearNoon')
        self.declare_parameter('objects_json_path', '')
        self.declare_parameter('carla_orchestrator_timer_rate', 1.0)
        self.declare_parameter('carla_ros_bridge.passive', True)
        self.declare_parameter('carla_ros_bridge.synchronous_mode', True)
        self.declare_parameter(
            'carla_ros_bridge.synchronous_mode_wait_for_vehicle_control_command', False
        )
        self.declare_parameter('carla_ros_bridge.fixed_delta_seconds', 0.01)
        self.declare_parameter('carla_ros_bridge.register_all_sensors', True)
        self.declare_parameter('static_tf_publishers', [''])
        self.declare_parameter('setup_request_topic', '/setup_request')
        self.declare_parameter('setup_status_topic', '/setup_status')
        self.declare_parameter('auto_start_simulation', False)

        self.declare_parameter('object_ids', [''])
        self.object_ids = self.get_parameter('object_ids').get_parameter_value().string_array_value

        # Get parameters
        self.spawn_point = {}
        # self.object_pose_topics = {}
        for object_id in self.object_ids:
            self.declare_parameter(f'spawn_point.{object_id}', '')
            self.spawn_point[object_id] = (
                self.get_parameter(f'spawn_point.{object_id}').get_parameter_value().string_value
            )
            # self.declare_parameter(f'object_pose_topics.{object_id}', '')
            # self.object_pose_topics[object_id] = (
            #     self.get_parameter(f'object_pose_topics.{object_id}')
            #     .get_parameter_value()
            #     .string_value
            # )

        self.carla_script_path = get_full_file_path(
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
        self.relativemousemode = (
            self.get_parameter('relativemousemode').get_parameter_value().bool_value
        )
        self.weather = self.get_parameter('weather').get_parameter_value().string_value
        self.objects_json_path = (
            self.get_parameter('objects_json_path').get_parameter_value().string_value
        )
        self.objects_json_path = get_full_file_path(
            self.objects_json_path,
            os.path.join(get_package_share_directory(PACKAGE_NAME), 'config'),
        )
        self.carla_orchestrator_timer_rate = (
            self.get_parameter('carla_orchestrator_timer_rate').get_parameter_value().double_value
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
        self.carla_ros_bridge_params['port'] = self.port
        self.carla_ros_bridge_params['timeout'] = self.timeout
        self.carla_ros_bridge_params['town'] = self.town

        self.auto_start_simulation = (
            self.get_parameter('auto_start_simulation').get_parameter_value().bool_value
        )

        self.setup_request_topic = (
            self.get_parameter('setup_request_topic').get_parameter_value().string_value
        )
        self.setup_status_topic = (
            self.get_parameter('setup_status_topic').get_parameter_value().string_value
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

        # Create subscribers
        self.setup_request_subscriber = self.create_subscription(
            String,
            self.setup_request_topic,
            self.setup_request_callback,
            RELIABLE_TRANSIENT_LOCAL_QOS,
        )

        # Create publishers
        self.setup_status_publisher = self.create_publisher(
            SetupState, self.setup_status_topic, RELIABLE_TRANSIENT_LOCAL_QOS
        )

        self.setup_state_name_lookup = {
            value: name for name, value in SetupState.__dict__.items() if isinstance(value, int)
        }
        if len(self.static_tf_publishers) > 0:
            self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Create timers
        self.carla_orchestrator_timer = self.create_timer(
            1.0 / self.carla_orchestrator_timer_rate,
            self.carla_orchestrator_timer_callback,
        )

        # Initialize Attributes
        self.subprocesses = {}
        self.simulator_subprocess = None
        self.setup_status = SetupState()
        self.setup_status.state = SetupState.IDLE
        self.simulation_config = {}
        self.configured_weather = None
        self.configured_objects_json_path = None
        self.configured_ego_vehicle_role_name = None

    def carla_orchestrator_timer_callback(self):
        if self.simulator_subprocess is None and self.auto_start_simulation:
            self.start_simulation()
            return

        match self.setup_status.state:
            case SetupState.SETUP_INIT:
                self.setup_status_publisher.publish(self.setup_status)
                self.setup_status.state = SetupState.SETUP_ONGOING
            case SetupState.SETUP_ONGOING:
                self.setup_status_publisher.publish(self.setup_status)
                # restart_simulation = True

                # if (
                #     self.configured_weather == self.simulation_config['weather']
                #     and self.configured_objects_json_path
                #     == self.simulation_config['objects_json_path']
                #     and self.configured_ego_vehicle_role_name
                #     == self.simulation_config['ego_vehicle_role_name']
                # ):
                #     spawn_point_poses = {}
                #     for object_id in self.simulation_config['spawn_point'].keys():
                #         spawn_point_pose = PoseStamped()
                #         x, y, z, yaw, pitch, roll = map(
                #             float,
                #             self.simulation_config['spawn_point'][object_id].split(','),
                #         )
                #         spawn_point_pose.pose.position.x = x
                #         spawn_point_pose.pose.position.y = y
                #         spawn_point_pose.pose.position.z = 0.0  # Ignore z
                #         q = tf_transformations.quaternion_from_euler(yaw, pitch, roll)
                #         spawn_point_pose.pose.orientation = Quaternion(
                #             x=q[0], y=q[1], z=q[2], w=q[3]
                #         )
                #         spawn_point_poses[object_id] = spawn_point_pose
                #     if all(
                #         are_poses_equal(
                #             self.object_poses[object_id], spawn_point_poses[object_id], 0.1
                #         )
                #         for object_id in spawn_point_poses.keys()
                #     ):
                #         restart_simulation = False

                # if restart_simulation:

                # End any ongoing simulation
                self.end_current_simulation()
                self.start_simulation()

                self.setup_status.state = SetupState.SETUP_COMPLETED
            case SetupState.SETUP_COMPLETED:
                self.setup_status_publisher.publish(self.setup_status)
                self.setup_status.state = SetupState.IDLE
                self.simulation_config = {}
            case _:
                pass

    def setup_request_callback(self, msg):
        if (
            self.setup_status.state == SetupState.IDLE
            or self.setup_status.state == SetupState.SETUP_COMPLETED
        ):
            self.simulation_config = json.loads(msg.data)
            if 'objects_json_path' in self.simulation_config:
                self.simulation_config['objects_json_path'] = get_full_file_path(
                    self.simulation_config['objects_json_path'],
                    os.path.join(get_package_share_directory(PACKAGE_NAME), 'config'),
                )
            self.get_logger().info(f'Processing setup request: {self.simulation_config}')
            self.setup_status.state = SetupState.SETUP_INIT

    def start_simulation(self):
        # Setup the CARLA simulator and ros bridge
        if (self.simulator_subprocess is None) or (self.reset_carla_after_exec):
            self.setup_simulator()
        self.start_carla_ros_bridge()

        weather_request = self.weather
        if 'weather' in self.simulation_config:
            weather_request = self.simulation_config['weather']
        if weather_request in WEATHER_PRESETS:
            self.get_logger().info(f'Setting weather to {weather_request}')
        else:
            weather_request = 'ClearNoon'
            self.get_logger().warn(
                f'Invalid weather request: {weather_request}. Using ClearNoon preset.'
            )
        self.carla_world.set_weather(WEATHER_PRESETS[weather_request])
        self.configured_weather = weather_request

        objects_json_path = self.objects_json_path
        if 'objects_json_path' in self.simulation_config:
            objects_json_path = self.simulation_config['objects_json_path']
        objects_json_path = get_full_file_path(
            objects_json_path, os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')
        )
        if objects_json_path == '':
            self.get_logger().warn(
                f'Invalid objects_json_path: {objects_json_path}. Skipping spawn_objects.'
            )
            return
        self.configured_objects_json_path = objects_json_path

        spawn_objects_config = {}
        spawn_objects_config['objects_json_path'] = objects_json_path

        spawn_point = self.spawn_point
        if 'spawn_point' in self.simulation_config:
            spawn_point = self.simulation_config['spawn_point']
        spawn_objects_config['spawn_point'] = spawn_point

        ego_vehicle_role_name = self.ego_vehicle_role_name
        if 'ego_vehicle_role_name' in self.simulation_config:
            ego_vehicle_role_name = self.simulation_config['ego_vehicle_role_name']
        spawn_objects_config['ego_vehicle_role_name'] = ego_vehicle_role_name
        self.configured_ego_vehicle_role_name = ego_vehicle_role_name

        self.spawn_objects(spawn_objects_config)
        self.publish_static_tfs()
        time.sleep(self.sim_startup_time)
        self.get_logger().info('Simulation started.')

    def setup_simulator(self):
        """Start the CARLA simulator."""
        if self.carla_script_path:
            start_simulator_command = [self.carla_script_path]
            if self.town != '':
                start_simulator_command.append(f'/Game/Carla/Maps/{self.town}')
            start_simulator_command.append(f'-carla-port={self.port}')
            if self.render_off_screen:
                start_simulator_command.append('-RenderOffScreen')
            start_simulator_command.append('-vulkan')
            if not self.relativemousemode:
                start_simulator_command.append('-norelativemousemode')

            subprocess_name = 'simulator'
            self.simulator_subprocess = create_subprocess(self, start_simulator_command, subprocess_name)
            time.sleep(self.sim_startup_time)

        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(self.timeout)
        self.carla_world = self.client.get_world()
        carla_map = self.carla_world.get_map()
        carla_map_name = carla_map.name.split('/')[-1]
        if self.town != carla_map_name:
            if 'simulator' in self.subprocesses:
                raise ValueError(
                    f'Could not start carla with town {self.town}. "{carla_map_name}" town '
                    f'is loaded.'
                )
            else:
                self.client.load_world(self.town)
                time.sleep(2.0)
                self.get_logger().info(f'Loaded map: {self.town}')
        else:
            self.get_logger().info(f'Simulator is ready with map: {carla_map_name}')

    def start_carla_ros_bridge(self):
        """Start the carla_ros_bridge node."""
        command = ['ros2', 'run', 'carla_ros_bridge', 'bridge', '--ros-args']
        for param, value in self.carla_ros_bridge_params.items():
            command.extend(['-p', f'{param}:={value}'])

        subprocess_name = 'carla_ros_bridge'
        self.subprocesses[subprocess_name] = create_subprocess(self, command, subprocess_name)

    def spawn_objects(self, spawn_objects_config, timeout=10):
        """Start the carla_spawn_objects node."""
        spawn_objects_params = {}
        spawn_objects_params['use_sim_time'] = True
        spawn_objects_params['objects_definition_file'] = spawn_objects_config['objects_json_path']
        for id_, value in spawn_objects_config['spawn_point'].items():
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

        subprocess_name = 'carla_spawn_objects'
        self.subprocesses[subprocess_name] = create_subprocess(
            self,
            ['unbuffer'] + command,
            subprocess_name,
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

        if spawn_objects_config['ego_vehicle_role_name']:
            objects_json_path = spawn_objects_config['objects_json_path']
            try:
                with open(objects_json_path, 'r', encoding='utf-8') as f:
                    objects = json.load(f).get('objects', [])
                    # Extract spawn points for vehicle.* types
                    for object_ in objects:
                        if spawn_objects_config['ego_vehicle_role_name'] in object_['id']:
                            sensors = object_['sensors']
                            for sensor in sensors:
                                if 'actor.pseudo.control' in sensor['type']:
                                    self.set_initial_pose(
                                        spawn_objects_config['ego_vehicle_role_name'], sensor['id']
                                    )
                                    self.get_logger().info(
                                        f'Initialized vehicle control for '
                                        f'{spawn_objects_config["ego_vehicle_role_name"]} with '
                                        f'control id {sensor["id"]}.'
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

        subprocess_name = 'carla_set_initial_pose'
        self.subprocesses[subprocess_name] = create_subprocess(self, command, subprocess_name)

    def end_current_simulation(self):
        cleanup_subprocesses(self.subprocesses)
        if self.reset_carla_after_exec and self.simulator_subprocess is not None:
            terminate_subprocess(self.simulator_subprocess)

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
            print('Shutting down carla_orchestrator node.')
            cleanup_subprocesses(self.subprocesses)
            terminate_subprocess(self.simulator_subprocess)
        except Exception as e:
            print(f'Error during node destruction: {e}')
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    # Initialize and run the CarlaOrchestrator node
    carla_orchestrator_node = CarlaOrchestrator()
    executor = MultiThreadedExecutor()
    executor.add_node(carla_orchestrator_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        carla_orchestrator_node.get_logger().info('User requested shutdown with SIGINT.')
    finally:
        # Cleanup on exit
        try:
            carla_orchestrator_node.destroy_node()
        except Exception as e:
            print(f'Error during node destruction: {e}')
        # Only shutdown if the context is still valid
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
