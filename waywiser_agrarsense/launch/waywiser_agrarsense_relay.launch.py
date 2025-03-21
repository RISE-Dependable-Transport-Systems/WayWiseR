import json
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    # Get the path to the package and the YAML configuration file
    waywiser_agrarsense_dir = get_package_share_directory('waywiser_agrarsense')

    # Declare arguments that can be set from the command line or default values
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation clock'
    )

    sim_config_la = DeclareLaunchArgument(
        'sim_config',
        default_value=os.path.join(waywiser_agrarsense_dir, 'config/agrarsense_playground.yaml'),
        description='Full path to params file of simulator',
    )

    vehicle_config_la = DeclareLaunchArgument(
        'vehicle_config',
        default_value=os.path.join(waywiser_agrarsense_dir, 'config/forwarder.yaml'),
        description='Full path to params file of vehicle',
    )

    # Node configuration for waywiser_twist_transform
    waywiser_twist_to_agrarsense_control_node = Node(
        package='waywiser_agrarsense',
        executable='waywiser_twist_to_agrarsense_control.py',
        name='waywiser_twist_to_agrarsense_control_node',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
            LaunchConfiguration('vehicle_config'),
        ],
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        emulate_tty=True,
    )

    vehicle_tf_publishers_launch_action = OpaqueFunction(function=vehicle_tf_publishers_launch)

    # Create launch description
    ld = LaunchDescription()

    # Add declared launch arguments
    ld.add_action(use_sim_time_la)
    ld.add_action(sim_config_la)
    ld.add_action(vehicle_config_la)

    ld.add_action(waywiser_twist_to_agrarsense_control_node)
    ld.add_action(vehicle_tf_publishers_launch_action)

    return ld


def vehicle_tf_publishers_launch(context):
    nodes = []
    config_data = yaml_to_dict(LaunchConfiguration('sim_config').perform(context))
    sim_parameters = config_data['sim_orchestrator_node']['ros__parameters']

    spawn_objects_json_file = os.path.expanduser(sim_parameters['objects_json_path'])
    # If the path is relative, prepend the package's config directory
    if spawn_objects_json_file and not spawn_objects_json_file.startswith('/'):
        spawn_objects_json_file = os.path.join(
            get_package_share_directory('waywiser_agrarsense'),
            'config',
            spawn_objects_json_file,
        )

    with open(spawn_objects_json_file, 'r') as f:
        data = json.load(f)
        for obj in data['objects']:
            if obj['type'] == 'vehicle':
                vehicle_identifier = obj['id']
                nodes.append(
                    Node(
                        package='waywiser_agrarsense',
                        executable='vehicle_tf_publisher.py',
                        name=f'{vehicle_identifier}_tf_publisher',
                        output='screen',
                        emulate_tty=True,
                        parameters=[
                            {
                                'odom_topic': f'/agrarsense/out/vehicles/'
                                f'{vehicle_identifier}/odometry',
                                'input_transform': f'/agrarsense/out/'
                                f'sensors/{vehicle_identifier}/transform',
                                'base_link_frame': f'{vehicle_identifier}/base_link',
                                'odom_frame': f'{vehicle_identifier}/odom',
                                'use_sim_time': LaunchConfiguration('use_sim_time'),
                            }
                        ],
                    )
                )

    return nodes


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, 'r') as f:
        return yaml.load(f, Loader=yaml.SafeLoader)
