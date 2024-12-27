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

    # args that can be set from the command line or a default will be used
    sim_config_la = DeclareLaunchArgument(
        'sim_config',
        default_value=os.path.join(waywiser_agrarsense_dir, 'config/agrarsense.yaml'),
        description='Full path to params file of simulator',
    )

    # Setup processes, nodes, and launch files
    simulator_launch_action = OpaqueFunction(function=simulator_launch)

    # Create launch description
    ld = LaunchDescription()

    # declare launch arg
    ld.add_action(sim_config_la)

    # start nodes
    ld.add_action(simulator_launch_action)

    return ld


def simulator_launch(context):
    config_data = yaml_to_dict(LaunchConfiguration('sim_config').perform(context))
    sim_parameters = config_data['sim_orchestrator_node']['ros__parameters']
    sim_parameters['sim_script_path'] = os.path.expanduser(sim_parameters['sim_script_path'])
    ros_bridge_script_path = os.path.expanduser(sim_parameters['ros_bridge_script_path'])
    # If the path is relative, prepend the package's config directory
    if ros_bridge_script_path and not ros_bridge_script_path.startswith('/'):
        ros_bridge_script_path = os.path.join(
            get_package_share_directory('waywiser_agrarsense'),
            'ros_bridge',
            ros_bridge_script_path,
        )
    sim_parameters['ros_bridge_script_path'] = ros_bridge_script_path

    spawn_object_json_file = os.path.expanduser(sim_parameters['objects_json_path'])
    # If the path is relative, prepend the package's config directory
    if spawn_object_json_file and not spawn_object_json_file.startswith('/'):
        spawn_object_json_file = os.path.join(
            get_package_share_directory('waywiser_agrarsense'),
            'config',
            spawn_object_json_file,
        )
    sim_parameters['objects_json_path'] = spawn_object_json_file

    sim_configurations_json_path = os.path.expanduser(
        sim_parameters['sim_configurations_json_path']
    )
    # If the path is relative, prepend the package's config directory
    if sim_configurations_json_path and not sim_configurations_json_path.startswith('/'):
        sim_configurations_json_path = os.path.join(
            get_package_share_directory('waywiser_agrarsense'),
            'config',
            sim_configurations_json_path,
        )
    sim_parameters['sim_configurations_json_path'] = sim_configurations_json_path

    general_parameters = config_data['/**']['ros__parameters']
    sim_parameters.update(general_parameters)

    sim_orchestrator_node = Node(
        package='waywiser_agrarsense',
        executable='agrarsense_orchestrator.py',
        name='agrarsense_orchestrator_node',
        parameters=[sim_parameters],
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        sigterm_timeout=['20'],
    )

    return [sim_orchestrator_node]


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, 'r') as f:
        return yaml.load(f, Loader=yaml.SafeLoader)
