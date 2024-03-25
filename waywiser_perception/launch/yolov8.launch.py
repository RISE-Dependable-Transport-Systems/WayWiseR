import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    perception_dir = get_package_share_directory('waywiser_perception')

    # args that can be set from the command line or a default will be used
    yolov8_config_la = DeclareLaunchArgument(
        'yolov8_config',
        default_value=os.path.join(perception_dir, 'config/yolov8.yaml'),
        description='Full path to params file of yolov8',
    )

    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation/Gazebo clock'
    )

    # include launch files
    yolov8_launch_action = OpaqueFunction(function=yolov8_launch)

    # create launch description
    ld = LaunchDescription()

    # declare launch arg
    ld.add_action(yolov8_config_la)
    ld.add_action(use_sim_time_la)

    # start nodes
    ld.add_action(yolov8_launch_action)

    return ld


def yolov8_launch(context):
    config_data = yaml_to_dict(LaunchConfiguration('yolov8_config').perform(context))
    yolov8_parameters = config_data['yolov8_node']['ros__parameters']
    yolov8_parameters['model_file_path'] = os.path.expanduser(yolov8_parameters['model_file_path'])
    if yolov8_parameters['use_tracker']:
        tracker_config_filepath = os.path.expanduser(yolov8_parameters['tracker_config_filepath'])

        if not tracker_config_filepath.startswith('/'):
            tracker_config_filepath = os.path.join(
                get_package_share_directory('waywiser_perception'),
                'config/',
                tracker_config_filepath,
            )

        if not os.path.exists(tracker_config_filepath):
            raise FileNotFoundError(
                f"Tracker config file '{tracker_config_filepath}' does not exist."
            )
        else:
            yolov8_parameters['tracker_config_filepath'] = tracker_config_filepath

    yolov8_node = Node(
        package='waywiser_perception',
        executable='yolov8_node.py',
        name='yolov8_node',
        parameters=[
            yolov8_parameters,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
    )

    return [yolov8_node]


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, 'r') as f:
        return yaml.load(f, Loader=yaml.SafeLoader)
