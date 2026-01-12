import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from waywiser_py.waywiser_utils import RosUtils
import yaml


def generate_launch_description():
    perception_dir = get_package_share_directory('waywiser_perception')

    # args that can be set from the command line or a default will be used
    yolov8_config_la = DeclareLaunchArgument(
        'yolo_config',
        default_value=os.path.join(perception_dir, 'config/yolov8.yaml'),
        description='Full path to params file of yolo',
    )

    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='False', description='Use simulation/Gazebo clock'
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
    yolo_config = LaunchConfiguration('yolo_config').perform(context)
    yolo_parameters = RosUtils.get_node_params(yolo_config, 'yolo_node')

    if not yolo_parameters:
        return []

    yolo_parameters['model_file_path'] = os.path.expanduser(yolo_parameters['model_file_path'])
    if yolo_parameters['use_tracker']:
        tracker_config_filepath = os.path.expanduser(yolo_parameters['tracker_config_filepath'])

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
            yolo_parameters['tracker_config_filepath'] = tracker_config_filepath

    yolo_node = Node(
        package='waywiser_perception',
        executable='yolo_node.py',
        name='yolo_node',
        parameters=[
            yolo_parameters,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
    )

    return [yolo_node]


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, 'r') as f:
        return yaml.load(f, Loader=yaml.SafeLoader)
