import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from waywiser_py.waywiser_utils import FileUtils, RosUtils


def generate_launch_description():
    # Get package directory
    waywiser_perception_dir = get_package_share_directory('waywiser_perception')

    # Declare launch arguments
    octomap_config_la = DeclareLaunchArgument(
        'octomap_config',
        default_value=os.path.join(waywiser_perception_dir, 'config/octomap.yaml'),
        description='Full path to params file for OctoMap',
    )

    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time/Gazebo clock',
    )

    octomap_node = OpaqueFunction(function=octomap_launch)

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(octomap_config_la)
    ld.add_action(use_sim_time_la)

    # start nodes
    ld.add_action(octomap_node)

    return ld


def octomap_launch(context):
    nodes = []
    octomap_config = FileUtils.get_full_file_path(
        LaunchConfiguration('octomap_config').perform(context)
    )
    node_params_dict = RosUtils.get_node_params(octomap_config, 'octomap_node')

    # OctomapNode - creates octomaps from point clouds
    nodes.append(
        Node(
            package='waywiser_perception',
            executable='octomap_node',
            name='octomap_node',
            output='screen',
            parameters=[
                node_params_dict,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
            ],
        )
    )

    return nodes
