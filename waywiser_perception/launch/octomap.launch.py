import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    # OctomapNode - creates octomaps from point clouds
    octomap_node = Node(
        package='waywiser_perception',
        executable='octomap_node',
        name='octomap_node',
        output='screen',
        parameters=[
            LaunchConfiguration('octomap_config'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    # OctomapOcclusionsNode - detects occlusions and triggers emergency stops
    octomap_occlusions_node = Node(
        package='waywiser_perception',
        executable='octomap_occlusions_node',
        name='octomap_occlusions_node',
        output='screen',
        parameters=[
            LaunchConfiguration('octomap_config'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(octomap_config_la)
    ld.add_action(use_sim_time_la)

    # start nodes
    ld.add_action(octomap_node)
    ld.add_action(octomap_occlusions_node)

    return ld
