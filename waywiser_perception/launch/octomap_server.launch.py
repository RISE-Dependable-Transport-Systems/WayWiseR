import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    waywiser_perception_dir = get_package_share_directory('waywiser_perception')

    octomap_config_la = DeclareLaunchArgument(
        'octomap_config',
        default_value=os.path.join(waywiser_perception_dir, 'config/octomap_server.yaml'),
        description='Full path to params file for OctoMap server',
    )
    # assumes camera_node_name='camera' — override if using a different name
    cloud_topic_la = DeclareLaunchArgument(
        'cloud_topic',
        default_value='/sensors/camera/color/points',
        description='PointCloud2 topic published by the OAK-D driver',
    )

    # OctoMap’s ROS2 server builds the 3D occupancy map (octree) and consumes the PointCloud2
    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[LaunchConfiguration('octomap_config')],
        remappings=[
            # remap OctoMap's 'pointcloud' input to the OAK-D point cloud topic
            ('pointcloud', LaunchConfiguration('cloud_topic')),
        ],
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(octomap_config_la)
    ld.add_action(cloud_topic_la)

    # start nodes
    ld.add_action(octomap_server)

    return ld
