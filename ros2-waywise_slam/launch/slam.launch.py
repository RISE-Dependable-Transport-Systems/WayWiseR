import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_dir = get_package_share_directory("ros2-waywise_slam")

    # args that can be set from the command line or a default will be used
    slam_config_la = DeclareLaunchArgument(
        "slam_config",
        default_value=os.path.join(slam_dir, "config/slam.yaml"),
        description="Full path to params file for slam toolbox",
    )
    use_sim_time_la = DeclareLaunchArgument(
        "use_sim_time", default_value="True", description="Use simulation/Gazebo clock"
    )

    # start nodes and use args to set parameters
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            LaunchConfiguration("slam_config"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
        remappings={("/pose", "/visual_pose")},
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(slam_config_la)
    ld.add_action(use_sim_time_la)

    # start nodes
    ld.add_action(slam_toolbox_node)

    return ld
