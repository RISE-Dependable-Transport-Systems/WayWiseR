import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    localisation_dir = get_package_share_directory("ros2-waywise_localization")

    # args that can be set from the command line or a default will be used
    use_sim_time_la = DeclareLaunchArgument(
        "use_sim_time", default_value="True", description="Use simulation/Gazebo clock"
    )
    ekf_config_la = DeclareLaunchArgument(
        "ekf_config",
        default_value=os.path.join(localisation_dir, "config/ekf.yaml"),
        description="Full path to params file for ekf_node",
    )

    # start nodes and use args to set parameters
    ekf_filter_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            LaunchConfiguration("ekf_config"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
        remappings={("/odometry/filtered", "/odom/filtered")},
    )
    
    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(use_sim_time_la)
    ld.add_action(ekf_config_la)

    # start nodes
    ld.add_action(ekf_filter_node)

    return ld
