import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz2_dir = get_package_share_directory("ros2-waywise_rviz2")

    # args that can be set from the command line
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # args that can be set from the command line or a default will be used
    rviz_la = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(rviz2_dir, "rviz/odom_reference_frame.rviz"),
        description="Full path to rviz display config file",
    )
    use_sim_time_la = DeclareLaunchArgument(
        "use_sim_time", default_value="False", description="Use simulation/Gazebo clock"
    )

    # start nodes and use args to set parameters
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(rviz_la)
    ld.add_action(use_sim_time_la)

    # start nodes
    ld.add_action(rviz_node)

    return ld
