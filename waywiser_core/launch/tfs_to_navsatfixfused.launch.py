from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # === Launch arguments ===
    config_la = DeclareLaunchArgument(
        'config',
        default_value='',
        description='Full path to YAML parameter file for tfs_to_navsatfixfused_node',
    )

    node_name_la = DeclareLaunchArgument(
        'node_name',
        default_value='tfs_to_navsatfixfused_node',
        description='Name of the TF -> NavSatFixExtended conversion node',
    )

    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo/Ignition) clock',
    )

    # === Node definition ===
    tfs_to_navsatfix_node = Node(
        package='waywiser_core',
        executable='tfs_to_navsatfixfused_node',
        name=LaunchConfiguration('node_name'),
        parameters=[
            LaunchConfiguration('config'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        output='screen',
        emulate_tty=True,
    )

    # === Launch Description ===
    ld = LaunchDescription()

    ld.add_action(config_la)
    ld.add_action(node_name_la)
    ld.add_action(use_sim_time_la)

    ld.add_action(tfs_to_navsatfix_node)

    return ld
