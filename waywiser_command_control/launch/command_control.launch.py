import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hw_bringup_dir = get_package_share_directory('waywiser_command_control')

    # args that can be set from the command line or a default will be used
    command_control_config_la = DeclareLaunchArgument(
        'command_control_config',
        default_value=os.path.join(hw_bringup_dir, 'config/command_control.yaml'),
        description='Full path to params file',
    )

    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation/Gazebo clock'
    )

    # start nodes and use args to set parameters
    emergency_stop_monitor_node = Node(
        package='waywiser_command_control',
        executable='emergency_stop_monitor',
        name='emergency_stop_monitor',
        parameters=[
            LaunchConfiguration('command_control_config'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        remappings={('/cmd_vel_in', '/onboard_mux_vel')},
    )

    onboard_twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='onboard_twist_mux',
        parameters=[
            LaunchConfiguration('command_control_config'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        remappings={('/cmd_vel_out', '/onboard_mux_vel')},
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(command_control_config_la)
    ld.add_action(use_sim_time_la)

    # start nodes
    ld.add_action(emergency_stop_monitor_node)
    ld.add_action(onboard_twist_mux_node)

    return ld
