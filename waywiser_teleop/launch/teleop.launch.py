import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    teleop_dir = get_package_share_directory('waywiser_teleop')

    # args that can be set from the command line or a default will be used
    teleop_config_la = DeclareLaunchArgument(
        'teleop_config',
        default_value=os.path.join(teleop_dir, 'config/teleop.yaml'),
        description='Full path to params file',
    )

    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation/Gazebo clock'
    )

    # start nodes and use args to set parameters
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[
            LaunchConfiguration('teleop_config'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[
            LaunchConfiguration('teleop_config'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        remappings={('/cmd_vel', '/joy_vel')},
    )

    twist_angular_correction_node = Node(
        package='waywiser_teleop',
        executable='twist_angular_correction',
        name='twist_angular_correction',
        parameters=[
            LaunchConfiguration('teleop_config'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    joy_emergency_stop_node = Node(
        package='waywiser_teleop',
        executable='joy_emergency_stop',
        name='joy_emergency_stop',
        parameters=[
            LaunchConfiguration('teleop_config'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    teleop_twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='teleop_twist_mux',
        parameters=[
            LaunchConfiguration('teleop_config'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        remappings={('/cmd_vel_out', '/teleop_mux_vel')},
    )

    twist_keyboard_conditional_launch_action = OpaqueFunction(
        function=twist_keyboard_conditional_launch
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(teleop_config_la)
    ld.add_action(use_sim_time_la)

    # start nodes
    ld.add_action(joy_node)
    ld.add_action(teleop_twist_joy_node)
    ld.add_action(joy_emergency_stop_node)
    ld.add_action(twist_angular_correction_node)
    ld.add_action(twist_keyboard_conditional_launch_action)
    ld.add_action(teleop_twist_mux_node)

    return ld


def twist_keyboard_conditional_launch(context):
    with open(LaunchConfiguration('teleop_config').perform(context)) as f:
        config_data = yaml.safe_load(f)
        if 'twist_keyboard' in config_data:
            twist_keyboard_params = config_data['twist_keyboard']['ros__parameters']
            enable_twist_keyboard = twist_keyboard_params['enable']
            if enable_twist_keyboard:
                twist_keyboard_node = Node(
                    package='waywiser_teleop',
                    executable='twist_keyboard.py',
                    name='twist_keyboard',
                    output='screen',
                    parameters=[
                        twist_keyboard_params,
                        {'use_sim_time': LaunchConfiguration('use_sim_time')},
                    ],
                    remappings={('/cmd_vel', '/key_vel')},
                )
                return [twist_keyboard_node]

    return []
