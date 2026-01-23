import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
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
        'use_sim_time', default_value='False', description='Use simulation/Gazebo clock'
    )

    control_vehicle_node_fqn_la = DeclareLaunchArgument(
        'control_vehicle_node_fqn',
        default_value='',
        description='Fully qualified name of the vehicle node to control',
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

    twist_keyboard_conditional_launch_action = OpaqueFunction(
        function=twist_keyboard_conditional_launch
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(teleop_config_la)
    ld.add_action(use_sim_time_la)
    ld.add_action(control_vehicle_node_fqn_la)

    # start nodes
    ld.add_action(joy_node)
    ld.add_action(teleop_twist_joy_node)
    ld.add_action(twist_angular_correction_node)
    ld.add_action(twist_keyboard_conditional_launch_action)

    return ld


def twist_keyboard_conditional_launch(context):
    with open(LaunchConfiguration('teleop_config').perform(context)) as f:
        config_data = yaml.safe_load(f)
        if 'twist_keyboard' in config_data:
            twist_keyboard_params = config_data['twist_keyboard']['ros__parameters']
            enable_twist_keyboard = twist_keyboard_params['enable']
            control_vehicle_node_fqn = LaunchConfiguration('control_vehicle_node_fqn').perform(
                context
            )
            if control_vehicle_node_fqn != '':
                twist_keyboard_params['control_vehicle_node_fqn'] = control_vehicle_node_fqn
            if enable_twist_keyboard:
                if 'DISPLAY' in os.environ:
                    twist_keyboard_node = Node(
                        package='waywiser_teleop',
                        executable='twist_keyboard.py',
                        name='twist_keyboard',
                        output='screen',
                        parameters=[
                            twist_keyboard_params,
                            {'use_sim_time': LaunchConfiguration('use_sim_time')},
                        ],
                        remappings={},
                    )
                    return [twist_keyboard_node]
                else:
                    log_no_display = LogInfo(
                        msg='No GUI display detected. twist_keyboard_node will not be launched.'
                    )
                    return [log_no_display]

    return []
