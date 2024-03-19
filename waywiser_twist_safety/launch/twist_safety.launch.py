import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    waywiser_twist_safety_dir = get_package_share_directory('waywiser_twist_safety')

    # args that can be set from the command line or a default will be used
    twist_safety_config_la = DeclareLaunchArgument(
        'twist_safety_config',
        default_value=os.path.join(waywiser_twist_safety_dir, 'config/twist_safety.yaml'),
        description='Full path to params file',
    )

    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation/Gazebo clock'
    )

    enable_collision_monitor_la = DeclareLaunchArgument(
        'enable_collision_monitor',
        default_value='False',
        description='Use Nav2 collision monitoring',
    )

    # start nodes and use args to set parameters
    onboard_twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='onboard_twist_mux',
        parameters=[
            LaunchConfiguration('twist_safety_config'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        remappings={('/cmd_vel_out', '/onboard_mux_vel')},
    )

    conditional_launch_action = OpaqueFunction(function=conditional_launch_setup)

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(twist_safety_config_la)
    ld.add_action(use_sim_time_la)
    ld.add_action(enable_collision_monitor_la)

    # start nodes
    ld.add_action(onboard_twist_mux_node)
    ld.add_action(conditional_launch_action)

    return ld


def conditional_launch_setup(context):
    enable_collision_monitor = (
        LaunchConfiguration('enable_collision_monitor').perform(context)
    ).lower() == 'true'

    launch_actions = []

    if enable_collision_monitor:
        collision_monitor = GroupAction(
            actions=[
                Node(
                    package='nav2_collision_monitor',
                    executable='collision_monitor',
                    name='collision_monitor',
                    output='screen',
                    emulate_tty=True,  # https://github.com/ros2/launch/issues/188
                    parameters=[
                        LaunchConfiguration('twist_safety_config'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')},
                    ],
                    remappings={('/bond', '/bond_nav2')},
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_collision_monitor',
                    output='screen',
                    emulate_tty=True,  # https://github.com/ros2/launch/issues/188
                    parameters=[
                        {
                            'use_sim_time': LaunchConfiguration('use_sim_time'),
                            'autostart': True,
                            'node_names': ['collision_monitor'],
                        }
                    ],
                    remappings={('/bond', '/bond_collision_monitor')},
                ),
            ],
        )

        emergency_stop_monitor_node = Node(
            package='waywiser_twist_safety',
            executable='emergency_stop_monitor',
            name='emergency_stop_monitor',
            parameters=[
                LaunchConfiguration('twist_safety_config'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
            ],
            remappings={('/cmd_vel_in', '/collision_monitor_vel')},
        )
        launch_actions.extend([collision_monitor, emergency_stop_monitor_node])
    else:
        emergency_stop_monitor_node = Node(
            package='waywiser_twist_safety',
            executable='emergency_stop_monitor',
            name='emergency_stop_monitor',
            parameters=[
                LaunchConfiguration('twist_safety_config'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
            ],
            remappings={('/cmd_vel_in', '/onboard_mux_vel')},
        )
        launch_actions.append(emergency_stop_monitor_node)

    return launch_actions
