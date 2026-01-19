import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    waywiser_teleop_dir = get_package_share_directory('waywiser_teleop')

    # args that can be set from the command line or a default will be used
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='False', description='Use simulation/Gazebo clock'
    )
    rviz_config_la = DeclareLaunchArgument(
        'rviz_config',
        default_value='',
        description='Full path of rviz display config file or path to their directory',
    )
    teleop_config_la = DeclareLaunchArgument(
        'teleop_config',
        default_value=os.path.join(waywiser_teleop_dir, 'config/teleop.yaml'),
        description='Full path to params file',
    )
    teleop_la = DeclareLaunchArgument(
        'teleop',
        default_value='True',
        description='Launch teleop',
    )
    rviz2_la = DeclareLaunchArgument(
        'rviz2',
        default_value='True',
        description='Launch rviz2',
    )
    control_vehicle_node_fqn_la = DeclareLaunchArgument(
        'control_vehicle_node_fqn',
        default_value='',
        description='Name of the vehicle node to control',
    )

    # create opaque functions to launch nodes using context
    teleop_rviz2_launch_action = OpaqueFunction(function=teleop_rviz2_launch)

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(use_sim_time_la)
    ld.add_action(rviz_config_la)
    ld.add_action(teleop_config_la)
    ld.add_action(teleop_la)
    ld.add_action(rviz2_la)
    ld.add_action(control_vehicle_node_fqn_la)

    # start nodes
    ld.add_action(teleop_rviz2_launch_action)

    return ld


def teleop_rviz2_launch(context):
    teleop = (LaunchConfiguration('teleop').perform(context)).lower() == 'true'
    rviz2 = (LaunchConfiguration('rviz2').perform(context)).lower() == 'true'
    nodes = []
    if teleop:
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory('waywiser_teleop'),
                            'launch',
                            'teleop.launch.py',
                        )
                    ]
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'teleop_config': LaunchConfiguration('teleop_config'),
                    'control_vehicle_node_fqn': LaunchConfiguration('control_vehicle_node_fqn'),
                }.items(),
            )
        )
    if rviz2:
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory('waywiser_rviz2'),
                            'launch',
                            'rviz.launch.py',
                        )
                    ]
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'rviz_config': LaunchConfiguration('rviz_config'),
                }.items(),
            )
        )

    return nodes
