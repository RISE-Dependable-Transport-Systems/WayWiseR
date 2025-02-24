import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    waywiser_hwbringup_dir = get_package_share_directory('waywiser_hwbringup')
    waywiser_twist_safety_dir = get_package_share_directory('waywiser_twist_safety')
    waywiser_rviz2_dir = get_package_share_directory('waywiser_rviz2')
    teleop_dir = get_package_share_directory('waywiser_teleop')

    # args that can be set from the command line or a default will be used
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='False', description='Use simulation/Gazebo clock'
    )
    vehicle_config_la = DeclareLaunchArgument(
        'vehicle_config',
        default_value=os.path.join(waywiser_hwbringup_dir, 'config/truck_small_scale.yaml'),
        description='Full path to params file of vehicle',
    )
    enable_collision_monitor_la = DeclareLaunchArgument(
        'enable_collision_monitor',
        default_value='False',
        description='Use Nav2 collision monitoring',
    )
    rviz_config_la = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(waywiser_rviz2_dir, 'rviz/odom_reference_frame_truck.rviz'),
        description='Full path of rviz display config file or path to their directory',
    )
    teleop_config_la = DeclareLaunchArgument(
        'teleop_config',
        default_value=os.path.join(teleop_dir, 'config/teleop.yaml'),
        description='Full path to params file',
    )
    teleop_la = DeclareLaunchArgument(
        'teleop',
        default_value='False',
        description='Launch teleop',
    )
    rviz2_la = DeclareLaunchArgument(
        'rviz2',
        default_value='False',
        description='Launch rviz2',
    )

    # include launch files
    truck = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_hwbringup_dir,
                    'launch',
                    'waywise_truck.launch.py',
                )
            ]
        ),
        launch_arguments={
            'vehicle_config': LaunchConfiguration('vehicle_config'),
        }.items(),
    )

    truck_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_hwbringup_dir,
                    'launch',
                    'waywise_state_publisher.launch.py',
                )
            ]
        ),
        launch_arguments={
            'vehicle_config': LaunchConfiguration('vehicle_config'),
        }.items(),
    )

    twist_safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_twist_safety_dir,
                    'launch',
                    'twist_safety.launch.py',
                )
            ]
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'enable_collision_monitor': LaunchConfiguration('enable_collision_monitor'),
        }.items(),
    )

    # include launch files
    teleop_rviz2_launch_action = OpaqueFunction(function=teleop_rviz2_launch)

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(use_sim_time_la)
    ld.add_action(vehicle_config_la)
    ld.add_action(enable_collision_monitor_la)
    ld.add_action(rviz_config_la)
    ld.add_action(teleop_config_la)
    ld.add_action(teleop_la)
    ld.add_action(rviz2_la)

    # start nodes
    ld.add_action(truck)
    ld.add_action(truck_state_publisher)
    ld.add_action(twist_safety)
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
