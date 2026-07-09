import os
import socket

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap

from waywiser_py.waywiser_utils import FileUtils, RosUtils


def generate_launch_description():
    waywiser_agrarsense_dir = get_package_share_directory('waywiser_agrarsense')
    waywiser_core_dir = get_package_share_directory('waywiser_core')
    waywiser_rviz2_dir = get_package_share_directory('waywiser_rviz2')
    waywiser_teleop_dir = get_package_share_directory('waywiser_teleop')
    waywiser_twist_safety_dir = get_package_share_directory('waywiser_twist_safety')
    waywiser_perception_dir = get_package_share_directory('waywiser_perception')

    # args that can be set from the command line or a default will be used
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation clock'
    )
    ego_vehicle_role_name_la = DeclareLaunchArgument(
        'ego_vehicle_role_name',
        default_value='forwarder',
        description='Name of the ego vehicle',
    )
    agrarsense_orchestrator_config_la = DeclareLaunchArgument(
        'agrarsense_orchestrator_config',
        default_value=os.path.join(
            waywiser_agrarsense_dir, 'config/agrarsense_orchestrator_vindeln.yaml'
        ),
        description='Full path to params file for agrarsense orchestrator',
    )
    enable_nav2_collision_monitor_la = DeclareLaunchArgument(
        'enable_nav2_collision_monitor',
        default_value='False',
        description='Use Nav2 collision monitoring',
    )
    rviz_config_la = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            waywiser_rviz2_dir, 'config/map_reference_frame_agrarsense.rviz'
        ),
        description='Full path of rviz display config file or path to their directory',
    )
    teleop_config_la = DeclareLaunchArgument(
        'teleop_config',
        default_value=os.path.join(waywiser_teleop_dir, 'config/teleop.yaml'),
        description='Full path to params file',
    )
    vehicle_config_la = DeclareLaunchArgument(
        'vehicle_config',
        default_value=os.path.join(waywiser_agrarsense_dir, 'config/forwarder.yaml'),
        description='Full path to params file of vehicle',
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
    control_vehicle_node_name_la = DeclareLaunchArgument(
        'control_vehicle_node_name',
        default_value='waywiser_car_node',
        description='Name of the vehicle node to control',
    )
    localization_node_name_la = DeclareLaunchArgument(
        'localization_node_name',
        default_value='waywiser_car_localization_node',
        description='Name of the node to be launched',
    )
    vehicle_name_la = DeclareLaunchArgument(
        'vehicle_name',
        default_value='forwarder',
        description='Name of the vehicle',
    )

    vehicle_name = LaunchConfiguration('vehicle_name')
    frame_prefix = [vehicle_name, '/']

    # include launch files
    waywiser_car_launch = GroupAction(
        actions=[
            PushRosNamespace(vehicle_name),
            SetRemap(src='/tf', dst='/tf'),
            SetRemap(src='/tf_static', dst='/tf_static'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            waywiser_core_dir,
                            'launch',
                            'waywiser_car.launch.py',
                        )
                    ]
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'vehicle_config': LaunchConfiguration('vehicle_config'),
                    'frame_prefix': frame_prefix,
                }.items(),
            ),
        ]
    )

    vehicle_tf_navsatfix_extended_wrapper = GroupAction(
        actions=[
            PushRosNamespace(vehicle_name),
            SetRemap(src='/tf', dst='/tf'),
            SetRemap(src='/tf_static', dst='/tf_static'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory('waywiser_core'),
                            'launch',
                            'navsatfix_extended_wrapper.launch.py',
                        )
                    ]
                ),
                launch_arguments={
                    'config': LaunchConfiguration('vehicle_config'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'frame_prefix': frame_prefix,
                }.items(),
            ),
        ]
    )

    vehicle_localization = GroupAction(
        actions=[
            PushRosNamespace(vehicle_name),
            SetRemap(src='/tf', dst='/tf'),
            SetRemap(src='/tf_static', dst='/tf_static'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            waywiser_core_dir,
                            'launch',
                            'waywiser_localization.launch.py',
                        )
                    ]
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'localization_config': LaunchConfiguration('vehicle_config'),
                    'localization_node_name': LaunchConfiguration('localization_node_name'),
                    'frame_prefix': frame_prefix,
                }.items(),
            ),
        ]
    )

    vehicle_twist_safety = GroupAction(
        actions=[
            PushRosNamespace(vehicle_name),
            SetRemap(src='/tf', dst='/tf'),
            SetRemap(src='/tf_static', dst='/tf_static'),
            IncludeLaunchDescription(
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
                    'enable_nav2_collision_monitor': LaunchConfiguration(
                        'enable_nav2_collision_monitor'
                    ),
                    'twist_safety_config': LaunchConfiguration('vehicle_config'),
                    'frame_prefix': frame_prefix,
                }.items(),
            ),
        ]
    )

    vehicle_yolo = GroupAction(
        actions=[
            PushRosNamespace(vehicle_name),
            SetRemap(src='/tf', dst='/tf'),
            SetRemap(src='/tf_static', dst='/tf_static'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            waywiser_perception_dir,
                            'launch',
                            'yolo.launch.py',
                        )
                    ]
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'yolo_config': LaunchConfiguration('vehicle_config'),
                }.items(),
            ),
        ]
    )

    agrarsense_orchestrator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_agrarsense_dir,
                    'launch',
                    'agrarsense_orchestrator.launch.py',
                )
            ]
        ),
        launch_arguments={
            'config': LaunchConfiguration('agrarsense_orchestrator_config'),
            'ego_vehicle_role_name': LaunchConfiguration('ego_vehicle_role_name'),
        }.items(),
    )

    waywiser_agrarsense_relay = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_agrarsense_dir,
                    'launch',
                    'waywiser_agrarsense_relay.launch.py',
                )
            ]
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'sim_config': LaunchConfiguration('agrarsense_orchestrator_config'),
        }.items(),
    )

    teleop_rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('waywiser'),
                    'launch',
                    'teleop_rviz2.launch.py',
                )
            ]
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'teleop_config': LaunchConfiguration('teleop_config'),
            'teleop': LaunchConfiguration('teleop'),
            'rviz2': LaunchConfiguration('rviz2'),
            'control_vehicle_node_fqn': [
                vehicle_name,
                '/',
                LaunchConfiguration('control_vehicle_node_name'),
            ],
        }.items(),
    )

    waywiser_collision_monitor = GroupAction(
        actions=[
            PushRosNamespace(vehicle_name),
            SetRemap(src='/tf', dst='/tf'),
            SetRemap(src='/tf_static', dst='/tf_static'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            waywiser_perception_dir,
                            'launch',
                            'collision_monitor.launch.py',
                        )
                    ]
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'collision_monitor_config': LaunchConfiguration('vehicle_config'),
                }.items(),
            ),
        ]
    )

    # Define OpaqueFunction actions to launch nodes with context
    waywiser_to_agrarsense_control_launch_action = OpaqueFunction(
        function=waywiser_to_agrarsense_control_launch
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(use_sim_time_la)
    ld.add_action(ego_vehicle_role_name_la)
    ld.add_action(agrarsense_orchestrator_config_la)
    ld.add_action(enable_nav2_collision_monitor_la)
    ld.add_action(rviz_config_la)
    ld.add_action(teleop_config_la)
    ld.add_action(vehicle_config_la)
    ld.add_action(teleop_la)
    ld.add_action(rviz2_la)
    ld.add_action(control_vehicle_node_name_la)
    ld.add_action(localization_node_name_la)
    ld.add_action(vehicle_name_la)
    # start nodes
    ld.add_action(SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'))
    ld.add_action(agrarsense_orchestrator)
    ld.add_action(waywiser_agrarsense_relay)
    ld.add_action(waywiser_car_launch)
    ld.add_action(waywiser_to_agrarsense_control_launch_action)
    ld.add_action(vehicle_tf_navsatfix_extended_wrapper)
    ld.add_action(vehicle_localization)
    ld.add_action(vehicle_twist_safety)
    ld.add_action(teleop_rviz2)
    ld.add_action(vehicle_yolo)
    ld.add_action(waywiser_collision_monitor)

    return ld


def waywiser_to_agrarsense_control_launch(context):
    vehicle_config = FileUtils.get_full_file_path(
        LaunchConfiguration('vehicle_config').perform(context)
    )
    node_params_dict = RosUtils.get_node_params(vehicle_config, 'waywiser_to_agrarsense_control')
    ego_vehicle_role_name = LaunchConfiguration('ego_vehicle_role_name').perform(context)
    return [
        Node(
            package='waywiser_agrarsense',
            executable='waywiser_to_agrarsense_control_node.py',
            name='waywiser_to_agrarsense_control',
            namespace=ego_vehicle_role_name,
            parameters=[
                node_params_dict,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'ego_vehicle_role_name': LaunchConfiguration('ego_vehicle_role_name'),
                },
            ],
            output='screen',
            emulate_tty=True,
        )
    ]
