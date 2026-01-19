import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, SetRemap

from waywiser_py.waywiser_utils import FileUtils, RosUtils


def generate_launch_description():
    waywiser_core_dir = get_package_share_directory('waywiser_core')
    waywiser_hwbringup_dir = get_package_share_directory('waywiser_hwbringup')
    waywiser_twist_safety_dir = get_package_share_directory('waywiser_twist_safety')
    waywiser_teleop_dir = get_package_share_directory('waywiser_teleop')

    # args that can be set from the command line or a default will be used
    vehicle_config_la = DeclareLaunchArgument(
        'vehicle_config',
        default_value=os.path.join(waywiser_hwbringup_dir, 'config/truck_small_scale.yaml'),
        description='Full path to params file of vehicle',
    )
    rviz_config_la = DeclareLaunchArgument(
        'rviz_config',
        default_value='map_reference_frame_truck.rviz',
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
    control_vehicle_node_name_la = DeclareLaunchArgument(
        'control_vehicle_node_name',
        default_value='waywiser_truck_node',
        description='Name of the vehicle node to control',
    )
    vehicle_name_la = DeclareLaunchArgument(
        'vehicle_name',
        default_value='semitruck',
        description='Name of the vehicle',
    )
    localization_node_name_la = DeclareLaunchArgument(
        'localization_node_name',
        default_value='waywiser_truck_localization_node',
        description='Name of the node to be launched',
    )

    vehicle_name = LaunchConfiguration('vehicle_name')
    frame_prefix = [vehicle_name, '/']

    # include launch files
    waywiser_truck_launch = GroupAction(
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
                            'waywiser_truck.launch.py',
                        )
                    ]
                ),
                launch_arguments={
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
                    'enable_nav2_collision_monitor': 'False',
                    'twist_safety_config': LaunchConfiguration('vehicle_config'),
                    'frame_prefix': frame_prefix,
                }.items(),
            ),
        ]
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

    # create opaque functions to launch nodes using context
    urm14_ultrasonic_array_launch_action = OpaqueFunction(function=urm14_ultrasonic_array_launch)

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(vehicle_config_la)
    ld.add_action(vehicle_name_la)
    ld.add_action(localization_node_name_la)
    ld.add_action(rviz_config_la)
    ld.add_action(teleop_config_la)
    ld.add_action(teleop_la)
    ld.add_action(rviz2_la)
    ld.add_action(control_vehicle_node_name_la)

    # start nodes
    ld.add_action(waywiser_truck_launch)
    ld.add_action(vehicle_localization)
    ld.add_action(vehicle_twist_safety)
    ld.add_action(teleop_rviz2)
    ld.add_action(urm14_ultrasonic_array_launch_action)
    ld.add_action(vehicle_tf_navsatfix_extended_wrapper)

    return ld


def urm14_ultrasonic_array_launch(context):
    nodes = []

    vehicle_config = FileUtils.get_full_file_path(
        LaunchConfiguration('vehicle_config').perform(context)
    )
    if vehicle_config == '':
        return nodes

    node_params_dict = RosUtils.get_node_params(vehicle_config, 'waywiser_truck_node')

    if 'urm14_sensor_array_config' in node_params_dict:
        vehicle_name = LaunchConfiguration('vehicle_name').perform(context)
        frame_prefix = (
            vehicle_name + '/'
        )  # TODO: use frame_prefix in urm14_ultrasonic_array launch

        urm14_sensor_array_config = FileUtils.get_full_file_path(
            node_params_dict['urm14_sensor_array_config'],
            os.path.join(get_package_share_directory('waywiser_hwbringup'), 'config'),
        )

        nodes.append(
            GroupAction(
                actions=[
                    PushRosNamespace(vehicle_name),
                    SetRemap(src='/tf', dst='/tf'),
                    SetRemap(src='/tf_static', dst='/tf_static'),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [
                                os.path.join(
                                    get_package_share_directory('waywiser_hwbringup'),
                                    'launch',
                                    'urm14_ultrasonic_array.launch.py',
                                )
                            ]
                        ),
                        launch_arguments={
                            'urm14_sensor_array_config': urm14_sensor_array_config,
                        }.items(),
                    ),
                ]
            )
        )

    return nodes
