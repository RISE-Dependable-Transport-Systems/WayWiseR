# launch file to bring up truck nodes

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from waywiser_py.waywiser_utils import get_full_file_path
import yaml


def generate_launch_description():
    waywiser_core_dir = get_package_share_directory('waywiser_core')
    waywiser_hwbringup_dir = get_package_share_directory('waywiser_hwbringup')

    # args that can be set from the command line or a default will be used
    vehicle_config_la = DeclareLaunchArgument(
        'vehicle_config',
        default_value=os.path.join(waywiser_hwbringup_dir, 'config/truck_small_scale.yaml'),
        description='Full path to params file of truck',
    )
    frame_prefix_la = DeclareLaunchArgument(
        'frame_prefix',
        default_value='/',
        description='Prefix to publish robot transforms in',
    )
    localization_node_name_la = DeclareLaunchArgument(
        'localization_node_name',
        default_value='waywiser_truck_localization_node',
        description='Name of the node to be launched',
    )

    # include launch files
    waywiser_truck_launch = IncludeLaunchDescription(
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
            'frame_prefix': LaunchConfiguration('frame_prefix'),
        }.items(),
    )

    waywiser_truck_localization_launch = IncludeLaunchDescription(
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
        }.items(),
    )

    # create opaque functions to launch nodes using context
    urm14_ultrasonic_array_launch_action = OpaqueFunction(function=urm14_ultrasonic_array_launch)

    # create launch description
    ld = LaunchDescription()

    # declare launch arg
    ld.add_action(vehicle_config_la)
    ld.add_action(frame_prefix_la)
    ld.add_action(localization_node_name_la)

    # start nodes
    ld.add_action(waywiser_truck_launch)
    ld.add_action(waywiser_truck_localization_launch)
    ld.add_action(urm14_ultrasonic_array_launch_action)

    return ld


def urm14_ultrasonic_array_launch(context):
    nodes = []

    vehicle_config = get_full_file_path(LaunchConfiguration('vehicle_config').perform(context))
    if vehicle_config == '':
        return nodes

    with open(vehicle_config, 'r', encoding='utf-8') as f:
        config_data = yaml.safe_load(f)
        node_params_dict = config_data['/**']['ros__parameters']
        node_params_dict.update(config_data['waywiser_truck_node']['ros__parameters'])

        if 'urm14_sensor_array_config' in node_params_dict:
            urm14_sensor_array_config = get_full_file_path(
                node_params_dict['urm14_sensor_array_config'],
                os.path.join(get_package_share_directory('waywiser_hwbringup'), 'config'),
            )

            nodes.append(
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
                )
            )

    return nodes
