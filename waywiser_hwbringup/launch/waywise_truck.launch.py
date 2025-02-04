# launch file to bring up truck nodes

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
import yaml


def generate_launch_description():
    hw_bringup_dir = get_package_share_directory('waywiser_hwbringup')

    # args that can be set from the command line or a default will be used
    vehicle_config_la = DeclareLaunchArgument(
        'vehicle_config',
        default_value=os.path.join(hw_bringup_dir, 'config/truck_small_scale.yaml'),
        description='Full path to params file of truck',
    )

    frame_prefix_la = DeclareLaunchArgument(
        'frame_prefix',
        default_value='/',
        description='Prefix to publish robot transforms in',
    )

    # start nodes and use args to set parameters
    waywise_node_launch_action = OpaqueFunction(function=waywise_node_launch)
    # create launch description
    ld = LaunchDescription()

    # declare launch arg
    ld.add_action(vehicle_config_la)
    ld.add_action(frame_prefix_la)

    # start nodes
    ld.add_action(waywise_node_launch_action)

    return ld


def waywise_node_launch(context):
    nodes = []

    with open(LaunchConfiguration('vehicle_config').perform(context)) as f:
        config_data = yaml.safe_load(f)
        node_params_dict = config_data['/**']['ros__parameters']
        node_params_dict.update(config_data['waywise_truck_node']['ros__parameters'])

        if 'urdf_file' in node_params_dict:
            urdf_file = os.path.expanduser(node_params_dict['urdf_file'])
            # If the path is relative, prepend the package's config directory
            if urdf_file:
                if not urdf_file.startswith('/'):
                    urdf_file = os.path.join(
                        get_package_share_directory('waywiser_description'),
                        'urdf',
                        urdf_file,
                    )

                urdf_scale = 1.0
                if 'urdf_scale' in node_params_dict:
                    urdf_scale = node_params_dict['urdf_scale']

                has_trailer = False
                if 'has_trailer' in node_params_dict:
                    has_trailer = node_params_dict['has_trailer']

                urdf_scaler_script_path = os.path.join(
                    get_package_share_directory('waywiser_description'),
                    'scripts',
                    'scale_urdf.sh',
                )
                node_params_dict['urdf_file'] = ParameterValue(
                    Command(
                        ' '.join(
                            [
                                urdf_scaler_script_path,
                                urdf_file,
                                str(urdf_scale),
                                'has_trailer:=' + str(has_trailer),
                            ]
                        )
                    ),
                    value_type=str,
                )

        nodes.append(
            Node(
                package='waywiser_node',
                executable='waywise_truck_node',
                name='waywise_truck_node',
                parameters=[
                    node_params_dict,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ],
                remappings=[('/cmd_vel', '/cmd_vel_out')],
                arguments=['--ros-args', '--log-level', 'info'],
            )
        )

    return nodes
