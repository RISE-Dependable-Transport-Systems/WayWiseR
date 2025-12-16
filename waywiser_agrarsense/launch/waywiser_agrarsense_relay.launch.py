import json
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml


def generate_launch_description():
    # Get the path to the package and the YAML configuration file
    waywiser_agrarsense_dir = get_package_share_directory('waywiser_agrarsense')

    # Declare arguments that can be set from the command line or default values
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation clock'
    )
    ego_vehicle_role_name_la = DeclareLaunchArgument(
        'ego_vehicle_role_name',
        default_value='truck',
        description='Name of the ego vehicle',
    )
    sim_config_la = DeclareLaunchArgument(
        'sim_config',
        default_value=os.path.join(waywiser_agrarsense_dir, 'config/agrarsense_orchestrator.yaml'),
        description='Full path to params file of simulator',
    )

    vehicle_config_la = DeclareLaunchArgument(
        'vehicle_config',
        default_value=os.path.join(waywiser_agrarsense_dir, 'config/forwarder.yaml'),
        description='Full path to params file of vehicle',
    )

    convert_rgb_to_grayscale_depth_la = DeclareLaunchArgument(
        'convert_rgb_to_grayscale_depth',
        default_value='true',
        description='Whether to convert rgb depth image to grayscale depth',
    )

    # Node configuration for waywiser_twist_transform
    waywiser_to_agrarsense_control = Node(
        package='waywiser_agrarsense',
        executable='waywiser_to_agrarsense_control.py',
        name='waywiser_to_agrarsense_control',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'ego_vehicle_role_name': LaunchConfiguration('ego_vehicle_role_name'),
            },
            LaunchConfiguration('vehicle_config'),
        ],
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        emulate_tty=True,
    )

    vehicle_tf_publishers_launch_action = OpaqueFunction(function=vehicle_tf_publishers_launch)
    camera_publishers_launch_action = OpaqueFunction(function=camera_publishers_launch)

    # Create launch description
    ld = LaunchDescription()

    # Add declared launch arguments
    ld.add_action(use_sim_time_la)
    ld.add_action(ego_vehicle_role_name_la)
    ld.add_action(sim_config_la)
    ld.add_action(vehicle_config_la)
    ld.add_action(convert_rgb_to_grayscale_depth_la)

    ld.add_action(waywiser_to_agrarsense_control)
    ld.add_action(vehicle_tf_publishers_launch_action)
    ld.add_action(camera_publishers_launch_action)

    return ld


def vehicle_tf_publishers_launch(context):
    nodes = []
    config_data = yaml_to_dict(LaunchConfiguration('sim_config').perform(context))
    sim_parameters = config_data['agrarsense_orchestrator_node']['ros__parameters']

    vehicle_ids = []
    if 'vehicle_ids' in sim_parameters:
        vehicle_ids = sim_parameters['vehicle_ids']

    vehicle_tf_publisher_params = {}
    if 'vehicle_tf_publisher' in sim_parameters:
        vehicle_tf_publisher_params = sim_parameters['vehicle_tf_publisher']

    for vehicle_id in vehicle_ids:
        if vehicle_id in vehicle_tf_publisher_params:
            vehicle_tf_publisher_param = vehicle_tf_publisher_params[vehicle_id]
            vehicle_tf_publisher_param['role_name'] = vehicle_id
            nodes.append(
                Node(
                    package='waywiser_agrarsense',
                    executable='vehicle_tf_publisher.py',
                    name=f'{vehicle_id}_tf_publisher',
                    output='screen',
                    emulate_tty=True,
                    parameters=[
                        vehicle_tf_publisher_param,
                        {
                            'use_sim_time': LaunchConfiguration('use_sim_time'),
                        },
                    ],
                )
            )

    return nodes


def camera_publishers_launch(context):
    nodes = []
    config_data = yaml_to_dict(LaunchConfiguration('sim_config').perform(context))
    sim_parameters = config_data['agrarsense_orchestrator_node']['ros__parameters']

    spawn_objects_json_file = os.path.expanduser(sim_parameters['objects_json_path'])
    # If the path is relative, prepend the package's config directory
    if spawn_objects_json_file and not spawn_objects_json_file.startswith('/'):
        spawn_objects_json_file = os.path.join(
            get_package_share_directory('waywiser_agrarsense'),
            'config',
            spawn_objects_json_file,
        )

    convert_rgb_to_grayscale_depth = LaunchConfiguration('convert_rgb_to_grayscale_depth').perform(
        context
    )

    with open(spawn_objects_json_file, 'r') as f:
        data = json.load(f)
        for obj in data['objects']:
            if obj['type'] == 'vehicle':
                role_name = obj['id']
                if 'sensors' in obj:
                    sensors = obj['sensors']
                    for sensor in sensors:
                        if sensor['model'] == 'RGBCamera':
                            parameters = sensor['parameters']
                            nodes.append(
                                Node(
                                    package='waywiser_perception',
                                    executable='camera_info_publisher.py',
                                    name=f'{role_name}_{sensor["id"]}_camera_info_publisher',
                                    output='screen',
                                    emulate_tty=True,
                                    parameters=[
                                        {
                                            'use_sim_time': LaunchConfiguration('use_sim_time'),
                                            'width': parameters['width'],
                                            'height': parameters['height'],
                                            'fov': parameters['fOV'],
                                            'camera_frame': f'{role_name}/{sensor["id"]}',
                                            'base_frame': 'base_link',
                                            'topic_name': (
                                                f'/agrarsense/out/sensors/{sensor["id"]}'
                                                f'/camera_info'
                                            ),
                                            'spawn_point': json.dumps(sensor['spawnPoint']),
                                        }
                                    ],
                                )
                            )
                        if sensor['model'] == 'DepthCamera':
                            parameters = sensor['parameters']['cameraParameters']
                            nodes.append(
                                Node(
                                    package='waywiser_perception',
                                    executable='camera_info_publisher.py',
                                    name=f'{role_name}_{sensor["id"]}_camera_info_publisher',
                                    output='screen',
                                    emulate_tty=True,
                                    parameters=[
                                        {
                                            'use_sim_time': LaunchConfiguration('use_sim_time'),
                                            'width': parameters['width'],
                                            'height': parameters['height'],
                                            'fov': parameters['fOV'],
                                            'camera_frame': f'{role_name}/{sensor["id"]}',
                                            'base_frame': 'base_link',
                                            'topic_name': (
                                                f'/agrarsense/out/sensors/{sensor["id"]}/'
                                                'camera_info'
                                            ),
                                            'spawn_point': json.dumps(sensor['spawnPoint']),
                                        }
                                    ],
                                )
                            )
                            if convert_rgb_to_grayscale_depth:
                                nodes.append(
                                    Node(
                                        package='waywiser_perception',
                                        executable='rgb_to_grayscale.py',
                                        name=f'{role_name}_{sensor["id"]}_rgb_to_grayscale',
                                        output='screen',
                                        emulate_tty=True,
                                        parameters=[
                                            {
                                                'use_sim_time': LaunchConfiguration(
                                                    'use_sim_time'
                                                ),
                                                'far_plane': 1000.0,
                                                'frame_id_override': f'{role_name}/{sensor["id"]}',
                                                'rgb_topic': (
                                                    f'/agrarsense/out/sensors/{sensor["id"]}'
                                                ),
                                                'depth_topic': (
                                                    f'/agrarsense/out/sensors/{sensor["id"]}'
                                                ),
                                            }
                                        ],
                                    )
                                )

    return nodes


def rgbd_to_pointcloud_launch(context):
    waywiser_perception_dir = get_package_share_directory('waywiser_perception')
    nodes = []
    with open(LaunchConfiguration('sim_config').perform(context)) as f:
        config_data = yaml.safe_load(f)
        node_params = config_data['/**']['ros__parameters']
        rgbd_to_pointcloud_sources = node_params['rgbd_to_pointcloud_sources']
        for rgbd_to_pointcloud_source in rgbd_to_pointcloud_sources:
            rgbd_to_pointcloud_source_params = node_params[rgbd_to_pointcloud_source]
            namespace = '/agrarsense/out/sensors'

            nodes.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [
                            os.path.join(
                                waywiser_perception_dir,
                                'launch',
                                'rgbd_to_pointcloud.launch.py',
                            )
                        ]
                    ),
                    launch_arguments={
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'namespace': namespace,
                        'rgb_topic': rgbd_to_pointcloud_source_params['rgb_camera'],
                        'depth_topic': rgbd_to_pointcloud_source_params['depth_camera'] + '_raw',
                        'rgb_camera_info_topic': rgbd_to_pointcloud_source_params['rgb_camera']
                        + '/camera_info',
                        'pointcloud_topic': rgbd_to_pointcloud_source_params['depth_camera']
                        + '/color/points',
                        'optical_to_ros_transform': 'False',
                    }.items(),
                )
            )

    return nodes


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, 'r') as f:
        return yaml.load(f, Loader=yaml.SafeLoader)
