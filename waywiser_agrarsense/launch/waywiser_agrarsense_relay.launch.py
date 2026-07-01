import json
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
from launch_ros.actions import Node, PushRosNamespace, SetRemap

from waywiser_py.waywiser_utils import FileUtils, RosUtils


def generate_launch_description():
    # Get the path to the package and the YAML configuration file
    waywiser_agrarsense_dir = get_package_share_directory('waywiser_agrarsense')

    # Declare arguments that can be set from the command line or default values
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation clock'
    )
    sim_config_la = DeclareLaunchArgument(
        'sim_config',
        default_value=os.path.join(
            waywiser_agrarsense_dir, 'config/agrarsense_orchestrator_playground.yaml'
        ),
        description='Full path to params file of simulator',
    )

    convert_rgb_to_grayscale_depth_la = DeclareLaunchArgument(
        'convert_rgb_to_grayscale_depth',
        default_value='true',
        description='Whether to convert rgb depth image to grayscale depth',
    )

    # Define OpaqueFunction actions to launch nodes with context
    vehicle_tf_publishers_launch_action = OpaqueFunction(function=vehicle_tf_publishers_launch)
    camera_publishers_launch_action = OpaqueFunction(function=camera_publishers_launch)
    rgbd_to_pointcloud_launch_action = OpaqueFunction(function=rgbd_to_pointcloud_launch)

    # Create launch description
    ld = LaunchDescription()

    # Add declared launch arguments
    ld.add_action(use_sim_time_la)
    ld.add_action(sim_config_la)
    ld.add_action(convert_rgb_to_grayscale_depth_la)

    ld.add_action(vehicle_tf_publishers_launch_action)
    ld.add_action(camera_publishers_launch_action)
    ld.add_action(rgbd_to_pointcloud_launch_action)

    return ld


def vehicle_tf_publishers_launch(context):
    nodes = []
    sim_config = FileUtils.get_full_file_path(LaunchConfiguration('sim_config').perform(context))
    node_params_dict = RosUtils.get_node_params(sim_config, 'agrarsense_orchestrator_node')

    vehicle_ids = []
    if 'vehicle_ids' in node_params_dict:
        vehicle_ids = node_params_dict['vehicle_ids']

    vehicle_tf_publisher_params = {}
    if 'vehicle_tf_publisher' in node_params_dict:
        vehicle_tf_publisher_params = node_params_dict['vehicle_tf_publisher']

    for vehicle_id in vehicle_ids:
        if vehicle_id in vehicle_tf_publisher_params:
            vehicle_tf_publisher_param = vehicle_tf_publisher_params[vehicle_id]
            vehicle_tf_publisher_param['role_name'] = vehicle_id
            nodes.append(
                Node(
                    package='waywiser_agrarsense',
                    executable='vehicle_tf_publisher_node.py',
                    namespace=vehicle_id,
                    name='tf_publisher',
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
    sim_config = FileUtils.get_full_file_path(LaunchConfiguration('sim_config').perform(context))
    node_params_dict = RosUtils.get_node_params(sim_config, 'agrarsense_orchestrator_node')

    spawn_objects_json_file = os.path.expanduser(node_params_dict['objects_json_path'])
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

    with open(spawn_objects_json_file, 'r', encoding='utf-8') as f:
        data = json.load(f)
        for obj in data['objects']:
            if obj['type'] == 'vehicle':
                role_name = obj['id']
                if 'sensors' in obj:
                    sensors = obj['sensors']
                    for sensor in sensors:
                        if sensor['model'] == 'RGBCamera' or sensor['model'] == 'DepthCamera':
                            if sensor['model'] == 'RGBCamera':
                                parameters = sensor['parameters']
                                parameters['out_topic'] = f'sensors/{sensor["id"]}/image_rect'
                            else:
                                parameters = sensor['parameters']['cameraParameters']
                                parameters['out_topic'] = f'sensors/{sensor["id"]}/image_rect'

                            parameters['fov'] = parameters.pop('fOV')
                            parameters['camera_frame'] = RosUtils.join_frame(
                                role_name, sensor['id']
                            )
                            parameters['base_frame'] = RosUtils.join_frame(role_name, 'base_link')
                            parameters['topic_name'] = f'sensors/{sensor["id"]}/camera_info'
                            parameters['spawn_point'] = json.dumps(sensor['spawnPoint'])
                            if not (
                                sensor['model'] == 'DepthCamera' and convert_rgb_to_grayscale_depth
                            ):
                                parameters['in_topic'] = f'/agrarsense/out/sensors/{sensor["id"]}'

                            nodes.append(
                                Node(
                                    package='waywiser_perception',
                                    executable='camera_info_publisher_node.py',
                                    namespace=role_name,
                                    name=f'{sensor["id"]}_camera_info_publisher',
                                    output='screen',
                                    emulate_tty=True,
                                    parameters=[
                                        parameters,
                                        {
                                            'use_sim_time': LaunchConfiguration('use_sim_time'),
                                        },
                                    ],
                                )
                            )

                            if sensor['model'] == 'DepthCamera' and convert_rgb_to_grayscale_depth:
                                nodes.append(
                                    Node(
                                        package='waywiser_perception',
                                        executable='rgb_to_grayscale_node.py',
                                        namespace=role_name,
                                        name=f'{sensor["id"]}_rgb_to_grayscale',
                                        output='screen',
                                        emulate_tty=True,
                                        parameters=[
                                            {
                                                'use_sim_time': LaunchConfiguration(
                                                    'use_sim_time'
                                                ),
                                                'far_plane': 1000.0,
                                                'frame_id_override': RosUtils.join_frame(
                                                    role_name, sensor['id']
                                                ),
                                                'depth_in_topic': (
                                                    f'/agrarsense/out/sensors/{sensor["id"]}'
                                                ),
                                                'depth_out_topic': (
                                                    f'sensors/{sensor["id"]}/image_rect'
                                                ),
                                            }
                                        ],
                                    )
                                )

    return nodes


def rgbd_to_pointcloud_launch(context):
    waywiser_perception_dir = get_package_share_directory('waywiser_perception')
    nodes = []
    sim_config = FileUtils.get_full_file_path(LaunchConfiguration('sim_config').perform(context))
    node_params_dict = RosUtils.get_node_params(sim_config, 'agrarsense_orchestrator_node')
    rgbd_to_pointcloud_sources = node_params_dict['rgbd_to_pointcloud_sources']
    for rgbd_to_pointcloud_source in rgbd_to_pointcloud_sources:
        rgbd_to_pointcloud_source_params = node_params_dict[rgbd_to_pointcloud_source]
        namespace = f'/{rgbd_to_pointcloud_source_params.get("attached_to", "agrarsense")}/sensors'

        nodes.append(
            GroupAction(
                actions=[
                    PushRosNamespace(namespace),
                    SetRemap(src='/tf', dst='/tf'),
                    SetRemap(src='/tf_static', dst='/tf_static'),
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
                            'node_name': rgbd_to_pointcloud_source_params['depth_camera']
                            + '_pc_publisher',
                            'rgb_topic': rgbd_to_pointcloud_source_params['rgb_camera']
                            + '/image_rect',
                            'depth_topic': rgbd_to_pointcloud_source_params['depth_camera']
                            + '/image_rect',
                            'rgb_camera_info_topic': rgbd_to_pointcloud_source_params['rgb_camera']
                            + '/camera_info',
                            'pointcloud_topic': rgbd_to_pointcloud_source_params['depth_camera']
                            + '/color/points',
                            'optical_to_ros_transform': 'False',
                        }.items(),
                    ),
                ]
            )
        )

    return nodes
