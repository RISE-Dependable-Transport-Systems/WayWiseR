import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    waywiser_carla_dir = get_package_share_directory('waywiser_carla')

    # args that can be set from the command line or a default will be used
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation clock'
    )
    carla_config_la = DeclareLaunchArgument(
        'carla_config',
        default_value=os.path.join(waywiser_carla_dir, 'config/carla.yaml'),
        description='Full path to params file for carla',
    )

    # use context to start nodes
    carla_orchestrator_launch_action = OpaqueFunction(function=carla_orchestrator_launch)
    rgbd_to_pointcloud_launch_action = OpaqueFunction(function=rgbd_to_pointcloud_launch)

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(use_sim_time_la)
    ld.add_action(carla_config_la)

    # start nodes
    ld.add_action(carla_orchestrator_launch_action)
    ld.add_action(rgbd_to_pointcloud_launch_action)

    return ld


def carla_orchestrator_launch(context):
    with open(LaunchConfiguration('carla_config').perform(context)) as f:
        config_data = yaml.safe_load(f)
        node_params = config_data['/**']['ros__parameters']
        node_params.update(config_data['carla_orchestrator_node']['ros__parameters'])

    if 'carla_script_path' in node_params:
        carla_script_path = node_params['carla_script_path']
        carla_script_path = os.path.expanduser(carla_script_path)
        node_params['carla_script_path'] = carla_script_path

    if 'objects_json_path' in node_params:
        objects_json_path = node_params['objects_json_path']
        objects_json_path = os.path.expanduser(objects_json_path)

        # If the path is relative, prepend the package's config directory
        if objects_json_path and not objects_json_path.startswith('/'):
            objects_json_path = os.path.join(
                get_package_share_directory('waywiser_carla'),
                'config/',
                objects_json_path,
            )
        node_params['objects_json_path'] = objects_json_path

    if 'sim_configurations_json_path' in node_params:
        sim_configurations_json_path = node_params['sim_configurations_json_path']
        sim_configurations_json_path = os.path.expanduser(sim_configurations_json_path)

        # If the path is relative, prepend the package's config directory
        if sim_configurations_json_path and not sim_configurations_json_path.startswith('/'):
            sim_configurations_json_path = os.path.join(
                get_package_share_directory('waywiser_carla'),
                'config/',
                sim_configurations_json_path,
            )
        node_params['sim_configurations_json_path'] = sim_configurations_json_path

    if 'rosbag_output_dir' in node_params:
        rosbag_output_dir = node_params['rosbag_output_dir']
        rosbag_output_dir = os.path.expanduser(rosbag_output_dir)
        node_params['rosbag_output_dir'] = rosbag_output_dir

    carla_orchestrator = Node(
        package='waywiser_carla',
        executable='carla_orchestrator.py',
        name='carla_orchestrator_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            node_params,
        ],
        sigterm_timeout=['30'],
    )

    return [carla_orchestrator]


def rgbd_to_pointcloud_launch(context):
    waywiser_perception_dir = get_package_share_directory('waywiser_perception')
    nodes = []
    with open(LaunchConfiguration('carla_config').perform(context)) as f:
        config_data = yaml.safe_load(f)
        node_params = config_data['/**']['ros__parameters']
        ego_vehicle_role_name = node_params['ego_vehicle_role_name']
        rgbd_to_pointcloud_sources = node_params['rgbd_to_pointcloud_sources']
        for rgbd_to_pointcloud_source in rgbd_to_pointcloud_sources:
            rgbd_to_pointcloud_source_params = node_params[rgbd_to_pointcloud_source]
            attached_to_ego_vehicle = rgbd_to_pointcloud_source_params['attached_to_ego_vehicle']
            namespace = '/carla'
            if attached_to_ego_vehicle:
                namespace = namespace + '/' + ego_vehicle_role_name

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
                        'rgb_camera_node_name': rgbd_to_pointcloud_source_params['rgb_camera'],
                        'depth_camera_node_name': rgbd_to_pointcloud_source_params['depth_camera'],
                    }.items(),
                )
            )

    return nodes
