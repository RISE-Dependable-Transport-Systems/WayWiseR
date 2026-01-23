import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, PushRosNamespace, SetRemap
from launch_ros.descriptions import ComposableNode

import yaml


def generate_launch_description():
    waywiser_hwbringup_dir = get_package_share_directory('waywiser_hwbringup')
    depthai_ros_driver_dir = get_package_share_directory('depthai_ros_driver')

    # args that can be set from the command line or a default will be used
    namespace_la = DeclareLaunchArgument(
        'namespace',
        default_value='/sensors',
        description='namespace for all components',
    )

    camera_node_name_la = DeclareLaunchArgument(
        'camera_node_name',
        default_value='camera',
        description='name for camera node',
    )

    oakd_config_la = DeclareLaunchArgument(
        'oakd_config',
        default_value=os.path.join(waywiser_hwbringup_dir, 'config/oak_d.yaml'),
        description='Full path to params file for OAKD',
    )

    # include launch files
    oakd_camera_launch = GroupAction(
        [
            PushRosNamespace(LaunchConfiguration('namespace')),
            # Topic remaps
            SetRemap(
                src='/sensors/camera/nn/spatial_detections', dst='/sensors/camera/detections'
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        depthai_ros_driver_dir,
                        'launch',
                        'camera.launch.py',
                    )
                ),
                launch_arguments={
                    'name': LaunchConfiguration('camera_node_name'),
                    'params_file': LaunchConfiguration('oakd_config'),
                }.items(),
            ),
        ]
    )

    # use context to start nodes
    color_pointcloud_launch_action = OpaqueFunction(function=color_pointcloud_launch)

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(namespace_la)
    ld.add_action(camera_node_name_la)
    ld.add_action(oakd_config_la)

    # start nodes
    ld.add_action(oakd_camera_launch)
    ld.add_action(color_pointcloud_launch_action)

    return ld


def color_pointcloud_launch(context):
    camera_params_dict = {}
    publish_color_pointcloud = False
    camera_node_name = LaunchConfiguration('camera_node_name').perform(context)
    namespace_with_camera_name = (
        LaunchConfiguration('namespace').perform(context) + '/' + camera_node_name
    )

    with open(LaunchConfiguration('oakd_config').perform(context)) as f:
        config_data = yaml.safe_load(f)
        camera_params_dict = config_data['/**']['ros__parameters']
        if camera_node_name in config_data:
            camera_params_dict.update(config_data[camera_node_name]['ros__parameters'])
        if 'publish_color_pointcloud' in camera_params_dict:
            publish_color_pointcloud = camera_params_dict['publish_color_pointcloud']

    composable_nodes = [
        ComposableNode(
            package='depth_image_proc',
            plugin='depth_image_proc::PointCloudXyzrgbNode',
            name='point_cloud_xyzrgb_node',
            namespace=namespace_with_camera_name,
            condition=IfCondition(str(publish_color_pointcloud)),
            remappings=[
                ('rgb/image_rect_color', 'rgb/image_rect'),
                ('depth_registered/image_rect', 'stereo/image_raw'),
                ('points', 'color/points'),
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]

    # Use default oakd container
    load_composable_nodes = LoadComposableNodes(
        composable_node_descriptions=composable_nodes,
        target_container=f'{namespace_with_camera_name}_container',
    )

    return [load_composable_nodes]


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, 'r') as f:
        return yaml.load(f, Loader=yaml.SafeLoader)
