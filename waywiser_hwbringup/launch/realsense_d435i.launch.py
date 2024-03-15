import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import yaml


def generate_launch_description():
    hw_bringup_dir = get_package_share_directory('waywiser_hwbringup')

    # args that can be set from the command line or a default will be used
    namespace_la = DeclareLaunchArgument(
        'namespace',
        default_value='/sensors/camera',
        description='namespace for all components',
    )

    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='False', description='Use simulation/Gazebo clock'
    )

    log_level_la = DeclareLaunchArgument(
        'log_level', default_value='info', description='Log level'
    )

    camera_config_la = DeclareLaunchArgument(
        'camera_config',
        default_value=os.path.join(hw_bringup_dir, 'config/realsense_d435i.yaml'),
        description='Full path to params file of camera',
    )

    container_la = DeclareLaunchArgument(
        name='container',
        default_value='',
        description=(
            'Name of an existing node container to load launched nodes into. '
            'If unset, a new container will be created.'
        ),
    )

    # use context to start nodes
    camera_launch_action = OpaqueFunction(function=camera_launch)

    # create launch description
    ld = LaunchDescription()

    # declare launch arg
    ld.add_action(namespace_la)
    ld.add_action(use_sim_time_la)
    ld.add_action(log_level_la)
    ld.add_action(camera_config_la)
    ld.add_action(container_la)

    # start nodes
    ld.add_action(camera_launch_action)

    return ld


def camera_launch(context):
    camera_params_dict = {}
    # Fix frame_id for depth camera point cloud: https://github.com/IntelRealSense/realsense-ros/tree/ros2-development?tab=readme-ov-file#ros2robot-vs-opticalcamera-coordination-systems # noqa
    enable_pointcloud_tranformation = False
    publish_color_pointcloud = False
    pointcloud_tranformation_params_dict = {}

    with open(LaunchConfiguration('camera_config').perform(context)) as f:
        camera_params_dict = yaml.safe_load(f)
        if 'pointcloud_tranformation' in camera_params_dict:
            pointcloud_tranformation_params_dict = camera_params_dict['pointcloud_tranformation']
            if 'enable' in pointcloud_tranformation_params_dict:
                enable_pointcloud_tranformation = pointcloud_tranformation_params_dict['enable']
        if 'publish_color_pointcloud' in camera_params_dict:
            publish_color_pointcloud = camera_params_dict['publish_color_pointcloud']

    composable_nodes = [
        ComposableNode(
            package='realsense2_camera',
            plugin='realsense2_camera::RealSenseNodeFactory',
            name='camera_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[camera_params_dict],
            remappings=[
                (
                    'depth/color/points',
                    'depth/points_raw',
                ),
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
        ComposableNode(
            package='waywiser_perception',
            plugin='waywiser_perception::PointCloudTransformer',
            name='point_cloud_transformer_node',
            condition=IfCondition(str(enable_pointcloud_tranformation)),
            parameters=[pointcloud_tranformation_params_dict],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
        ComposableNode(
            package='image_proc',
            plugin='image_proc::DebayerNode',
            name='debayer_node',
            namespace=[LaunchConfiguration('namespace'), '/color'],
            extra_arguments=[{'use_intra_process_comms': True}],
            remappings=[
                ('image_color', 'image'),
            ],
        ),
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_node',
            namespace=[LaunchConfiguration('namespace'), '/color'],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
        ComposableNode(
            package='depth_image_proc',
            plugin='depth_image_proc::ConvertMetricNode',
            name='convert_metric_node',
            namespace=[LaunchConfiguration('namespace'), '/aligned_depth_to_color'],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_node',
            namespace=[LaunchConfiguration('namespace'), '/aligned_depth_to_color'],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
        ComposableNode(
            package='depth_image_proc',
            plugin='depth_image_proc::PointCloudXyzrgbNode',
            name='point_cloud_xyzrgb_node',
            namespace=LaunchConfiguration('namespace'),
            condition=IfCondition(str(publish_color_pointcloud)),
            remappings=[
                ('rgb/camera_info', 'color/camera_info'),
                ('rgb/image_rect_color', 'color/image_rect'),
                ('depth_registered/image_rect', 'aligned_depth_to_color/image_rect'),
                ('points', 'color/points'),
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]

    # If an existing container is not provided, start a container and load nodes into it
    image_processing_container = ComposableNodeContainer(
        condition=LaunchConfigurationEquals('container', ''),
        name='pc_proc_container',
        namespace=LaunchConfiguration('namespace'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
    )

    # If an existing container name is provided, load composable nodes into it
    # This will block until a container with the provided name is available and nodes are loaded
    load_composable_nodes = LoadComposableNodes(
        condition=LaunchConfigurationNotEquals('container', ''),
        composable_node_descriptions=composable_nodes,
        target_container=LaunchConfiguration('container'),
    )

    return [image_processing_container, load_composable_nodes]


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, 'r') as f:
        return yaml.load(f, Loader=yaml.SafeLoader)
