from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # args that can be set from the command line or a default will be used
    namespace_la = DeclareLaunchArgument(
        'namespace',
        default_value='/sensors/camera',
        description='namespace for all components',
    )

    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation/Gazebo clock'
    )

    log_level_la = DeclareLaunchArgument(
        'log_level', default_value='info', description='Log level'
    )

    container_la = DeclareLaunchArgument(
        name='container',
        default_value='',
        description=(
            'Name of an existing node container to load launched nodes into. '
            'If unset, a new container will be created.'
        ),
    )

    # start nodes and use args to set parameters
    composable_nodes = [
        ComposableNode(
            package='image_proc',
            plugin='image_proc::DebayerNode',
            name='debayer_node',
            namespace=[LaunchConfiguration('namespace'), '/color'],
            extra_arguments=[{'use_intra_process_comms': True}],
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }
            ],
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
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }
            ],
        ),
        ComposableNode(
            package='depth_image_proc',
            plugin='depth_image_proc::ConvertMetricNode',
            name='convert_metric_node',
            namespace=[LaunchConfiguration('namespace'), '/aligned_depth_to_color'],
            extra_arguments=[{'use_intra_process_comms': True}],
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }
            ],
        ),
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_node',
            namespace=[LaunchConfiguration('namespace'), '/aligned_depth_to_color'],
            extra_arguments=[{'use_intra_process_comms': True}],
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }
            ],
        ),
        ComposableNode(
            package='depth_image_proc',
            plugin='depth_image_proc::PointCloudXyzrgbNode',
            name='point_cloud_xyzrgb_node',
            namespace=LaunchConfiguration('namespace'),
            remappings=[
                ('rgb/camera_info', 'color/camera_info'),
                ('rgb/image_rect_color', 'color/image_rect'),
                ('depth_registered/image_rect', 'aligned_depth_to_color/image_rect'),
                ('points', 'color/points'),
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }
            ],
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

    # create launch description
    ld = LaunchDescription()

    # declare launch arg
    ld.add_action(namespace_la)
    ld.add_action(use_sim_time_la)
    ld.add_action(log_level_la)
    ld.add_action(container_la)

    # start nodes
    ld.add_action(image_processing_container)
    ld.add_action(load_composable_nodes)

    return ld
