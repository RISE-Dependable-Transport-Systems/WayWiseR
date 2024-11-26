from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    # args that can be set from the command line or a default will be used
    namespace_la = DeclareLaunchArgument(
        'namespace',
        default_value='/sensors',
        description='namespace for all components',
    )

    rgb_camera_node_name_la = DeclareLaunchArgument(
        'rgb_camera_node_name',
        default_value='camera',
        description='name for rgb camera node',
    )
    depth_camera_node_name_la = DeclareLaunchArgument(
        'depth_camera_node_name',
        default_value='camera',
        description='name for depth camera node',
    )
    max_depth_meters_la = DeclareLaunchArgument(
        'max_depth_meters',
        default_value='100.0',
        description='Max. depth in meters to filter point clouds',
    )
    optical_to_ros_transform_la = DeclareLaunchArgument(
        'optical_to_ros_transform',
        default_value='False',
        description='Whether to do optical to ros coordinate transformation of depth msgs',
    )

    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='False', description='Use simulation/Gazebo clock'
    )

    log_level_la = DeclareLaunchArgument(
        'log_level', default_value='warn', description='Log level'
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
    rgbd_to_pointcloud_launch_action = OpaqueFunction(function=rgbd_to_pointcloud_launch)

    # create launch description
    ld = LaunchDescription()

    # declare launch arg
    ld.add_action(namespace_la)
    ld.add_action(rgb_camera_node_name_la)
    ld.add_action(depth_camera_node_name_la)
    ld.add_action(max_depth_meters_la)
    ld.add_action(optical_to_ros_transform_la)
    ld.add_action(use_sim_time_la)
    ld.add_action(log_level_la)
    ld.add_action(container_la)

    # start nodes
    ld.add_action(rgbd_to_pointcloud_launch_action)

    return ld


def rgbd_to_pointcloud_launch(context):
    rgb_camera_node_name = LaunchConfiguration('rgb_camera_node_name').perform(context)
    depth_camera_node_name = LaunchConfiguration('depth_camera_node_name').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)

    rgbd_to_pointcloud_node = Node(
        package='waywiser_perception',
        executable='rgbd_to_pointcloud.py',
        name=depth_camera_node_name + '_pointcloud',
        namespace=namespace,
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'rgb_topic': rgb_camera_node_name + '/image',
                'depth_topic': depth_camera_node_name + '/image',
                'pointcloud_topic': depth_camera_node_name + '/color/points',
                'camera_info_topic': rgb_camera_node_name + '/camera_info',
                'max_depth_meters': LaunchConfiguration('max_depth_meters'),
                'optical_to_ros_transform': LaunchConfiguration('optical_to_ros_transform'),
            },
        ],
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
    )

    return [rgbd_to_pointcloud_node]


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, 'r') as f:
        return yaml.load(f, Loader=yaml.SafeLoader)
