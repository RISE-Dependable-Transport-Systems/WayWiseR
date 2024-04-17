from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # args that can be set from the command line or a default will be used
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation clock'
    )
    carla_ego_vehicle_role_name_la = DeclareLaunchArgument(
        'carla_ego_vehicle_role_name',
        default_value='truck',
        description='Role name of ego vehicle in carla',
    )

    # start nodes and use args to set parameters
    waywiser_twist_relay = Node(
        package='topic_tools',
        executable='relay',
        name='waywiser_twist_relay',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'input_topic': '/cmd_vel_out'},
            {
                'output_topic': [
                    '/carla/',
                    LaunchConfiguration('carla_ego_vehicle_role_name'),
                    '/twist',
                ]
            },
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    carla_map_to_odom_tf_publisher = Node(
        package='waywiser_carla',
        executable='map_to_odom_tf_publisher',
        name='carla_map_to_odom_tf_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'base_link_frame': LaunchConfiguration('carla_ego_vehicle_role_name')},
            {'odom_frame': 'odom'},
            {
                'odom_topic': [
                    '/carla/',
                    LaunchConfiguration('carla_ego_vehicle_role_name'),
                    '/odometry',
                ]
            },
        ],
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(use_sim_time_la)
    ld.add_action(carla_ego_vehicle_role_name_la)

    # start nodes
    ld.add_action(waywiser_twist_relay)
    ld.add_action(carla_map_to_odom_tf_publisher)

    return ld
