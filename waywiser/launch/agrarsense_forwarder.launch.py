import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    waywiser_agrarsense_dir = get_package_share_directory('waywiser_agrarsense')
    waywiser_core_dir = get_package_share_directory('waywiser_core')
    waywiser_rviz2_dir = get_package_share_directory('waywiser_rviz2')
    waywiser_teleop_dir = get_package_share_directory('waywiser_teleop')
    waywiser_twist_safety_dir = get_package_share_directory('waywiser_twist_safety')
    waywiser_perception_dir = get_package_share_directory('waywiser_perception')

    # args that can be set from the command line or a default will be used
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation clock'
    )
    ego_vehicle_role_name_la = DeclareLaunchArgument(
        'ego_vehicle_role_name',
        default_value='forwarder',
        description='Name of the ego vehicle',
    )
    agrarsense_orchestrator_config_la = DeclareLaunchArgument(
        'agrarsense_orchestrator_config',
        default_value=os.path.join(
            waywiser_agrarsense_dir, 'config/agrarsense_orchestrator_vindeln.yaml'
        ),
        description='Full path to params file for agrarsense orchestrator',
    )
    enable_collision_monitor_la = DeclareLaunchArgument(
        'enable_collision_monitor',
        default_value='True',
        description='Use Nav2 collision monitoring',
    )
    rviz_config_la = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            waywiser_rviz2_dir, 'config/map_reference_frame_agrarsense.rviz'
        ),
        description='Full path of rviz display config file or path to their directory',
    )
    teleop_config_la = DeclareLaunchArgument(
        'teleop_config',
        default_value=os.path.join(waywiser_teleop_dir, 'config/teleop_sim.yaml'),
        description='Full path to params file',
    )
    vehicle_config_la = DeclareLaunchArgument(
        'vehicle_config',
        default_value=os.path.join(waywiser_agrarsense_dir, 'config/forwarder.yaml'),
        description='Full path to params file of vehicle',
    )
    ego_vehicle_identifier_la = DeclareLaunchArgument(
        'ego_vehicle_identifier',
        default_value='forwarder',
        description='Identifier of ego vehicle',
    )
    teleop_la = DeclareLaunchArgument(
        'teleop',
        default_value='True',
        description='Launch teleop',
    )
    rviz2_la = DeclareLaunchArgument(
        'rviz2',
        default_value='True',
        description='Launch rviz2',
    )
    yolo_config_la = DeclareLaunchArgument(
        'yolo_config',
        default_value=os.path.join(waywiser_perception_dir, 'config/yolov3_tinyu.yaml'),
        description='Full path to params file of yolo',
    )
    collision_monitor_config_la = DeclareLaunchArgument(
        'collision_monitor_config',
        default_value=os.path.join(waywiser_perception_dir, 'config/collision_monitor.yaml'),
        description='Full path to params file for CollisionMonitor node.',
    )
    control_vehicle_node_name_la = DeclareLaunchArgument(
        'control_vehicle_node',
        default_value='waywiser_car_node',
        description='Name of the vehicle node to control',
    )

    # include launch files
    agrarsense_orchestrator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_agrarsense_dir,
                    'launch',
                    'agrarsense_orchestrator.launch.py',
                )
            ]
        ),
        launch_arguments={
            'config': LaunchConfiguration('agrarsense_orchestrator_config'),
            'ego_vehicle_role_name': LaunchConfiguration('ego_vehicle_role_name'),
        }.items(),
    )

    waywiser_agrarsense_relay = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_agrarsense_dir,
                    'launch',
                    'waywiser_agrarsense_relay.launch.py',
                )
            ]
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'sim_config': LaunchConfiguration('agrarsense_orchestrator_config'),
            'vehicle_config': LaunchConfiguration('vehicle_config'),
            'ego_vehicle_identifier': LaunchConfiguration('ego_vehicle_identifier'),
        }.items(),
    )

    waywiser_car = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_core_dir,
                    'launch',
                    'waywiser_car.launch.py',
                )
            ]
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'vehicle_config': LaunchConfiguration('vehicle_config'),
        }.items(),
    )

    twist_safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_twist_safety_dir,
                    'launch',
                    'twist_safety.launch.py',
                )
            ]
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'enable_collision_monitor': LaunchConfiguration('enable_collision_monitor'),
            'twist_safety_config': LaunchConfiguration('vehicle_config'),
        }.items(),
    )

    teleop_rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('waywiser'),
                    'launch',
                    'teleop_rviz2.launch.py',
                )
            ]
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'teleop_config': LaunchConfiguration('teleop_config'),
            'teleop': LaunchConfiguration('teleop'),
            'rviz2': LaunchConfiguration('rviz2'),
            'control_vehicle_node': LaunchConfiguration('control_vehicle_node'),
        }.items(),
    )

    yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_perception_dir,
                    'launch',
                    'yolo.launch.py',
                )
            ]
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'yolo_config': LaunchConfiguration('yolo_config'),
        }.items(),
    )
    collision_monitor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_perception_dir,
                    'launch',
                    'collision_monitor.launch.py',
                )
            ]
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'collision_monitor_config': LaunchConfiguration('collision_monitor_config'),
        }.items(),
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(use_sim_time_la)
    ld.add_action(ego_vehicle_role_name_la)
    ld.add_action(agrarsense_orchestrator_config_la)
    ld.add_action(enable_collision_monitor_la)
    ld.add_action(rviz_config_la)
    ld.add_action(teleop_config_la)
    ld.add_action(vehicle_config_la)
    ld.add_action(ego_vehicle_identifier_la)
    ld.add_action(teleop_la)
    ld.add_action(rviz2_la)
    ld.add_action(yolo_config_la)
    ld.add_action(collision_monitor_config_la)
    ld.add_action(control_vehicle_node_name_la)

    # start nodes
    ld.add_action(agrarsense_orchestrator)
    ld.add_action(waywiser_agrarsense_relay)
    ld.add_action(waywiser_car)
    ld.add_action(twist_safety)
    ld.add_action(teleop_rviz2)
    ld.add_action(yolo)
    ld.add_action(collision_monitor)

    return ld
