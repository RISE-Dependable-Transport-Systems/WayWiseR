# Orchestrating launch file — Shuttle #TODO NOT TESTED
# (needs to be on the same network as the Drone) 
 
# 1. "gnss_print_verbose" is printed to informs when ublox-F9R enters fusion mode (set to true in shuttle_small_scale.yaml)

# 2. Start OAK-D camera, publishes Person detections to "detections_topic". Command: "ros2 launch depthai_ros_driver camera.launch.py params_file:=waywiser_hwbringup/config/oak_d.yaml"

# 3. Start VESC via shuttle.launch.py

# 4. Run teleop.launch.py to enable controller/keyboard input (topic "/joy")

# 5. Continuously transform detections_topic_global back to camera-relative coordinates and publish to detections_topic #TODO

# 6. Start Collision Monitor via collision_monitor.launch.py, listens to "detections_topic" and outputs "emergency_stop_topic"

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    waywiser_hwbringup_dir = get_package_share_directory('waywiser_hwbringup')
    waywiser_teleop_dir = get_package_share_directory('waywiser_teleop')
    waywiser_perception_dir = get_package_share_directory('waywiser_perception')
    depthai_dir = get_package_share_directory('depthai_ros_driver')

    # args that can be set from the command line or a default will be used
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='True', 
        description='Use simulation clock'
    )
    collision_monitor_config_la = DeclareLaunchArgument(
        'collision_monitor_config',
        default_value=os.path.join(waywiser_perception_dir, 'config/collision_monitor.yaml'),
        description='Full path to params file for CollisionMonitor node.',
    )

    # include launch files
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    depthai_dir,
                    'launch',
                    'camera.launch.py',
                )
            ]
        ),
        launch_arguments={
            'params_file': os.path.join(waywiser_hwbringup_dir, 'config', 'oak_d.yaml'),
        }.items(),
    )

    shuttle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_hwbringup_dir,
                    'launch',
                    'shuttle.launch.py',
                )
            ]
        )
    )
    
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_teleop_dir,
                    'launch',
                    'teleop.launch.py',
                )
            ]
        )
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
    ld.add_action(camera)
    ld.add_action(shuttle)
    ld.add_action(teleop)
    ld.add_action(collision_monitor_config_la)
    ld.add_action(collision_monitor)

    return ld
