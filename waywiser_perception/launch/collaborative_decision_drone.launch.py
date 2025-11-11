# Orchestrating launch file — Drone #TODO NOT TESTED
# (needs to be on the same network as the Shuttle) 

# 1. Start OAK-D camera, remap "detections_topic" to "detections_topic_drone" for the next step. Command: "ros2 launch depthai_ros_driver camera.launch.py params_file:=waywiser_hwbringup/config/oak_d.yaml"

# 2. Continuously convert detections_topic_drone (camera-relative coordinates) to global coordinates and publish as detections_topic_global #TODO
#     - Use coordinatetransforms.h infrastructure from WayWise 
#     - Access global coordinates of Drone from /nav_sat_fix topic 

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction, Node
from launch_ros.actions import PushNodeRemapping

def generate_launch_description():
    waywiser_hwbringup_dir = get_package_share_directory('waywiser_hwbringup')
    depthai_dir = get_package_share_directory('depthai_ros_driver')

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

    # remap output for global coordinates conversion
    remappings = [
        ('detections_topic', 'detections_topic_drone'),
    ]

    # Wrap camera launch in a GroupAction with PushNodeRemapping so all the nodes started by it gets the remapping applied
    camera_group = GroupAction(
        actions=[
            PushNodeRemapping(remappings=remappings),
            camera,
        ]
    )

    # Create the global coordinate converter node
    global_converter = Node(
        package='waywiser_perception',
        executable='detection_global_converter_node.py',
        name='detection_global_converter'
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(camera_group)
    ld.add_action(global_converter)

    return ld