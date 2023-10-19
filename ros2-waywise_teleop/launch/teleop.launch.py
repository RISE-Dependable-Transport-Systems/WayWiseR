import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():     
    teleop_dir = get_package_share_directory('ros2-waywise_teleop')
    
    joy_la = DeclareLaunchArgument(
        'joy_config', default_value=os.path.join(teleop_dir, 'config/joy_teleop.yaml'),
        description='Full path to params file')
    
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_config')]
    )    
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[LaunchConfiguration('joy_config')]
    )   
    
    # create launch description
    ld = LaunchDescription()
    
    # declare launch args
    ld.add_action(joy_la)
        
    # start nodes    
    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)
    return ld
