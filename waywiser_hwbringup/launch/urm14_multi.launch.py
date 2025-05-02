import os
import yaml # You might need to install pyyaml: pip install pyyaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction # Reverted GroupAction import
from launch_ros.actions import PushROSNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the share directory of your package
    hw_bringup_dir = get_package_share_directory("waywiser_hwbringup")
    # Define the path to the original single sensor launch file
    single_launch_file_path = os.path.join(hw_bringup_dir, 'launch', 'urm14_single.launch.py')

    # Declare an argument for the configuration file listing the instances
    # This allows you to specify a different config file when launching
    multi_config_file = os.path.join(hw_bringup_dir, "config", "urm14_instances.yaml")
    instances_config_arg = DeclareLaunchArgument(
        "instances_config_file",
        default_value=multi_config_file, # Default path to your instances config
        description="Full path to the YAML file listing the instances to launch"
    )

    # List to hold all the launch actions for the different instances
    launch_description_actions = []

    # Read the configuration file and create launch actions for each instance
    try:
        with open(multi_config_file, 'r') as f:
            config = yaml.safe_load(f)
            # Get the list of instances from the 'instances' key in the YAML
            instances = config.get('instances', [])

            if not instances:
                print("Warning: No instances found in the configuration file.")

            # Iterate through each instance defined in the YAML
            for instance in instances:
                # Get the namespace and relative config file path for the instance
                namespace = instance.get('namespace')
                relative_config_path = instance.get('config_file') # Path from YAML

                # Skip if essential information is missing
                if not namespace or not relative_config_path:
                    print(f"Warning: Skipping instance with missing namespace or config_file: {instance}")
                    continue

                # Construct the absolute path for the instance's parameter file
                absolute_config_path = os.path.join(hw_bringup_dir, relative_config_path)

                # Create a group of actions for this instance to apply the namespace
                # GroupAction is used to apply actions like PushROSNamespace to a set of subsequent actions
                instance_group = GroupAction(
                    actions=[
                        # Apply the specific namespace for this instance
                        PushROSNamespace(namespace),

                        # Include the original launch file for this instance
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(single_launch_file_path),
                            launch_arguments={
                                # Pass the specific config file path to the original launch file's argument
                                'urm14_config': absolute_config_path,
                            }.items() # .items() converts the dict to a list of tuples for launch arguments
                        )
                    ]
                )
                # Add the group of actions for this instance to the main list
                launch_description_actions.append(instance_group)

    except FileNotFoundError:
        print(f"Error: Instances configuration file not found at {multi_config_file}")
        # Return an empty launch description so ROS 2 launch doesn't crash completely
        return LaunchDescription([])
    except yaml.YAMLError as e:
        print(f"Error parsing YAML file {multi_config_file}: {e}")
        return LaunchDescription([])
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return LaunchDescription([])


    # Create the main LaunchDescription object
    ld = LaunchDescription(launch_description_actions)

    # Add the configuration file argument declaration itself to the launch description
    # This allows the user to override the default config file path
    ld.add_action(instances_config_arg)

    return ld
