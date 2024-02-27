# WayWiseR ﹘ [WayWise](https://github.com/RISE-Dependable-Transport-Systems/WayWise) ❤️ [ROS2](https://docs.ros.org/)

![Workflow build result](https://github.com/RISE-Dependable-Transport-Systems/WayWiseR/actions/workflows/build.yaml/badge.svg)

WayWise**R** is the integration of WayWise, the rapid prototyping library for connected, autonomous vehicles developed at the RISE Dependable Transport Systems group, with ROS2.
Both WayWise and WayWiseR are focused on our research projects.
Broadly speaking, there are two main use cases:

1. **Accelerating bringup of ROS2-powered vehicles.**
   In this case, the main functionality of the vehicle is implemented using ROS2 packages and nodes, e.g., [Nav2](https://github.com/ros-planning/navigation2) or any other package you like.
   WayWise is used for the low-level functionality like talking to motor controller, servo, IMU and GNSS (optional) to publish Odom messages and subscribe to Twist messages through WayWiseR.
   The main purpose of WayWiseR here is to make it easy to bringup ROS2-powered vehicles using Waywise.
   <img width="1465" alt="Use Case 1" src="https://github.com/RISE-Dependable-Transport-Systems/WayWiseR/assets/2404625/468456d0-2130-4602-a803-2553a65fd220">

2. **Extended functionality for WayWise-powered vehicles.**
   In this case, the main functionality is implemented using WayWise (e.g., route following using pure pursuit, control and planning using [ControlTower](https://github.com/RISE-Dependable-Transport-Systems/ControlTower)).
   ROS2 adds additional functionality like localization using [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) or advanced simulations in combination with, e.g., [Gazebo](https://gazebosim.org).
   Compared to use case 1, directions of Odom and Twist messages would tentatively reverse: WayWise would publish Twist messages through WayWiseR to control the simulated vehicle and subscribe to Odom messages.
   <img width="1895" alt="Use Case 2" src="https://github.com/RISE-Dependable-Transport-Systems/WayWiseR/assets/2404625/543b7e36-41ea-4ade-ad8f-f54034e85162">

In both cases, WayWiseR provides an abstraction layer between WayWise and ROS2, as well as launch files and configuration to quickly get you started.
A typical project might start out in simulation using ROS2 and Gazebo. Then, WayWiseR is used to bring it to a real vehicle (1. use case).
Alternatively, a project could start out with a ROS2-supported sensor that you want to gather data with (say a LiDAR). Then, WayWise could be used to make the sensor mobile with exact positioning and waypoint following (2. use case).
Considering the modularity of ROS2 and WayWise, a project could also do something entirely different that does not clearly fit into one of the use cases. 😊

Main authors are (firstname.lastname@ri.se):

- Ramana Reddy Avula
- Rickard Häll
- Marvin Damschen

## How to use it and what to expect

This ROS2 package is meant to be cloned into a ROS2 workspace.
We do not do releases (for the time being) and do not promise a stable API but stick to standard ROS messages wherever possible.
In general, our development resources are scarce and dedicated to fulfill use cases of research projects we are part of. We do our best to avoid it, but things will break from time to time.

### How to install and build (on Ubuntu 22.04)

Install ROS2 (required):

- [Install ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

Install MAVSDK 2.0 or newer (required):

- [Download MAVSDK pre-built releses](https://github.com/mavlink/MAVSDK/releases)

  `sudo dpkg -i libmavsdk-dev*.deb`

To instead build MAVSDK from source (optional):

- [Scripts can be found in the WayWise repository](https://github.com/RISE-Dependable-Transport-Systems/WayWise/tree/main/tools/build_MAVSDK)

Gazebo simulator can be installed on dektop (optional):

- [Install Gazebo](https://gazebosim.org/docs/fortress/install_ubuntu)

To automatically source the ROS2 environment when a new terminal is opened:

`echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc`

Setup workspace and build our package:

    sudo apt update && sudo apt install -y libunwind-dev libqt5serialport5-dev git build-essential cmake python3-colcon-common-extensions

    mkdir -p ~/waywiser_ws/src
    cd ~/waywiser_ws/src
    git clone --recurse-submodules git@github.com:RISE-Dependable-Transport-Systems/WayWiseR.git
    cd ..
    rosdep install -i --from-path src --rosdistro humble -r -y
    colcon build --symlink-install
    colcon test

Before sourcing the overlay, it is very important that you open a new terminal, separate from the one where you built the workspace. Sourcing an overlay in the same terminal where you built, or likewise building where an overlay is sourced, may create complex issues.

`source install/local_setup.bash`

### Current state

The current state presents a foundation that our research projects [AGRARSENSE](https://www.ri.se/en/what-we-do/projects/agrarsense) and [SUNRISE](https://www.ri.se/en/what-we-do/projects/safety-assurance-framework-for-connected-automated-mobility-systems) will build upon during 2024 to investigate safety-critical situational awareness in the forestry and road vehicle contexts, respectively.

## Organization

WayWiseR is divided into several ROS2 packages. Make sure to have a look into the respecitve package.xml files.

- **waywiser**: A meta package that depends on all packages below to be able to refer to WayWiseR as a whole.
- **waywiser_description**: Contains vehicle descriptions in the form of [xacro](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html) files. Currently a single vehicle is available that corresponds to a [Traxxas](https://traxxas.com/) Slash incl. camera, depth camera, LiDAR and IMU.
- **waywiser_gazebo**: Everything related to simulation using [Gazebo](https://gazebosim.org).
- **waywiser_hwbringup**: Configuration and launch files to get real (not simulated) vehicles running.
- **waywiser_nav2**: Package that provides dynamic path planning using Nav2.
- **waywiser_node**: Wraps [WayWise](https://github.com/RISE-Dependable-Transport-Systems/WayWise) into ROS2 nodes (currently a single node).
- **waywiser_perception**: Provides image-processing and computer vision functionalities using [YOLOv8](https://docs.ultralytics.com/).
- **waywiser_rviz2**: Configuration and launch files for RViz2.
- **waywiser_slam**: Configuration and launch files for [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox).
- **waywiser_teleop**: Configuration and launch files for teleop packages (handling keyboard or gamepad input) and a node to arbitrate between them.
- **waywiser_twist_safety**: Contains configuration and launch files to manage twist commands to vehicle from different sources. It also includes a composable node for emergency stop monitoring.

## Examples

Use case 1 ﹘ Nav2 on top of WayWise:

https://github.com/RISE-Dependable-Transport-Systems/WayWiseR/assets/2404625/99751b42-a983-4826-a795-4b80ddd2bc28

Use case 2 ﹘ WayWise autopilot driving in Gazebo:

https://github.com/RISE-Dependable-Transport-Systems/WayWiseR/assets/2404625/c936d089-d462-4c81-a0ff-e2c9cdb1e4ab

## Funded by

<img src="https://user-images.githubusercontent.com/2404625/202213271-a4006999-49d5-4e61-9f3d-867a469238d1.png" width="120" height="81" align="left" alt="EU logo" />
This project has received funding from the European Union’s Horizon Europe research and innovation programme under grant agreement nº 101095835 and nº 101069573. The results reflect only the authors' view and the Agency is not responsible
for any use that may be made of the information it contains.
