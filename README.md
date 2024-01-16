# WayWiseR ﹘ [WayWise](https://github.com/RISE-Dependable-Transport-Systems/WayWise) ❤️ [ROS2](https://docs.ros.org/)
WayWise**R** is the integration of WayWise, the rapid prototyping library for connected, autonomous vehicles developed at the RISE Dependable Transport Systems group, with ROS2.  
Broadly speaking, there are two main use cases:
1. **Accelerating bringup of ROS2-powered vehicles.**  
   In this case, the main functionality of the vehicle implemented using ROS2 packages and nodes, e.g., [Nav2](https://github.com/ros-planning/navigation2) or any other package you like.
   WayWise is used for the low-level functionality like talking to motor controller, servo, IMU and GNSS (optional) to publish Odom messages and subscribe to Twist messages through WayWiseR.  
   The main purpose of WayWiseR here is to make it easy to control WayWise-powered vehicles using ROS2.
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
- Marvin Damschen
- Rickard Häll
- Ramana Reddy Avula

## How to use it and what to expect
TODO

### Current state
TODO

## Organization
- **waywiser**: ...
- **waywiser_description**: ...
- **waywiser_gazebo**: ...
- **waywiser_hwbringup**: ...
- **waywiser_node**: ...
- **waywiser_rviz2**: ...
- **waywiser_slam**: ...
- **waywiser_teleop**: ...

## Examples
Use case 1:

https://github.com/RISE-Dependable-Transport-Systems/WayWiseR/assets/2404625/99751b42-a983-4826-a795-4b80ddd2bc28

Use case 2:

https://github.com/RISE-Dependable-Transport-Systems/WayWiseR/assets/2404625/c936d089-d462-4c81-a0ff-e2c9cdb1e4ab

