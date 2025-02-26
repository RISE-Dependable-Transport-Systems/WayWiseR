## Overview

The waywiser_core package contains ROS 2 nodes that wrap the [WayWise](https://github.com/RISE-Dependable-Transport-Systems/WayWise/tree/8334fa6159e43885464b456e87908ba62f550b12) vehicle prototyping library into ROS interfaces​. These nodes bridge low-level vehicle control and sensor data with ROS topics and services, enabling ROS-based autonomy or autopilot features on WayWise-powered vehicles. These nodes can be grouped into two categories: Hardware Interface Nodes and Autopilot Nodes.

### Hardware Interface Nodes

Connects to the vehicle’s hardware (motor controller, steering servo, IMU, GNSS, etc.) via WayWise and exposes it to ROS. These nodes allow a ROS system (e.g. Nav2) to command the vehicle and receive odometry and sensor feedback. They essentially make a physical vehicle controllable through ROS topics.

There are currently 2 such nodes in this category:

- waywise_car
- waywise_truck

### Autopilot Nodes

Implements an onboard autopilot using WayWise’s route-following controllers (e.g. Pure Pursuit). These nodes lets the vehicle drive autonomously by generating velocity commands based on

- a target path generated dynamically by a higher-level ROS2-based path planner (e.g. Nav2)
- a list of waypoints (set manually or using automated scripts) comminicated thorugh MAVLINK, using [ControlTower](https://github.com/RISE-Dependable-Transport-Systems/ControlTower).

These nodes essentially consume localization data and output drive commands, and makes a physical or simulated vehicle navigate autonomously.

There are currently 2 such nodes in this category:

- waywise_car_autopilot
- waywise_truck_autopilot

## Node Hierarchy

Following the waywise vehicle state heirarchy, the waywise_truck and waywise_truck_autopilot are designed with waywise_car and waywise_car_autopilot as base node classes. This means that all the functionalities of waywise_car and waywise_car_autopilot are inherently available to the waywise_truck and waywise_truck_autopilot nodes.
