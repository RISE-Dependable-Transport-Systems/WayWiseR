## Overview

The `waywiser_core` package contains ROS 2 nodes that wrap the [WayWise](https://github.com/das-rise/WayWise) vehicle prototyping library into ROS interfaces. These nodes bridge low-level vehicle control and sensor data with ROS topics and services, enabling ROS-based autonomy or autopilot features on WayWise-powered vehicles.

## Architecture

The package uses a component-based architecture with two main components:

- **Vehicle Interface Component** – Connects to the vehicle's hardware (motor controller, steering servo, IMU, GNSS, etc.) via WayWise and exposes it to ROS. This component allows a ROS system (e.g., Nav2) to command the vehicle and receive odometry and sensor feedback.

- **Autopilot Component** – Implements onboard autopilot using WayWise's route-following controllers (e.g., Pure Pursuit). This component enables autonomous driving by generating velocity commands based on:
  - a target path generated dynamically by a higher-level ROS2-based path planner (e.g. Nav2)
  - a list of waypoints (set manually or using automated scripts) comminicated thorugh MAVLINK, using [ControlTowerNode](https://github.com/das-rise/ControlTowerNode).

## Available Nodes

- `waywiser_car_node` – ROS 2 node for car-like vehicles using the vehicle interface and autopilot components
- `waywiser_truck_node` – ROS 2 node for truck-like vehicles (extends `waywiser_car_node` functionality)
- `waywiser_copter_node` – ROS 2 node for copter-like vehicles (drones) using PX4 (if built with PX4 support)

## Node Hierarchy

Following the WayWise vehicle state hierarchy, `waywiser_truck_node` is designed with `waywiser_car_node` as the base node class. This means that all functionalities of `waywiser_car_node` are inherently available to `waywiser_truck_node`.
