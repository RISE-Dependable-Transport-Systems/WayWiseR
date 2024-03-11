## (Optional) FastDDS discovery server and client setup

### On server

- Open a new terminal and run (optionally, add these lines to ~/.bashrc):

  ```
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_DOMAIN_ID=0
  ```

- In the same terminal, run:

  ```
  ./waywiser_ws/install/waywiser/discovery/server_setup.sh
  # To enable remote server, run:
  # ./waywiser_ws/install/waywiser/discovery/server_setup.sh -r -s <server_ip>
  # To display help for this script, run:
  # ./waywiser_ws/install/waywiser/discovery/server_setup.sh -h
  ```

### On client

- Open a new terminal and run (optionally, add these lines to ~/.bashrc):

  ```
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_DOMAIN_ID=0
  ```

- In the same terminal, run:

  ```
  ./waywiser_ws/install/waywiser/discovery/client_setup.sh
  # If running on remote machine, run:
  # ./waywiser_ws/install/waywiser/discovery/client_setup.sh -r -s <server_ip> -c <client_ip>
  # To display help for this script, run:
  # ./waywiser_ws/install/waywiser/discovery/client_setup.sh -h
  ros2 daemon stop && ros2 daemon start
  ros2 run demo_nodes_cpp talker
  ```

- Open a new terminal and run:

  ```
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_DOMAIN_ID=0
  ./waywiser_ws/install/waywiser/discovery/client_setup.sh
  # If running on remote machine, run:
  # ./waywiser_ws/install/waywiser/discovery/client_setup.sh -r -s <server_ip> -c <client_ip>
  ros2 run demo_nodes_cpp listener
  ```

- Open a new terminal and run:

  ```
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_DOMAIN_ID=0
  ./waywiser_ws/install/waywiser/discovery/client_setup.sh -su
  # If running on remote machine, run:
  # ./waywiser_ws/install/waywiser/discovery/client_setup.sh -su -r -s <server_ip> -c <client_ip>
  ros2 node list
  ```

Few system-level network parameter tunings can address some issues faced while using various DDS implementations on Linux in real-world situations. See [here](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html#cross-vendor-tuning) for additional guidance.
