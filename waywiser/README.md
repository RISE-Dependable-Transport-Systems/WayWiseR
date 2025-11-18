### Email Configuration

To enable email functionality in WayWiseR nodes (for example, notifications from `waywiser_test_runner`), you need to provide email credentials in a `.env` file and set the environment variable `WAYWISER_DOTENV_PATH` to point to it.

    export WAYWISER_DOTENV_PATH=$WAYWISER_WS/.env
    echo "export WAYWISER_DOTENV_PATH=$WAYWISER_DOTENV_PATH" >> .venv/bin/activate

The `.env` file should contain the following variables:

    EMAIL_USER=''
    EMAIL_PASSWORD=''
    EMAIL_RECIPIENT=''
    SMTP_SERVER=smtp.gmail.com
    SMTP_PORT=465

Note: If you are using Gmail, use an App Password for the `EMAIL_PASSWORD` variable.

## (Optional) FastDDS discovery server and client setup

### On server

- Open a new terminal and run (optionally, add these lines to ~/.bashrc):

  ```
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_DOMAIN_ID=0
  ```

- In the same terminal, run:

  ```
  cd ~/waywiser_ws
  ./install/waywiser/discovery/server_setup.sh
  # To enable remote server, run:
  # ./install/waywiser/discovery/server_setup.sh -r -s <server_ip>
  # To display help for this script, run:
  # ./install/waywiser/discovery/server_setup.sh -h
  ```

### On client

- Open a new terminal and run (optionally, add these lines to ~/.bashrc):

  ```
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_DOMAIN_ID=0
  ```

- In the same terminal, run:

  ```
  cd ~/waywiser_ws
  source install/waywiser/discovery/client_setup.sh
  # If running on remote machine, run:
  # source install/waywiser/discovery/client_setup.sh -r -s <server_ip> -c <client_ip>
  # To display help for this script, run:
  # ./install/waywiser/discovery/client_setup.sh -h
  ros2 daemon stop && ros2 daemon start
  ros2 run demo_nodes_cpp talker
  ```

- Open a new terminal and run:

  ```
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_DOMAIN_ID=0
  cd ~/waywiser_ws
  source install/waywiser/discovery/client_setup.sh
  # If running on remote machine, run:
  # source install/waywiser/discovery/client_setup.sh -r -s <server_ip> -c <client_ip>
  ros2 run demo_nodes_cpp listener
  ```

- Open a new terminal and run:

  ```
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_DOMAIN_ID=0
  cd ~/waywiser_ws
  source install/waywiser/discovery/client_setup.sh -su
  # If running on remote machine, run:
  # source install/waywiser/discovery/client_setup.sh -su -r -s <server_ip> -c <client_ip>
  ros2 node list
  ```

Few system-level network parameter tunings can address some issues faced while using various DDS implementations on Linux in real-world situations. See [here](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html#cross-vendor-tuning) for additional guidance.
