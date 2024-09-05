# Based on https://robotics.stackexchange.com/questions/97296/building-ros1-bridge-fails-in-docker-container-noetic-foxy
# with apt key fix from https://github.com/osrf/docker_images/issues/607#issuecomment-1058734931
FROM osrf/ros:foxy-ros1-bridge

# Install dependencies
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
  && apt-get update \
  && apt-get install -y ssh build-essential gcc g++ gdb clang cmake rsync nano tar screen python3 ros-foxy-launch* \
  && apt-get install -y ros-noetic-rosauth ros-noetic-rosbridge-suite \
  && apt-get clean

# Set up workspace and clone repositories
WORKDIR "/home/workspace"
RUN colcon build --symlink-install --packages-skip ros1_bridge

# build AGRARSENSE_ROS_Messages
RUN git clone https://AMKFrostBit@dev.azure.com/AMKFrostBit/AGRARSENSE/_git/AGRARSENSE_ROS_Messages src/AGRARSENSE_ROS_Messages
WORKDIR "/home/workspace/src/AGRARSENSE_ROS_Messages/ros1ws/"
RUN . /opt/ros/noetic/setup.sh && catkin_make
WORKDIR "/home/workspace/src/AGRARSENSE_ROS_Messages/ros2ws/"
RUN . /opt/ros/foxy/setup.sh && colcon build

# Build ros1_bridge
WORKDIR "/home/workspace"
RUN git clone --single-branch --branch foxy https://github.com/ros2/ros1_bridge src/ros1_bridge
WORKDIR "/home/workspace/src/ros1_bridge"
RUN . /opt/ros/noetic/setup.sh \
  && . /opt/ros/foxy/setup.sh \
  && . /home/workspace/src/AGRARSENSE_ROS_Messages/ros1ws/devel/setup.sh \
  && . /home/workspace/src/AGRARSENSE_ROS_Messages/ros2ws/install/setup.sh \
  && colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure

# Set environment variables
ENV ROS_DOMAIN_ID=0
ENV ROS_PORT=9090

# Create start scripts for each bridge service
WORKDIR "/home/workspace"
RUN echo '#!/bin/bash' > start_ue5_ros1_bridge \
  && echo 'source /opt/ros/noetic/setup.bash' >> start_ue5_ros1_bridge \
  && echo 'source src/AGRARSENSE_ROS_Messages/ros1ws/devel/setup.bash' >> start_ue5_ros1_bridge \
  && echo 'exec roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=True port:=${ROS_PORT}' >> start_ue5_ros1_bridge \
  && chmod +x start_ue5_ros1_bridge

RUN echo '#!/bin/bash' > start_ros1_ros2_bridge \
  && echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> start_ros1_ros2_bridge \
  && echo 'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}' >> start_ros1_ros2_bridge \
  && echo 'echo ROS_DOMAIN_ID is set to ${ROS_DOMAIN_ID}' >> start_ros1_ros2_bridge \
  && echo 'source /opt/ros/noetic/setup.bash' >> start_ros1_ros2_bridge \
  && echo 'source /opt/ros/foxy/setup.bash' >> start_ros1_ros2_bridge \
  && echo 'source src/AGRARSENSE_ROS_Messages/ros2ws/install/setup.bash' >> start_ros1_ros2_bridge \
  && echo 'exec ros2 run ros1_bridge dynamic_bridge --bridge-all-topics' >> start_ros1_ros2_bridge \
  && chmod +x start_ros1_ros2_bridge


CMD ["/bin/bash"]
