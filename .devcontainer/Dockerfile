# Base: Ubuntu 24.04 + ROS 2 Jazzy
FROM osrf/ros:jazzy-desktop

ENV LANG=C.UTF-8 LC_ALL=C.UTF-8

# Tools & deps (keep handy to speed up devcontainer; Makefile is still idempotent)
RUN apt update && apt install -y --no-install-recommends \
    git build-essential cmake ninja-build pkg-config \
    python3-pip python3-venv python3-colcon-common-extensions \
    python3-rosdep python3-vcstool \
    libunwind-dev libqt5serialport5-dev \
    curl ca-certificates \
 && rm -rf /var/lib/apt/lists/*

# rosdep init at build time (root)
RUN rosdep init || true

# Default shell sources ROS for interactive sessions
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc

WORKDIR /workspaces/WayWiseR
