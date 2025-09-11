# Base: Ubuntu 24.04 + ROS 2 Jazzy
FROM osrf/ros:jazzy-desktop

ARG MAVSDK_VERSION=2.14.1
ENV MAVSDK_VERSION=${MAVSDK_VERSION}
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8

# Tools & deps you listed (no sudo/user mgmt)
RUN apt update && apt install -y --no-install-recommends \
    git build-essential cmake ninja-build pkg-config \
    python3-pip python3-venv python3-colcon-common-extensions \
    python3-rosdep python3-vcstool \
    libunwind-dev libqt5serialport5-dev \
    curl ca-certificates \
&& rm -rf /var/lib/apt/lists/*

# rosdep init at build time (root)
RUN rosdep init || true

# Install MAVSDK from prebuilt .deb (auto-pick by arch)
# amd64 -> Ubuntu 22.04 package (works on 24.04); arm64 -> Debian 12 package
ARG MAVSDK_PKG_OVERRIDE=""
ENV MAVSDK_PKG_OVERRIDE=${MAVSDK_PKG_OVERRIDE}
RUN set -eu; \
    base="https://github.com/mavlink/MAVSDK/releases/download/v${MAVSDK_VERSION}"; \
    arch="$(dpkg --print-architecture)"; \
    if [ -n "${MAVSDK_PKG_OVERRIDE}" ]; then \
        pkg="${MAVSDK_PKG_OVERRIDE}"; \
    else \
        case "${arch}" in \
        amd64) pkg="libmavsdk-dev_${MAVSDK_VERSION}_ubuntu22.04_amd64.deb" ;; \
        arm64) pkg="libmavsdk-dev_${MAVSDK_VERSION}_debian12_arm64.deb" ;; \
        *) echo "Unsupported architecture: ${arch}"; exit 1 ;; \
        esac; \
    fi; \
    echo "Installing MAVSDK ${MAVSDK_VERSION} (${pkg})"; \
    curl -fsSL -o /tmp/mavsdk.deb "${base}/${pkg}"; \
    dpkg -i /tmp/mavsdk.deb || { apt update && apt install -y -f && dpkg -i /tmp/mavsdk.deb; }; \
    rm -f /tmp/mavsdk.deb


# Default shell sources ROS for interactive sessions
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc

WORKDIR /workspaces/WayWiseR
