#!/usr/bin/env bash
set -eo pipefail

# Ensure env vars (defaults if not injected)
export WAYWISER_WS=${WAYWISER_WS:-/workspaces/waywiser_ws}
export WAYWISER_SKIPPED_PACKAGES=${WAYWISER_SKIPPED_PACKAGES:-"waywiser_agrarsense waywiser_carla waywiser_gazebo"}

# Update apt cache
apt update

# Source ROS env
source /opt/ros/jazzy/setup.bash

# rosdep permissions & update (init done in Dockerfile)
sudo rosdep fix-permissions || true
rosdep update

# Create workspace and link this repo into src
mkdir -p "$WAYWISER_WS/src"
if [ ! -e "$WAYWISER_WS/src/WayWiseR" ]; then
    ln -sfn /workspaces/WayWiseR "$WAYWISER_WS/src/WayWiseR"
fi

# Always use HTTPS for GitHub (covers nested submodules too)
git config --global url."https://github.com/".insteadof git@github.com:

# Ensure the WayWise submodule uses HTTPS and then pull it
cd "$WAYWISER_WS/src/WayWiseR"
git config -f .gitmodules submodule.waywiser_core/WayWise.url https://github.com/RISE-Dependable-Transport-Systems/WayWise.git
git submodule sync --recursive
git submodule update --init --recursive --jobs 4

# Python venv + requirements
cd "$WAYWISER_WS"
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r src/WayWiseR/requirements.txt

# Install ROS package deps (respecting the skip list)
cd "$WAYWISER_WS"
PKG_PATHS=$(colcon list --paths-only | { [ -n "$WAYWISER_SKIPPED_PACKAGES" ] && grep -v -w -E "$(echo $WAYWISER_SKIPPED_PACKAGES | tr ' ' '|')" || cat; })
rosdep install --from-paths $PKG_PATHS --ignore-src --rosdistro jazzy -r -y

# Build (skip simulators by default)
colcon build --symlink-install --packages-skip $WAYWISER_SKIPPED_PACKAGES

# Persist env for new terminals (important per your note)
echo "export WAYWISER_WS=$WAYWISER_WS" >> ~/.bashrc
echo "export WAYWISER_SKIPPED_PACKAGES=\"$WAYWISER_SKIPPED_PACKAGES\"" >> ~/.bashrc
echo "source \$WAYWISER_WS/install/local_setup.bash" >> ~/.bashrc

echo "✅ WayWiseR devcontainer setup complete. MAVSDK installed from .deb. Open a NEW terminal for the overlay."