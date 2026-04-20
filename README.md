# WayWiseR

![Workflow build result](https://github.com/das-rise/WayWiseR/actions/workflows/build.yaml/badge.svg) [![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/das-rise/WayWiseR)

**WayWiseR** is a [ROS2](https://docs.ros.org/)-based rapid prototyping platform designed for **Connected and Automated Vehicle (CAV)** validation research. Extending the low-level functionalities provided by the [WayWise](https://github.com/das-rise/WayWise) library with standardized ROS2 interfaces, WayWiseR enables systematic scenario-based validation across both simulated and physical environments.

WayWiseR is divided into modular ROS2 packages:

| Package                                      | Description                                                       |
| :------------------------------------------- | :---------------------------------------------------------------- |
| `waywiser`                                   | FastDDS discovery configuration and global utils.                 |
| `waywiser_core`                              | ROS2 wrappers for the core WayWise functionalities.               |
| `waywiser_description`                       | Vehicle and sensor descriptions (URDF/Xacro).                     |
| `waywiser_hwbringup`                         | Configuration and launch files for physical hardware.             |
| `waywiser_perception`                        | Image processing and computer vision.                             |
| `waywiser_nav2`                              | Dynamic path planning via Nav2.                                   |
| `waywiser_rviz2`                             | RViz2 configuration and launch files.                             |
| `waywiser_slam`                              | SLAM Toolbox configuration and launch files.                      |
| `waywiser_teleop`                            | Multi-source teleoperation and arbitration.                       |
| `waywiser_test_runner`                       | Test orchestration for both simulators and hardware.              |
| `waywiser_twist_safety`                      | Onboard safety features such as command arbitration, E-Stop, etc. |
| `waywiser_gazebo` / `_carla` / `_agrarsense` | Simulation-specific integration and environments.                 |

---

## Installation & Build (Ubuntu 22.04)

1. **Prerequisites:**
   - Install [uv](https://github.com/astral-sh/uv?tab=readme-ov-file#installation) for Python package management:

     ```bash
     curl -LsSf https://astral.sh/uv/install.sh | sh
     ```

   - Install ROS2 Humble following the [official guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

   - Install the latest MAVSDK 2.x version (3.x is not yet supported) by downloading the latest deb package from [prebuilt releases](https://github.com/mavlink/MAVSDK/releases) and running:

     ```bash
     sudo dpkg -i libmavsdk-dev*.deb
     ```

     Alternatively, MAVSDK can be built from source by using the scripts [here](https://github.com/das-rise/WayWise/tree/main/tools/build_MAVSDK).

   - Install system dependencies:

     ```bash
     sudo apt update && sudo apt install -y \
         libunwind-dev \
         libqt5serialport5-dev \
         git \
         build-essential \
         cmake \
         python3-colcon-common-extensions \
         python3-pyqt5
     ```

2. **Initialize the workspace:**

   First, define the path to your workspace. It is recommended to add this to your `~/.bashrc` to make it persistent across all your terminal sessions:

   ```bash
   export WAYWISER_WS=~/waywiser_ws # Update with your desired path
   grep -q '^export WAYWISER_WS=' ~/.bashrc && { sed -i "s|^export WAYWISER_WS=.*|export WAYWISER_WS=$WAYWISER_WS|" ~/.bashrc && echo "Updated WAYWISER_WS=$WAYWISER_WS in ~/.bashrc"; } || { echo "export WAYWISER_WS=$WAYWISER_WS" >> ~/.bashrc && echo "Added WAYWISER_WS=$WAYWISER_WS to ~/.bashrc"; }
   ```

   Now, create the workspace and clone the repository:

   ```bash
   mkdir -p $WAYWISER_WS/src
   git clone git@github.com:das-rise/WayWiseR.git $WAYWISER_WS/src/WayWiseR
   git -C $WAYWISER_WS/src/WayWiseR submodule update --init waywiser_core/WayWise
   ```

   WayWiseR uses a `.env` file to manage ROS configuration and skipped packages. First, copy the `.env.example` to establish your environment settings:

   ```bash
   cp $WAYWISER_WS/src/WayWiseR/.env.example $WAYWISER_WS/src/WayWiseR/.env
   ```

   Open `.env` and verify/update `WAYWISER_SKIPPED_PACKAGES` and your desired configurations. 
   
   > ⚠️ **Important**: Because the simulation packages (`waywiser_agrarsense`, `waywiser_carla`, `waywiser_gazebo`) require significant additional system setup, it is **strictly recommended** to include them in the `WAYWISER_SKIPPED_PACKAGES` list for this initial workspace build. You should proceed with building the core workspace first, and then build each of these packages later by following the specialized instructions in their respective package `README.md` files.

3. **Setup Python virtual environment:**

   ```bash
   cd $WAYWISER_WS
   export PY_VER=$(cat src/WayWiseR/.python-version)
   uv venv --python $PY_VER --clear
   source .venv/bin/activate
   export PYTHONPATH=$WAYWISER_WS/.venv/lib/python${PY_VER}/site-packages:$PYTHONPATH
   uv pip install -e src/WayWiseR
   ```

4. **Build the workspace:**

   ```bash
   cd $WAYWISER_WS
   source /opt/ros/humble/setup.bash
   set -a && source $WAYWISER_WS/src/WayWiseR/.env && set +a
   rosdep install --from-paths $(colcon list --paths-only | grep -Evw "$(echo "$WAYWISER_SKIPPED_PACKAGES" | tr ' ' '|')") --ignore-src --rosdistro $ROS_DISTRO -r -y
   colcon build --symlink-install --packages-skip $WAYWISER_SKIPPED_PACKAGES
   ```

   > 💡 **Tip**: To persist environment variables, append the setup block to your `.venv/bin/activate` by running the following command in your terminal:
   >
   > ```bash
   > cat <<EOT >> $WAYWISER_WS/.venv/bin/activate
   > # WayWiseR Environment Setup
   > set -a; source "\$WAYWISER_WS/src/WayWiseR/.env"; set +a
   > source /opt/ros/humble/setup.bash
   > export PYTHONPATH=\$WAYWISER_WS/.venv/lib/python${PY_VER}/site-packages:\$PYTHONPATH
   >
   > if [ -f "\$WAYWISER_WS/install/setup.bash" ]; then
   >   source "\$WAYWISER_WS/install/setup.bash"
   > fi
   > EOT
   > ```

   To build simulator-related packages such as waywiser_agrarsense, waywiser_carla, and waywiser_gazebo, follow the instructions in the respective packages.

5. **Source the workspace:**

   ```bash
   source $WAYWISER_WS/install/setup.bash
   ```

---

## Developer Guide

For detailed developer setup configuration, refer to the [Developer Guide](.github/DEVELOPER_GUIDE.md).

---

## Demos

### Autonomous reverse docking of a semi-truck (1:14 scaled) prototype

https://github.com/user-attachments/assets/10458e63-c195-4a39-ae8a-7fcdc91dceaf

### Safety-critical human detection and emergency braking in forestry simulations

https://github.com/user-attachments/assets/b987e2b1-8aab-4627-a7d1-eda75a33960e

---

## Maintainers

Current maintainers are: (firstname.middlename.lastname@ri.se):

- Ramana Reddy Avula
- Aria Mirzai
- Karl Lundgren
- Ashfaq Farooqui

Previous maintainers:

- Marvin Damschen
- Rickard Häll

---

## Citation

If you use WayWiseR in your research, please cite:

```bibtex
@INPROCEEDINGS{11369551,
  author={Avula, Ramana Reddy and Damschen, Marvin and Mirzai, Aria and Lundgren, Karl and Farooqui, Ashfaq and Thors\'{e}n, Anders},
  booktitle={2025 13th International Conference on Control, Mechatronics and Automation (ICCMA)},
  title={WayWiseR: A Rapid Prototyping Platform for Validating Connected and Automated Vehicles},
  year={2025},
  pages={306-311},
  doi={10.1109/ICCMA67641.2025.11369551}
}
```

## Funding

<img src="https://user-images.githubusercontent.com/2404625/202213271-a4006999-49d5-4e61-9f3d-867a469238d1.png" width="120" height="81" align="left" alt="EU logo" />
This project has received funding from the European Union’s Horizon Europe research and innovation programme under grant agreements 101095835 and 101069573. The results reflect only the authors' view, and the Agency is not responsible
for any use that may be made of the information it contains.
