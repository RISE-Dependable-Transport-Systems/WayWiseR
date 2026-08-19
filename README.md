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

The simulation-specific integrations are maintained in seperate repositories: [WayWiseR_Agrarsense](https://github.com/das-rise/WayWiseR_Agrarsense), [WayWiseR_Carla](https://github.com/das-rise/WayWiseR_Carla), and [WayWiseR_Gazebo](https://github.com/das-rise/WayWiseR_Gazebo).

---

## Installation (Ubuntu 22.04)

1. **Clone the repository:**

   ```bash
   export WAYWISER_WS=~/waywiser_ws   # update to your desired path
   mkdir -p $WAYWISER_WS/src
   git clone git@github.com:das-rise/WayWiseR.git $WAYWISER_WS/src/WayWiseR
   ln -s $WAYWISER_WS/src/WayWiseR/Makefile $WAYWISER_WS/Makefile
   ```

2. **Build the workspace:**

   ```bash
   cd $WAYWISER_WS
   make all
   ```

   This will:
   - Prompt you to configure your `.env` (packages to build, RMW, middleware settings, etc.)
   - Install all system prerequisites
   - Create and populate the Python virtual environment with only the Python extras needed by the selected packages
   - Install ROS dependencies via rosdep
   - Build the workspace with colcon

   For a list of all available make targets:

   ```bash
   make help
   ```

   > 💡 **Notes**:
   > - **MAVSDK**: Automatically installed on Ubuntu 22.04 (amd64). On other architectures, install it manually and skip prereqs with `make all ARGS="--skip-prereqs"`.
   > - **Ground-only builds**: Disable PX4/drone support via `WAYWISER_BUILD_PX4_DRONE=0` in `.env` (`make configure`) or pass `ARGS="--skip-px4-drone"` (e.g., `make all ARGS="--skip-px4-drone"`).

3. **Activate the workspace:**

   ```bash
   source $WAYWISER_WS/.venv/bin/activate
   ```

   The venv activation script automatically sources ROS2, the workspace install, and your `.env`.

## Quick Start

Launch a rover with teleoperation:

```bash
cd $WAYWISER_WS
source .venv/bin/activate
ros2 launch waywiser hwbringup_rover.launch.py
```

The rover dynamics are simulated by WayWise using a simple bicycle model.

---

## Developer Guide

For detailed developer setup configuration, refer to the [Developer Guide](.github/DEVELOPER_GUIDE.md).

---

## Demos

### Autonomous reverse docking of a semi-truck (1:14 scaled) prototype

https://github.com/user-attachments/assets/10458e63-c195-4a39-ae8a-7fcdc91dceaf

### Safety-critical human detection and emergency braking in forestry simulations

https://github.com/user-attachments/assets/b987e2b1-8aab-4627-a7d1-eda75a33960e

### Autonomous UAV mission execution in Gazebo using PX4 SITL

https://github.com/user-attachments/assets/139a8611-7e69-4d19-9e98-9aea803044eb

---

## Maintainers

Current maintainers are: (firstname.middlename.lastname@ri.se):

- Aria Mirzai
- Karl Lundgren
- Ashfaq Farooqui

Previous maintainers:

- Ramana Reddy Avula
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
