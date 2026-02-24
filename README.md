# rh56dfx_description

A ROS 2 URDF/xacro description package for the **Inspire Robotics RH56DFX** 6-DOF dexterous robotic hand. Supports both **left** and **right** hand variants with full mesh geometry, joint limits, and ros2_control integration.

> **Credits:** The RH56DFX hand hardware is designed and manufactured by [Inspire Robotics](http://www.inspire-robots.com). This ROS 2 description package provides URDF/xacro models and ros2_control configuration for simulation and integration purposes.

---

## Overview

The RH56DFX is a high-performance dexterous hand with 6 degrees of freedom featuring:
- **4 fingers** (index, middle, ring, little) each with 2 links + rubber pads + fingertip
- **1 thumb** with 4 links + rubber pads + fingertip
- **Wrist** with yaw and base rotation joints
- **Mimic joints** for realistic multi-link finger curling from a single actuator command
- **Palm** link with soft contact geometry
- Compact aluminium construction designed for mounting on industrial robot arms


This package provides everything needed to:
- Visualise the hand in RViz
- Integrate it into a larger robot URDF via xacro macros
- Control it with ros2_control JointTrajectoryController

---

## Package Contents

```
rh56dfx_description/
├── config/
│   ├── left/
│   │   ├── joint_limits.yaml       # Left hand joint position/velocity limits
│   │   ├── joint_names.yaml        # Left hand joint name definitions
│   │   └── ros2_controllers.yaml   # Left hand ros2_control config
│   └── right/
│       ├── joint_limits.yaml       # Right hand joint position/velocity limits
│       ├── joint_names.yaml        # Right hand joint name definitions
│       └── ros2_controllers.yaml   # Right hand ros2_control config
├── launch/
│   └── display.launch.py           # Visualisation launch with RViz + joint sliders
├── meshes/
│   ├── left/
│   │   ├── collision/              # Simplified collision STL meshes (left)
│   │   └── visual/                 # High-resolution visual STL meshes (left)
│   └── right/
│       ├── collision/              # Simplified collision STL meshes (right)
│       └── visual/                 # High-resolution visual STL meshes (right)
├── rviz/
│   ├── rh56dfx_left.rviz           # Pre-configured RViz layout for left hand
│   └── rh56dfx_right.rviz          # Pre-configured RViz layout for right hand
└── urdf/
    ├── rh56dfx.xacro               # Top-level hand URDF (standalone)
    ├── rh56dfx_macro.xacro         # Reusable xacro macro for embedding in larger robots
    ├── rh56dfx_left.csv            # Left hand DH/joint parameter data
    └── rh56dfx_right.csv           # Right hand DH/joint parameter data
```

---

## Requirements

| Dependency | Version |
|---|---|
| ROS 2 | Jazzy (Ubuntu 24.04) |
| robot_state_publisher | — |
| joint_state_publisher_gui | — |
| rviz2 | — |
| ros2_control | — |
| xacro | — |

```bash
sudo apt install \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-rviz2 \
  ros-jazzy-ros2-control \
  ros-jazzy-xacro
```

---

## Installation

```bash
# Clone into your ROS 2 workspace
cd ~/ros2_ws/src
git clone https://github.com/ookkshirsagar/rh56dfx_description.git

# Build
cd ~/ros2_ws
colcon build --packages-select rh56dfx_description --symlink-install
source install/setup.bash
```

---

## Usage

### Visualise in RViz

Launch the hand in RViz with an interactive joint slider GUI:

```bash
# Left hand (default)
ros2 launch rh56dfx_description display.launch.py

# Right hand
ros2 launch rh56dfx_description display.launch.py side:=right
```

**Launch arguments:**

| Argument | Default | Description |
|---|---|---|
| `side` | `left` | Hand variant — `left` or `right` |
| `prefix` | `""` | Prefix added to all link and joint names |
| `use_world` | `false` | Attach a fixed `world` link as root for standalone use |

This opens RViz with the hand model and a Joint State Publisher GUI for manually moving all finger joints.

> RViz configs are pre-saved in `rviz/rh56dfx_left.rviz` and `rviz/rh56dfx_right.rviz` and loaded automatically by the launch file.

---

### Embed in Your Robot URDF

Use the provided xacro macro to attach the RH56DFX hand to any humanoid/robot arm:

```xml
<!-- In your robot's xacro file -->
<xacro:include filename="$(find rh56dfx_description)/urdf/rh56dfx.xacro"/>

<!-- Attach left hand to your robot's tool flange -->
<xacro:rh56dfx_hand
    side="left"
    prefix="left_"/>

<!-- Then connect left_hand_root to your robot's flange with a fixed joint -->
<joint name="flange_to_hand" type="fixed">
  <parent link="tool0"/>
  <child link="left_hand_root"/>
</joint>

<!-- Attach right hand to your robot's tool flange -->
<xacro:rh56dfx_hand
    side="right"
    prefix="right_"/>

<!-- Then connect right_hand_root to your robot's flange with a fixed joint -->
<joint name="flange_to_hand" type="fixed">
  <parent link="tool0"/>
  <child link="right_hand_root"/>
</joint>
```

**Macro parameters:**

| Parameter | Description |
|---|---|
| `side` | Hand variant — `left` or `right` |
| `prefix` | Prepended to all link/joint names. Use e.g. `left_` or `right_` to avoid name collisions |

> **Note:** The macro creates a `{prefix}hand_root` link as the kinematic anchor. Connect your robot's flange to this link with a fixed joint.

---

## Kinematic Structure

```
{prefix}hand_root
└── {prefix}wrist_base_link       (fixed)
    └── {prefix}wrist_yaw          (revolute — wrist_yaw_joint)
        └── {prefix}hand_base      (revolute — hand_base_joint)
            ├── {prefix}thumb_1    (revolute — thumb_1_joint)
            │   └── {prefix}thumb_2  (revolute — thumb_2_joint)
            │       ├── {prefix}thumb_rubber_1  (fixed)
            │       └── {prefix}thumb_3  (revolute — mimic thumb_2)
            │           ├── {prefix}thumb_rubber_2  (fixed)
            │           └── {prefix}thumb_4  (revolute — mimic thumb_3)
            │               ├── {prefix}thumb_rubber_3  (fixed)
            │               │   ├── {prefix}thumb_pad  (fixed)
            │               │   └── {prefix}thumb_tip  (fixed)
            ├── {prefix}index_1    (revolute — index_1_joint)
            │   ├── {prefix}index_rubber_1  (fixed)
            │   └── {prefix}index_2  (revolute — mimic index_1)
            │       ├── {prefix}index_rubber_2  (fixed)
            │       │   ├── {prefix}index_pad  (fixed)
            │       │   └── {prefix}index_tip  (fixed)
            ├── {prefix}middle_1   (revolute — middle_1_joint)  [same structure as index]
            ├── {prefix}ring_1     (revolute — ring_1_joint)    [same structure as index]
            ├── {prefix}little_1   (revolute — little_1_joint)  [same structure as index]
            └── {prefix}palm       (fixed)
```

---

## Joint Reference

### Independently Controlled Joints (8 total)

| Joint | Description | Lower | Upper |
|---|---|---|---|
| `{prefix}wrist_yaw_joint` | Wrist lateral rotation | -0.445 rad | +0.445 rad |
| `{prefix}hand_base_joint` | Wrist pitch / hand base rotation | -0.3955 rad | +0.386 rad |
| `{prefix}thumb_1_joint` | Thumb ab/adduction | 0 rad | 1.310 rad |
| `{prefix}thumb_2_joint` | Thumb proximal flexion | 0 rad | 0.523 rad |
| `{prefix}index_1_joint` | Index finger flexion | 0 rad | 1.344 rad |
| `{prefix}middle_1_joint` | Middle finger flexion | 0 rad | 1.344 rad |
| `{prefix}ring_1_joint` | Ring finger flexion | 0 rad | 1.344 rad |
| `{prefix}little_1_joint` | Little finger flexion | 0 rad | 1.344 rad |

### Mimic Joints (driven automatically)

| Joint | Mimics | Multiplier | Offset |
|---|---|---|---|
| `{prefix}thumb_3_joint` | `thumb_2_joint` | 1.1425 | 0 |
| `{prefix}thumb_4_joint` | `thumb_3_joint` | 0.7508 | 0 |
| `{prefix}index_2_joint` | `index_1_joint` | 1.1169 | -0.15 |
| `{prefix}middle_2_joint` | `middle_1_joint` | 1.1169 | -0.15 |
| `{prefix}ring_2_joint` | `ring_1_joint` | 1.1169 | -0.15 |
| `{prefix}little_2_joint` | `little_1_joint` | 1.1169 | -0.15 |

> Mimic joints are driven by their parent joint automatically — you only need to command the 8 independently controlled joints.

---

### ros2_control Integration

The package includes ready-to-use ros2_control configuration. To spawn the hand controller:

```bash
# Load controller config (left hand)
ros2 param load /controller_manager \
  $(ros2 pkg prefix rh56dfx_description)/share/rh56dfx_description/config/left/ros2_controllers.yaml

# Activate hand controller
ros2 control load_controller --set-state active hand_controller
```

---

## Credits

The **RH56DFX** dexterous hand is designed and manufactured by:

**Inspire Robotics**
- Website: [http://www.inspire-robots.com](http://www.inspire-robots.com)
- Product: RH56DFX 6-DOF Dexterous Hand

This ROS 2 description package was developed independently for research and simulation purposes. All mesh files and kinematic parameters are derived from the official RH56DFX hardware documentation and CAD models provided by Inspire Robotics.

If you use this package in academic work, please cite Inspire Robotics as the hardware manufacturer.

---

## License

This package is licensed under the **MIT License** — see [LICENSE](LICENSE) for details.

Note: The mesh files (STL) included in this package are derived from CAD models provided by Inspire Robotics and remain subject to Inspire Robotics' terms of use.

---

## Contributing

Contributions are welcome — see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

Areas where help is particularly useful:
- Right hand pose tuning and validation
- Additional example launch files
- Gazebo/simulation plugin integration
- Hardware-in-the-loop testing with real RH56DFX hardware
