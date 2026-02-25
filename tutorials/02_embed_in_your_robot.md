# Tutorial 02 — Embedding the Hand in Your Own Robot

This tutorial walks you through integrating the RH56DFX hand into your own robot's URDF using the provided xacro macro. By the end you will have a working unified kinematic chain with correct TF and no link name collisions.

---

## Prerequisites

- `rh56dfx_description` is built and sourced
- You have an existing robot URDF/xacro with a tool flange link (e.g. `tool0`)
- Basic familiarity with ROS 2 xacro

---

## Overview

The hand is provided as a **reusable xacro macro** defined in `urdf/rh56dfx_macro.xacro`. The macro creates the complete hand kinematic chain starting from a `{prefix}hand_root` link. You connect your robot's flange to this root with a fixed joint.

The macro takes two parameters:

| Parameter | Description |
|---|---|
| `side` | `left` or `right` |
| `prefix` | Prepended to all link and joint names — use this to avoid name collisions |

---

## Step 1 — Include the Macro

In your robot's top-level xacro file, add the include at the top:

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Your existing robot includes -->
  <xacro:include filename="$(find my_robot_description)/urdf/my_robot.xacro"/>

  <!-- Include the RH56DFX hand macro -->
  <xacro:include filename="$(find rh56dfx_description)/urdf/rh56dfx_macro.xacro"/>

</robot>
```

---

## Step 2 — Instantiate the Macro

Call the macro to create the hand links and joints. Choose a `prefix` that matches your setup:

```xml
<!-- Left hand mounted on the robot -->
<xacro:rh56dfx_hand
    side="left"
    prefix="left_"/>
```

```xml
<!-- Right hand mounted on the robot -->
<xacro:rh56dfx_hand
    side="right"
    prefix="right_"/>
```

> **Important:** Always set a `prefix` when embedding in a larger robot. Without a prefix, link names like `hand_base` and `palm` may conflict with other links in your URDF.

---

## Step 3 — Connect the Hand to Your Robot's Flange

The macro creates a `{prefix}hand_root` link as the kinematic anchor. Add a fixed joint to connect it to your robot's tool flange:

```xml
<joint name="flange_to_hand" type="fixed">
  <parent link="tool0"/>           <!-- Your robot's tool flange link -->
  <child link="left_hand_root"/>   <!-- {prefix}hand_root -->
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

Adjust `xyz` and `rpy` to match the physical mounting offset and orientation of the hand on your specific robot.

---

## Step 4 — Complete Example

Here is a minimal complete xacro file for a robot with a left RH56DFX hand:

```xml
<?xml version="1.0"?>
<robot name="my_robot_with_hand" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Arguments -->
  <xacro:arg name="side" default="left"/>

  <!-- Robot base -->
  <xacro:include filename="$(find my_robot_description)/urdf/my_robot.xacro"/>
  <xacro:my_robot/>

  <!-- RH56DFX hand -->
  <xacro:include filename="$(find rh56dfx_description)/urdf/rh56dfx_macro.xacro"/>
  <xacro:rh56dfx_hand side="$(arg side)" prefix="$(arg side)_"/>

  <!-- Mount hand to robot flange -->
  <joint name="flange_to_hand" type="fixed">
    <parent link="tool0"/>
    <child link="$(arg side)_hand_root"/>
    <origin xyz="0 0 0.01" rpy="0 0 0"/>
  </joint>

</robot>
```

---

## Step 5 — Verify the TF Tree

Build and launch your robot:

```bash
colcon build --packages-select my_robot_description --symlink-install
source install/setup.bash
ros2 launch my_robot_description display.launch.py
```

Then verify the TF chain is continuous from your robot base all the way to the fingertips:

```bash
ros2 run tf2_tools view_frames
```

Open the generated `frames.pdf` and check that:
- `tool0` → `left_hand_root` → `left_wrist_base_link` → ... → `left_palm` is a continuous chain
- No broken TF links (red arrows in the PDF)
- No duplicate link names

You can also check a specific transform:

```bash
ros2 run tf2_ros tf2_echo base_link left_palm
```

---

## Step 6 — Two Hands on the Same Robot

To mount both hands (e.g. on a dual-arm robot), instantiate the macro twice with different prefixes:

```xml
<!-- Left arm hand -->
<xacro:rh56dfx_hand side="left" prefix="left_"/>
<joint name="left_flange_to_hand" type="fixed">
  <parent link="left_tool0"/>
  <child link="left_hand_root"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>

<!-- Right arm hand -->
<xacro:rh56dfx_hand side="right" prefix="right_"/>
<joint name="right_flange_to_hand" type="fixed">
  <parent link="right_tool0"/>
  <child link="right_hand_root"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

Because every link and joint name is prefixed, there are no naming conflicts between the two hands.

---

## Common Mistakes

**Forgetting the prefix**
→ Link names like `palm`, `hand_base`, `thumb_1` will clash with other links in your URDF. Always set a prefix.

**Connecting to the wrong child link**
→ The joint must connect to `{prefix}hand_root`, not `{prefix}wrist_base_link`. The `hand_root` is the intended kinematic anchor.

**Wrong mounting orientation**
→ If the hand appears rotated or flipped in RViz, adjust the `rpy` in your flange joint. The hand's Z axis points away from the wrist base along the arm.

**Not rebuilding after changes**
→ With `--symlink-install`, xacro files are symlinked and changes take effect immediately on relaunch. Python launch files also benefit from this. However, changes to `CMakeLists.txt` or `package.xml` still require a full rebuild.

---

## What's Next

- [Tutorial 03](03_understanding_mimic_joints.md) — Understand mimic joints and how to command the hand correctly