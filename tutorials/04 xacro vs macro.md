# Tutorial 04 — rh56dfx.xacro vs rh56dfx_macro.xacro

A common point of confusion when integrating the RH56DFX hand is knowing which xacro file to include and when. This tutorial explains the purpose of each file, why they are separate, and the exact rule for choosing between them.

---

## The Two Files

| File | Purpose |
|---|---|
| `urdf/rh56dfx.xacro` | Standalone complete robot file — for visualisation only |
| `urdf/rh56dfx_macro.xacro` | Macro definition only — for embedding in your robot |

---

## `rh56dfx.xacro` — The Standalone File

This is a **complete, self-contained robot description**. Open it and you will see:

```xml
<robot name="rh56dfx" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Declares arguments: side, prefix, use_world -->
  <xacro:arg name="side" default="left"/>
  <xacro:arg name="prefix" default=""/>
  <xacro:arg name="use_world" default="false"/>

  <!-- Includes the macro definition -->
  <xacro:include filename="$(find rh56dfx_description)/urdf/rh56dfx_macro.xacro"/>

  <!-- Instantiates the hand immediately -->
  <xacro:rh56dfx_hand side="${side_s}" prefix="${prefix_p}"/>

</robot>
```

It has three key characteristics:
1. It has a `<robot>` tag — making it a complete robot description
2. It already includes `rh56dfx_macro.xacro` internally
3. It already calls `<xacro:rh56dfx_hand>` — the hand is instantiated inside it

This file is used **only** by `display.launch.py` for standalone visualisation:

```bash
ros2 launch rh56dfx_description display.launch.py
ros2 launch rh56dfx_description display.launch.py side:=right
```

---

## `rh56dfx_macro.xacro` — The Macro Definition

This file only **defines** the macro — it does not instantiate anything. Open it and you will see:

```xml
<robot xmlns:xacro="http://ros.org/wiki/xacro">

  <xacro:macro name="rh56dfx_hand" params="side prefix">

    <!-- All link and joint definitions -->
    <link name="${prefix}hand_root"/>
    <link name="${prefix}wrist_base_link"> ... </link>
    ...

  </xacro:macro>

</robot>
```

Notice:
- No `<robot name="...">` — it has no robot identity of its own
- No `<xacro:arg>` declarations — it takes its parameters from whoever calls it
- No instantiation — defining the macro does nothing until you call it

This is the file you include when embedding the hand into your robot.

---

## Why You Cannot Use `rh56dfx.xacro` for Embedding

There are two reasons using `rh56dfx.xacro` in your robot's URDF will break things:

### Reason 1 — Nested `<robot>` tags

Your robot's top-level xacro already has a `<robot>` tag:

```xml
<robot name="my_robot" xmlns:xacro="...">
  ...
</robot>
```

`rh56dfx.xacro` also has its own `<robot>` tag. Including it would nest two `<robot>` tags inside each other, which is invalid URDF and will cause a parse error.

### Reason 2 — Double instantiation

`rh56dfx.xacro` already calls `<xacro:rh56dfx_hand>` inside itself. If you include it and then call the macro again yourself, the hand gets instantiated **twice** — producing duplicate link and joint names, which is also invalid URDF.

---

## The Golden Rule

```
display.launch.py only       →  rh56dfx.xacro
embedding in your robot      →  rh56dfx_macro.xacro
```

---

## Correct Integration Pattern

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Your robot -->
  <xacro:include filename="$(find my_robot_description)/urdf/my_robot.xacro"/>
  <xacro:my_robot/>

  <!-- RH56DFX — include the MACRO file, not rh56dfx.xacro -->
  <xacro:include filename="$(find rh56dfx_description)/urdf/rh56dfx_macro.xacro"/>

  <!-- Instantiate the hand yourself with your chosen side and prefix -->
  <xacro:rh56dfx_hand side="right" prefix="right_"/>

  <!-- Connect hand root to your robot's flange -->
  <joint name="flange_to_hand" type="fixed">
    <parent link="tool0"/>
    <child link="right_hand_root"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

</robot>
```

---

## Wrong Integration Patterns

### Wrong — including rh56dfx.xacro instead of the macro

```xml
<!-- WRONG — rh56dfx.xacro has its own <robot> tag and instantiates the hand already -->
<xacro:include filename="$(find rh56dfx_description)/urdf/rh56dfx.xacro"/>
```

### Wrong — including the macro file but forgetting to instantiate it

```xml
<!-- This only defines the macro — the hand does not exist yet -->
<xacro:include filename="$(find rh56dfx_description)/urdf/rh56dfx_macro.xacro"/>

<!-- Missing: <xacro:rh56dfx_hand side="right" prefix="right_"/> -->
```

### Wrong — including the macro and then calling rh56dfx.xacro as well

```xml
<!-- This would instantiate the hand twice -->
<xacro:include filename="$(find rh56dfx_description)/urdf/rh56dfx_macro.xacro"/>
<xacro:include filename="$(find rh56dfx_description)/urdf/rh56dfx.xacro"/>
```

---

## Quick Reference

| I want to... | Use |
|---|---|
| Visualise the hand standalone in RViz | `rh56dfx.xacro` via `display.launch.py` |
| Add the hand to my robot URDF | `rh56dfx_macro.xacro` — include + instantiate |
| Use the hand in a multi-robot setup | `rh56dfx_macro.xacro` — instantiate twice with different prefixes |

---

## What's Next

- [Back to Tutorial 02 — Embedding in Your Robot](02_embed_in_your_robot.md)
- [Back to README](../README.md)