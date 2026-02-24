# Tutorial 01 — Visualising the Hand

This tutorial walks you through launching the RH56DFX hand in RViz, understanding the display, navigating the TF tree, and using the joint slider GUI to move individual fingers.

---

## Prerequisites

Make sure the package is built and sourced:

```bash
cd ~/ros2_ws
colcon build --packages-select rh56dfx_description --symlink-install
source install/setup.bash
```

---

## Step 1 — Launch the Display

### Left hand (default)

```bash
ros2 launch rh56dfx_description display.launch.py
```

### Right hand

```bash
ros2 launch rh56dfx_description display.launch.py side:=right
```

Two windows will open:
- **RViz** — 3D visualisation of the hand
- **Joint State Publisher GUI** — sliders for moving each joint

---

## Step 2 — Understanding the RViz Display

When RViz opens you will see the hand model rendered with two materials:

| Material | Colour | Meaning |
|---|---|---|
| White | `rgba(1 1 1 1)` | Rigid aluminium links (finger phalanges, palm base) |
| Dark grey | `rgba(0.298 0.298 0.298 1)` | Soft rubber contact pads and fingertips |

If you see **red links** — this is the collision geometry overlay. To hide it, go to **MotionPlanning display → uncheck "Show Robot Collision"**, or simply ignore it as it is cosmetic.

### Navigating in RViz

| Action | Control |
|---|---|
| Rotate view | Left click + drag |
| Pan view | Middle click + drag (or Shift + left drag) |
| Zoom | Scroll wheel |
| Reset view | Press `R` |

---

## Step 3 — Understanding the TF Tree

The TF tree shows the kinematic chain of the hand. To visualise it, in RViz:

1. Click **Add** in the Displays panel
2. Select **TF** → click OK
3. Expand the TF display and check **Show Names**

You will see the full link hierarchy:

```
hand_root
└── wrist_base_link
    └── wrist_yaw
        └── hand_base
            ├── thumb_1
            │   └── thumb_2
            │       └── thumb_3 → thumb_4
            ├── index_1 → index_2
            ├── middle_1 → middle_2
            ├── ring_1 → ring_2
            ├── little_1 → little_2
            └── palm
```

> **Note:** The `prefix` argument prepends a string to every link name. For example with `prefix:=left_`, the root becomes `left_hand_root`.

To inspect the TF tree from the terminal:

```bash
ros2 run tf2_tools view_frames
```

This generates a `frames.pdf` in your current directory showing the complete TF graph.

---

## Step 4 — Moving the Joints with the GUI

The **Joint State Publisher GUI** window has a slider for each of the 8 independently controlled joints:

| Slider | Controls |
|---|---|
| `wrist_yaw_joint` | Wrist lateral rotation |
| `hand_base_joint` | Wrist pitch / hand rotation |
| `thumb_1_joint` | Thumb ab/adduction (spread) |
| `thumb_2_joint` | Thumb proximal curl |
| `index_1_joint` | Index finger curl |
| `middle_1_joint` | Middle finger curl |
| `ring_1_joint` | Ring finger curl |
| `little_1_joint` | Little finger curl |

Move any slider and watch the corresponding links update in RViz in real time.

> **You will notice:** When you move `index_1_joint`, the second finger link (`index_2`) also moves automatically. This is a **mimic joint** — see [Tutorial 03](03_understanding_mimic_joints.md) for a full explanation.

---

## Step 5 — Trying Common Hand Poses

Try recreating these poses using the sliders:

### Open hand (all zeros)
All sliders at `0.0` — fully open hand.

### Power grasp
```
index_1_joint:  1.30
middle_1_joint: 1.30
ring_1_joint:   1.30
little_1_joint: 1.30
thumb_1_joint:  1.10
thumb_2_joint:  0.40
```

### Pinch grasp (index + thumb)
```
index_1_joint:  1.20
thumb_1_joint:  0.80
thumb_2_joint:  0.35
middle_1_joint: 0.0
ring_1_joint:   0.0
little_1_joint: 0.0
```

### Point gesture
```
index_1_joint:  0.0
middle_1_joint: 1.30
ring_1_joint:   1.30
little_1_joint: 1.30
thumb_1_joint:  0.60
thumb_2_joint:  0.20
```

---

## Step 6 — Switching Between Left and Right

Close the current launch and relaunch with `side:=right`:

```bash
ros2 launch rh56dfx_description display.launch.py side:=right
```

The right hand is a mirror of the left. Joint names, limits, and structure are identical — only the mesh geometry and joint origins differ to reflect the physical mirroring.

---

## What's Next

- [Tutorial 02](02_embed_in_your_robot.md) — Embed the hand into your own robot URDF
- [Tutorial 03](03_understanding_mimic_joints.md) — Understand mimic joints and how to command the hand correctly