# Tutorial 03 — Understanding Mimic Joints

This tutorial explains what mimic joints are, why the RH56DFX uses them, and how to correctly command the hand so that all finger links move together naturally.

---

## What Is a Mimic Joint?

A **mimic joint** is a joint that automatically follows another joint's position according to a simple linear relationship:

```
mimic_position = (multiplier × parent_position) + offset
```

It is defined in URDF with the `<mimic>` tag:

```xml
<joint name="index_2_joint" type="revolute">
  <mimic joint="index_1_joint" multiplier="1.1169" offset="-0.15"/>
  ...
</joint>
```

This means `index_2_joint` will always be set to `(1.1169 × index_1_joint) - 0.15` automatically — you never need to command it directly.

---

## Why Does the RH56DFX Use Mimic Joints?

The RH56DFX is an **underactuated hand**. Each finger has 2 physical links (phalanges) but only **1 actuator**. The second link is mechanically coupled to the first via a tendon linkage — when the first phalange curls, the second follows proportionally.

Mimic joints model this mechanical coupling in simulation so that:
- Commanding a single joint value produces a realistic full-finger curl
- The visual model matches the physical hand behaviour
- MoveIt and other planners only need to plan for the 8 actuated joints

---

## Which Joints Are Mimic Joints?

The RH56DFX has **6 mimic joints**:

| Mimic Joint | Follows | Multiplier | Offset |
|---|---|---|---|
| `{prefix}thumb_3_joint` | `thumb_2_joint` | 1.1425 | 0 |
| `{prefix}thumb_4_joint` | `thumb_3_joint` | 0.7508 | 0 |
| `{prefix}index_2_joint` | `index_1_joint` | 1.1169 | -0.15 |
| `{prefix}middle_2_joint` | `middle_1_joint` | 1.1169 | -0.15 |
| `{prefix}ring_2_joint` | `ring_1_joint` | 1.1169 | -0.15 |
| `{prefix}little_2_joint` | `little_1_joint` | 1.1169 | -0.15 |

The **8 joints you actually command** are:

```
wrist_yaw_joint
hand_base_joint
thumb_1_joint
thumb_2_joint
index_1_joint
middle_1_joint
ring_1_joint
little_1_joint
```

---

## How the Finger Curl Works

When you send `index_1_joint = 1.0 rad`:

```
index_1_joint  = 1.0 rad          ← you command this
index_2_joint  = (1.1169 × 1.0) - 0.15 = 0.9669 rad  ← automatic
```

The proximal phalange curls to 1.0 rad and the distal phalange curls to 0.9669 rad, producing a natural finger curl.

For the thumb, the chain goes three levels deep:

```
thumb_2_joint = 0.4 rad           ← you command this
thumb_3_joint = 1.1425 × 0.4 = 0.457 rad   ← automatic
thumb_4_joint = 0.7508 × 0.457 = 0.343 rad ← automatic
```

---

## Common Mistake — Commanding Mimic Joints Directly

**Do not** include mimic joints in your JointTrajectory commands. This is the most common error when integrating the hand with ros2_control or MoveIt.

### Wrong — includes mimic joints:
```python
joint_names = [
    'left_index_1_joint',
    'left_index_2_joint',   # ← WRONG — mimic joint, do not command
    'left_middle_1_joint',
    'left_middle_2_joint',  # ← WRONG
]
```

### Correct — only the 8 actuated joints:
```python
joint_names = [
    'left_wrist_yaw_joint',
    'left_hand_base_joint',
    'left_thumb_1_joint',
    'left_thumb_2_joint',
    'left_index_1_joint',
    'left_middle_1_joint',
    'left_ring_1_joint',
    'left_little_1_joint',
]
```

---

## Sending a Joint Trajectory Command

Here is a minimal Python example to command the hand to a power grasp using `rclpy`:

```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class HandCommander(Node):
    def __init__(self):
        super().__init__('hand_commander')
        self.pub = self.create_publisher(
            JointTrajectory,
            '/hand_controller/joint_trajectory',
            10
        )
        # Wait for subscriber
        self.create_timer(1.0, self.send_command)

    def send_command(self):
        msg = JointTrajectory()

        # Only the 8 actuated joints — no mimic joints
        msg.joint_names = [
            'left_wrist_yaw_joint',
            'left_hand_base_joint',
            'left_thumb_1_joint',
            'left_thumb_2_joint',
            'left_index_1_joint',
            'left_middle_1_joint',
            'left_ring_1_joint',
            'left_little_1_joint',
        ]

        point = JointTrajectoryPoint()
        # Power grasp pose
        point.positions = [0.0, 0.0, 1.10, 0.40, 1.30, 1.30, 1.30, 1.30]
        point.time_from_start = Duration(sec=2)

        msg.points = [point]
        self.pub.publish(msg)
        self.get_logger().info('Sent power grasp command')

def main():
    rclpy.init()
    node = HandCommander()
    rclpy.spin_once(node, timeout_sec=2.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Verifying Mimic Joints in RViz

When using the **Joint State Publisher GUI** in the display launch, move the `index_1_joint` slider and observe:
- `index_1` link moves to your commanded angle
- `index_2` link automatically follows at a proportionally larger angle

This confirms that mimic joints are working correctly. If `index_2` does not move, check that `joint_state_publisher` is configured to publish mimic joint states — the `display.launch.py` handles this automatically.

---

## Mimic Joints and MoveIt

When configuring MoveIt for the RH56DFX:

- **Planning group joints** should list only the 8 actuated joints — not mimic joints
- **SRDF passive joints** — mimic joints should be listed as passive so MoveIt does not attempt to plan for them
- **Controllers** — your `hand_controller` JointTrajectoryController should only accept the 8 actuated joint names

Example SRDF passive joint declarations:
```xml
<passive_joint name="left_index_2_joint"/>
<passive_joint name="left_middle_2_joint"/>
<passive_joint name="left_ring_2_joint"/>
<passive_joint name="left_little_2_joint"/>
<passive_joint name="left_thumb_3_joint"/>
<passive_joint name="left_thumb_4_joint"/>
```

---

## Summary

| Joint type | Count | You command it? |
|---|---|---|
| Actuated (independent) | 8 | Yes |
| Mimic (coupled) | 6 | No — automatic |
| Fixed (rubber pads, tips) | Many | No — rigid |

Always command only the **8 actuated joints**. The mimic joints will follow automatically based on the multiplier and offset defined in the URDF.

---

## What's Next

- [Tutorial 05 — ros2_control Integration](05_ros2_control_integration.md) *(coming soon)*
- [Back to README](../README.md)