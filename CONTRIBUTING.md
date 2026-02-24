# Contributing to rh56dfx_description

Thank you for your interest in contributing to this ROS 2 description package for the Inspire Robotics RH56DFX dexterous hand.

---

## Prerequisites

- **ROS 2 Jazzy** on Ubuntu 24.04
- Package builds cleanly:
  ```bash
  colcon build --packages-select rh56dfx_description --symlink-install
  ```
- Both hand variants visualise correctly:
  ```bash
  ros2 launch rh56dfx_description display.launch.py
  ros2 launch rh56dfx_description display.launch.py side:=right
  ```

---

## Reporting Issues

Before opening an issue:
1. Check [existing issues](../../issues) for duplicates
2. Verify the problem exists on a clean build

When opening an issue include:
- ROS 2 distro and Ubuntu version
- Which hand variant (`left` or `right`)
- Steps to reproduce
- Expected vs actual behaviour
- Relevant terminal output or screenshots

---

## Understanding the Xacro Structure

Before contributing changes to URDF/xacro files, make sure you understand the structure:

- **`rh56dfx.xacro`** — top-level standalone file. Accepts `side`, `prefix`, and `use_world` arguments. Instantiates the macro.
- **`rh56dfx_macro.xacro`** — the reusable `rh56dfx_hand` macro. All link and joint definitions live here. Takes `side` and `prefix` params.
- All link/joint names are prefixed with `${prefix}` to support multi-robot setups.
- Left and right variants differ in inertial origins, joint origins, and axis directions — changes to geometry must be validated for **both** sides.
- Mimic joints (`thumb_3`, `thumb_4`, `*_2` finger joints) are driven automatically and should not be independently commanded.

---

## Submitting Changes

### 1. Fork and clone
```bash
git clone https://github.com/ookkshirsagar/rh56dfx_description.git
```

### 2. Create a branch
```bash
git checkout -b feat/your-feature-name
```

| Prefix | Use for |
|---|---|
| `feat/` | New features |
| `fix/` | Bug fixes |
| `docs/` | Documentation only |
| `refactor/` | Cleanup, no behaviour change |

### 3. Test your changes

Always test both hand variants after any URDF/xacro change:

```bash
colcon build --packages-select rh56dfx_description --symlink-install
source install/setup.bash

# Verify left hand
ros2 launch rh56dfx_description display.launch.py

# Verify right hand
ros2 launch rh56dfx_description display.launch.py side:=right

# Verify with prefix (for multi-robot use)
ros2 launch rh56dfx_description display.launch.py prefix:=robot1_

# Verify with world link
ros2 launch rh56dfx_description display.launch.py use_world:=true
```

Check that:
- No TF errors appear in the terminal
- All finger links appear correctly in RViz
- Joint sliders move all fingers as expected
- Mimic joints follow their parent joints

### 4. Commit clearly
```bash
git commit -m "fix: correct right hand thumb_2 joint origin"
```

### 5. Open a Pull Request with a clear description referencing any related issue

---

## Code Style

- **Python:** PEP 8
- **URDF/xacro:** Add comments for non-obvious geometry values. Use `<!-- Left -->` and `<!-- Right -->` comments to separate side-specific blocks
- **YAML:** 2-space indentation throughout

---

## Areas Where Help Is Welcome

- Right hand fingertip and pad joint origin validation against real hardware
- Gazebo Classic and Gazebo Harmonic simulation plugin integration
- ROS 2 Hardware Interface for real RH56DFX serial communication
- Additional example launch files (e.g. with ros2_control active)
- Tutorials or usage examples

---

## License

By contributing you agree your changes will be licensed under the [MIT License](LICENSE).
