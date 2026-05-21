# guide.md

# Sailboat ROS2 + Gazebo Development Guide

Internal development and setup guide for the autonomous sailboat project.

---

# Table of Contents

* [1. Workspace Setup](#1-workspace-setup)
* [2. VRX Installation](#2-vrx-installation)
* [3. Repository Structure](#3-repository-structure)
* [4. Building the Workspace](#4-building-the-workspace)
* [5. Running the Simulation](#5-running-the-simulation)
* [6. Actuator Control](#6-actuator-control)
* [7. Important Topics](#7-important-topics)
* [8. Architecture Overview](#8-architecture-overview)
* [9. Development Workflow](#9-development-workflow)
* [10. Debugging](#10-debugging)
* [11. Known Issues / Notes](#11-known-issues--notes)

---

# 1. Workspace Setup

Create the ROS2 workspace:

```bash
mkdir -p ~/sailboat_ws/src
cd ~/sailboat_ws/src
```

The GitHub repository should be cloned directly into the `src` folder.

Example:

```bash
cd ~/sailboat_ws/src

git clone <REPOSITORY_URL>
```

The repository itself represents the contents of the `src` directory.

---

# 2. VRX Installation

The VRX repository is NOT included in the project repository and must be installed manually.

Inside `~/sailboat_ws/src`:

```bash
cd ~/sailboat_ws/src

git clone https://github.com/osrf/vrx.git
```
---

# 3. Repository Structure

Current project packages:

```text
sailboat_gazebo/
sailboat_control/
```

Old / deprecated package:

```text
sailboat_description/
```

This package is currently unused and planned for removal.

---

# sailboat_gazebo

Contains:

* Gazebo worlds
* Sailboat SDF model
* Launch files
* Physics configuration
* Joint controllers
* Buoyancy plugins

Important files:

```text
sailboat_gazebo/
├── launch/
│   └── sim.launch.py
├── worlds/
│   └── minimal_ocean.sdf
└── models/
    └── sailboat/
        └── model.sdf
```

---

# sailboat_control

Contains:

* ROS2 actuator nodes
* Future control logic
* Future autonomy logic
* Teleoperation
* Waypoint following
* Wind estimation

Current node:

```text
actuator_node.py
```

---

# 4. Building the Workspace

From workspace root:

```bash
cd ~/sailboat_ws
```

Build:

```bash
colcon build
```

Source workspace:

```bash
source install/setup.bash
```

IMPORTANT:

This must be done in EVERY new terminal before running ROS2 commands.

---

# 5. Running the Simulation

The system currently requires multiple terminals.

---

# Terminal 1 — Launch Gazebo

```bash
cd ~/sailboat_ws

source install/setup.bash

ros2 launch sailboat_gazebo sim.launch.py
```

This launches:

* Gazebo Harmonic
* VRX ocean world
* Sailboat model
* ROS-Gazebo bridge

---

# Terminal 2 — Run Actuator Node

```bash
cd ~/sailboat_ws

source install/setup.bash

ros2 run sailboat_control actuator_node
```

This node provides:

* sail position control
* rudder position control

---

# Terminal 3 — Send Actuator Commands

Boom / sail angle:

```bash
ros2 topic pub \
/cmd_baum_pos \
std_msgs/msg/Float64 \
"{data: 0.5}"
```

Center sail:

```bash
ros2 topic pub \
/cmd_baum_pos \
std_msgs/msg/Float64 \
"{data: 0.0}"
```

Rudder:

```bash
ros2 topic pub \
/cmd_rudder_pos \
std_msgs/msg/Float64 \
"{data: 0.3}"
```

---

# 6. Actuator Control

The system currently uses:

```text
Gazebo JointPositionController
```

for:

* boom_joint
* rudder_joint

---

# Control Philosophy

ROS publishes:

```text
desired joint angles
```

Gazebo handles:

```text
joint actuation and PID stabilization
```

This architecture is intentionally simple and stable.

---

# 7. Important Topics

## ROS Topics

Boom position command:

```text
/cmd_baum_pos
```

Rudder position command:

```text
/cmd_rudder_pos
```

Joint states:

```text
/model/sailboat/joint_state
```

---

## Gazebo Topics

Boom actuator:

```text
/baum_pos
```

Rudder actuator:

```text
/rudder_pos
```

---

# 8. Architecture Overview

Current system architecture:

```text
ROS2 command topics
        ↓
actuator_node
        ↓
Gazebo position controller
        ↓
joint actuation
        ↓
boat physics
```

Future planned architecture:

```text
autonomy node
    ↓
control logic
    ↓
actuator commands
    ↓
Gazebo simulation
```

---

# 9. Development Workflow

---

# Main Branch

The `main` branch should ALWAYS remain:

* stable
* runnable
* demonstrable

Never directly push experimental features to `main`.

---

# Feature Branches

Large features should be implemented in dedicated branches.

Examples:

```text
feature/sail-force-model
feature/wind-estimation
feature/waypoint-following
feature/tacking-logic
feature/camera-sensor
```

---

# Recommended Workflow

Create feature branch:

```bash
git checkout -b feature/sail-force-model
```

Commit regularly with meaningful commit messages.

Merge back into main after testing.

---

# Good Commit Examples

```text
Add boom position controller
Implement rudder joint hierarchy
Add buoyancy plugin
Refactor actuator architecture
```

Avoid:

```text
fix
test
working maybe
asdf
```

---

# 10. Debugging

List ROS topics:

```bash
ros2 topic list
```

Echo topic:

```bash
ros2 topic echo /cmd_baum_pos
```

List Gazebo topics:

```bash
gz topic -l
```

Check joint states:

```bash
ros2 topic echo /model/sailboat/joint_state
```

---

# 11. Known Issues / Notes

* Gazebo plugin naming is confusing:

  * `name=` refers to plugin class name, not custom instance name
* Gazebo Harmonic plugin APIs differ from older Gazebo Classic tutorials
* Resource paths must include:

  * sailboat models
  * VRX models
  * ROS Jazzy assets
* JointPositionController is currently the preferred control method
* sailboat_description package is deprecated
* Wind force model is not implemented yet

---
