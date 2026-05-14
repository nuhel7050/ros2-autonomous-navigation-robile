<div align="center">

# ROS2 Autonomous Navigation — Robile Platform

[![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?style=flat-square&logo=ros&logoColor=white)](#)
[![Python](https://img.shields.io/badge/Python-3.8+-3776AB?style=flat-square&logo=python&logoColor=white)](#)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?style=flat-square&logo=ubuntu&logoColor=white)](#)

**Autonomous navigation and exploration stack for the Robile mobile robot platform — integrating A\* path planning, potential field control, particle filter localization, and frontier-based exploration.**

</div>

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Project Structure](#project-structure)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Components](#components)
  - [Path Planning](#1-path-planning)
  - [Localization](#2-localization)
  - [Exploration](#3-exploration)
- [Launch Configuration](#launch-configuration)
- [Working with Robile](#working-with-robile)
- [Troubleshooting](#troubleshooting)
- [Acknowledgments](#acknowledgments)

---

## Overview

This project implements a complete hybrid navigation system for the [Robile](https://robile-amr.readthedocs.io/en/latest/) robot platform, developed as part of the Autonomous Mobile Robots course at HBRS. The system combines three core capabilities:

| Component | Algorithm | Purpose |
|-----------|-----------|---------|
| **Global Planning** | A* Search | Compute obstacle-free path from start to goal |
| **Local Planning** | Potential Field | Navigate between waypoints while avoiding obstacles |
| **Localization** | Monte Carlo Particle Filter | Estimate robot pose within a known map |
| **Exploration** | Frontier-Based | Autonomously discover and map unknown environments |

For detailed project objectives, see [project.md](project.md).

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Launch System                             │
│              hybrid_navigation.launch.py                     │
├──────────┬──────────┬──────────┬──────────┬─────────────────┤
│  SLAM    │  AMCL    │  A*      │ Potential│  Frontier       │
│ Toolbox  │  Node    │ Planner  │  Field   │  Explorer       │
├──────────┴──────────┴──────────┴──────────┴─────────────────┤
│                     ROS2 Topics                              │
│  /map  /scan  /cmd_vel  /path  /goal_pose  /pose            │
├─────────────────────────────────────────────────────────────┤
│               Robile Hardware / Gazebo Sim                   │
│            LiDAR · Odometry · Base Controller                │
└─────────────────────────────────────────────────────────────┘
```

**Data Flow:**
```
Exploration → selects frontier goal
    → A* Planner → computes global path (waypoints)
        → Potential Field → drives between waypoints, avoids obstacles
            → /cmd_vel → Robot motion
```

---

## Project Structure

```
ros2-autonomous-navigation-robile/
├── launch/
│   ├── hybrid_navigation.launch.py   # Full navigation stack launcher
│   └── map_and_robile.launch.py      # Map server + Robile launcher
│
├── src/
│   ├── localisation/
│   │   ├── particle_filter.py        # Monte Carlo particle filter (29 KB)
│   │   ├── amcl.py                   # AMCL integration node
│   │   └── amcl_structure.py         # AMCL structure definitions
│   │
│   ├── path_and_motion_planning/
│   │   ├── global_planner.py         # A* global planner ROS2 node
│   │   ├── local_planner.py          # Potential field local planner
│   │   ├── astar.py                  # Standalone A* implementation
│   │   ├── global_planner_optimized.py
│   │   ├── local_planner_optimized.py
│   │   └── potential_field_and_astar.py  # Combined planner
│   │
│   └── exploration/
│       └── exploration.py            # Frontier-based exploration node
│
├── maps/                             # Pre-built environment maps
├── lifecycle_manager.sh              # Lifecycle node management script
├── project.md                        # Course project specification
└── AMR_Team_03_report.pdf            # Project report
```

---

## Prerequisites

| Requirement | Version |
|------------|---------|
| Ubuntu | 22.04+ |
| ROS2 | Humble |
| Python | 3.8+ |
| Robile AMR | Simulation environment |

**Key ROS2 packages:** `nav2_amcl`, `slam_toolbox`, `tf2_ros`, `nav2_map_server`

---

## Installation

### 1. Install ROS2 Humble

```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install -y ros-humble-desktop
```

### 2. Set Up Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repository
git clone https://github.com/nuhel7050/ros2-autonomous-navigation-robile.git

# Install dependencies
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 3. Set Up Robile Simulation

Follow the [Robile AMR documentation](https://robile-amr.readthedocs.io/en/latest/) for simulation setup.

```bash
colcon build --packages-select robile_description robile_gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/install/robile_description/share/robile_description/models
```

### 4. Source the Workspace

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Usage

### Quick Start — Full Navigation Stack

```bash
# Terminal 1: Launch Robile in Gazebo
ros2 launch robile_gazebo gazebo_4_wheel.launch.py

# Terminal 2: Launch hybrid navigation (SLAM + A* + Potential Field + Explorer)
ros2 launch ros2-autonomous-navigation hybrid_navigation.launch.py

# Terminal 3: Manage lifecycle nodes
./lifecycle_manager.sh
```

### Running Individual Components

```bash
# A* Global Planner only
python3 src/path_and_motion_planning/global_planner.py

# Potential Field Local Planner only
python3 src/path_and_motion_planning/local_planner.py

# Particle Filter Localization only
python3 src/localisation/particle_filter.py

# Frontier Explorer only (requires A* + Potential Field running)
python3 src/exploration/exploration.py
```

---

## Components

### 1. Path Planning

**Global Planner (A\*)**
- Subscribes to `/map` (OccupancyGrid) and `/goalpose`
- Inflates obstacles using morphological dilation for safe clearance
- Computes shortest path via A* with 8-connected grid neighbors
- Simplifies path by removing collinear waypoints
- Publishes path on `/path_line` and `/path_points`

**Local Planner (Potential Field)**
- Navigates between A* waypoints using attractive + repulsive forces
- Attractive force pulls robot toward next waypoint
- Repulsive force pushes robot away from obstacles (from LiDAR scan)
- Publishes velocity commands on `/cmd_vel`

### 2. Localization

**Particle Filter (Monte Carlo)**
- Custom implementation of Adaptive Monte Carlo Localization
- Particle initialization, motion model, sensor model, resampling
- Integrates with `/scan` (LiDAR) and `/odom` topics
- Publishes estimated pose for downstream planners

### 3. Exploration

**Frontier-Based Explorer**
- Detects frontier cells (boundary between known free space and unknown space)
- Clusters frontiers using depth-first search
- Filters unsafe frontiers (too close to obstacles)
- Selects nearest reachable frontier as next goal
- Sends goals to A* planner, monitors progress
- Handles timeouts and unreachable goals with automatic retry
- Saves map when exploration is complete

---

## Launch Configuration

The `hybrid_navigation.launch.py` starts the full stack:

| Node | Package | Description |
|------|---------|-------------|
| `async_slam_toolbox_node` | slam_toolbox | Online SLAM for mapping |
| `amcl` | nav2_amcl | Adaptive Monte Carlo Localization |
| `astar` | global_path_planner | A* global planner |
| `local_planner` | local_path_planner | Potential field local planner |
| `integrated_explorer` | exploration | Frontier-based exploration |
| `tf2_monitor` | tf2_ros | TF tree debugging |

**Configurable Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_sim_time` | `true` | Use simulation clock |
| `min_threshold` | `5` | A* obstacle inflation threshold |
| `k_rep` | `0.5` | Potential field repulsive gain |
| `k_att` | `0.8` | Potential field attractive gain |

---

## Working with Robile

### Control Modes

```bash
# Keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Joystick control
ros2 launch robile_navigation joystick_control.launch.py
```

### Programmatic Control

```python
import rclpy
from geometry_msgs.msg import Twist

rclpy.init()
node = rclpy.create_node('robile_controller')
publisher = node.create_publisher(Twist, '/cmd_vel', 10)

msg = Twist()
msg.linear.x = 0.5   # Forward at 0.5 m/s
msg.angular.z = 0.0   # No rotation
publisher.publish(msg)
```

### Real Robot Setup

```bash
# Set ROS Domain ID (3 for robile3, 4 for robile4)
export ROS_DOMAIN_ID=3
echo "export ROS_DOMAIN_ID=3" >> ~/.bashrc
source ~/.bashrc
```

---

## Troubleshooting

| Error | Cause | Solution |
|-------|-------|---------|
| `RTPS_TRANSPORT_SHM Error` | Stale shared memory locks | `rm -rf /dev/shm/fastrtps_*` |
| `Process already running` | Previous process didn't release resources | `ps aux \| grep <process> && kill -9 <pid>` |
| `map_server activate: no node found` | Lifecycle node not registered | Restart map_server, then `ros2 lifecycle set /map_server configure && activate` |
| `TransactionFailed: unable to append transforms` | Conflicting TF publishers | Check TF tree: `ros2 run tf2_tools view_frames` |
| Nodes can't discover each other | Mismatched `ROS_DOMAIN_ID` | Set consistent `ROS_DOMAIN_ID` in all terminals |
| Subscribers not receiving messages | QoS incompatibility | Match reliability/durability settings between pub/sub |
| `Failed to set parameters: node not found` | Wrong node name | Verify with `ros2 node list` |

---

## Acknowledgments

- **Prof. Alex Mitrevski** — Course lectures and supervision
- **ROS2 Community** — Framework and tooling support
- **AMR Team 03** — Collaborative development, testing, and debugging