# ROS2 Autonomous Navigation and Exploration

This repository contains a comprehensive implementation of autonomous navigation and exploration algorithms for mobile robots using ROS2. The project involves localization, path planning, and autonomous exploration techniques to enable efficient navigation through unknown environments.

## Project Overview

This project implements a hybrid navigation system for the Robile robot platform. It includes:

- **Localization**: Monte-Carlo based particle filter implementation
- **Path Planning**: A* algorithm and potential field planner nodes
- **Autonomous Exploration**: Frontier-based exploration node
- **Navigation**: Combined global and local planning algorithms

For detailed project objectives, refer to the [project.md](project.md) file.

## Prerequisites

- Ubuntu 22.04 or later
- ROS2 Humble (or later)
- Python 3.8+
- Robile AMR simulation environment

## Installation

### 1. Set up ROS2

```bash
# Install ROS2 Humble (if not already installed)
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop
```

### 2. Set up Robile Simulation
Follow the link for Robile setup [here](https://robile-amr.readthedocs.io/en/latest/)
```bash
# Create a new workspace if you don't have one
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Install dependencies
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 3. Source the workspace

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Usage

### 1. Launch Robile in Simulation

To start the Robile simulation with Gazebo and RViz:

```bash
ros2 launch robile_gazebo gazebo_4_wheel.launch.py
```

### 2. Launch Hybrid Navigation

Start the complete navigation stack(Potential Field, A* and Exploration Nodes):

```bash
ros2 launch ros2-autonomous-navigation hybrid_navigation.launch.py
```

### 3. Manage Lifecycle Nodes

A convenience script is provided to manage lifecycle nodes:

```bash
./lifecycle_manager.sh
```
## Project Components

### Localization

The localization module implements a particle filter approach similar to AMCL (Adaptive Monte Carlo Localization):

```bash
python3 src/localisation/particle_filter.py
```

### Path Planning

The path planning module provides both global and local planning capabilities:

- **Global Planner**: A* algorithm implementation to generate path to goal and waypoints
  ```bash
  python3 src/path_and_motion_planning/global_planner.py
  ```

- **Local Planner**: Dynamic path adjustments using potential fields and requires A* to be running
  ```bash
  python3 src/path_and_motion_planning/local_planner.py
  ```
### Exploration

The exploration module implements frontier-based exploration strategies and requires the nodes A* and Potential Field to be running:

```bash
python3 src/exploration/exploration.py
```

## Troubleshooting Guide

### RTPS_TRANSPORT_SHM Error
**Problem**: "[RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7413: open_and_lock_file failed -> Function open_port_internal"

**Cause**: Another process is already running or a previous process didn't release the lock properly.

**Solution**: 
```bash

rm -rf /dev/shm/fastrtps_*

```

### Process already running/Resource already in use
**Problem**: Some launch files do not handle what to do when someone abruptly terminates the programs(nodes). This locks the processes and resources making it unavailable to use for another instance.

**Cause**: Another process is already running or a previous process didn't release the lock properly.

**Solution**: 
```bash
# Find and kill any running processes (gazebo, ros2)
ps aux | grep 
kill -9 <process_id>

```


### Map Server Activation Error
**Problem**: "map_server activate configure: no node found error"

**Cause**: The map_server lifecycle node isn't properly registered or addressable.

**Solution**:
```bash
# Check if the node is running
ros2 node list | grep map_server

# If not running, restart the map server
ros2 launch nav2_map_server map_server.launch.py use_sim_time:=True map:=$(pwd)/maps/map_01.yaml

# Manually transition the lifecycle node
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

### Transaction Failed Error
**Problem**: "Error: TransactionFailed: unable to append transforms" 

**Cause**: Typically a TF (Transform) related issue where conflicting transforms are being published.

**Solution**:
```bash
# Check current TF tree
ros2 run tf2_tools view_frames

# Identify conflicting transforms
ros2 topic echo /tf_static | grep frame_id

# Restart the nodes publishing conflicting transforms
ros2 lifecycle set /conflicting_node shutdown
ros2 lifecycle set /conflicting_node configure
ros2 lifecycle set /conflicting_node activate
```


### ROS Domain ID Conflicts
This is when communicating with real robot (robile3 or robile4)

**Problem**: Nodes can't communicate or discover each other

**Cause**: Different ROS_DOMAIN_ID settings between terminals

**Solution**:
```bash
# Set consistent ROS_DOMAIN_ID in all terminals
export ROS_DOMAIN_ID=3 #4 for robile4

# Add to your ~/.bashrc for persistence
echo "export ROS_DOMAIN_ID=3" >> ~/.bashrc
source ~/.bashrc
```

#### QoS Incompatibility Issues
**Problem**: Subscribers not receiving messages from publishers

**Cause**: Incompatible Quality of Service (QoS) settings

**Solution**:
```bash
# Use compatible QoS settings or explicit QoS overrides
# For example, match sensor data with a reliable QoS override:
ros2 topic pub --qos-reliability reliable --qos-durability transient_local /topic_name ...
```

#### Parameter Server Errors
**Problem**: "Failed to set parameters: node not found"

**Cause**: Node name mismatch or node not running

**Solution**:
```bash
# Verify node name
ros2 node list

# Correct parameter target with proper node name
ros2 param set /correct_node_name parameter_name parameter_value
```

## Working with Robile

### Robile Simulation Setup

Robile is a mobile robot platform used in this project. To set up Robile:

1. Ensure you've cloned the Robile repository as mentioned in the installation section.

2. Configure the Robile simulation environment:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select robile_description robile_gazebo
   ```

3. Set environment variables:
   ```bash
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/install/robile_description/share/robile_description/models
   ```

### Robile Control

Robile can be controlled in several ways:

1. **Keyboard Teleop**:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

2. **Joystick Control**:
   ```bash
   ros2 launch robile_navigation joystick_control.launch.py
   ```

3. **Programmatic Control**:
   ```python
   # Python example to move Robile
   import rclpy
   from geometry_msgs.msg import Twist
   
   rclpy.init()
   node = rclpy.create_node('robile_controller')
   publisher = node.create_publisher(Twist, '/cmd_vel', 10)
   
   msg = Twist()
   msg.linear.x = 0.5  # Move forward at 0.5 m/s
   msg.angular.z = 0.0  # No rotation
   
   publisher.publish(msg)
   ```

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Prof. Alex Mitrevski for his  lectures and support
- ROS2 community for continuous support
- The team for constant co-ordination and support in implementing, testing and debugging the project