# Heterogeneous Multi-Robot Task Allocation (MRTA) Simulation

A ROS2 + Gazebo simulation framework for multi-robot task allocation using deep reinforcement learning, inspired by the paper [*"Heterogeneous Multi-Robot Task Allocation: A Deep Reinforcement Learning Approach"*](https://www.marmotlab.org/publications/73-RAL2025-HetMRTA.pdf).

<p align="center">
  <img src="https://raw.githubusercontent.com/sohamkarandikar391/multi_robot_task_allocation_env/main/docs/scenario_screenshot.png" alt="Simulation Overview" width="600"/>
</p>
<p align="center"><em>Multi-robot team collaborating on spatially distributed tasks with obstacles</em></p>
Watch the full simulation in action: [YouTube Demo](https://www.youtube.com/watch?v=Crc_LIuiWpk)


---

## Project Overview

This project implements a complete simulation environment for **heterogeneous multi-robot task allocation (MRTA)**, where teams of robots with different capabilities must coordinate to complete spatially distributed tasks that require specific skill combinations.

### Key Features
- **Heterogeneous Robot Teams**: Multiple species of robots with distinct skill sets
- **Skill-Based Task Requirements**: Tasks require specific combinations of skills that must be satisfied by team assembly
- **RL-Based Task Allocation**: Uses attention-based neural networks to compute optimal task assignments
- **Gazebo Simulation**: Full physics simulation with obstacle avoidance and collision detection
- **Dynamic Team Formation**: Robots autonomously navigate and assemble at task locations
- **ROS2 Architecture**: Modular design with separate nodes for allocation, control, and visualization

---

## Simulation Environment

### Current Environment Properties

**World Configuration:**
- **Dimensions**: 20m × 20m workspace
- **Obstacles**: Static obstacles requiring dynamic path planning (Orange obstacles are short and block only ground robots while Purple obstacles are tall that block aerial and ground robots from going through them)
- **Depot Locations**: Species-specific home bases for robot deployment (Marked with green cones in the environment)
- **Task Sites**: Spatially distributed locations requiring team coordination (Marked with yellow cones in the environment)

**Robot Species:**
1. **Red TurtleBots** (Ground robots)
   - Skills: `[1, 0, 0, 0, 1]`
   - Count: 4 robots
   - Max velocity: 0.5 m/s

2. **Blue TurtleBots** (Ground robots)
   - Skills: `[0, 1, 0, 0, 1]`
   - Count: 4 robots
   - Max velocity: 0.5 m/s

3. **Green Turtlebots** (Ground robots)
   - Skills: `[0, 0, 1, 1, 0]`
   - Count: 3 robots
   - Operating altitude: 2.0m

**Task Characteristics:**
- Each task has a **skill requirement vector** (e.g., `[1, 0, 0, 1, 0]`)
- **Team assembly required**: Tasks start only when combined robot skills ≥ requirements
- **Variable duration**: Tasks take 15-30 seconds to complete
- **Sequential execution**: Robots follow computed routes through multiple tasks

### Environment Dynamics

**Motion Control:**
- **Ground Robots**: Differential drive kinematics with non-holonomic constraints
- **Potential Field Navigation**: Attractive forces to goals, repulsive forces from obstacles and other robots
- **Collision Avoidance**: Dynamic obstacle detection with safety margins

**Task Execution:**
1. **Assignment**: RL policy computes full task routes for all robots
2. **Navigation**: Robots navigate to assigned task locations using potential fields
3. **Waiting**: Robots arrive and wait for teammates at task sites
4. **Execution**: Task begins when skill requirements are satisfied
5. **Completion**: Robots proceed to next task or return to depot

---

##  Original RL Paper

This implementation is based on the work by **Chen et al.**:

> **"Heterogeneous Multi-Robot Task Allocation: A Deep Reinforcement Learning Approach"**  
> *arXiv:2301.09932, 2023*

![Paper Method Overview](https://raw.githubusercontent.com/sohamkarandikar391/multi_robot_task_allocation_env/main/docs/paper_method.gif)
*Attention-based neural network architecture for task allocation (from original paper)*

### Key Contributions from Paper:
- **Attention Mechanism**: Uses self-attention to encode relationships between robots and tasks
- **Heterogeneous Skills**: Explicitly models different robot capabilities and task requirements
- **Scalability**: Handles variable numbers of robots and tasks
- **Coordination**: Learns implicit team coordination strategies through reward shaping

### Our Implementation:
- Integrated trained RL policy into ROS2/Gazebo simulation
- Added real-world physics and navigation constraints
- Implemented distributed robot controllers with potential field planning

---

##  Installation & Setup

### Prerequisites
- **Ubuntu 22.04** or later
- **ROS2 Jazzy** (or Humble with modifications)
- **Gazebo Classic 11** or **Ignition Gazebo**
- **Python 3.10+**
- **PyTorch 1.12+**

### Dependencies

```bash
# ROS2 packages
sudo apt install ros-jazzy-desktop ros-jazzy-gazebo-ros-pkgs

# Python dependencies
pip install torch numpy pyyaml
```

### Clone and Build

```bash
# Create workspace
mkdir -p ~/mrta_ws/src
cd ~/mrta_ws/src

# Clone repository
git clone https://github.com/sohamkarandikar391/mrta_simulation.git

# Clone RL paper implementation
cd mrta_simulation
git clone https://github.com/marmotlab/HeteroMRTA/tree/main
# Build workspace
cd ~/mrta_ws
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

### Download Pre-trained Model

```bash
# Place trained checkpoint in model directory
cd ~/mrta_ws/src/mrta_simulation/HeteroMRTA/model/save_5_task_2/
# Download checkpoint.pth from [link to model weights]
```

---

##  Running the Simulation

### Quick Start

```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch mrta_simulation test_launch.py
# This command launches the gazebo simulation, starts the robot controllers and the RL-trained allocator.
```

### Launch File Options

```bash
# Custom scenario
ros2 launch mrta_simulation multi_robot_sim.launch.py \
    scenario:=scenario_simple.yaml
```

---

---

##  Creating Custom Scenarios

Edit `config/scenario_simple.yaml`:

```yaml
species:
  red_turtlebot:
    count: 4
    skills: [1, 0, 0, 0, 1]
    depot_position: [-8.4, -8.4, 0]
  
tasks:
  0:
    id: 0
    position: [2.5, 5.1, 0]
    requirements: [1, 0, 0, 1, 0]
    duration: 10

obstacles:
  obstacle_1:
    position: [0, 0, 0]
    size: [2, 2, 1]
```

---

## Monitoring & Visualization

### ROS2 Topics

```bash
# View task assignments
ros2 topic echo /task_assignments

# Monitor robot positions
ros2 topic echo /robot_positions

# Check task status
ros2 topic echo /task_status
```
---
