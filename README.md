# ChessGame with RL

## Overview
This project is under the construction of WSL1 with github repositories shown below
  1. Chess_manipulator(panda chess_env) (https://github.com/ZeinBarhoum/chess_manipulator.git)
  2. Universal_Robots_ROS2_Gazebo_Simulation(UR env) (https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git)
  3. chess-alpha-zero(for sending reinforcement learning) (https://github.com/Zeta36/chess-alpha-zero.git)

## Project Structure
```bash
ros2_ws/src/
├── chess_robot_ai/
│  ├── chess_robot_ai/
│  │  ├── chess_robot_node.py # This is for panda manipulator
│  │  ├── chess_robot_node_UR.py # This is for UR manipulator
├── panda_env
├── ur_chess_controller
├── ur_env

```

## Prerequisites
- ROS2 [humble]
- Ubuntu [22.04.5 LTS]
- Gazebo [version 11.10.2]


## 🚀Quick Install Guide
```bash
# cd your ros2 workspace
cd [ros2_ws] # modified it with your own workspace name

# cd src
cd src

# Clone the repository
git clone git@github.com:Jack23162329/ChessGame_Auto.git


# Install dependencies
[dependency installation commands]

# Build the workspace
colcon build
