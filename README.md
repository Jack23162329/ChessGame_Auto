# ChessGame with RL

## 😸Overview
This project is implemented in WSL2 with GitHub repositories as shown below:
  1. [Chess_manipulator(panda_env)](https://github.com/ZeinBarhoum/chess_manipulator.git)
  2. [Universal_Robots_ROS2_Gazebo_Simulation(ur_env)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git)
  3. [chess-alpha-zero(for sending reinforcement learning model)](https://github.com/Zeta36/chess-alpha-zero.git)

We aim to use ROS2 for system control with Gazebo for simulation, creating a Reinforcement Learning chess game for Panda and UR manipulator to play with humans. Currently, we have implemented the basic mechanism for the system.

## 👍Prerequisites
- ROS2 [humble]
- Ubuntu on WSL2[22.04.5 LTS]
- Gazebo [version 11.10.2]
- Python 3.8+

Note on Prerequisites: This project relies on various ROS2 and Gazebo packages. The exact requirements may vary depending on your system configuration and needs, download those packages if you encounter errors.
Also, if you want to be able to use UR manipulator arms, we need to install all the dependies shown in Universal_Robots_ROS2_Gazebo_Simulation repo.



## Environmental Setup 
Add these lines to your ~/.bashrc: (It's typically located inside your workspace)
```bash
# ROS2 and Development Tools
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# Workspace
source ~/ros2_ws/install/setup.bash

# WSL2 Display Configuration
export DISPLAY=:0
```

## 🚀Quick Install Guide
We need to create both Ubuntu and Anaconda environments for this project to avoid environment conflicts.

For Ubuntu:
```bash
# cd your ros2 workspace
cd ros2_ws # modify it with your own workspace name

# cd src
cd src

# Clone the repository
git clone git@github.com:Jack23162329/ChessGame_Auto.git

# back to workspace directory
cd ..

# Build the workspace
colcon build
source install/setup.bash
```
For Anaconda:
Installation follows chess-alpha-zero repo. 
[Details](https://github.com/Zeta36/chess-alpha-zero.git)
```bash
conda create -n env_name python=3.6.3
conda activate env_name
conda install tensorflow-gpu=1.3.0
conda install keras=2.0.8
```
After creating Anaconda Environment (clone repository):
```bash
cd [path] # Where you want to install repo
git@github.com:Zeta36/chess-alpha-zero.git
```
Download requirements for chess-alpha-zero:
```bash
pip install -r requirements.txt
```

## Project Structure
After installation, we need to replace `chess_model_server.py` from Ubuntu to your Anaconda env (chess-alpha-zero), then we'll have Project Structure as shown below.

👽Anaconda (chess-alpha-zero)
```bash
# only to instruct where to put chess_model_server.py here
chess-alpha-zero/
├── src/
│  ├── chess_model_server.py # put it here
│  ├── __init__.py
│  └── chess_zero
├── notebooks
├── data
├── binder
├── requirements.txt
└── readme.md
```
👾Ubuntu
```bash
ChessGame_Auto/
├── chess_robot_ai/
│  ├── chess_robot_ai/
│  │    ├── chess_robot_node.py # This is for panda manipulator
│  └──  └── chess_robot_node_UR.py # This is for UR manipulator
├── panda_env
├── ur_robot_env
├── notes
└── README.md
```

# 🐈Quick Start
After everything is set up:
## For Franka Emika Panda robot
Launch the world (Keep the world running):
```bash
ros2 launch panda_env simulation.launch.py
```
For testing movement (ChessWorld must remain running):
```bash
# In another terminal
ros2 run panda_env example_game
```
For Reinforcement Learning ChessGame (ChessWorld must remain running):

👽Anaconda:
```bash
# Need to be started inside your Anaconda environment
conda activate env_name
cd chess-alpha-zero\src # where you install the repo
python chess_model_server.py # Turn on server for RL model, sending RL model
```
👾Ubuntu:
```bash
# In another terminal
ros2 run chess_robot_ai chess_robot_node # For receiving RL model
```

## For Universal Robots
Launch the world (Keep the world running):
```bash
ros2 launch ur_robot_env ur_sim_control.launch.py ur_type:=ur10e #(3, 5, 5e, 10, 10e, etc…)
```
For testing movement (ChessWorld must remain running):
```bash
# In another terminal
ros2 run ur_robot_env ur_move
```
For Reinforcement Learning ChessGame (ChessWorld must remain running):

👽Anaconda:
```bash
# Need to be started inside your Anaconda environment
conda activate env_name
cd chess-alpha-zero\src # where you install the repo
python chess_model_server.py # Turn on server for RL model, sending RL model
```
👾Ubuntu:
```bash
# In another terminal
ros2 run chess_robot_ai chess_robot_node_ur # For receiving RL model
```
## ▶️Demo
Try to use RL to make auto chess game play

https://github.com/user-attachments/assets/47fa09a7-7d71-4cb4-9595-e0c2054d9e79

## Known Problems
1. We cannot play with KNIGHT, BISHOP and other pieces since we have not implemented the piece grasping function. (fixed with a simple hand made gripper)
2. Due to the manipulator's workspace limitations, we cannot reach the last row of the chessboard (a8, b8, etc.).(solved by using UR robot)









