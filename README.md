# ChessGame with RL

## 😸Overview
This project is under the construction of WSL1 with github repositories as shown below
  1. [Chess_manipulator(panda_env)](https://github.com/ZeinBarhoum/chess_manipulator.git)
  2. [Universal_Robots_ROS2_Gazebo_Simulation(ur_env)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git)
  3. [chess-alpha-zero(for sending reinforcement learning model)](https://github.com/Zeta36/chess-alpha-zero.git)

We aimed to use ROS2 for system control with Gazebo for simulation, creating Reinfocement learning chess game for Panda and UR manipulator to play with Human, right now we only build the machanism for it.




## 👍Prerequisites
- ROS2 [humble]
- Ubuntu [22.04.5 LTS]
- Gazebo [version 11.10.2]


## 🚀Quick Install Guide
We need to create both Ubuntu and Anaconda Environment for this project to avoid Environmental collision 

For Ubuntu:
```bash
# cd your ros2 workspace
cd ros2_ws # modified it with your own workspace name

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
Installation follow chess-alpha-zero repo. 
[Details](https://github.com/Zeta36/chess-alpha-zero.git)
```bash
conda create -n env_name python=3.6.3
conda activate env_name
conda install tensorflow-gpu=1.3.0
conda install keras=2.0.8
```
After creating Anaconda Environment (clone repository)
```bash
cd [path] # Where you want to install repo
git@github.com:Zeta36/chess-alpha-zero.git
```
Download requirements for chess-alpha-zero
```bash
pip install -r requirements.txt
```


## Project Structure
After installation, we need to replace `chess_model_server.py` from Ubuntu to your Aconda env (chess-alpha-zero), then we'll have Project Structure as shown below.

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
├── ur_chess_controller
├── ur_env
├── notes
└── README.md
```

# 🐈Quick Start
After everthing is setup
## For Franka Emika Panda robot
Open the world (Keep world open)
```bash
ros2 launch panda_env simulation.launch.py
```
For testing movement (ChessWorld needs keep open)
```bash
# In another terminal
ros2 run panda_env example_game
```
For Reinforcement Learning ChessGame (ChessWorld needs keep open)

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
Open the world (Keep world open)
```bash
ros2 launch ur_env ur_sim_control.launch.py ur_type:=ur10e #(3, 5, 5e, 10, 10e, etc…)

```
For testing movement (ChessWorld needs keep open)
```bash
# In another terminal
ros2 run ur_chess_controller example_game
```
For Reinforcement Learning ChessGame (ChessWorld needs keep open)

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



