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
1. We cannot play with KNIGHT, BISHOP and other pieces since we have not implemented the piece grasping function.
2. Due to the manipulator's workspace limitations, we cannot reach the last row of the chessboard (a8, b8, etc.).(solved by using UR robot)

---

RL 棋局遊戲
😸概述
本專案在 WSL2 環境下實作，並使用以下 GitHub 倉庫：

Chess_manipulator(panda_env)

Universal_Robots_ROS2_Gazebo_Simulation(ur_env)

chess-alpha-zero（用於傳送強化學習模型）

我們的目標是利用 ROS2 進行系統控制，並藉由 Gazebo 進行模擬，創造一個由 Panda 與 UR 機械手臂與人類對弈的強化學習棋局遊戲。目前，我們已實作系統的基本機制。

👍先決條件
ROS2 [humble]

在 WSL2 上執行的 Ubuntu [22.04.5 LTS]

Gazebo [版本 11.10.2]

Python 3.8+

先決條件說明：本專案依賴多個 ROS2 與 Gazebo 套件，具體需求可能會根據您的系統配置和需求有所不同，若遇到錯誤請下載相應的套件。
此外，若您希望使用 UR 機械手臂，則必須安裝 Universal_Robots_ROS2_Gazebo_Simulation 倉庫中所列的所有依賴項。

環境設置
請在您的 ~/.bashrc 中加入以下行（通常位於您的工作區中）：

bash
複製
# ROS2 與開發工具
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# 工作區
source ~/ros2_ws/install/setup.bash

# WSL2 顯示設定
export DISPLAY=:0
🚀快速安裝指南
為避免環境衝突，我們需要為本專案建立 Ubuntu 與 Anaconda 兩個環境。

對於 Ubuntu：
bash
複製
# 進入您的 ros2 工作區
cd ros2_ws  # 請根據您的工作區名稱修改

# 進入 src 目錄
cd src

# 複製倉庫
git clone git@github.com:Jack23162329/ChessGame_Auto.git

# 返回工作區目錄
cd ..

# 建置工作區
colcon build
source install/setup.bash
對於 Anaconda：
安裝流程參考 chess-alpha-zero 倉庫。
詳情

bash
複製
conda create -n env_name python=3.6.3
conda activate env_name
conda install tensorflow-gpu=1.3.0
conda install keras=2.0.8
建立 Anaconda 環境後（複製倉庫）：

bash
複製
cd [path]  # 您希望安裝倉庫的目錄
git clone git@github.com:Zeta36/chess-alpha-zero.git
下載 chess-alpha-zero 的需求套件：

bash
複製
pip install -r requirements.txt
專案結構
安裝完成後，我們需要將 chess_model_server.py 從 Ubuntu 移至您的 Anaconda 環境（chess-alpha-zero），專案結構如下所示：

👽Anaconda (chess-alpha-zero)
bash
複製
# 此處指示將 chess_model_server.py 放置的位置
chess-alpha-zero/
├── src/
│   ├── chess_model_server.py  # 放置於此
│   ├── __init__.py
│   └── chess_zero
├── notebooks
├── data
├── binder
├── requirements.txt
└── readme.md
👾Ubuntu
bash
複製
ChessGame_Auto/
├── chess_robot_ai/
│   ├── chess_robot_ai/
│   │   ├── chess_robot_node.py  # 用於 Panda 機械手臂
│   └── └── chess_robot_node_UR.py  # 用於 UR 機械手臂
├── panda_env
├── ur_robot_env
├── notes
└── README.md
🐈快速啟動
完成所有設定後：

適用於 Franka Emika Panda 機械手臂
啟動模擬世界（請保持世界模擬持續運行）：

bash
複製
ros2 launch panda_env simulation.launch.py
測試運動（ChessWorld 必須保持運行）：

bash
複製
# 在另一個終端機中執行
ros2 run panda_env example_game
啟動強化學習棋局遊戲（ChessWorld 必須保持運行）：

👽Anaconda：
bash
複製
# 請在您的 Anaconda 環境中啟動
conda activate env_name
cd chess-alpha-zero\src  # 進入倉庫安裝目錄
python chess_model_server.py  # 啟動 RL 模型伺服器，傳送 RL 模型
👾Ubuntu：
bash
複製
# 在另一個終端機中執行
ros2 run chess_robot_ai chess_robot_node  # 用於接收 RL 模型
適用於 Universal Robots
啟動模擬世界（請保持世界模擬持續運行）：

bash
複製
ros2 launch ur_robot_env ur_sim_control.launch.py ur_type:=ur10e  # (可選型號：3, 5, 5e, 10, 10e 等)
測試運動（ChessWorld 必須保持運行）：

bash
複製
# 在另一個終端機中執行
ros2 run ur_robot_env ur_move
啟動強化學習棋局遊戲（ChessWorld 必須保持運行）：

👽Anaconda：
bash
複製
# 請在您的 Anaconda 環境中啟動
conda activate env_name
cd chess-alpha-zero\src  # 進入倉庫安裝目錄
python chess_model_server.py  # 啟動 RL 模型伺服器，傳送 RL 模型
👾Ubuntu：
bash
複製
# 在另一個終端機中執行
ros2 run chess_robot_ai chess_robot_node_ur  # 用於接收 RL 模型
▶️示範
嘗試使用強化學習進行自動棋局對弈

示範連結

已知問題
由於尚未實作棋子抓取功能，我們無法與 KNIGHT（騎士）、BISHOP（主教）及其他棋子進行對弈。

由於機械手臂工作範圍的限制，我們無法觸及棋盤的最末一行（如 a8, b8 等）。(已透過使用 UR 機械手臂解決此問題)


