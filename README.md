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
├── ur_chess_controller
├── ur_env
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
ros2 launch ur_env ur_sim_control.launch.py ur_type:=ur10e #(3, 5, 5e, 10, 10e, etc…)
```
For testing movement (ChessWorld must remain running):
```bash
# In another terminal
ros2 run ur_chess_controller example_game
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
2. Due to the manipulator's workspace limitations, we cannot reach the last row of the chessboard (a8, b8, etc.).

---

# ChessGame with RL

## 😸概述
本專案在 WSL2 環境下開發，使用了以下 GitHub 儲存庫:
  1. [Chess_manipulator(panda_env)](https://github.com/ZeinBarhoum/chess_manipulator.git)
  2. [Universal_Robots_ROS2_Gazebo_Simulation(ur_env)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git)
  3. [chess-alpha-zero(強化學習模型)](https://github.com/Zeta36/chess-alpha-zero.git)

我們使用 ROS2 進行系統控制，Gazebo 進行模擬，開發了一個基於強化學習的西洋棋遊戲系統，讓 Panda 和 UR 機械手臂能與人類對弈。目前已完成系統的基本機制實現。

## 👍環境需求
- ROS2 [humble]
- Ubuntu on WSL2 [22.04.5 LTS]
- Gazebo [版本 11.10.2]
- Python 3.8+

注意：本專案依賴多個 ROS2 和 Gazebo 套件。具體需求可能因系統配置而異，如遇到錯誤請安裝相應套件。

## 環境設置
請將以下指令加入您的 ~/.bashrc 檔案（通常位於工作區內）：
```bash
# ROS2 和開發工具
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# 工作區
source ~/ros2_ws/install/setup.bash

# WSL2 顯示設置
export DISPLAY=:0
```

## 🚀快速安裝指南
本專案需要同時建立 Ubuntu 和 Anaconda 環境以避免環境衝突。

Ubuntu 環境：
```bash
# 進入 ROS2 工作區
cd ros2_ws # 請改為您的工作區名稱

# 進入 src 目錄
cd src

# 克隆儲存庫
git clone git@github.com:Jack23162329/ChessGame_Auto.git

# 返回工作區目錄
cd ..

# 建置工作區
colcon build
source install/setup.bash
```

Anaconda 環境：
請依照 chess-alpha-zero 儲存庫的說明進行安裝。
[詳細說明](https://github.com/Zeta36/chess-alpha-zero.git)
```bash
conda create -n env_name python=3.6.3
conda activate env_name
conda install tensorflow-gpu=1.3.0
conda install keras=2.0.8
```

建立 Anaconda 環境後（克隆儲存庫）：
```bash
cd [路徑] # 選擇要安裝儲存庫的位置
git@github.com:Zeta36/chess-alpha-zero.git
```

安裝 chess-alpha-zero 所需套件：
```bash
pip install -r requirements.txt
```

## 專案結構
完成安裝後，需將 `chess_model_server.py` 從 Ubuntu 環境複製到 Anaconda 環境（chess-alpha-zero）中。專案結構如下：

👽Anaconda (chess-alpha-zero)
```bash
# 僅說明 chess_model_server.py 的放置位置
chess-alpha-zero/
├── src/
│   ├── chess_model_server.py # 放置於此
│   ├── __init__.py
│   └── chess_zero
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
│   ├── chess_robot_ai/
│   │    ├── chess_robot_node.py # Panda 機械手臂控制程式
│   └──  └── chess_robot_node_UR.py # UR 機械手臂控制程式
├── panda_env
├── ur_chess_controller
├── ur_env
├── notes
└── README.md
```

# 🐈快速開始
環境設置完成後：

## Franka Emika Panda 機器人
啟動模擬環境（保持運行）：
```bash
ros2 launch panda_env simulation.launch.py
```

測試移動功能（需保持模擬環境運行）：
```bash
# 在新終端中執行
ros2 run panda_env example_game
```

運行強化學習西洋棋遊戲（需保持模擬環境運行）：

👽Anaconda 環境：
```bash
# 在 Anaconda 環境中執行
conda activate env_name
cd chess-alpha-zero\src # 進入儲存庫安裝位置
python chess_model_server.py # 啟動伺服器，發送強化學習模型
```

👾Ubuntu 環境：
```bash
# 在新終端中執行
ros2 run chess_robot_ai chess_robot_node # 接收強化學習模型
```

## Universal Robots
啟動模擬環境（保持運行）：
```bash
ros2 launch ur_env ur_sim_control.launch.py ur_type:=ur10e #(可選 3, 5, 5e, 10, 10e 等型號)
```

測試移動功能（需保持模擬環境運行）：
```bash
# 在新終端中執行
ros2 run ur_chess_controller example_game
```

運行強化學習西洋棋遊戲（需保持模擬環境運行）：

👽Anaconda 環境：
```bash
# 在 Anaconda 環境中執行
conda activate env_name
cd chess-alpha-zero\src # 進入儲存庫安裝位置
python chess_model_server.py # 啟動伺服器，發送強化學習模型
```

👾Ubuntu 環境：
```bash
# 在新終端中執行
ros2 run chess_robot_ai chess_robot_node_ur # 接收強化學習模型
```

## ▶️演示
嘗試使用強化學習進行自動西洋棋對弈

https://github.com/user-attachments/assets/47fa09a7-7d71-4cb4-9595-e0c2054d9e79

## 已知問題
1. 目前無法操作騎士、主教等棋子，因為尚未實現抓取這些棋子的功能。
2. 由於機械手臂工作空間的限制，無法觸及棋盤最後一行（a8, b8 等位置）。
```


