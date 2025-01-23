# ChessGame with RL

## 😸Overview
This project is implemented in WSL1 with GitHub repositories as shown below: (Updated: it's okay to use WSL2)
  1. [Chess_manipulator(panda_env)](https://github.com/ZeinBarhoum/chess_manipulator.git)
  2. [Universal_Robots_ROS2_Gazebo_Simulation(ur_env)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git)
  3. [chess-alpha-zero(for sending reinforcement learning model)](https://github.com/Zeta36/chess-alpha-zero.git)

We aim to use ROS2 for system control with Gazebo for simulation, creating a Reinforcement Learning chess game for Panda and UR manipulator to play with humans. Currently, we have implemented the basic mechanism for the system.

## 👍Prerequisites
- ROS2 [humble]
- Ubuntu [22.04.5 LTS]
- Gazebo [version 11.10.2]

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

# 強化學習西洋棋遊戲

## 😸概述
本專案在 WSL1 環境下開發，使用以下 GitHub 儲存庫：
  1. [Chess_manipulator(panda_env)](https://github.com/ZeinBarhoum/chess_manipulator.git)
  2. [Universal_Robots_ROS2_Gazebo_Simulation(ur_env)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git)
  3. [chess-alpha-zero(強化學習模型)](https://github.com/Zeta36/chess-alpha-zero.git)

我們使用 ROS2 進行系統控制，Gazebo 進行模擬，創建一個讓 Panda 和 UR 機械手臂與人類對弈的強化學習西洋棋遊戲。目前已完成系統基礎機制的實作。

## 👍系統需求
- ROS2 [humble]
- Ubuntu [22.04.5 LTS]
- Gazebo [version 11.10.2]

## 🚀快速安裝指南
為避免環境衝突，我們需要建立 Ubuntu 和 Anaconda 兩個環境。

Ubuntu 環境：
```bash
# 切換至 ros2 工作區
cd ros2_ws # 請改為您的工作區名稱

# 進入 src 目錄
cd src

# 複製儲存庫
git clone git@github.com:Jack23162329/ChessGame_Auto.git

# 返回工作區目錄
cd ..

# 建置工作區
colcon build
source install/setup.bash
```
Anaconda 環境：
請依照 chess-alpha-zero 儲存庫說明安裝。
[詳細資訊](https://github.com/Zeta36/chess-alpha-zero.git)
```bash
conda create -n env_name python=3.6.3
conda activate env_name
conda install tensorflow-gpu=1.3.0
conda install keras=2.0.8
```
建立 Anaconda 環境後（複製儲存庫）：
```bash
cd [path] # 選擇要安裝的位置
git@github.com:Zeta36/chess-alpha-zero.git
```
下載 chess-alpha-zero 相關套件：
```bash
pip install -r requirements.txt
```

## 專案結構
安裝完成後，需要將 Ubuntu 環境中的 `chess_model_server.py` 複製到 Anaconda 環境 (chess-alpha-zero)，之後的專案結構如下：

👽Anaconda (chess-alpha-zero)
```bash
# 說明 chess_model_server.py 的放置位置
chess-alpha-zero/
├── src/
│  ├── chess_model_server.py # 放在這裡
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
│  │    ├── chess_robot_node.py # Panda 機械手臂控制
│  └──  └── chess_robot_node_UR.py # UR 機械手臂控制
├── panda_env
├── ur_chess_controller
├── ur_env
├── notes
└── README.md
```

# 🐈快速開始
完成所有設定後：
## Franka Emika Panda 機器人
啟動世界（保持運行）：
```bash
ros2 launch panda_env simulation.launch.py
```
測試移動（ChessWorld 必須保持運行）：
```bash
# 在另一個終端機
ros2 run panda_env example_game
```
強化學習西洋棋遊戲（ChessWorld 必須保持運行）：

👽Anaconda：
```bash
# 需在 Anaconda 環境中啟動
conda activate env_name
cd chess-alpha-zero\src # 儲存庫安裝位置
python chess_model_server.py # 啟動 RL 模型伺服器，傳送 RL 模型
```
👾Ubuntu：
```bash
# 在另一個終端機
ros2 run chess_robot_ai chess_robot_node # 接收 RL 模型
```

## Universal Robots
啟動世界（保持運行）：
```bash
ros2 launch ur_env ur_sim_control.launch.py ur_type:=ur10e #(可選 3, 5, 5e, 10, 10e 等)
```
測試移動（ChessWorld 必須保持運行）：
```bash
# 在另一個終端機
ros2 run ur_chess_controller example_game
```
強化學習西洋棋遊戲（ChessWorld 必須保持運行）：

👽Anaconda：
```bash
# 需在 Anaconda 環境中啟動
conda activate env_name
cd chess-alpha-zero\src # 儲存庫安裝位置
python chess_model_server.py # 啟動 RL 模型伺服器，傳送 RL 模型
```
👾Ubuntu：
```bash
# 在另一個終端機
ros2 run chess_robot_ai chess_robot_node_ur # 接收 RL 模型
```
## ▶️示範
嘗試使用強化學習進行自動西洋棋對弈

https://github.com/user-attachments/assets/47fa09a7-7d71-4cb4-9595-e0c2054d9e79

## 已知問題
1. 由於尚未實作棋子抓取功能，目前無法使用騎士、主教等棋子。
2. 受限於機械手臂工作空間，無法觸及棋盤最後一列（a8、b8 等位置）。
