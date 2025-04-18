import rclpy
from rclpy.node import Node
import requests
import numpy as np
import time
import sys
import os

# Directly set the path to your source files
manipulator_path = "/home/jack/ros2_ws/src/ChessGame_Auto/ur_chess_controller/ur_chess_controller"

print(f"Adding to path: {manipulator_path}")
if os.path.exists(manipulator_path):
    print(f"Path exists! Contents: {os.listdir(manipulator_path)}")
    sys.path.append(manipulator_path)
else:
    print("Path does not exist!")

try:
    from URClass import ur10_6dof  # Changed from panda_7dof
    from example_game import ChessRobotController
    print("Successfully imported URClass and ChessRobotController")
except Exception as e:
    print(f"Error importing modules: {e}")
    print(f"Current sys.path: {sys.path}")


class ChessRobotNodeUR(Node):
    def __init__(self):
        super().__init__('chess_robot_node_ur')
        
        # Model server configuration
        self.model_url = 'http://host.docker.internal:5000'

        self.alpha_number = {
            'a': 8, 'b': 7, 'c': 6, 'd': 5,
            'e': 4, 'f': 3, 'g': 2, 'h': 1
        }
        
        try:
            # Just create the chess controller - it will create its own robot
            self.chess_controller = ChessRobotController()
            # Get the robot reference from the controller
            self.robot = self.chess_controller.robot
            self.get_logger().info("UR10 robot initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize UR10 robot: {e}")
            raise

        self.create_timer(5.0, self.check_server_health)
        self.wait_for_server()
        self.Start_game()

    def Start_game(self):
        """Test full pipeline: model prediction and robot movement"""
        try:
            # Test server connection
            response = requests.get(f"{self.model_url}/health")
            if response.status_code == 200:
                self.get_logger().info("Successfully connected to model server!")
                
                # we start to play the game
                self.play_game()
            else:
                self.get_logger().info("Failed to connect to model server")
                # Test getting a move prediction
                # self.get_logger().info("Requesting prediction from model...")
                # move = self.get_ai_move()
                
                # if move:
                #     self.get_logger().info(f"Received move prediction: {move}")
                    
                #     # Test robot movement with the predicted move
                #     success = self.execute_move(move)
                    
                #     if success:
                #         self.get_logger().info("Test move executed successfully!")
                #         # Start the continuous game loop after successful test
                #         self.play_game()
                #     else:
                #         self.get_logger().error("Failed to execute test move")
                # else:
                #     self.get_logger().error("Failed to get move prediction")
                
        except Exception as e:
            self.get_logger().error(f"Test failed: {str(e)}")

    
    def check_server_health(self):
        try:
            response = requests.get(f"{self.model_url}/health")
            if response.status_code == 200:
                self.get_logger().debug("Model server is healthy")
            else:
                self.get_logger().warn("Model server health check failed")
        except requests.exceptions.ConnectionError:
            self.get_logger().error("Lost connection to model server")

    def wait_for_server(self):
        """Wait for model server to be available"""
        self.get_logger().info("Waiting for model server...")
        while rclpy.ok():
            try:
                response = requests.get(f"{self.model_url}/health")
                if response.status_code == 200:
                    self.get_logger().info("Connected to model server!")
                    break
            except requests.exceptions.ConnectionError:
                self.get_logger().warn("Waiting for model server to start...")
                time.sleep(2)
    
    def get_prediction(self, board_state):
        """Get prediction from model server"""
        try:
            # Now we just need the move, not policy/value
            response = requests.post(
                f"{self.model_url}/predict",
                json={'board_state': board_state.tolist() if board_state is not None else None}
            )
            
            if response.status_code == 200:
                result = response.json()
                if result.get('status') == 'success' and 'move' in result:
                    return result['move']
                else:
                    self.get_logger().error(f"Prediction failed: {result}")
                    return None
                    
            else:
                self.get_logger().error(f"Prediction request failed with status {response.status_code}")
                return None
                    
        except Exception as e:
            self.get_logger().error(f"Error getting prediction: {str(e)}")
            return None
    
    def execute_move(self, chess_move):
        """Execute move using ChessRobotController"""
        try:
            from_square = chess_move[:2]
            to_square = chess_move[2:]
            
            self.get_logger().info(f"UR10 moving from {from_square} to {to_square}")
            
            from_pos = [int(from_square[1]), self.alpha_number[from_square[0]]]
            to_pos = [int(to_square[1]), self.alpha_number[to_square[0]]]
            
            self.get_logger().info(f"UR10 coordinates - From: {from_pos}, To: {to_pos}")
            
            # Use the chess controller instance
            self.chess_controller.piece2piece(from_pos, to_pos)
            self.robot.execute_plan(5, erase=True)
            self.robot.reset()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error executing UR10 move: {str(e)}")
            return False
        
    def get_ai_move(self):
        """Get move from AI server"""
        try:
            # Request prediction without sending board state since server maintains it
            response = requests.post(f"{self.model_url}/predict")
            if response.status_code == 200:
                result = response.json()
                if result['status'] == 'success' and 'move' in result:
                    move = result['move']
                    self.get_logger().info(f"AI suggests move: {move}")
                    
                    # Log current board state if available
                    if 'board_state' in result:
                        self.get_logger().info(f"Current board state: \n{result['board_state']}")
                    
                    return move
                else:
                    self.get_logger().error(f"Invalid response format: {result}")
                    return None
            else:
                self.get_logger().error(f"Server returned status code: {response.status_code}")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Error getting AI move: {str(e)}")
            return None

    def play_game(self):
        """Main game loop"""
        try:
            # Create a timer for the game loop
            self.create_timer(0.5, self._game_loop_callback)  # 2 second interval
            self.get_logger().info("Game loop started")
            
        except Exception as e:
            self.get_logger().error(f"Error setting up game loop: {str(e)}")

    def _game_loop_callback(self):
        """Callback for the game loop timer with error handling"""
        try:
            # Get AI's move
            move = self.get_ai_move()
            
            if move:
                self.get_logger().info(f"Executing AI move: {move}")
                
                # Add delay between moves
                time.sleep(1)  # Add small delay between moves
                
                success = self.execute_move(move)
                
                if success:
                    # Update server's board state
                    response = requests.post(
                        f"{self.model_url}/update_board", 
                        json={'move': move}
                    )
                    
                    if response.status_code == 200:
                        self.get_logger().info("Move executed and board updated successfully!")
                        # Add delay after successful move
                        time.sleep(2)  # Wait before next move
                    else:
                        self.get_logger().error("Failed to update server board state")
                else:
                    self.get_logger().error("Failed to execute move!")
                    # Add recovery mechanism
                    self.robot.reset()  # Reset robot on failure
                    time.sleep(1)  # Wait before retry
                    
        except Exception as e:
            self.get_logger().error(f"Error in game loop: {str(e)}")
            # Add recovery mechanism
            self.robot.reset()  # Reset robot on error
            time.sleep(1)  # Wait before continuing

def main(args=None):
    rclpy.init(args=args)
    node = ChessRobotNodeUR()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
