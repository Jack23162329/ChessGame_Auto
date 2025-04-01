#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from ur_robot_env.URClass import ur10_6dof

# Persistent publisher node for gripper commands.
class GripperPublisher(Node):
    def __init__(self):
        super().__init__('gripper_command_publisher')
        # Create publisher once, which will remain active
        self.publisher = self.create_publisher(Float64MultiArray, '/gripper_effort_controller/commands', 10)

    def publish_command(self, command_data):
        """
        Publish a gripper command without reinitializing the node.
        
        Args:
            command_data (list): List of effort values, e.g. [-0.25, 0.25] for closing and [0.25, -0.25] for opening.
        """
        msg = Float64MultiArray()
        msg.data = command_data
        self.get_logger().info("Publishing gripper command: " + str(command_data))
        self.publisher.publish(msg)

# Modified ChessRobotController that uses the persistent gripper publisher.
class ChessRobotController:
    def __init__(self, gripper_publisher):
        self.robot = ur10_6dof()
        # Save the persistent gripper publisher instance
        self.gripper_publisher = gripper_publisher
        self.dx = 0.076
        self.x0 = 0.235
        self.y0 = -0.266
        self.ey = 0.003
        self.moving = 0.02
        self.positions = self._initialize_positions()

    def _initialize_positions(self):
        positions = [[[0, 0] for _ in range(8)] for _ in range(8)]
        for i in range(8):
            for j in range(8):
                positions[i][j][0] = -(self.x0 + i * self.dx + 0.29)
                positions[i][j][1] = -(self.y0 + j * self.dx)
        return positions

    def go_to_piece(self, piece, error_y=0):
        pos = self.positions[piece[0] - 1][piece[1] - 1]
        target_pos = [pos[0], pos[1] + self.ey + error_y, 0.4]
        self.robot.move_to_pos_1step(target_pos, 0.5, True)

    def go_down(self):
        self.robot.move_straight('vertical', -self.moving, 0.8, n=3, plan=True)

    def go_up(self):
        self.robot.move_straight('vertical', self.moving, 0.8, n=3, plan=True)

    def go_right(self, g):
        self.robot.move_straight('side', g * -self.moving, 0.8, n=7, plan=True)

    def go_left(self, g):
        self.robot.move_straight('side', g * -self.moving, 0.8, n=7, plan=True)

    def go_forward(self, g):
        self.robot.move_straight('forward', g * -self.moving, 0.8, n=7, plan=True)

    def go_backward(self, g):
        self.robot.move_straight('forward', g * -self.moving, 0.8, n=7, plan=True)

    def piece2piece(self, piece1, piece2, error_y=0, d=0.09):
        print("Starting movement sequence...")

        # Step 1: Move to source position
        print("Moving to source position...")
        self.go_to_piece(piece1, error_y)
        self.robot.execute_plan(5, erase=True)
        time.sleep(1.0)

        # Step 2: Lower to grasp position
        print("Lowering to grasp position...")
        self.go_down()
        self.robot.execute_plan(5, erase=True)
        time.sleep(2.0)

        # Step 3: Close gripper to grasp the piece
        print("Closing gripper to grasp piece...")
        self.gripper_publisher.publish_command([-0.75, 0.75])
        time.sleep(2.0)

        # Step 4: Raise with the piece
        print("Raising with the piece...")
        self.go_up()
        self.robot.execute_plan(5, erase=True)
        time.sleep(2.0)

        # Step 5: Move from source to destination
        delta_x = piece2[0] - piece1[0]
        delta_y = piece2[1] - piece1[1]
        print(f"Moving: delta_x = {delta_x}, delta_y = {delta_y}")
        if delta_x > 0:
            self.go_forward(delta_x)
        elif delta_x < 0:
            self.go_backward(delta_x)
        if delta_y > 0:
            self.go_right(delta_y)
        elif delta_y < 0:
            self.go_left(delta_y)

        # Step 6: Lower to placement position
        print("Lowering to place the piece...")
        self.go_down()
        self.robot.execute_plan(5, erase=True)
        time.sleep(2.0)

        # Step 7: Open gripper to release the piece
        print("Opening gripper to release piece...")
        self.gripper_publisher.publish_command([0.25, -0.25])
        time.sleep(2.0)

        # Step 8: Raise after releasing the piece
        self.go_up()

        print("Movement sequence completed!")
        return True

def main():
    # Initialize ROS only once
    rclpy.init(args=None)

    # Create a persistent gripper publisher node
    gripper_publisher = GripperPublisher()
    # Pass the gripper publisher to the robot controller
    controller = ChessRobotController(gripper_publisher)

    # Mapping chess board letters to numbers
    alpha_number = {
        'a': 8, 'b': 7, 'c': 6, 'd': 5,
        'e': 4, 'f': 3, 'g': 2, 'h': 1
    }

    while True:
        try:
            move = input("Enter move (e.g. 'd2 to d4' or 'd2d4', 'quit' to exit, 'reset' to reset): ").lower().strip()
            if move == 'quit':
                print("Exiting program...")
                break
            if move == 'reset':
                print("Resetting the robot...")
                controller.robot.reset()
                continue

            # Parse move input
            if ' to ' in move:
                from_square, to_square = move.split(' to ')
            else:
                if len(move) != 4:
                    print("Invalid move format. Please use 'd2d4' or 'd2 to d4' format")
                    continue
                from_square = move[:2]
                to_square = move[2:]

            # Validate chess square notation
            if (from_square[0] not in alpha_number or
                to_square[0] not in alpha_number or
                not from_square[1].isdigit() or
                not to_square[1].isdigit() or
                not (1 <= int(from_square[1]) <= 8) or
                not (1 <= int(to_square[1]) <= 8)):
                print("Invalid chess square. Use valid chess notation (a-h)(1-8)")
                continue

            print(f"Moving from {from_square} to {to_square}")
            controller.piece2piece(
                [int(from_square[1]), alpha_number[from_square[0]]],
                [int(to_square[1]), alpha_number[to_square[0]]]
            )
            controller.robot.execute_plan(5, erase=True)
            print("Resetting robot...")
            controller.robot.reset()

        except KeyboardInterrupt:
            print("\nProgram interrupted. Stopping safely...")
            break

    # Clean up the persistent publisher node and shutdown ROS once done.
    gripper_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
