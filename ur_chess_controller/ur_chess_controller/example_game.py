#!/usr/bin/env python3
from ur_chess_controller.URClass import ur10_6dof
import numpy as np
import time

class ChessRobotController:
    def __init__(self):
        self.robot = ur10_6dof()
        self.dx = 0.076
        self.x0 = 0.235
        self.y0 = -0.266
        self.ey = 0.003
        self.moving = 0.02
        self.positions = self._initialize_positions()

    def _initialize_positions(self):
        positions = [[[0,0] for i in range(8)] for j in range(8)]
        for i in range(8):
            for j in range(8):
                positions[i][j][0] = -(self.x0 + i*self.dx + 0.30) # change it if you change the position of the chessworld
                positions[i][j][1] = -(self.y0 + j*self.dx)
        return positions

    def go_to_piece(self, piece,error_y = 0):
        self.robot.move_to_pos_1step([self.positions[piece[0]-1][piece[1]-1][0],
                             self.positions[piece[0]-1][piece[1]-1][1] + self.ey + error_y,0.4],0.5,True)

    def go_down(self):
        self.robot.move_straight('vertical', -self.moving, 0.8, n=7, plan=True)
    def go_up(self):
        self.robot.move_straight('vertical', self.moving, 0.8, n=7, plan=True)
    #g represent grid
    def go_right(self,g):
        self.robot.move_straight('side',g * -self.moving, 0.8, n=7, plan=True)
    def go_left(self,g):
        self.robot.move_straight('side',g * -self.moving, 0.8, n=7, plan=True)
    def go_forward(self,g):
        self.robot.move_straight('forward',g * -self.moving, 0.8, n=7, plan=True)
    def go_backward(self,g):
        self.robot.move_straight('forward',g * -self.moving, 0.8, n=7, plan=True)

    def piece2piece(self, piece1, piece2, error_y=0):
        """Execute complete chess piece movement"""
        # epsilon = 1e-6
        print("Starting movement sequence...")
        
        # Step 1: Move to source position
        print("Moving to source position...")
        self.go_to_piece(piece1, error_y)
        
        # Step 2: Go down to grab piece
        # print("Going down...")
        self.go_down()

        # Calculate movement
        delta_x = piece2[0] - piece1[0]
        delta_y = piece2[1] - piece1[1]
        print(f"delta_x :{delta_x}, delta_y :{delta_y}")

        if delta_x > 0: #which means we need to move forwarddet
            self.go_forward(delta_x)
        elif delta_x < 0:
            self.go_backward(delta_x)
        else:
            pass
        
        if delta_y > 0:
            self.go_right(delta_y)
        elif delta_y < 0:
            self.go_left(delta_y)
        else:
            pass

        self.go_up()    
       
        print("Movement sequence completed!")
        return True

def main():
    controller = ChessRobotController()
    alpha_number = {
        'a': 8, 'b': 7, 'c': 6, 'd': 5,
        'e': 4, 'f': 3, 'g': 2, 'h': 1
    }

    # for i in range(1, 8+1):
    #     for j in range(1, 8+1):
    #         controller.go_to_piece([j, i])
    #         controller.robot.execute_plan(5, erase=True)
    #         print("Reset")
    #         controller.robot.reset()


    while True:
        try:
            move = input("Enter move (e.g. 'd2 to d4' or 'd2d4', or 'quit' to exit, 'reset' to reset: ").lower().strip()
            
            if move == 'quit':
                print("Exiting program...")
                break
            if move == 'reset':
                print("reseting the robot")
                controller.robot.reset()
            
            # Parse move
            if ' to ' in move:
                from_square, to_square = move.split(' to ')
            else:
                if len(move) != 4:
                    print("Invalid move format. Please use format like 'd2d4' or 'd2 to d4'")
                    continue
                from_square = move[:2]
                to_square = move[2:]
            
            # Validate input
            if (from_square[0] not in alpha_number or 
                to_square[0] not in alpha_number or 
                not from_square[1].isdigit() or 
                not to_square[1].isdigit() or
                not (1 <= int(from_square[1]) <= 8) or 
                not (1 <= int(to_square[1]) <= 8)):
                print("Invalid square position. Please use valid chess notation (a-h)(1-8)")
                continue
            
            print(f"Move from {from_square} to {to_square}")
            controller.piece2piece(
                [int(from_square[1]), alpha_number[from_square[0]]], 
                [int(to_square[1]), alpha_number[to_square[0]]]
            )
            controller.robot.execute_plan(10, erase=True)
            print("Reset")
            controller.robot.reset()
                         
        except KeyboardInterrupt:
            print("\nProgram interrupted. Stopping safely...")
            controller.robot.stop()
            controller.robot.reset()
            break
        except Exception as e:
            print(f"Error executing move: {str(e)}")
            print("Please try again.")

if __name__ == '__main__':
    main()