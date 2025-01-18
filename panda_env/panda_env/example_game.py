
import numpy as np
import RobotClass

robot = RobotClass.panda_7dof()

# initial setup

dx = 0.076  # Distance between chess squares in meters
x0 = 0.235  # Starting x-coordinate
y0 = -0.266 # Starting y-coordinate
ey = 0.003  # Small error adjustment in y direction

moving = 0.02

positions = [[[0,0] for i in range(8)] for j in range(8)]

for i in range(8):
    for j in range(8):
        positions [i][j][0] = x0 + i*dx
        positions [i][j][1] = y0 + j*dx

def go_to_piece(piece,error_y = 0):
    robot.move_to_pos_1step([positions[piece[0]-1][piece[1]-1][0],
                             positions[piece[0]-1][piece[1]-1][1] + ey + error_y,0.4],0.5,True)

def go_down():
    robot.move_straight('vertical', -moving, 0.8, n=8, plan=True)
def go_up():
    robot.move_straight('vertical', moving, 0.8, n=7, plan=True)
#g represent grid
def go_right(g):
    robot.move_straight('side',g * moving, 0.8, n=7, plan=True)
def go_left(g):
    robot.move_straight('side',g * moving, 0.8, n=7, plan=True)
def go_forward(g):
    robot.move_straight('forward',g * moving, 0.8, n=7, plan=True)
def go_backward(g):
    robot.move_straight('forward',g * moving, 0.8, n=7, plan=True)

    
# def go_down(d = 0.09):
#     robot.direction('d',d,1,5,True)
# def go_up(d = 0.09):
#     robot.direction('u',d,1,5,True)
# def go_to_corner():
#     robot.direction('l',dx/2,1,5,True)
#     robot.direction('f',dx/2,1,5,True)
# def go_to_center():
#     robot.direction('r',dx/2,1,4,True)
#     robot.direction('b',dx/2+0,1,4,True)
# def centralize():
#     robot.direction('l',dx/8,1,4,True)
#     robot.direction('b',dx/8+0,1,4,True)
#     robot.direction('f',dx/8+0,1,4,True)    

def piece2piece(piece1,piece2,error_y = 0, d=0.09):
    
    #step one--> get the position
    go_to_piece(piece1,error_y)
    #step two--> go down and grab it
    go_down()

    #step three--> calculate how much should I move?
    x1 = piece1[0]
    y1 = piece1[1]

    x2 = piece2[0]
    y2 = piece2[1]
    
    delta_x = x2 - x1
    delta_y = y2 - y1
    print(f"delta_x :{delta_x}, delta_y :{delta_y}")

    if delta_x > 0: #which means we need to move forwarddet
        go_forward(delta_x)
    elif delta_x < 0:
        go_backward(delta_x)
    else:
        pass
    
    if delta_y > 0:
        go_right(delta_y)
    elif delta_y < 0:
        go_left(delta_y)
    else:
        pass
    #step four--> go up and reset
    go_up()

    
#It's originall code from github
    # go_down(d)   
    # go_to_corner()
    # x1 = piece1[0]
    # y1 = piece1[1]

    # x2 = piece2[0]
    # y2 = piece2[1]
    
    # delta_x = x2 - x1
    # delta_y = y2 - y1
    # if delta_x>0:
    #     robot.direction('f',delta_x*dx,1,2 + abs(delta_x),True)
    # else:
    #     robot.direction('b',-delta_x*dx,1,2 + abs(delta_x),True)
    
    # if delta_y>0:
    #     robot.direction('l',delta_y*dx,1,2 + abs(delta_y),True)
    # else:
    #     robot.direction('r',-delta_y*dx,1,2 + abs(delta_y),True)
    
    # go_to_center()
    
    # # centralize()
    
    # go_up(d)
    
def main():
    
    # Use while True to keep the program running indefinitely
    while True:
        try:
            # Get input from user
            move = input("Enter move (e.g. 'd2 to d4' or 'd2d4', or 'quit' to exit): ").lower().strip()
            
            # Add exit condition
            if move == 'quit':
                print("Exiting program...")
                break
                
            # Handle different input formats
            if ' to ' in move:
                from_square, to_square = move.split(' to ')
            else:
                # Assume format like 'd2d4'
                if len(move) != 4:
                    print("Invalid move format. Please use format like 'd2d4' or 'd2 to d4'")
                    continue
                from_square = move[:2]  # d2
                to_square = move[2:]    # d4
                
            print(f"Move from {from_square} to {to_square}")
            
            alpha_number = {
                'a': 8,
                'b': 7,
                'c': 6,
                'd': 5,
                'e': 4,
                'f': 3,
                'g': 2,
                'h': 1
            }
            
            # Validate input before processing
            if (from_square[0] not in alpha_number or 
                to_square[0] not in alpha_number or 
                not from_square[1].isdigit() or 
                not to_square[1].isdigit()):
                print("Invalid square position. Please use valid chess notation (a-h)(1-8)")
                continue
                
            eng1 = from_square[0]
            eng2 = to_square[0]
            
            # Execute the move
            piece2piece([int(from_square[1]), alpha_number[eng1]], 
                       [int(to_square[1]), alpha_number[eng2]])
            robot.execute_plan(5, erase=True)
            robot.reset()
            
            
        except ValueError as e:
            print(f"Error: {e}")
            print("Please try again.")
        except Exception as e:
            print(f"Unexpected error: {e}")
            print("Please try again.")

    #@@# print("Let's move all the pawn forward")
    # row = 2
    # for i in range(1, 9):
    #     print(i)
    #     piece2piece([row,i],[row+2,i])
    #     robot.execute_plan(20,erase=True) # change the number here to change the operation time
    #     robot.reset()
    
    # row = 4
    # print("Now let's put it back")
    # for j in range(1, 9):
    #     print(j)
    #     piece2piece([row,j],[row-2,j])
    #     robot.execute_plan(20,erase=True)
    #     robot.reset()
    
    # row = 7
    # print("This time is the opponite pawn")
    # for j in range(1, 9):
    #     print(j)
    #     piece2piece([row,j],[row-2,j])
    #     robot.execute_plan(20,erase=True)
    #     robot.reset()
    


    #@@# testing for manipulator arm can go to each points of the checkerboard
    # print("let't go to every corner on the checkboard")
    # for i in range(1, 9):
    #     for j in range(1, 9):
    #         piece2piece([i, j], [i, j])
    #         robot.execute_plan(5, erase= True)
    #         robot.reset()
    
    # print('Executing d2 to d4')
    # piece2piece([2,5],[4,5])
    # robot.execute_plan(30,erase=True)

    # print('Executing d1 takes d4')

    # piece2piece([1,5],[4,5])
    # robot.execute_plan(60,erase=True)
        
if __name__ == '__main__':
    
    main()
    
