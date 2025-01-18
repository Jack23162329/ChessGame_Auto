import numpy as np
import RobotClass

def test_corner_center_movement():
    robot = RobotClass.panda_7dof()
    
    # Parameters
    dx = 0.076
    x0 = 0.235
    y0 = -0.266
    ey = 0.003
    
    try:
        # First reach position above pawn
        print("Moving to position [2,4]...")
        robot.move_to_pos_1step([x0 + dx, y0 + 3*dx, 0.4], 5, True)
        
        # Move straight down with more control
        print("Moving down to catch the pawn...")
        robot.move_vertical_straight(-0.025, 0.8, 8, True)  # Small distance, more steps for smoothness
        print("Moving up a little bit before it change a position")
        robot.move_vertical_straight(0.025, 0.8, 8, True)  # Small distance, more steps for smoothness
        
        print("Moving to position [4,4]...")
        robot.move_to_pos_1step([x0 + 3 * dx, y0 + 3*dx, 0.4], 5, True)
        print("Moving down to catch the pawn...")
        robot.move_vertical_straight(-0.025, 0.8, 8, True)  # Small distance, more steps for smoothness
        print("Moving up a little bit before it change a position")
        robot.move_vertical_straight(0.025, 0.8, 8, True)

        # Execute plan
        print("Executing movement plan...")
        robot.execute_plan(20, erase=True)

        robot.reset()
        
    except Exception as e:
        print(f"Error occurred: {e}")

if __name__ == "__main__":
    test_corner_center_movement()