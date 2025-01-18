from spatialmath import SE3
from roboticstoolbox.models.DH import Panda
from roboticstoolbox import DHRobot
from roboticstoolbox.tools import ctraj,jtraj
import numpy as np
import os


class panda_7dof(Panda):
    def __init__(self,q0 = None):
        super().__init__()
        
        if(q0 == None):
            self.q0 = [0]*7
            self.q0[6]=np.pi/4
        else:
            self.q0 = q0
        self.q = self.q0
        self.p = self.fk_pos(self.q)
        
        self.plan = []
        
        self.set_q(self.q,1)
    
        
    def ik(self, T):
        q = self.ikine_LM(T, joint_limits = True).q
        return q
    
    def fk(self, q):
        return self.fkine(q)
    
    def fk_pos(self,q):
        return self.fkine(q).t
    
    def traj (self, p1, p2, n):
        ps = jtraj(p1,p2,n).q
        q = []
        for p in ps:
            T = SE3.Trans(p)* SE3.Rx(np.pi)
            q.append(self.ik(T))
        return q
            
    def string_qs(self, qs):
        s=''
        for q in qs:
            for value in q:
                s+=str(value)+' '
        return s
    def string_q(self, q):
        s=''
        for value in q:
            s+=str(value)+' '
        return s
    
    def move_linear(self, p, t, n=3, plan=False):
        p0 = self.fk_pos(self.q)
        qs = self.traj(p0,p,n)
        
        dt = t/n
        if(plan):
            self.plan.extend(qs)
        else:
            os.system(f'ros2 run chess_manipulator multi_point_controller {int(n)} {int(dt*1e9)} {self.string_qs(qs)}')
        self.update(qs[-1])
        self.p = p
    
    def direction(self, direction, distance, t, n=3, plan = False):
        p = self.p.copy()
        if direction=='f':
            p[0]+=distance
        elif direction=='b':
            p[0]-=distance
        elif direction=='l':
            p[1]+=distance
        elif direction=='r':
            p[1]-=distance  
        elif direction=='u':
            p[2]+=distance
        elif direction=='d':
            p[2]-=distance
        else:
            print('Wrong Direction')
            return
        self.move_linear(p,t,n,plan)  

    # def move_straight(self, direction, distance, t, n=5, plan=False):
    #     """
    #     Move in a straight line while maintaining orientation
    #     Args:
    #         direction: 'vertical', 'forward', 'backward', 'left', 'right'
    #         distance: distance to move in meters
    #         t: time duration
    #         n: number of steps
    #         plan: whether to add to plan or execute immediately
    #     """
    #     current_q = self.q.copy()
    #     current_pose = self.fk(current_q)
        
    #     # Generate a series of poses along the straight line
    #     steps = []
    #     for i in range(n):
    #         # Create new transform maintaining orientation
    #         new_T = current_pose.copy()
            
    #         # Update position based on direction
    #         step_distance = distance * (i+1) / n

    #         if direction == 'vertical':
    #             new_T.t[2] += step_distance
    #         elif direction == 'forward':
    #             new_T.t[0] += step_distance
    #         elif direction == 'backward':
    #             new_T.t[0] -= step_distance
    #         elif direction == 'left':
    #             new_T.t[1] += step_distance
    #         elif direction == 'right':
    #             new_T.t[1] -= step_distance
    #         else:
    #             print("Invalid direction. Use: vertical, forward, backward, left, right")
    #             return
            
    #         # Try to find IK solution close to current configuration
    #         try:
    #             sol = self.ikine_LM(new_T, q0=current_q)
    #             if sol.success and not np.any(np.isnan(sol.q)):
    #                 steps.append(sol.q)
    #                 # Update current_q to use as next initial guess
    #                 current_q = sol.q
    #             else:
    #                 print(f"Warning: Invalid solution at step {i+1}")
    #                 continue
    #         except Exception as e:
    #             print(f"Error at step {i+1}: {e}")
    #             continue
        
    #     if len(steps) == 0:
    #         print("No valid solutions found")
    #         return
        
    #     if plan:
    #         self.plan.extend(steps)
    #     else:
    #         dt = t/len(steps)
    #         for step in steps:
    #             os.system(f'ros2 run chess_manipulator multi_point_controller 1 {int(dt*1e9)} {self.string_q(step)}')
    #             self.update(step)
                
    #     return True
    def move_straight(self, direction, distance, t, n=5, plan=False):
        """
        Move in a straight line while maintaining orientation
        """
        # print(f"Starting from: {self.fk(self.q).t}")
        
        steps = []
        current_T = self.fk(self.q)
        
        for i in range(n):
            new_T = current_T.copy()
            
            # Update position based on direction
            if direction == 'vertical':
                new_T.t[2] += (distance * (i + 1) / n)
            elif direction == 'forward':
                new_T.t[0] += (distance * (i + 1) / n)
            elif direction == 'side':
                new_T.t[1] += (distance * (i + 1) / n)
                
            # Get IK solution
            sol = self.ikine_LM(new_T, q0=self.q)
            if sol.success:
                steps.append(sol.q)
                # print(f"Step {i+1} target: {self.fk(sol.q).t}")
            else:
                print(f"Failed at step {i+1}")
                return
        
        if plan:
            self.plan.extend(steps)
            # Update internal state to reflect the planned final position
            self.q = steps[-1]
            self.p = self.fk_pos(self.q)
        else:
            dt = t/n
            for step in steps:
                os.system(f'ros2 run chess_manipulator multi_point_controller 1 {int(dt*1e9)} {self.string_q(step)}')
                self.update(step)
        
        # print(f"Final position: {self.fk(self.q).t}")


            
    def execute_plan(self,t,erase = True):
        qs = self.plan
        n= len(qs)
        dt = t/n
        os.system(f'ros2 run chess_manipulator multi_point_controller {n} {int(dt*1e9)} {self.string_qs(qs)}')
        self.update(qs[-1])
        if(erase):
            self.plan = []
        
            
    
    def move_to_pos_1step(self, p, t, plan = False):
        T = SE3.Trans(p)* SE3.Rx(np.pi)
        q = self.ik(T)
        
        if(plan):
            self.plan.extend([q])
        else:
            os.system(f'ros2 run chess_manipulator multi_point_controller 1 {int(t*1e9)} {self.string_q(q)}')
        self.update(q)
        
        
        
    def set_q(self, q, t):
        self.update(q)
        os.system(f'ros2 run chess_manipulator multi_point_controller 1 {int(t*1e9)} {self.string_q(q)} ')
    
    def update(self,q):
        self.q = q
        self.p = self.fk_pos(q)
        
    def reset(self):
        self.set_q(self.q0,1)
    
    def move_single_joint(self, joint_index, angle, t, plan=False):
        """
        Move a single joint to a specific angle
        joint_index: 0-6 (for 7 DOF Panda)
        angle: target angle in radians
        t: time in seconds
        """
        new_q = self.q.copy()  # Copy current joint positions
        new_q[joint_index] = angle
        
        if(plan):
            self.plan.extend([new_q])
        else:
            os.system(f'ros2 run chess_manipulator multi_point_controller 1 {int(t*1e9)} {self.string_q(new_q)}')
        self.update(new_q)
        
            
if __name__ == '__main__':
    robot = panda_7dof()

    robot.move_to_pos([0.5,0,0.3],3)

