#!/usr/bin/env python3
from spatialmath import SE3
from roboticstoolbox.models.DH import UR10
from roboticstoolbox import DHRobot
from roboticstoolbox.tools import ctraj,jtraj
# from .path_planner import UR10PathPlanner # use it to optimize the path in the future work

import numpy as np
import os

class ur10_6dof(UR10):
    def __init__(self,q0 = None):
        super().__init__()
        
        if(q0 == None):
            self.q0 = [0, -1.57, 0, -1.57, -1.57, np.pi]
        else:
            self.q0 = q0
        self.q = self.q0
        self.p = self.fk_pos(self.q)
        
        self.plan = []
        self.set_q(self.q,1)
    
    def ik(self, T, q0=None):
        if q0 == None:  # Very important, our result sometimes will crash is bcs of we didn't designate qo for ik(), 
            #so we give it current self.q for initial guess, makes it stable and continuous
            q0 = self.q
        sol = self.ikine_LM(T, q0=q0, joint_limits=True)
        if sol.success:
            return sol.q
        else:
            raise ValueError("IK solution failed for the given transformation.")

    
    def fk(self, q):
        return self.fkine(q)
    
    def fk_pos(self,q):
        return self.fkine(q).t
    
    def traj(self, p1, p2, n):
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
            os.system(f'ros2 run ur_robot_env multi_point_controller {int(n)} {int(dt*1e9)} {self.string_qs(qs)}')
        self.update(qs[-1])
        self.p = p
    def set_q(self, q, t):
        self.update(q)
        os.system(f'ros2 run ur_robot_env multi_point_controller 1 {int(t*1e9)} {self.string_q(q)} ')

    def update(self,q):
        self.q = q
        self.p = self.fk_pos(q)

    def reset(self):
        self.set_q(self.q0,1)

    def move_to_pos_1step(self, p, t, plan = False):
        T = SE3.Trans(p)* SE3.Rx(np.pi)
        q = self.ik(T)
        
        if(plan):
            self.plan.extend([q])
        else:
            os.system(f'ros2 run ur_robot_env multi_point_controller 1 {int(t*1e9)} {self.string_q(q)}')
        self.update(q)

    def execute_plan(self,t,erase = True):
        qs = self.plan
        n = len(qs)
        dt = t/n
        os.system(f'ros2 run ur_robot_env multi_point_controller {n} {int(dt*1e9)} {self.string_qs(qs)}')
        self.update(qs[-1])
        if(erase):
            self.plan = []

    def move_straight(self, direction, distance, t, n=5, plan=False):
        """
        Move the robot in a straight line in the specified direction while maintaining its orientation.
        
        Parameters:
            direction (str): 'vertical', 'forward', or 'side'
            distance (float): The distance to move in the specified direction.
            t (float): Total time for the movement.
            n (int): Number of interpolation steps.
            plan (bool): If True, append the movement steps to the plan; otherwise, execute immediately.
        """
        steps = []
        current_T = self.fk(self.q)
        
        for i in range(n):
            new_T = current_T.copy()
            
            if direction == 'vertical':
                new_T.t[2] += (distance * (i + 1) / n)
            elif direction == 'forward':
                new_T.t[0] += (distance * (i + 1) / n)
            elif direction == 'side':
                new_T.t[1] += (distance * (i + 1) / n)
                
            sol = self.ikine_LM(new_T, q0=self.q)
            if sol.success:
                steps.append(sol.q)
            else:
                print(f"Failed at step {i+1}")
                return
        
        if plan:
            self.plan.extend(steps)
            self.q = steps[-1]
            self.p = self.fk_pos(self.q)
        else:
            dt = t/n
            for step in steps:
                os.system(f'ros2 run ur_robot_env multi_point_controller 1 {int(dt*1e9)} {self.string_q(step)}')
                self.update(step)







