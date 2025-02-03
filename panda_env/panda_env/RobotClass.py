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
            os.system(f'ros2 run panda_env multi_point_controller {int(n)} {int(dt*1e9)} {self.string_qs(qs)}')
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
                os.system(f'ros2 run panda_env multi_point_controller 1 {int(dt*1e9)} {self.string_q(step)}')
                self.update(step)
        
        # print(f"Final position: {self.fk(self.q).t}")


            
    def execute_plan(self,t,erase = True):
        qs = self.plan
        n= len(qs)
        dt = t/n
        os.system(f'ros2 run panda_env multi_point_controller {n} {int(dt*1e9)} {self.string_qs(qs)}')
        self.update(qs[-1])
        if(erase):
            self.plan = []
        
            
    
    def move_to_pos_1step(self, p, t, plan = False):
        T = SE3.Trans(p)* SE3.Rx(np.pi)
        q = self.ik(T)
        
        if(plan):
            self.plan.extend([q])
        else:
            os.system(f'ros2 run panda_env multi_point_controller 1 {int(t*1e9)} {self.string_q(q)}')
        self.update(q)
        
        
        
    def set_q(self, q, t):
        self.update(q)
        os.system(f'ros2 run panda_env multi_point_controller 1 {int(t*1e9)} {self.string_q(q)} ')
    
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
            os.system(f'ros2 run panda_env multi_point_controller 1 {int(t*1e9)} {self.string_q(new_q)}')
        self.update(new_q)
        
            
if __name__ == '__main__':
    robot = panda_7dof()

    robot.move_to_pos([0.5,0,0.3],3)

