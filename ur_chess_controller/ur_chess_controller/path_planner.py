import numpy as np
from scipy.optimize import minimize
from typing import List, Tuple, Optional

class UR10PathPlanner:
    def __init__(self):
        # UR10 joint limits (in radians)
        self.joint_limits = {
            'lower': np.array([-2*np.pi, -2*np.pi, -2*np.pi, -2*np.pi, -2*np.pi, -2*np.pi]),
            'upper': np.array([2*np.pi, 2*np.pi, 2*np.pi, 2*np.pi, 2*np.pi, 2*np.pi])
        }
        
        # IK parameters
        self.max_iterations = 100
        self.damping = 0.5
        self.tolerance = 1e-4
        
        # Path planning parameters
        self.interpolation_points = 50
        self.singularity_threshold = 0.1

    def compute_manipulability(self, J: np.ndarray) -> float:
        """
        Compute manipulability measure to avoid singularities
        
        Args:
            J: Jacobian matrix
            
        Returns:
            float: Manipulability measure
        """
        return np.sqrt(np.linalg.det(J @ J.T))

    def ik_cost_function(self, q: np.ndarray, T_target: np.ndarray, q_current: np.ndarray) -> float:
        """
        Cost function for IK optimization
        
        Args:
            q: Joint angles to evaluate
            T_target: Target transformation matrix
            q_current: Current joint angles
            
        Returns:
            float: Cost value
        """
        # Forward kinematics to get current pose
        T_current = self.fk(q)  # You'll need to implement fk method
        
        # Position and orientation error
        pos_error = np.linalg.norm(T_current[:3, 3] - T_target[:3, 3])
        rot_error = np.linalg.norm(T_current[:3, :3] - T_target[:3, :3], 'fro')
        
        # Joint limit avoidance
        limit_error = np.sum(np.maximum(0, q - self.joint_limits['upper'])**2 + 
                           np.maximum(0, self.joint_limits['lower'] - q)**2)
        
        # Joint movement minimization
        movement = np.linalg.norm(q - q_current)
        
        # Weights for different components
        w_pos = 1.0
        w_rot = 0.5
        w_limit = 0.1
        w_movement = 0.05
        
        return (w_pos * pos_error + 
                w_rot * rot_error + 
                w_limit * limit_error + 
                w_movement * movement)

    def ik_with_optimization(self, T_target: np.ndarray, q_init: np.ndarray) -> np.ndarray:
        """
        Solve IK using optimization with multiple constraints
        
        Args:
            T_target: Target transformation matrix
            q_init: Initial joint angles
            
        Returns:
            np.ndarray: Optimized joint angles
        """
        bounds = list(zip(self.joint_limits['lower'], self.joint_limits['upper']))
        
        result = minimize(
            self.ik_cost_function,
            q_init,
            args=(T_target, q_init),
            method='SLSQP',
            bounds=bounds,
            options={'maxiter': self.max_iterations, 'ftol': self.tolerance}
        )
        
        if not result.success:
            print(f"IK optimization warning: {result.message}")
            
        return result.x

    def check_singularity(self, J: np.ndarray) -> bool:
        """
        Check if configuration is near singularity
        
        Args:
            J: Jacobian matrix
            
        Returns:
            bool: True if near singularity
        """
        manipulability = self.compute_manipulability(J)
        return manipulability < self.singularity_threshold

    def generate_cartesian_path(self, start_pose: np.ndarray, 
                              end_pose: np.ndarray) -> List[np.ndarray]:
        """
        Generate Cartesian path between two poses
        
        Args:
            start_pose: Starting transformation matrix
            end_pose: Ending transformation matrix
            
        Returns:
            List[np.ndarray]: List of interpolated poses
        """
        path = []
        for s in np.linspace(0, 1, self.interpolation_points):
            # Interpolate position
            pos = (1-s) * start_pose[:3, 3] + s * end_pose[:3, 3]
            
            # Interpolate rotation using SLERP
            R1 = start_pose[:3, :3]
            R2 = end_pose[:3, :3]
            R = self.slerp(R1, R2, s)
            
            # Combine into transformation matrix
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = pos
            
            path.append(T)
            
        return path

    def slerp(self, R1: np.ndarray, R2: np.ndarray, t: float) -> np.ndarray:
        """
        Spherical Linear Interpolation for rotation matrices
        
        Args:
            R1: Start rotation matrix
            R2: End rotation matrix
            t: Interpolation parameter (0 to 1)
            
        Returns:
            np.ndarray: Interpolated rotation matrix
        """
        R = R1 @ np.linalg.inv(R2)
        theta = np.arccos((np.trace(R) - 1) / 2)
        
        if abs(theta) < 1e-6:
            return R1
            
        return R1 @ (np.linalg.matrix_power(R, t))

    def plan_path(self, T_target: np.ndarray, q_current: np.ndarray) -> Optional[np.ndarray]:
        """
        Main path planning function with singularity avoidance
        
        Args:
            T_target: Target transformation matrix
            q_current: Current joint angles
            
        Returns:
            Optional[np.ndarray]: Planned joint angles if successful, None if failed
        """
        try:
            # Get current pose
            T_current = self.fk(q_current)
            
            # Generate Cartesian path
            path = self.generate_cartesian_path(T_current, T_target)
            
            # Initialize joint path
            joint_path = [q_current]
            
            # Follow path with IK
            for T in path:
                # Use previous solution as initial guess
                q_init = joint_path[-1]
                
                # Compute IK for this waypoint
                q_new = self.ik_with_optimization(T, q_init)
                
                # Check joint limits and singularities
                J = self.compute_jacobian(q_new)  # You'll need to implement this
                if self.check_singularity(J):
                    print("Warning: Near singularity, attempting to find alternative solution")
                    # Try alternative configuration
                    q_alt = self.find_alternative_config(T, q_init)
                    if q_alt is not None:
                        q_new = q_alt
                
                joint_path.append(q_new)
            
            return np.array(joint_path)
            
        except Exception as e:
            print(f"Path planning failed: {str(e)}")
            return None

    def find_alternative_config(self, T: np.ndarray, q_init: np.ndarray) -> Optional[np.ndarray]:
        """
        Find alternative configuration to avoid singularity
        
        Args:
            T: Target transformation matrix
            q_init: Initial joint angles
            
        Returns:
            Optional[np.ndarray]: Alternative configuration if found
        """
        # Try multiple initial configurations
        q_attempts = [
            q_init + np.random.uniform(-0.5, 0.5, size=6) 
            for _ in range(5)
        ]
        
        best_q = None
        best_manipulability = -float('inf')
        
        for q_try in q_attempts:
            q_sol = self.ik_with_optimization(T, q_try)
            J = self.compute_jacobian(q_sol)
            manipulability = self.compute_manipulability(J)
            
            if manipulability > best_manipulability:
                best_manipulability = manipulability
                best_q = q_sol
                
        return best_q

    def ik(self, T: np.ndarray, q_current: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Enhanced IK solver with path planning
        
        Args:
            T: Target transformation matrix
            q_current: Current joint angles (optional)
            
        Returns:
            np.ndarray: Solved joint angles
        """
        if q_current is None:
            # If no current configuration provided, use middle of joint ranges
            q_current = (self.joint_limits['upper'] + self.joint_limits['lower']) / 2
            
        # Plan full path
        joint_path = self.plan_path(T, q_current)
        
        if joint_path is None:
            raise RuntimeError("Failed to find valid IK solution")
            
        # Return final configuration
        return joint_path[-1]