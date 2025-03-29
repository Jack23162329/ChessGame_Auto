#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.duration import Duration
import sys

class JointControlClient(Node):
    def __init__(self):
        super().__init__(node_name='joint_controller')
        self._action_client = ActionClient(
            node=self, 
            action_type=FollowJointTrajectory,
            action_name='/joint_trajectory_controller/follow_joint_trajectory'
        )

    def send_goal(self, n, qs, dt_ns):
        goal_msg = FollowJointTrajectory.Goal()

        # Changed to UR10 joint names
        joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        points = []
        for i in range(n):
            q = qs[6*i:6*(i+1)]  # Changed from 7 to 6 joints
            point = JointTrajectoryPoint()
            point.time_from_start = Duration(nanoseconds=int(dt_ns*(i+1))).to_msg()
            point.positions = q
            points.append(point)

        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: ' + str(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

def main(args=None):
    rclpy.init()

    action_client = JointControlClient()

    n = int(sys.argv[1])
    dt = int(sys.argv[2])
    qs = []
    for i in range(n*6):  # Changed from 7 to 6 for UR10
        qs.append(float(sys.argv[i+3]))
        
    future = action_client.send_goal(n, qs, dt)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()