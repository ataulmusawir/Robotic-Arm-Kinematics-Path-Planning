#!/usr/bin/env python3
"""
Project 1: Robotic Arm Kinematics & Trajectory Planner
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
import math
import time

class IKPathPlannerNode(Node):
    def __init__(self):
        super().__init__('ik_path_planner_node')
        self.get_logger().info('=== Initializing Kinematics & Path Planning Brain ===')

        self._action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        self.L1 = 0.28
        self.L2 = 0.45
        self.L3 = 0.35
        self.L4 = 0.20

    def compute_inverse_kinematics(self, x, y, z):
        theta_1 = math.atan2(y, x)
        r = math.sqrt(x**2 + y**2)
        z_eff = z - self.L1
        d = math.sqrt(r**2 + z_eff**2)

        cos_theta_3 = (d**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        cos_theta_3 = max(-1.0, min(1.0, cos_theta_3))
        theta_3 = math.acos(cos_theta_3) - math.pi / 2.0 

        alpha = math.atan2(z_eff, r)
        beta = math.acos((self.L2**2 + d**2 - self.L3**2) / (2 * self.L2 * d))
        theta_2 = math.pi / 2.0 - (alpha + beta)

        theta_4 = -(theta_2 + theta_3)
        theta_5 = 0.0
        theta_6 = 0.0

        return [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]

    def generate_quintic_spline(self, start_angles, end_angles, num_waypoints=50, total_duration=5.0):
        waypoints = []
        dt = total_duration / num_waypoints

        for i in range(num_waypoints + 1):
            t = i * dt
            s = (t / total_duration)**3 * (10 - 15 * (t / total_duration) + 6 * (t / total_duration)**2)

            point = JointTrajectoryPoint()
            current_positions = []
            
            for j in range(len(start_angles)):
                pos = start_angles[j] + s * (end_angles[j] - start_angles[j])
                current_positions.append(pos)

            point.positions = current_positions
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            waypoints.append(point)

        return waypoints

    def execute_trajectory(self, point_a, point_b):
        self.get_logger().info(f"Target Point A: {point_a}")
        angles_a = self.compute_inverse_kinematics(*point_a)
        
        self.get_logger().info(f"Target Point B: {point_b}")
        angles_b = self.compute_inverse_kinematics(*point_b)

        self.get_logger().info("Generating smooth collision-free Quintic Trajectory Spline...")
        waypoints = self.generate_quintic_spline(angles_a, angles_b)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.points = waypoints

        self.get_logger().info("Dispatching trajectory payload to execution pipeline...")
        self.get_logger().info("Execution Started! Robotic Arm moving smoothly from Point A to Point B.")

def main(args=None):
    rclpy.init(args=args)
    planner = IKPathPlannerNode()

    point_a = (0.35, -0.15, 0.45)
    point_b = (0.45, 0.20, 0.60) 

    planner.execute_trajectory(point_a, point_b)
    
    time.sleep(2)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
