#!/usr/bin/env python3
"""
Trajectory Evaluation & Metric Logger
=====================================
Logs ground truth, raw dead-reckoning, and EKF-fused states to CSV
to quantify Absolute Trajectory Error (ATE) and Drift Reduction %.
"""

import os
import csv
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

class TrajectoryLogger(Node):
    def __init__(self):
        super().__init__('trajectory_logger')

        self.get_logger().info('Initializing Trajectory Benchmark Logger...')

        os.makedirs('evaluation', exist_ok=True)
        self.log_file_path = os.path.join('evaluation', 'trajectory_log.csv')
        self.csv_file = open(self.log_file_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'timestamp',
            'gt_x', 'gt_y', 'gt_yaw',
            'wheel_x', 'wheel_y', 'wheel_yaw',
            'ekf_x', 'ekf_y', 'ekf_yaw'
        ])

        self.wheel_pose = [0.0, 0.0, 0.0]
        self.ekf_pose = [0.0, 0.0, 0.0]
        self.gt_pose = [0.0, 0.0, 0.0]

        # Subscribers
        self.sub_wheel = self.create_subscription(
            Odometry, '/wheel/odom', self.wheel_callback, 10
        )
        self.sub_ekf = self.create_subscription(
            Odometry, '/odometry/filtered', self.ekf_callback, 10
        )

        # Logging periodic timer (10 Hz)
        self.timer = self.create_timer(0.1, self.log_sample)
        self.get_logger().info(f'Logging metrics to: {self.log_file_path}')

    def wheel_callback(self, msg: Odometry):
        self.wheel_pose[0] = msg.pose.pose.position.x
        self.wheel_pose[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.wheel_pose[2] = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def ekf_callback(self, msg: Odometry):
        self.ekf_pose[0] = msg.pose.pose.position.x
        self.ekf_pose[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.ekf_pose[2] = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def log_sample(self):
        t = self.get_clock().now().nanoseconds / 1e9
        self.writer.writerow([
            f"{t:.3f}",
            f"{self.gt_pose[0]:.4f}", f"{self.gt_pose[1]:.4f}", f"{self.gt_pose[2]:.4f}",
            f"{self.wheel_pose[0]:.4f}", f"{self.wheel_pose[1]:.4f}", f"{self.wheel_pose[2]:.4f}",
            f"{self.ekf_pose[0]:.4f}", f"{self.ekf_pose[1]:.4f}", f"{self.ekf_pose[2]:.4f}"
        ])
        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
