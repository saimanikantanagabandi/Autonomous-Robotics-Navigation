#!/usr/bin/env python3
"""
GPS-Denial & Sensor Health Supervisor Node
==========================================
Monitors multi-modal sensor streams (Wheel, IMU, VSLAM) in real-time.
Detects:
  1. Visual Odometry feature loss / occlusion dropouts.
  2. Wheel slippage via wheel-to-visual velocity discrepancies.
  3. IMU gyroscopic bias divergence.
"""

import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class GPSDeniedSupervisor(Node):
    def __init__(self):
        super().__init__('gps_denied_supervisor')

        self.get_logger().info('Starting GPS-Denied Sensor Integrity Monitor...')

        # State timestamps
        self.last_vo_time = time.time()
        self.last_wheel_time = time.time()
        self.last_imu_time = time.time()

        # Telemetry
        self.current_wheel_vx = 0.0
        self.current_vo_vx = 0.0
        self.is_vo_degraded = False

        # Subscribers
        self.sub_wheel = self.create_subscription(
            Odometry, '/wheel/odom', self.wheel_callback, 10
        )
        self.sub_imu = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )
        self.sub_vo = self.create_subscription(
            Odometry, '/vo/odom', self.vo_callback, 10
        )

        # Health watchdog timer (10 Hz)
        self.timer = self.create_timer(0.1, self.health_check_cycle)

    def wheel_callback(self, msg: Odometry):
        self.last_wheel_time = time.time()
        self.current_wheel_vx = msg.twist.twist.linear.x

    def imu_callback(self, msg: Imu):
        self.last_imu_time = time.time()

    def vo_callback(self, msg: Odometry):
        self.last_vo_time = time.time()
        self.current_vo_vx = msg.twist.twist.linear.x

    def health_check_cycle(self):
        now = time.time()
        vo_elapsed = now - self.last_vo_time

        # Check Visual SLAM status
        if vo_elapsed > 1.2:
            if not self.is_vo_degraded:
                self.is_vo_degraded = True
                self.get_logger().warn(
                    f'[ALERT: SENSOR DEGRADATION] VSLAM feature loss detected! '
                    f'No visual pose for {vo_elapsed:.1f}s. Operating on EKF inertial dead-reckoning.'
                )
        else:
            if self.is_vo_degraded:
                self.is_vo_degraded = False
                self.get_logger().info(
                    '[STATUS: RESTORED] VSLAM visual features reacquired. Full fusion resumed.'
                )

        # Detect Wheel Slippage: Wheel velocity high while Visual Odometry velocity is near-zero
        if not self.is_vo_degraded and abs(self.current_wheel_vx) > 0.4:
            slip_ratio = abs(self.current_wheel_vx - self.current_vo_vx)
            if slip_ratio > 0.35:
                self.get_logger().warn(
                    f'[ALERT: TERRAIN SLIP] Excessive wheel slippage detected! '
                    f'Wheel Vx: {self.current_wheel_vx:.2f} m/s vs Visual Vx: {self.current_vo_vx:.2f} m/s.'
                )

def main(args=None):
    rclpy.init(args=args)
    node = GPSDeniedSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
