"""
Standalone EKF Sensor Fusion Simulator for GPS-Denied Navigation
================================================================
Models an autonomous Unmanned Ground Vehicle (UGV) performing a reconnaissance
patrol mission in a GPS-denied bunker/facility.

Sensors Simulated:
  1. Wheel Odometry: Fast update, high systematic slip & cumulative drift.
  2. Visual Odometry (VSLAM): Accurate position, but subject to feature dropouts & occlusion.
  3. 9-DoF IMU: Fast angular velocity with white noise + random-walk bias drift.

Filter:
  Extended Kalman Filter (EKF) with multi-rate sensor updates and dynamic covariance.

Output:
  - Trajectory comparison plot saved to 'results/ekf_trajectory_benchmark.png'
  - Absolute Trajectory Error (ATE) & Root Mean Square Error (RMSE) quantification.
"""

import os
import math
import numpy as np
import matplotlib.pyplot as plt

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle

class ExtendedKalmanFilter2D:
    """
    2D Non-linear EKF for mobile robot state estimation.
    State vector: x = [px, py, theta, v, omega]^T
      px: x position (m)
      py: y position (m)
      theta: heading angle (rad)
      v: linear velocity (m/s)
      omega: angular velocity (rad/s)
    """
    def __init__(self, init_x=0.0, init_y=0.0, init_theta=0.0):
        # Initial state
        self.x = np.array([init_x, init_y, init_theta, 0.0, 0.0], dtype=float)

        # State covariance matrix P
        self.P = np.diag([0.05, 0.05, 0.01, 0.1, 0.05])

        # Process noise covariance Q
        self.Q = np.diag([
            0.01,   # px process noise
            0.01,   # py process noise
            0.005,  # theta process noise
            0.08,   # v acceleration noise
            0.05    # omega acceleration noise
        ])

        # Measurement noise covariances
        # Wheel odometry: measures [v, omega]
        self.R_wheel = np.diag([0.15**2, 0.10**2])

        # IMU: measures [omega]
        self.R_imu = np.array([[0.03**2]])

        # Visual Odometry (VSLAM): measures [px, py]
        self.R_vo = np.diag([0.08**2, 0.08**2])

    def predict(self, dt):
        """Kinematic prediction step."""
        px, py, theta, v, omega = self.x

        # State transition function f(x)
        px_new = px + v * np.cos(theta) * dt
        py_new = py + v * np.sin(theta) * dt
        theta_new = normalize_angle(theta + omega * dt)
        v_new = v
        omega_new = omega

        self.x = np.array([px_new, py_new, theta_new, v_new, omega_new])

        # Jacobian matrix F = df/dx
        F = np.eye(5)
        F[0, 2] = -v * np.sin(theta) * dt
        F[0, 3] = np.cos(theta) * dt
        F[1, 2] = v * np.cos(theta) * dt
        F[1, 3] = np.sin(theta) * dt
        F[2, 4] = dt

        # Update covariance P = F * P * F^T + Q
        self.P = F @ self.P @ F.T + self.Q

    def update_wheel(self, z_wheel):
        """
        Measurement update from wheel encoders.
        z_wheel = [v_meas, omega_meas]
        """
        # Measurement matrix H (measures state indices 3 and 4)
        H = np.zeros((2, 5))
        H[0, 3] = 1.0
        H[1, 4] = 1.0

        z_pred = H @ self.x
        y = z_wheel - z_pred  # Innovation residual

        S = H @ self.P @ H.T + self.R_wheel
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.x[2] = normalize_angle(self.x[2])
        self.P = (np.eye(5) - K @ H) @ self.P

    def update_imu(self, z_imu_omega):
        """
        Measurement update from IMU gyroscope.
        z_imu_omega = [omega_meas]
        """
        H = np.zeros((1, 5))
        H[0, 4] = 1.0

        z_pred = H @ self.x
        y = np.array([z_imu_omega]) - z_pred

        S = H @ self.P @ H.T + self.R_imu
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + (K @ y).flatten()
        self.x[2] = normalize_angle(self.x[2])
        self.P = (np.eye(5) - K @ H) @ self.P

    def update_visual_odometry(self, z_vo):
        """
        Measurement update from Visual Odometry / VSLAM.
        z_vo = [px_meas, py_meas]
        """
        H = np.zeros((2, 5))
        H[0, 0] = 1.0
        H[1, 1] = 1.0

        z_pred = H @ self.x
        y = z_vo - z_pred

        S = H @ self.P @ H.T + self.R_vo
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.x[2] = normalize_angle(self.x[2])
        self.P = (np.eye(5) - K @ H) @ self.P


def run_simulation():
    print("=" * 65)
    print("  AUTONOMOUS UGV GPS-DENIED EKF SENSOR FUSION SIMULATION")
    print("=" * 65)

    np.random.seed(42)
    dt = 0.05               # 20 Hz base loop
    total_time = 60.0       # 60 seconds mission
    time_steps = int(total_time / dt)

    # Waypoints for patrol mission (x, y)
    waypoints = [
        (0.0, 0.0),
        (15.0, 0.0),
        (25.0, 10.0),
        (25.0, 25.0),
        (10.0, 30.0),
        (0.0, 15.0),
        (0.0, 0.0)
    ]

    # Initialize Ground Truth State: [x, y, theta, v, omega]
    gt_x = 0.0
    gt_y = 0.0
    gt_theta = 0.0
    gt_v = 1.0
    wp_idx = 1

    # Raw Unfused Dead-Reckoning accumulator (wheel odom only)
    dr_x, dr_y, dr_theta = 0.0, 0.0, 0.0

    # Initialize EKF
    ekf = ExtendedKalmanFilter2D(init_x=0.0, init_y=0.0, init_theta=0.0)

    # Storage for analysis
    hist_time = []
    hist_gt = []
    hist_dr = []
    hist_ekf = []
    hist_vo = []

    # Sensor noise parameters
    imu_bias = 0.015  # drifting gyro bias (rad/s)
    wheel_slip_factor = 0.94  # 6% systematic wheel slip

    print(f"[*] Simulating patrol mission over {total_time}s across {len(waypoints)} checkpoints...")

    for step in range(time_steps):
        t = step * dt
        hist_time.append(t)

        # -------------------------------------------------------------
        # 1. Ground Truth Robot Motion (Pure Pursuit towards waypoint)
        # -------------------------------------------------------------
        target_wp = waypoints[wp_idx]
        dx = target_wp[0] - gt_x
        dy = target_wp[1] - gt_y
        dist = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_err = normalize_angle(target_angle - gt_theta)

        # Reached waypoint -> advance to next
        if dist < 1.0 and wp_idx < len(waypoints) - 1:
            wp_idx += 1

        # Proportional angular controller
        gt_omega = np.clip(1.5 * angle_err, -0.8, 0.8)
        gt_v = 1.2 if dist > 1.0 else 0.5

        # Update Ground Truth
        gt_theta = normalize_angle(gt_theta + gt_omega * dt)
        gt_x += gt_v * np.cos(gt_theta) * dt
        gt_y += gt_v * np.sin(gt_theta) * dt
        hist_gt.append([gt_x, gt_y, gt_theta])

        # -------------------------------------------------------------
        # 2. Simulate Sensor Measurements
        # -------------------------------------------------------------
        # A. Wheel Odometry (affected by slip and encoder noise)
        meas_v = gt_v * wheel_slip_factor + np.random.normal(0, 0.08)
        meas_omega_wheel = gt_omega + np.random.normal(0, 0.05)

        # Accumulate Unfused Dead Reckoning (to demonstrate failure of naive odometry)
        dr_theta = normalize_angle(dr_theta + meas_omega_wheel * dt)
        dr_x += meas_v * np.cos(dr_theta) * dt
        dr_y += meas_v * np.sin(dr_theta) * dt
        hist_dr.append([dr_x, dr_y, dr_theta])

        # B. 9-DoF IMU (100 Hz capability, sampled here at 20 Hz)
        imu_bias += np.random.normal(0, 0.0002)  # slow bias drift
        meas_gyro = gt_omega + imu_bias + np.random.normal(0, 0.02)

        # C. Visual Odometry (VSLAM - 10 Hz, feature dropout between t=20s and t=32s)
        vo_meas = None
        feature_dropout = (20.0 <= t <= 32.0)  # e.g., dark hallway / optical glare
        if not feature_dropout and (step % 2 == 0):  # 10 Hz rate
            vo_x = gt_x + np.random.normal(0, 0.06)
            vo_y = gt_y + np.random.normal(0, 0.06)
            vo_meas = np.array([vo_x, vo_y])
            hist_vo.append([t, vo_x, vo_y])

        # -------------------------------------------------------------
        # 3. EKF Execution: Prediction + Multi-Sensor Update
        # -------------------------------------------------------------
        ekf.predict(dt)
        ekf.update_wheel(np.array([meas_v, meas_omega_wheel]))
        ekf.update_imu(meas_gyro)

        if vo_meas is not None:
            ekf.update_visual_odometry(vo_meas)

        hist_ekf.append([ekf.x[0], ekf.x[1], ekf.x[2]])

    # Convert to arrays
    hist_gt = np.array(hist_gt)
    hist_dr = np.array(hist_dr)
    hist_ekf = np.array(hist_ekf)
    hist_vo = np.array(hist_vo)

    # -------------------------------------------------------------
    # 4. Error Metrics & Quantitative Evaluation (ATE RMSE)
    # -------------------------------------------------------------
    dr_pos_err = np.linalg.norm(hist_dr[:, :2] - hist_gt[:, :2], axis=1)
    ekf_pos_err = np.linalg.norm(hist_ekf[:, :2] - hist_gt[:, :2], axis=1)

    dr_rmse = np.sqrt(np.mean(dr_pos_err**2))
    ekf_rmse = np.sqrt(np.mean(ekf_pos_err**2))
    drift_reduction = (1.0 - (ekf_rmse / dr_rmse)) * 100.0

    print("-" * 65)
    print("BENCHMARK EVALUATION RESULTS (GPS-Denied Environment):")
    print(f"  > Raw Dead-Reckoning Drift (RMSE) : {dr_rmse:.3f} m")
    print(f"  > Max Dead-Reckoning Position Error: {np.max(dr_pos_err):.3f} m")
    print(f"  > Fused EKF Estimation (RMSE)      : {ekf_rmse:.3f} m")
    print(f"  > Max EKF Position Error          : {np.max(ekf_pos_err):.3f} m")
    print(f"  > Cumulative Drift Reduction      : {drift_reduction:.1f}% IMPROVEMENT")
    print("-" * 65)

    # -------------------------------------------------------------
    # 5. Visualization Generation
    # -------------------------------------------------------------
    os.makedirs("results", exist_ok=True)
    plot_path = os.path.join("results", "ekf_trajectory_benchmark.png")

    fig = plt.figure(figsize=(14, 10))

    # Subplot 1: 2D Spatial Trajectory
    ax1 = fig.add_subplot(2, 2, (1, 3))
    ax1.plot(hist_gt[:, 0], hist_gt[:, 1], 'k-', linewidth=2.5, label='Ground Truth Trajectory')
    ax1.plot(hist_dr[:, 0], hist_dr[:, 1], 'r--', linewidth=1.5, label='Raw Wheel Odometry (High Slip Drift)')
    if len(hist_vo) > 0:
        ax1.scatter(hist_vo[::3, 1], hist_vo[::3, 2], c='magenta', s=8, alpha=0.4, label='Visual Odometry (VSLAM Points)')
    ax1.plot(hist_ekf[:, 0], hist_ekf[:, 1], 'b-', linewidth=2.0, label='EKF Sensor Fusion (Proposed)')

    # Mark Waypoints
    wp_arr = np.array(waypoints)
    ax1.scatter(wp_arr[:, 0], wp_arr[:, 1], c='green', marker='X', s=120, zorder=5, label='Mission Waypoints')
    for i, (wx, wy) in enumerate(waypoints):
        ax1.annotate(f"WP{i}", (wx + 0.5, wy + 0.5), fontsize=9, fontweight='bold')

    # Highlight feature dropout region
    ax1.set_title("UGV 2D Trajectory: GPS-Denied Reconnaissance Sweep", fontsize=12, fontweight='bold')
    ax1.set_xlabel("X Position (meters)", fontsize=11)
    ax1.set_ylabel("Y Position (meters)", fontsize=11)
    ax1.grid(True, linestyle=':', alpha=0.7)
    ax1.legend(loc='lower left', fontsize=9)
    ax1.axis('equal')

    # Subplot 2: Position Error Over Time
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(hist_time, dr_pos_err, 'r--', label=f'Raw Wheel Odom (RMSE: {dr_rmse:.2f}m)')
    ax2.plot(hist_time, ekf_pos_err, 'b-', label=f'EKF Fused (RMSE: {ekf_rmse:.2f}m)')
    ax2.axvspan(20, 32, color='orange', alpha=0.2, label='VSLAM Feature Dropout Zone (12s)')
    ax2.set_title("Euclidean Position Error vs. Mission Time", fontsize=11, fontweight='bold')
    ax2.set_xlabel("Time (seconds)")
    ax2.set_ylabel("Error (meters)")
    ax2.grid(True, linestyle=':', alpha=0.7)
    ax2.legend(fontsize=8)

    # Subplot 3: Heading (Yaw) Error
    ax3 = fig.add_subplot(2, 2, 4)
    dr_yaw_err = [abs(normalize_angle(d - g)) * (180.0 / np.pi) for d, g in zip(hist_dr[:, 2], hist_gt[:, 2])]
    ekf_yaw_err = [abs(normalize_angle(e - g)) * (180.0 / np.pi) for e, g in zip(hist_ekf[:, 2], hist_gt[:, 2])]
    ax3.plot(hist_time, dr_yaw_err, 'r--', label='Raw Wheel Heading Error')
    ax3.plot(hist_time, ekf_yaw_err, 'b-', label='EKF Heading Error (IMU Aided)')
    ax3.axvspan(20, 32, color='orange', alpha=0.2)
    ax3.set_title("Heading (Yaw) Error vs. Mission Time", fontsize=11, fontweight='bold')
    ax3.set_xlabel("Time (seconds)")
    ax3.set_ylabel("Yaw Error (degrees)")
    ax3.grid(True, linestyle=':', alpha=0.7)
    ax3.legend(fontsize=8)

    plt.tight_layout()
    plt.savefig(plot_path, dpi=300)
    print(f"[+] Benchmark plot successfully generated and saved to: {plot_path}")
    print("=" * 65)

if __name__ == '__main__':
    run_simulation()
