"""
Autonomous UGV Simulation Engine & State Estimation
===================================================
Real-time simulation engine executing:
  - Non-linear kinematics of differential drive UGV
  - Realistic multi-sensor stream generation (Wheel, 9-DoF IMU, RGB-D VSLAM, LiDAR)
  - 5-State Extended Kalman Filter (EKF) with covariance propagation & ellipse extraction
  - Pure Pursuit / DWB local trajectory tracking
  - Fault injection (Wheel slip, visual dropout, IMU bias)
  - GPS-denied supervisor watchdog
  - Real-time ATE / RPE metrics calculation & CSV logging
"""

import math
import time
import threading
import numpy as np
from typing import Dict, Any, List, Optional
from mission_control.bunker_map import BunkerMap

def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class ExtendedKalmanFilter2D:
    def __init__(self, init_x=0.0, init_y=0.0, init_theta=0.0):
        # State vector: [px, py, theta, v, omega]
        self.x = np.array([init_x, init_y, init_theta, 0.0, 0.0], dtype=float)
        self.P = np.diag([0.05, 0.05, 0.01, 0.1, 0.05])
        self.Q = np.diag([0.01, 0.01, 0.005, 0.08, 0.05])
        self.R_wheel = np.diag([0.15**2, 0.10**2])
        self.R_imu = np.array([[0.03**2]])
        self.R_vo = np.diag([0.08**2, 0.08**2])

    def predict(self, dt: float):
        px, py, theta, v, omega = self.x
        px_new = px + v * math.cos(theta) * dt
        py_new = py + v * math.sin(theta) * dt
        theta_new = normalize_angle(theta + omega * dt)
        v_new = v
        omega_new = omega
        self.x = np.array([px_new, py_new, theta_new, v_new, omega_new])

        F = np.eye(5)
        F[0, 2] = -v * math.sin(theta) * dt
        F[0, 3] = math.cos(theta) * dt
        F[1, 2] = v * math.cos(theta) * dt
        F[1, 3] = math.sin(theta) * dt
        F[2, 4] = dt
        self.P = F @ self.P @ F.T + self.Q

    def update_wheel(self, z_wheel):
        H = np.zeros((2, 5))
        H[0, 3] = 1.0
        H[1, 4] = 1.0
        y = z_wheel - H @ self.x
        S = H @ self.P @ H.T + self.R_wheel
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.x[2] = normalize_angle(self.x[2])
        self.P = (np.eye(5) - K @ H) @ self.P

    def update_imu(self, z_omega):
        H = np.zeros((1, 5))
        H[0, 4] = 1.0
        y = np.array([z_omega]) - H @ self.x
        S = H @ self.P @ H.T + self.R_imu
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ y).flatten()
        self.x[2] = normalize_angle(self.x[2])
        self.P = (np.eye(5) - K @ H) @ self.P

    def update_vo(self, z_vo):
        H = np.zeros((2, 5))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        y = z_vo - H @ self.x
        S = H @ self.P @ H.T + self.R_vo
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.x[2] = normalize_angle(self.x[2])
        self.P = (np.eye(5) - K @ H) @ self.P

    def get_covariance_ellipse(self) -> Dict[str, float]:
        """Calculates 2-sigma position covariance uncertainty ellipse."""
        P_pos = self.P[0:2, 0:2]
        eigvals, eigvecs = np.linalg.eigh(P_pos)
        eigvals = np.maximum(eigvals, 1e-6)
        # 2-sigma radius
        r_x = float(2.0 * math.sqrt(eigvals[1]))
        r_y = float(2.0 * math.sqrt(eigvals[0]))
        angle = float(math.atan2(eigvecs[1, 1], eigvecs[0, 1]))
        return {
            "rx": round(max(0.1, min(r_x, 5.0)), 3),
            "ry": round(max(0.1, min(r_y, 5.0)), 3),
            "angle": round(angle, 3),
            "trace": round(float(np.trace(self.P)), 4)
        }

class SimulationEngine:
    def __init__(self):
        self.bunker = BunkerMap()
        self.lock = threading.Lock()

        # Mission mode: 'PATROL', 'WAYPOINT', 'MANUAL', 'IDLE'
        self.mode = "PATROL"
        self.is_paused = False

        # Ground Truth state: [x, y, theta, v, omega]
        self.gt_x = 0.0
        self.gt_y = 0.0
        self.gt_theta = 0.0
        self.gt_v = 0.0
        self.gt_omega = 0.0

        # Raw Odometry state (dead-reckoning with uncorrected drift)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.odom_v = 0.0
        self.odom_omega = 0.0

        # EKF Filter
        self.ekf = ExtendedKalmanFilter2D(self.gt_x, self.gt_y, self.gt_theta)

        # Fault Injection Flags
        self.fault_wheel_slip = False
        self.fault_vslam_blackout = False
        self.fault_imu_bias = False

        # Teleop velocity commands
        self.cmd_v = 0.0
        self.cmd_omega = 0.0

        # Patrol & Navigation Waypoints
        self.checkpoints = self.bunker.default_checkpoints
        self.current_wp_idx = 0
        self.current_goal: Optional[Dict[str, float]] = None
        self.planned_path: List[List[float]] = []

        # Supervisor Watchdog State
        self.last_vo_time = time.time()
        self.is_vslam_degraded = False
        self.is_wheel_slipping = False
        self.collision_alert = False
        self.watchdog_alerts: List[Dict[str, Any]] = []

        # Trajectory History for Real-Time Canvas Rendering
        self.gt_trail: List[List[float]] = []
        self.odom_trail: List[List[float]] = []
        self.ekf_trail: List[List[float]] = []
        self.max_trail_points = 250

        # Benchmark Statistics
        self.mission_start_time = time.time()
        self.log_records: List[Dict[str, float]] = []
        self.ate_odom_history: List[float] = []
        self.ate_ekf_history: List[float] = []

        # Sensor Telemetry Cache
        self.latest_lidar_scan = []
        self.last_imu_gyro = 0.0

        # Background Simulation Thread
        self.running = True
        self.sim_thread = threading.Thread(target=self._sim_loop, daemon=True)
        self.sim_thread.start()

        # Initialize first patrol goal
        self._set_patrol_goal(0)

    def _set_patrol_goal(self, index: int):
        if not self.checkpoints:
            return
        idx = index % len(self.checkpoints)
        self.current_wp_idx = idx
        target = self.checkpoints[idx]
        self.current_goal = {"x": target["x"], "y": target["y"], "name": target["name"]}
        # Plan path using A*
        raw_path = self.bunker.plan_path_astar((self.ekf.x[0], self.ekf.x[1]), (target["x"], target["y"]))
        self.planned_path = [[round(p[0], 2), round(p[1], 2)] for p in raw_path]

    def set_custom_goal(self, gx: float, gy: float):
        """Dispatches UGV to an interactive user-clicked destination."""
        with self.lock:
            # Clamp to bunker bounds
            gx = max(self.bunker.min_x + 1.0, min(self.bunker.max_x - 1.0, gx))
            gy = max(self.bunker.min_y + 1.0, min(self.bunker.max_y - 1.0, gy))
            self.mode = "WAYPOINT"
            self.current_goal = {"x": gx, "y": gy, "name": f"Tactical Target ({gx:.1f}, {gy:.1f})"}
            raw_path = self.bunker.plan_path_astar((self.ekf.x[0], self.ekf.x[1]), (gx, gy))
            self.planned_path = [[round(p[0], 2), round(p[1], 2)] for p in raw_path]
            self._add_alert("NAV", f"Target dispatched: ({gx:.1f}m, {gy:.1f}m)", "INFO")

    def teleop(self, v: float, omega: float):
        """Manual control command."""
        with self.lock:
            self.mode = "MANUAL"
            self.cmd_v = max(-1.2, min(1.5, v))
            self.cmd_omega = max(-1.8, min(1.8, omega))

    def set_faults(self, wheel_slip: Optional[bool] = None, vslam_blackout: Optional[bool] = None, imu_bias: Optional[bool] = None):
        with self.lock:
            if wheel_slip is not None:
                self.fault_wheel_slip = wheel_slip
                msg = "Wheel slippage injected (surface friction loss)" if wheel_slip else "Wheel traction restored"
                self._add_alert("SLIP", msg, "WARN" if wheel_slip else "INFO")
            if vslam_blackout is not None:
                self.fault_vslam_blackout = vslam_blackout
                msg = "Optical blackout injected (IR jamming / low features)" if vslam_blackout else "Optical sensors restored"
                self._add_alert("VSLAM", msg, "WARN" if vslam_blackout else "INFO")
            if imu_bias is not None:
                self.fault_imu_bias = imu_bias
                msg = "IMU gyro bias drift active (+0.08 rad/s)" if imu_bias else "IMU bias calibrated"
                self._add_alert("IMU", msg, "WARN" if imu_bias else "INFO")

    def toggle_pause(self):
        with self.lock:
            self.is_paused = not self.is_paused

    def reset_simulation(self):
        with self.lock:
            self.gt_x = 0.0
            self.gt_y = 0.0
            self.gt_theta = 0.0
            self.gt_v = 0.0
            self.gt_omega = 0.0

            self.odom_x = 0.0
            self.odom_y = 0.0
            self.odom_theta = 0.0
            self.odom_v = 0.0
            self.odom_omega = 0.0

            self.ekf = ExtendedKalmanFilter2D(0.0, 0.0, 0.0)
            self.gt_trail.clear()
            self.odom_trail.clear()
            self.ekf_trail.clear()
            self.log_records.clear()
            self.ate_odom_history.clear()
            self.ate_ekf_history.clear()
            self.watchdog_alerts.clear()
            self.mission_start_time = time.time()
            self.mode = "PATROL"
            self._set_patrol_goal(0)
            self._add_alert("SYSTEM", "Mission simulation reset to Base Station (0, 0)", "INFO")

    def _add_alert(self, category: str, text: str, level: str = "INFO"):
        self.watchdog_alerts.append({
            "timestamp": time.strftime("%H:%M:%S"),
            "category": category,
            "text": text,
            "level": level
        })
        if len(self.watchdog_alerts) > 30:
            self.watchdog_alerts.pop(0)

    def _sim_loop(self):
        dt = 0.05  # 20 Hz loop
        trail_tick = 0
        log_tick = 0

        while self.running:
            time.sleep(dt)
            if self.is_paused:
                continue

            with self.lock:
                # 1. Trajectory tracking & Controller
                if self.mode in ("PATROL", "WAYPOINT") and self.planned_path:
                    # Find lookahead waypoint
                    curr_x, curr_y = self.ekf.x[0], self.ekf.x[1]
                    target_pt = self.planned_path[0]
                    dist_to_pt = math.hypot(target_pt[0] - curr_x, target_pt[1] - curr_y)

                    if dist_to_pt < 0.8 and len(self.planned_path) > 1:
                        self.planned_path.pop(0)
                        target_pt = self.planned_path[0]
                        dist_to_pt = math.hypot(target_pt[0] - curr_x, target_pt[1] - curr_y)

                    if len(self.planned_path) <= 1 and dist_to_pt < 0.6:
                        # Reached target
                        if self.mode == "PATROL":
                            self._add_alert("PATROL", f"Checkpoint cleared: {self.current_goal['name']}", "SUCCESS")
                            self._set_patrol_goal(self.current_wp_idx + 1)
                        else:
                            self._add_alert("NAV", "Custom destination reached. Holding position.", "SUCCESS")
                            self.mode = "IDLE"
                            self.cmd_v = 0.0
                            self.cmd_omega = 0.0
                            self.planned_path.clear()
                    else:
                        # Pure Pursuit steering
                        angle_to_goal = math.atan2(target_pt[1] - curr_y, target_pt[0] - curr_x)
                        heading_err = normalize_angle(angle_to_goal - self.ekf.x[2])

                        self.cmd_v = max(0.2, min(1.2, 0.9 * dist_to_pt))
                        # Slow down in sharp turns
                        if abs(heading_err) > 0.6:
                            self.cmd_v *= 0.4
                        self.cmd_omega = max(-1.5, min(1.5, 2.2 * heading_err))
                elif self.mode == "IDLE":
                    self.cmd_v = 0.0
                    self.cmd_omega = 0.0

                # 2. Physics & Kinematics update (Ground Truth)
                target_v = self.cmd_v
                target_omega = self.cmd_omega

                # Apply acceleration smoothing
                self.gt_v += np.clip(target_v - self.gt_v, -2.0 * dt, 2.0 * dt)
                self.gt_omega += np.clip(target_omega - self.gt_omega, -3.0 * dt, 3.0 * dt)

                next_gt_x = self.gt_x + self.gt_v * math.cos(self.gt_theta) * dt
                next_gt_y = self.gt_y + self.gt_v * math.sin(self.gt_theta) * dt
                next_gt_theta = normalize_angle(self.gt_theta + self.gt_omega * dt)

                # Collision check
                if self.bunker.is_collision(next_gt_x, next_gt_y, margin=0.35):
                    self.gt_v = 0.0
                    self.collision_alert = True
                else:
                    self.gt_x = next_gt_x
                    self.gt_y = next_gt_y
                    self.gt_theta = next_gt_theta
                    self.collision_alert = False

                # 3. Simulate Sensors
                # Wheel odometry with realistic systematic slip and scaling error
                slip_factor = 0.65 if self.fault_wheel_slip else 0.96
                wheel_noise_v = np.random.normal(0, 0.04)
                wheel_noise_omega = np.random.normal(0, 0.03)

                meas_wheel_v = self.gt_v * slip_factor + wheel_noise_v
                meas_wheel_omega = self.gt_omega * (1.02 if not self.fault_wheel_slip else 0.88) + wheel_noise_omega

                # Dead-reckoning integration (raw odometry drift)
                self.odom_theta = normalize_angle(self.odom_theta + meas_wheel_omega * dt)
                self.odom_x += meas_wheel_v * math.cos(self.odom_theta) * dt
                self.odom_y += meas_wheel_v * math.sin(self.odom_theta) * dt
                self.odom_v = meas_wheel_v
                self.odom_omega = meas_wheel_omega

                # 9-DoF IMU measurement (angular velocity with bias)
                imu_bias = 0.08 if self.fault_imu_bias else 0.005
                meas_imu_omega = self.gt_omega + imu_bias + np.random.normal(0, 0.015)
                self.last_imu_gyro = meas_imu_omega

                # RGB-D Visual Odometry (VSLAM)
                in_dropout_zone = self.bunker.is_in_dropout_zone(self.gt_x, self.gt_y)
                is_vo_available = not (in_dropout_zone or self.fault_vslam_blackout)

                meas_vo_x = self.gt_x + np.random.normal(0, 0.06)
                meas_vo_y = self.gt_y + np.random.normal(0, 0.06)

                # 4. Extended Kalman Filter (EKF) Step
                # Prediction
                self.ekf.predict(dt)

                # Wheel velocity update
                self.ekf.update_wheel(np.array([meas_wheel_v, meas_wheel_omega]))

                # IMU gyro update
                self.ekf.update_imu(meas_imu_omega)

                # VSLAM update (if features available)
                if is_vo_available:
                    self.ekf.update_vo(np.array([meas_vo_x, meas_vo_y]))
                    self.last_vo_time = time.time()

                # 5. Supervisor Watchdog Integrity Monitor
                now = time.time()
                vo_elapsed = now - self.last_vo_time
                if vo_elapsed > 1.2:
                    if not self.is_vslam_degraded:
                        self.is_vslam_degraded = True
                        reason = "IR Jamming / Manual Blackout" if self.fault_vslam_blackout else "Low Feature Density Zone"
                        self._add_alert("WATCHDOG", f"VSLAM dropout: {reason}. Operating on inertial EKF.", "WARN")
                else:
                    if self.is_vslam_degraded:
                        self.is_vslam_degraded = False
                        self._add_alert("WATCHDOG", "VSLAM visual loop re-acquired. Full pose fusion restored.", "INFO")

                slip_diff = abs(meas_wheel_v - self.ekf.x[3])
                if slip_diff > 0.25:
                    if not self.is_wheel_slipping:
                        self.is_wheel_slipping = True
                        self._add_alert("WATCHDOG", f"High wheel slip detected (Δv={slip_diff:.2f} m/s)!", "WARN")
                else:
                    self.is_wheel_slipping = False

                # 6. LiDAR Scan (raycasting)
                self.latest_lidar_scan = self.bunker.get_lidar_scan(self.gt_x, self.gt_y, self.gt_theta, num_beams=32)

                # 7. Record Trajectory Trails (every 3 ticks = 150ms)
                trail_tick += 1
                if trail_tick >= 3:
                    trail_tick = 0
                    self.gt_trail.append([round(self.gt_x, 2), round(self.gt_y, 2)])
                    self.odom_trail.append([round(self.odom_x, 2), round(self.odom_y, 2)])
                    self.ekf_trail.append([round(float(self.ekf.x[0]), 2), round(float(self.ekf.x[1]), 2)])
                    if len(self.gt_trail) > self.max_trail_points:
                        self.gt_trail.pop(0)
                        self.odom_trail.pop(0)
                        self.ekf_trail.pop(0)

                # 8. Benchmark Metrics Logging (every 4 ticks = 200ms)
                log_tick += 1
                if log_tick >= 4:
                    log_tick = 0
                    err_odom = math.hypot(self.gt_x - self.odom_x, self.gt_y - self.odom_y)
                    err_ekf = math.hypot(self.gt_x - self.ekf.x[0], self.gt_y - self.ekf.x[1])
                    self.ate_odom_history.append(err_odom)
                    self.ate_ekf_history.append(err_ekf)
                    if len(self.ate_odom_history) > 200:
                        self.ate_odom_history.pop(0)
                        self.ate_ekf_history.pop(0)

                    self.log_records.append({
                        "time": round(now - self.mission_start_time, 2),
                        "gt_x": round(self.gt_x, 3),
                        "gt_y": round(self.gt_y, 3),
                        "gt_theta": round(self.gt_theta, 3),
                        "odom_x": round(self.odom_x, 3),
                        "odom_y": round(self.odom_y, 3),
                        "ekf_x": round(float(self.ekf.x[0]), 3),
                        "ekf_y": round(float(self.ekf.x[1]), 3),
                        "ate_odom": round(err_odom, 3),
                        "ate_ekf": round(err_ekf, 3)
                    })
                    if len(self.log_records) > 2000:
                        self.log_records.pop(0)

    def get_state_payload(self) -> Dict[str, Any]:
        """Telemetry payload sent via WebSocket / HTTP to UI."""
        with self.lock:
            rmse_odom = math.sqrt(np.mean(np.square(self.ate_odom_history))) if self.ate_odom_history else 0.0
            rmse_ekf = math.sqrt(np.mean(np.square(self.ate_ekf_history))) if self.ate_ekf_history else 0.0
            drift_reduction = (1.0 - (rmse_ekf / max(1e-4, rmse_odom))) * 100.0 if rmse_odom > 0.05 else 0.0
            max_err_odom = max(self.ate_odom_history) if self.ate_odom_history else 0.0
            max_err_ekf = max(self.ate_ekf_history) if self.ate_ekf_history else 0.0

            curr_ate_odom = math.hypot(self.gt_x - self.odom_x, self.gt_y - self.odom_y)
            curr_ate_ekf = math.hypot(self.gt_x - self.ekf.x[0], self.gt_y - self.ekf.x[1])

            ellipse = self.ekf.get_covariance_ellipse()

            return {
                "timestamp": round(time.time() - self.mission_start_time, 2),
                "mode": self.mode,
                "is_paused": self.is_paused,
                # Robot states
                "gt": {
                    "x": round(self.gt_x, 3),
                    "y": round(self.gt_y, 3),
                    "theta": round(self.gt_theta, 3),
                    "v": round(self.gt_v, 2),
                    "omega": round(self.gt_omega, 2)
                },
                "odom": {
                    "x": round(self.odom_x, 3),
                    "y": round(self.odom_y, 3),
                    "theta": round(self.odom_theta, 3),
                    "v": round(self.odom_v, 2),
                    "omega": round(self.odom_omega, 2)
                },
                "ekf": {
                    "x": round(float(self.ekf.x[0]), 3),
                    "y": round(float(self.ekf.x[1]), 3),
                    "theta": round(float(self.ekf.x[2]), 3),
                    "v": round(float(self.ekf.x[3]), 2),
                    "omega": round(float(self.ekf.x[4]), 2),
                    "ellipse": ellipse
                },
                # Trails
                "trails": {
                    "gt": self.gt_trail[-100:],
                    "odom": self.odom_trail[-100:],
                    "ekf": self.ekf_trail[-100:]
                },
                # LiDAR & Environmental
                "lidar": self.latest_lidar_scan,
                "planned_path": self.planned_path,
                "current_goal": self.current_goal,
                "checkpoints": self.checkpoints,
                "active_wp_idx": self.current_wp_idx,
                "in_dropout_zone": self.bunker.is_in_dropout_zone(self.gt_x, self.gt_y),
                # Sensor Integrity & Watchdog Status
                "watchdog": {
                    "gps_denied": True,
                    "vslam_degraded": self.is_vslam_degraded,
                    "wheel_slipping": self.is_wheel_slipping,
                    "collision_alert": self.collision_alert,
                    "fault_wheel_slip": self.fault_wheel_slip,
                    "fault_vslam_blackout": self.fault_vslam_blackout,
                    "fault_imu_bias": self.fault_imu_bias
                },
                # Benchmark Metrics
                "metrics": {
                    "curr_ate_odom": round(curr_ate_odom, 3),
                    "curr_ate_ekf": round(curr_ate_ekf, 3),
                    "rmse_odom": round(rmse_odom, 3),
                    "rmse_ekf": round(rmse_ekf, 3),
                    "max_err_odom": round(max_err_odom, 3),
                    "max_err_ekf": round(max_err_ekf, 3),
                    "drift_reduction_pct": round(drift_reduction, 1),
                    "recent_ate_odom": [round(x, 2) for x in self.ate_odom_history[-30:]],
                    "recent_ate_ekf": [round(x, 2) for x in self.ate_ekf_history[-30:]]
                },
                # Watchdog logs
                "alerts": self.watchdog_alerts[-10:]
            }

    def export_csv_data(self) -> str:
        """Returns CSV string of logged trajectory benchmark data."""
        with self.lock:
            lines = [
                "time_sec,gt_x,gt_y,gt_theta,odom_x,odom_y,ekf_x,ekf_y,ate_odom_err,ate_ekf_err"
            ]
            for r in self.log_records:
                lines.append(
                    f"{r['time']},{r['gt_x']},{r['gt_y']},{r['gt_theta']},"
                    f"{r['odom_x']},{r['odom_y']},{r['ekf_x']},{r['ekf_y']},"
                    f"{r['ate_odom']},{r['ate_ekf']}"
                )
            return "\n".join(lines)
