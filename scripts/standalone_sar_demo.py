#!/usr/bin/env python3
"""
AEGIS-SAR: Standalone Python Search & Rescue Mission & OpenCV Benchmark
======================================================================
Simulates an autonomous Search & Rescue UGV in a post-disaster environment:
  1. Systematic SAR Sector Search & Path Planning (A*)
  2. OpenCV-Powered Multi-Spectral Human Detection & Triage
  3. 15-State EKF Sensor Fusion (Wheel Odometry + 9-DoF IMU + VSLAM)
  4. Real-time Emergency Alert Generation & Evacuation Routing
  5. Quantitative Performance Benchmark (RMSE, Survivor Discovery Latency)

Outputs publication-grade benchmark plot to: results/sar_mission_benchmark.png
"""

import os
import sys
import math
import time

# Ensure project root is on sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import numpy as np
import matplotlib.pyplot as plt
import cv2

from mission_control.bunker_map import BunkerMap
from mission_control.simulation_engine import ExtendedKalmanFilter2D, normalize_angle
from mission_control.sar_vision import SARVisionDetector

def run_sar_simulation(duration_sec=75.0, dt=0.05):
    print("=" * 80)
    print("   AEGIS-SAR: AUTONOMOUS SEARCH & RESCUE ROBOT MISSION BENCHMARK")
    print("   OpenCV AI Vision • Multi-Sensor Fusion • Nav2 Path Planning • Triage")
    print("=" * 80)

    bunker = BunkerMap()
    init_x, init_y, init_theta = 0.0, -3.0, 0.0
    ekf = ExtendedKalmanFilter2D(init_x=init_x, init_y=init_y, init_theta=init_theta)
    vision = SARVisionDetector(width=320, height=240)

    # Mission Waypoints (Sector Search)
    sectors = bunker.default_checkpoints
    sector_idx = 0

    # Robot Physics Ground Truth
    gt_x, gt_y, gt_theta = init_x, init_y, init_theta
    gt_v, gt_omega = 0.0, 0.0

    # Dead-Reckoning (Uncorrected Odometry)
    odom_x, odom_y, odom_theta = init_x, init_y, init_theta

    # Survivors and Hazards
    survivors = [dict(s) for s in bunker.get_survivors()]
    hazards = bunker.get_hazards()
    rubble_piles = bunker.rubble_piles

    # Tracking History
    hist_t = []
    hist_gt = []
    hist_ekf = []
    hist_odom = []
    hist_ate_ekf = []
    hist_ate_odom = []
    discovered_survivors = []
    dispatched_alerts = []

    # Current planned path
    def plan_to(target):
        pts = bunker.plan_path_astar((ekf.x[0], ekf.x[1]), (target["x"], target["y"]))
        return list(pts)

    planned_path = plan_to(sectors[0])
    total_steps = int(duration_sec / dt)

    print(f"\n[+] Executing {duration_sec}s Disaster Sweep across {len(sectors)} sectors...")
    print(f"[+] Total Simulated Survivors to Locate: {len(survivors)}")
    print(f"[+] Active Hazards: {len(hazards)} structural fire/gas/collapse zones\n")

    t_start = time.time()
    for step in range(total_steps):
        t = step * dt

        # 1. Trajectory Controller
        if planned_path:
            curr_x, curr_y = ekf.x[0], ekf.x[1]
            target_pt = planned_path[0]
            dist_to_pt = math.hypot(target_pt[0] - curr_x, target_pt[1] - curr_y)

            if dist_to_pt < 0.85 and len(planned_path) > 1:
                planned_path.pop(0)
                target_pt = planned_path[0]
                dist_to_pt = math.hypot(target_pt[0] - curr_x, target_pt[1] - curr_y)

            if len(planned_path) <= 1 and dist_to_pt < 0.7:
                sector_idx = (sector_idx + 1) % len(sectors)
                planned_path = plan_to(sectors[sector_idx])
            else:
                target_angle = math.atan2(target_pt[1] - curr_y, target_pt[0] - curr_x)
                heading_err = normalize_angle(target_angle - ekf.x[2])
                cmd_v = max(0.3, min(1.35, 1.1 * dist_to_pt))
                if abs(heading_err) > 0.6:
                    cmd_v *= 0.45
                cmd_omega = max(-1.6, min(1.6, 2.4 * heading_err))
        else:
            cmd_v, cmd_omega = 0.0, 0.0

        # 2. Kinematics (Ground Truth)
        gt_v += np.clip(cmd_v - gt_v, -2.5 * dt, 2.5 * dt)
        gt_omega += np.clip(cmd_omega - gt_omega, -3.5 * dt, 3.5 * dt)

        next_x = gt_x + gt_v * math.cos(gt_theta) * dt
        next_y = gt_y + gt_v * math.sin(gt_theta) * dt
        next_th = normalize_angle(gt_theta + gt_omega * dt)

        if not bunker.is_collision(next_x, next_y, margin=0.35):
            gt_x, gt_y, gt_theta = next_x, next_y, next_th

        # 3. Sensor Streams
        # Slippery rubble introduces systematic wheel slip
        is_slipping = (15.0 <= t <= 28.0)
        slip_ratio = 0.62 if is_slipping else 0.96
        meas_v = gt_v * slip_ratio + np.random.normal(0, 0.035)
        meas_w = gt_omega * (1.02 if not is_slipping else 0.86) + np.random.normal(0, 0.025)

        # Raw Odometry dead-reckoning
        odom_theta = normalize_angle(odom_theta + meas_w * dt)
        odom_x += meas_v * math.cos(odom_theta) * dt
        odom_y += meas_v * math.sin(odom_theta) * dt

        # IMU Gyro
        meas_gyro = gt_omega + 0.005 + np.random.normal(0, 0.015)

        # Visual & Thermal SLAM (FLIR Thermal IR penetrates smoke zone)
        in_smoke = bunker.is_in_dropout_zone(gt_x, gt_y)
        vo_noise = 0.12 if in_smoke else 0.04
        meas_vo = np.array([gt_x + np.random.normal(0, vo_noise), gt_y + np.random.normal(0, vo_noise)])
        meas_vo_th = gt_theta + np.random.normal(0, 0.03 if in_smoke else 0.012)

        # 4. Extended Kalman Filter (EKF)
        ekf.predict(dt)
        ekf.update_wheel(np.array([meas_v, meas_w]))
        ekf.update_imu(meas_gyro)
        ekf.update_vo(meas_vo, meas_vo_th)

        # 5. OpenCV Vision & Survivor Detection (at 10 Hz)
        if step % 2 == 0:
            frame_b64, active_dets, new_alert = vision.render_and_detect(
                gt_x, gt_y, gt_theta,
                survivors, hazards, rubble_piles,
                in_smoke
            )
            if new_alert:
                surv_id = new_alert["id"]
                new_alert["time_sec"] = round(t, 1)
                dispatched_alerts.append(new_alert)
                for s in survivors:
                    if s["id"] == surv_id and not s["discovered"]:
                        s["discovered"] = True
                        discovered_survivors.append(s)
                        print(f"  [!] T={t:4.1f}s | ALERT DISPATCHED -> {s['name']} "
                              f"at (X:{new_alert['world_x']:.2f}m, Y:{new_alert['world_y']:.2f}m) | "
                              f"TRIAGE: {s['triage']} | Confidence: {new_alert['confidence']*100:.0f}%")

        # 6. Log telemetry
        ate_ekf = math.hypot(gt_x - ekf.x[0], gt_y - ekf.x[1])
        ate_odom = math.hypot(gt_x - odom_x, gt_y - odom_y)

        hist_t.append(t)
        hist_gt.append((gt_x, gt_y))
        hist_ekf.append((ekf.x[0], ekf.x[1]))
        hist_odom.append((odom_x, odom_y))
        hist_ate_ekf.append(ate_ekf)
        hist_ate_odom.append(ate_odom)

    # Statistical Evaluation
    rmse_ekf = math.sqrt(np.mean(np.square(hist_ate_ekf)))
    rmse_odom = math.sqrt(np.mean(np.square(hist_ate_odom)))
    drift_reduction = (1.0 - (rmse_ekf / rmse_odom)) * 100.0

    print("\n" + "=" * 80)
    print("   QUANTITATIVE BENCHMARK EVALUATION & SEARCH & RESCUE REPORT")
    print("=" * 80)
    print(f"Total Sweep Duration          : {duration_sec:.1f} s")
    disc_rate = (len(discovered_survivors) / max(1, len(survivors))) * 100.0
    print(f"Survivors Located             : {len(discovered_survivors)} / {len(survivors)} ({disc_rate:.0f}% Discovery Rate)")
    print(f"Raw Odometry Drift RMSE       : {rmse_odom:.3f} m (Peak Error: {max(hist_ate_odom):.2f} m)")
    print(f"EKF Multi-Sensor Fusion RMSE  : {rmse_ekf:.3f} m (Peak Error: {max(hist_ate_ekf):.2f} m)")
    print(f"Cumulative Drift Reduction    : +{drift_reduction:.1f}% ACCURACY IMPROVEMENT")
    print("=" * 80)

    # Generate Publication-Grade Plot
    generate_benchmark_plot(
        bunker, hist_gt, hist_ekf, hist_odom,
        hist_t, hist_ate_ekf, hist_ate_odom,
        discovered_survivors, dispatched_alerts,
        rmse_ekf, rmse_odom, drift_reduction
    )

def generate_benchmark_plot(
    bunker, hist_gt, hist_ekf, hist_odom,
    hist_t, hist_ate_ekf, hist_ate_odom,
    survivors, alerts,
    rmse_ekf, rmse_odom, drift_red
):
    os.makedirs("results", exist_ok=True)
    out_file = os.path.join("results", "sar_mission_benchmark.png")

    fig = plt.figure(figsize=(16, 9), facecolor="#090d16")

    # Subplot 1: 2D Spatial Map (Trajectory + Rubble + Hazards + Survivors)
    ax1 = plt.subplot(1, 2, 1, facecolor="#0c1220")
    ax1.set_title("AEGIS-SAR 2D Spatial Trajectory & Survivor Triage Map", color="#f8fafc", fontsize=13, fontweight="bold", pad=12)
    ax1.set_xlim(-21, 21)
    ax1.set_ylim(-21, 21)
    ax1.set_xlabel("X Coordinate (meters)", color="#94a3b8", fontsize=10)
    ax1.set_ylabel("Y Coordinate (meters)", color="#94a3b8", fontsize=10)
    ax1.tick_params(colors="#94a3b8")
    ax1.grid(True, color="#1e293b", linestyle="--", alpha=0.6)

    # Perimeter and Center Walls
    for w in bunker.wall_segments:
        ax1.plot([w[0], w[2]], [w[1], w[3]], color="#38bdf8", linewidth=2.5, zorder=3)

    # Rubble Piles
    for r in bunker.rubble_piles:
        circle = plt.Circle((r[0], r[1]), r[2], color="#64748b", alpha=0.7, zorder=4)
        ax1.add_patch(circle)
        ax1.text(r[0], r[1], "RUBBLE", color="#e2e8f0", fontsize=7, ha="center", va="center", zorder=5)

    # Active Fire / Gas Hazards
    for h in bunker.hazards:
        if h.get("temp_c", 20) > 100:
            circle = plt.Circle((h["x"], h["y"]), h["radius"], color="#ef4444", alpha=0.35, zorder=4)
            ax1.add_patch(circle)
            ax1.text(h["x"], h["y"], f"FIRE\n{h['temp_c']}°C", color="#fef08a", fontsize=8, ha="center", va="center", fontweight="bold", zorder=5)

    # Trajectories
    gt_arr = np.array(hist_gt)
    ekf_arr = np.array(hist_ekf)
    odom_arr = np.array(hist_odom)

    ax1.plot(odom_arr[:, 0], odom_arr[:, 1], color="#ef4444", linestyle="--", linewidth=1.6, label=f"Dead-Reckoning (Drift RMSE: {rmse_odom:.2f}m)", zorder=6)
    ax1.plot(ekf_arr[:, 0], ekf_arr[:, 1], color="#06b6d4", linewidth=2.2, label=f"EKF Sensor Fusion (RMSE: {rmse_ekf:.2f}m)", zorder=7)
    ax1.plot(gt_arr[:, 0], gt_arr[:, 1], color="#10b981", linewidth=1.8, label="Ground Truth Path", zorder=8)

    # Survivors
    for s in bunker.get_survivors():
        clr = "#ef4444" if s["triage"] == "RED" else ("#f59e0b" if s["triage"] == "YELLOW" else "#10b981")
        ax1.scatter(s["x"], s["y"], s=180, color=clr, edgecolors="#ffffff", linewidths=1.5, zorder=10)
        ax1.text(s["x"] + 0.6, s["y"], f"{s['id']}\n({s['triage']})", color="#ffffff", fontsize=8, fontweight="bold", va="center", zorder=11)

    # Base Station
    ax1.scatter(0, 0, s=150, color="#38bdf8", marker="P", edgecolors="#fff", zorder=12)
    ax1.text(0, -1.2, "BASE STAGING", color="#38bdf8", fontsize=9, fontweight="bold", ha="center")

    leg = ax1.legend(loc="upper left", facecolor="#111827", edgecolor="#1e293b", fontsize=9, labelcolor="#f8fafc")

    # Subplot 2: Trajectory Error Comparison & Discovery Timeline
    ax2 = plt.subplot(2, 2, 2, facecolor="#0c1220")
    ax2.set_title("Absolute Trajectory Error (ATE) Tracking Over Time", color="#f8fafc", fontsize=11, fontweight="bold")
    ax2.set_xlabel("Mission Elapsed Time (seconds)", color="#94a3b8", fontsize=9)
    ax2.set_ylabel("Position Error (meters)", color="#94a3b8", fontsize=9)
    ax2.tick_params(colors="#94a3b8")
    ax2.grid(True, color="#1e293b", linestyle="--", alpha=0.5)

    ax2.plot(hist_t, hist_ate_odom, color="#ef4444", linestyle="--", linewidth=1.5, label="Raw Odometry Drift")
    ax2.plot(hist_t, hist_ate_ekf, color="#06b6d4", linewidth=2.0, label="EKF Multi-Sensor Fusion")
    ax2.axvspan(15, 28, color="#f59e0b", alpha=0.15, label="Rubble Slippage Zone")
    ax2.legend(loc="upper left", facecolor="#111827", edgecolor="#1e293b", fontsize=8, labelcolor="#f8fafc")

    # Subplot 3: Survivor Discovery Timeline & Triage Bar
    ax3 = plt.subplot(2, 2, 4, facecolor="#0c1220")
    ax3.set_title("OpenCV Survivor Discovery & Triage Timeline", color="#f8fafc", fontsize=11, fontweight="bold")
    ax3.set_xlabel("Discovery Time (seconds)", color="#94a3b8", fontsize=9)
    ax3.set_ylabel("Detection Confidence (%)", color="#94a3b8", fontsize=9)
    ax3.tick_params(colors="#94a3b8")
    ax3.set_ylim(40, 105)
    ax3.grid(True, color="#1e293b", linestyle="--", alpha=0.5)

    for a in alerts:
        conf_pct = a["confidence"] * 100
        clr = "#ef4444" if a["triage"] == "RED" else ("#f59e0b" if a["triage"] == "YELLOW" else "#10b981")
        ax3.scatter(a["time_sec"], conf_pct, s=160, color=clr, edgecolors="#fff", zorder=5)
        ax3.annotate(
            f"{a['name']}\nConf: {conf_pct:.0f}% | {a['triage']}",
            (a["time_sec"], conf_pct),
            xytext=(0, 12), textcoords="offset points",
            color="#f8fafc", fontsize=8, ha="center",
            bbox=dict(boxstyle="round,pad=0.2", fc="#1f293d", ec=clr, lw=1)
        )

    plt.tight_layout()
    plt.savefig(out_file, dpi=200, facecolor=fig.get_facecolor(), edgecolor="none")
    plt.close()
    print(f"\n[+] High-Resolution Benchmark Plot Generated: {out_file}\n")

if __name__ == "__main__":
    run_sar_simulation(duration_sec=75.0)
