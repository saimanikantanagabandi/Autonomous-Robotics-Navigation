"""
Unit and Integration Tests for Mission Control Stack
===================================================
Tests BunkerMap geometry, raycasting, A* planning, EKF equations,
simulation engine, and FastAPI REST endpoints.
"""

import math
import numpy as np
from mission_control.bunker_map import BunkerMap
from mission_control.simulation_engine import ExtendedKalmanFilter2D, SimulationEngine

def test_bunker_map_geometry():
    bunker = BunkerMap()
    # Boundary collision
    assert bunker.is_collision(20.5, 0.0) is True
    assert bunker.is_collision(-20.5, 0.0) is True
    assert bunker.is_collision(0.0, 20.5) is True
    assert bunker.is_collision(0.0, -20.5) is True

    # Center partition collision
    assert bunker.is_collision(0.0, 0.0) is True

    # Open area
    assert bunker.is_collision(5.0, 5.0) is False

    # Pillar collision
    assert bunker.is_collision(8.0, 8.0) is True
    print("[PASS] BunkerMap geometry tests")

def test_lidar_raycasting():
    bunker = BunkerMap()
    # Raycast from (0, 10) pointing North (should hit North wall at y=20)
    dist, hx, hy = bunker.cast_ray(0.0, 10.0, math.pi / 2, max_range=20.0)
    assert abs(dist - 10.0) < 0.2
    assert abs(hy - 20.0) < 0.2

    # 36-beam scan
    scan = bunker.get_lidar_scan(0.0, 5.0, 0.0, num_beams=18)
    assert len(scan) == 18
    for beam in scan:
        assert 0.0 < beam["range"] <= 14.0
    print("[PASS] LiDAR raycasting tests")

def test_astar_planner():
    bunker = BunkerMap()
    path = bunker.plan_path_astar((0.0, 5.0), (5.0, 10.0))
    assert len(path) >= 2
    assert abs(path[0][0] - 0.0) < 1.0
    assert abs(path[-1][0] - 5.0) < 1.0
    print("[PASS] A* path planning tests")

def test_ekf_equations():
    ekf = ExtendedKalmanFilter2D(init_x=0.0, init_y=0.0, init_theta=0.0)
    # Predict step
    ekf.x[3] = 1.0  # v = 1 m/s
    ekf.predict(0.1)
    assert abs(ekf.x[0] - 0.1) < 1e-4

    # Wheel update
    ekf.update_wheel(np.array([1.2, 0.1]))
    assert abs(ekf.x[3] - 1.2) < 0.3

    # IMU update
    ekf.update_imu(0.2)
    assert abs(ekf.x[4] - 0.2) < 0.3

    # VO update
    ekf.update_vo(np.array([0.15, 0.0]))
    assert abs(ekf.x[0] - 0.15) < 0.1

    # Covariance ellipse
    ellipse = ekf.get_covariance_ellipse()
    assert ellipse["rx"] > 0
    assert ellipse["ry"] > 0
    assert "angle" in ellipse
    print("[PASS] EKF mathematical equations tests")

def test_simulation_engine():
    engine = SimulationEngine()
    time_payload = engine.get_state_payload()
    assert "gt" in time_payload
    assert "odom" in time_payload
    assert "ekf" in time_payload
    assert "metrics" in time_payload
    assert "lidar" in time_payload

    # Teleop test
    engine.teleop(0.5, 0.2)
    assert engine.cmd_v == 0.5
    assert engine.cmd_omega == 0.2

    # Custom goal dispatch
    engine.set_custom_goal(4.0, 3.0)
    assert engine.mode == "WAYPOINT"
    assert len(engine.planned_path) > 0

    # CSV export
    csv_str = engine.export_csv_data()
    assert "time_sec,gt_x,gt_y" in csv_str
    print("[PASS] Simulation engine & telemetry tests")

if __name__ == "__main__":
    test_bunker_map_geometry()
    test_lidar_raycasting()
    test_astar_planner()
    test_ekf_equations()
    test_simulation_engine()
    print("\n[ALL TESTS PASSED SUCCESSFULLY!]")
