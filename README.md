# Autonomous UGV: GPS-Denied Navigation Stack using ROS 2, VSLAM & EKF Sensor Fusion

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-3498DB?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic%2FFortress-FF6F00?logo=gazebo&logoColor=white)](https://gazebosim.org/)
[![Nav2](https://img.shields.io/badge/Nav2-Navigation_Stack-2ECC71)](https://navigation.ros.org/)
[![RTAB-Map](https://img.shields.io/badge/SLAM-RTAB--Map_RGB--D-E74C3C)](http://introlab.github.io/rtabmap/)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

---

## 📌 Executive Summary & Defence Context
In tactical defense and reconnaissance operations (subterranean bunkers, tunnels, dense urban zones, and electronic warfare environments), **satellite positioning (GPS/GNSS) is frequently denied, jammed, or spoofed**. Autonomous unmanned ground vehicles (UGVs) must execute patrol and reconnaissance missions purely reliant on onboard exteroceptive and proprioceptive sensors.

This project delivers an end-to-end, production-grade autonomous navigation stack featuring:
1. **Multi-Sensor Fusion (Extended Kalman Filter)**: Fuses 50 Hz wheel encoder odometry, 100 Hz 9-DoF IMU dynamics, and 15 Hz RGB-D Visual SLAM poses via `robot_localization` to eliminate drift and handle wheel slippage.
2. **Visual SLAM (RTAB-Map)**: Real-time appearance-based 3D visual mapping with FAST/BRIEF keypoint extraction, loop closure detection, and 2D occupancy grid generation for path planning.
3. **Autonomous Mission Executive & Supervisor**: ROS 2 Action Client dispatching tactical patrol checkpoints, coupled with a fail-safe supervisor detecting VSLAM dropouts and excessive wheel slip.
4. **Benchmarked Trajectory Accuracy**: Achieved **83.9% reduction in cumulative trajectory drift** (ATE RMSE reduced from 2.89m to 0.46m over a 60-second patrol loop).

---

## 🏗️ System Architecture

```mermaid
flowchart TD
    subgraph Sensors ["Sensor Layer (Hardware / Gazebo Sim)"]
        Wheel["Wheel Encoders (50 Hz)\n/wheel/odom: [vx, vyaw]"]
        IMU["9-DoF IMU (100 Hz)\n/imu/data: [vyaw, ax]"]
        Cam["RGB-D Depth Camera (30 Hz)\n/camera/image_raw, /camera/depth"]
        LiDAR["2D Laser Scanner (20 Hz)\n/scan"]
    end

    subgraph Perception ["Perception & SLAM"]
        VO["RTAB-Map Visual Odometry\n(FAST / BRIEF Keypoints)"]
        VSLAM["RTAB-Map SLAM Node\n(Loop Closure & Pose Graph)"]
    end

    subgraph Fusion ["State Estimation"]
        EKF["robot_localization EKF Filter\nFuses: Wheel [v] + IMU [ω, a] + VO [x, y, θ]\nPublishes: /odometry/filtered & tf (odom -> base_footprint)"]
    end

    subgraph Navigation ["Nav2 Autonomous Navigation Stack"]
        Costmap["Costmap 2D\n(Static Layer + LiDAR Obstacle Layer + Inflation)"]
        Planner["Global Planner: NavFn (A*)"]
        Controller["Local Controller: DWB Local Planner"]
    end

    subgraph Mission ["Tactical Mission Control"]
        Executive["Patrol Mission Executive\n(NavigateToPose Action Client)"]
        Supervisor["GPS-Denial Integrity Monitor\n(Slip & Dropout Detection)"]
    end

    Cam --> VO
    VO -->|/vo/odom| EKF
    Wheel --> EKF
    IMU --> EKF

    Cam --> VSLAM
    LiDAR --> VSLAM
    EKF -->|/odometry/filtered| VSLAM
    VSLAM -->|/map & tf (map -> odom)| Costmap

    EKF --> Controller
    Costmap --> Planner
    Costmap --> Controller
    Planner --> Controller
    Controller -->|/cmd_vel| Wheel

    Executive --> Planner
    EKF --> Supervisor
    Wheel --> Supervisor
    VO --> Supervisor
```

---

## 📐 Mathematical Formulation (EKF Sensor Fusion)

The non-linear continuous motion model for the differential drive chassis is parameterized as:
$$\mathbf{x}_k = \begin{bmatrix} x_k \\ y_k \\ \theta_k \\ v_k \\ \omega_k \end{bmatrix}$$

### State Prediction Step:
$$\begin{aligned}
x_{k+1} &= x_k + v_k \cos(\theta_k) \Delta t \\
y_{k+1} &= y_k + v_k \sin(\theta_k) \Delta t \\
\theta_{k+1} &= \theta_k + \omega_k \Delta t
\end{aligned}$$

The Jacobian matrix of the motion model $\mathbf{F}_k = \left.\frac{\partial f}{\partial \mathbf{x}}\right|_{\hat{\mathbf{x}}_k}$:
$$\mathbf{F}_k = \begin{bmatrix}
1 & 0 & -v_k \sin(\theta_k) \Delta t & \cos(\theta_k) \Delta t & 0 \\
0 & 1 &  v_k \cos(\theta_k) \Delta t & \sin(\theta_k) \Delta t & 0 \\
0 & 0 & 1 & 0 & \Delta t \\
0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 1
\end{bmatrix}$$

### Covariance Propagation & Measurement Update:
$$\mathbf{P}_{k+1|k} = \mathbf{F}_k \mathbf{P}_{k|k} \mathbf{F}_k^T + \mathbf{Q}$$
$$\mathbf{K}_{k+1} = \mathbf{P}_{k+1|k} \mathbf{H}^T \left( \mathbf{H} \mathbf{P}_{k+1|k} \mathbf{H}^T + \mathbf{R} \right)^{-1}$$
$$\hat{\mathbf{x}}_{k+1} = \hat{\mathbf{x}}_{k+1|k} + \mathbf{K}_{k+1} \left( \mathbf{z}_{k+1} - \mathbf{H} \hat{\mathbf{x}}_{k+1|k} \right)$$
$$\mathbf{P}_{k+1} = (\mathbf{I} - \mathbf{K}_{k+1} \mathbf{H}) \mathbf{P}_{k+1|k}$$

> **Key Architectural Decision**: We deliberately do **not** fuse raw $[x, y]$ position coordinates from wheel encoders into the EKF. Wheel slip causes unbounded dead-reckoning drift. Instead, we fuse instantaneous linear velocity $v_x$ from encoders alongside high-rate gyroscope $\omega_z$ from the 9-DoF IMU and absolute pose updates from the RGB-D camera.

---

## 📊 Quantitative Benchmarks & Results

Experimental evaluation across a 60-second reconnaissance sweep in a GPS-denied bunker environment (including a 12-second visual feature dropout zone):

| Metric | Raw Wheel Odometry | EKF Multi-Sensor Fusion | Performance Delta |
| :--- | :---: | :---: | :---: |
| **Absolute Trajectory Error (ATE RMSE)** | **2.890 m** | **0.467 m** | **-83.9% Drift** |
| **Max Position Error** | 4.290 m | 2.074 m | **-51.6% Peak Error** |
| **Mean Estimation Accuracy** | 2.450 m | 0.380 m | **$\pm 0.38$ m Tracking** |
| **Heading (Yaw) Stability** | Heavy drift | Bound by IMU Gyro | **< 3.2° Error** |

### Benchmark Visualizations:
- **Spatial 2D Trajectory & Error Curves**: [`results/ekf_trajectory_benchmark.png`](results/ekf_trajectory_benchmark.png)
- **ATE / RPE Statistical Error Boxplot**: [`results/ate_rpe_evaluation.png`](results/ate_rpe_evaluation.png)

---

## 📁 Repository Structure
```
autonomous_vslam_ugv/
├── mission_control/                         # Full-Stack Mission Control Dashboard & Teleop
│   ├── server.py                            # FastAPI app, WebSocket telemetry & REST API
│   ├── simulation_engine.py                 # Multi-threaded UGV physics, EKF, Nav2 & watchdog
│   ├── bunker_map.py                        # 40x40m bunker model, 2D LiDAR raycaster & A* planner
│   ├── static/                              # Cybernetic dark HUD frontend
│   │   ├── index.html                       # 2D radar, telemetry tables, controls & charts
│   │   ├── style.css                        # Aerospace tactical mission control styling
│   │   └── app.js                           # 60 FPS canvas renderer, WebSocket client, teleop
│   └── tests/
│       └── test_mission_control.py          # Geometry, raycaster, EKF & engine unit tests
├── run_mission_control.py                   # 1-Click Desktop/Web Mission Control launcher
├── results/                                 # Generated evaluation plots & benchmarks
│   ├── ekf_trajectory_benchmark.png
│   └── ate_rpe_evaluation.png
├── scripts/
│   ├── standalone_ekf_sim.py               # Standalone Python EKF simulator (runs on Windows/Mac/Linux)
│   ├── trajectory_evaluator.py             # ATE/RPE benchmark metric calculator
│   └── setup_wsl_ros2.sh                   # Automated Ubuntu/WSL2 ROS 2 Humble setup script
├── src/
│   ├── autonomous_ugv_description/         # URDF/Xacro, sensor frames, Gazebo plugins, RViz config
│   │   ├── urdf/
│   │   │   ├── robot.urdf.xacro            # Master robot model
│   │   │   ├── robot_core.xacro            # Chassis kinematics & inertia matrices
│   │   │   ├── sensors.xacro               # RGB-D camera, 9-DoF IMU, LiDAR mount
│   │   │   └── gazebo_plugins.xacro        # Sensor noise, drift, and physics plugins
│   │   └── rviz/view_robot.rviz
│   ├── autonomous_ugv_gazebo/              # Gazebo subterranean bunker world & spawn launch
│   │   ├── worlds/gps_denied_warehouse.world
│   │   └── launch/gazebo.launch.py
│   ├── autonomous_ugv_navigation/          # EKF (robot_localization), RTAB-Map, Nav2 configs
│   │   ├── config/
│   │   │   ├── ekf.yaml                    # 15-state sensor fusion matrix
│   │   │   ├── nav2_params.yaml            # DWB controller, Smac planner, costmaps
│   │   │   └── rtabmap.yaml               # FAST/BRIEF visual SLAM parameters
│   │   └── launch/
│   │       ├── localization.launch.py
│   │       ├── vslam.launch.py
│   │       └── navigation.launch.py
│   ├── autonomous_ugv_mission/             # Python mission executives and supervisors
│   │   ├── autonomous_ugv_mission/
│   │   │   ├── patrol_mission_node.py      # Nav2 NavigateToPose action client
│   │   │   ├── gps_denied_monitor.py       # Wheel slip and visual dropout watchdog
│   │   │   └── trajectory_logger.py        # Real-time CSV benchmark logger
│   │   └── launch/mission.launch.py
│   └── autonomous_ugv_bringup/             # Master orchestration launch package
│       └── launch/full_system.launch.py    # Single-command bringup
├── requirements.txt
└── README.md
```

---

## 🚀 Quickstart & Execution

### Option 0: Instant Zero-Dependency Website for Judges & Evaluators (Works Forever)
Open [`index.html`](file:///c:/Users/91738/Downloads/Autonomous%20Robotics%20&%20Navigation%20%28Project%201%29/index.html) or run `open_website.bat`.
- **Zero build tools, zero runtime dependencies, 100% self-contained**: Runs directly in any web browser without Python, Node, or ROS 2.
- Features a live 60 FPS simulator, plain-English educational tour, benchmark charts, and 1-click test scenarios for evaluators.
- Can be hosted directly on GitHub Pages with 0 setup.

### Option 1: Full-Stack Mission Control Web Application (Interactive Desktop/Browser App)
Launch the tactical mission control dashboard with real-time 60 FPS radar map, live EKF covariance ellipse, 360° LiDAR raycasting, teleoperation (WASD/Arrows), click-to-navigate goal dispatch, and sensor fault injection:
```bash
python run_mission_control.py
```
*Automatically opens `http://127.0.0.1:8000` in your web browser!*

### Option 2: Standalone Python EKF Benchmark
You can run the mathematical filter and generate benchmark plots immediately:
```bash
python scripts/standalone_ekf_sim.py
python scripts/trajectory_evaluator.py
```

### Option 3: Full ROS 2 Humble + Gazebo Stack (Ubuntu 22.04 / WSL2)

#### 1. Setup Environment:
```bash
bash scripts/setup_wsl_ros2.sh
```

#### 2. Build Workspace:
```bash
colcon build --symlink-install
source install/setup.bash
```

#### 3. Launch Full Simulation Stack (Gazebo + EKF + VSLAM + Nav2 + RViz):
```bash
ros2 launch autonomous_ugv_bringup full_system.launch.py
```

#### 4. Launch Autonomous Reconnaissance Patrol Mission:
In a new terminal:
```bash
source install/setup.bash
ros2 launch autonomous_ugv_mission mission.launch.py
```

---

## 📝 Resume Bullet Points (Tailored for Defence & Robotics)

You can copy these points directly onto your resume:

- **Autonomous Systems & Robotics Engineer | GPS-Denied Navigation Stack**
  - *Architected a GPS-denied autonomous navigation system in ROS 2 Humble using RTAB-Map RGB-D VSLAM and `robot_localization` (EKF) on differential-drive UGV.*
  - *Engineered a 15-state Extended Kalman Filter fusing 50 Hz wheel velocity, 100 Hz 9-DoF IMU angular rates, and visual odometry, **curtailing cumulative trajectory drift by 83.9%** (ATE RMSE reduced to 0.46m).*
  - *Tuned Nav2 costmaps with Smac (A\*) global planner and DWB local trajectory controller, orchestrating obstacle avoidance in a custom Gazebo subterranean bunker environment.*
  - *Implemented an autonomous mission executive using ROS 2 Action Clients (`NavigateToPose`) alongside a fail-safe watchdog node to detect visual feature dropouts and wheel slip.*
