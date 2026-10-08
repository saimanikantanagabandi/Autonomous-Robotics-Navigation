#!/usr/bin/env python3
"""
Autonomous UGV Mission Control Launcher
=======================================
Launches the full-stack Mission Control web application and automatically
opens the interactive tactical dashboard in your default browser.
"""

import sys
import time
import webbrowser
import threading

def check_dependencies():
    required = {
        "fastapi": "fastapi",
        "uvicorn": "uvicorn",
        "websockets": "websockets",
        "numpy": "numpy",
        "scipy": "scipy"
    }
    missing = []
    for mod, pkg in required.items():
        try:
            __import__(mod)
        except ImportError:
            missing.append(pkg)

    if missing:
        print("\n[!] Missing required Python packages:")
        for p in missing:
            print(f"    - {p}")
        print("\nPlease run: pip install -r requirements.txt\n")
        sys.exit(1)

def open_browser():
    time.sleep(1.2)
    url = "http://127.0.0.1:8000"
    print(f"\n[+] Opening Mission Control Tactical Dashboard: {url}")
    try:
        webbrowser.open(url)
    except Exception as e:
        print(f"[!] Could not open browser automatically: {e}")

def main():
    check_dependencies()

    banner = r"""
================================================================================
   ______  __  ________________  _   ______  __  _______  __  _______   _   __
  / ____/ / / / /_  __/ __ \/ __ \/ | / / __ \/  |/  / __ \/ / / / ___/  | | / /
 / /_    / / / / / / / / / / / / /  |/ / / / / /|_/ / / / / / / /\__ \   | |/ / 
/ __/   / /_/ / / / / /_/ / /_/ / /|  / /_/ / /  / / /_/ / /_/ /___/ /   | |/  
/_/     \____/ /_/  \____/\____/_/ |_/\____/_/  /_/\____/\____//____/    |___/   
          Autonomous UGV GPS-Denied Mission Control & Teleoperation             
================================================================================
 [STATUS] Host: http://127.0.0.1:8000
 [STATUS] Sensors: 50Hz Wheel Odom, 100Hz 9-DoF IMU, 15Hz RGB-D VSLAM, 2D LiDAR
 [STATUS] Filter : 15-State Extended Kalman Filter (robot_localization)
 [STATUS] Nav2   : A* Global Path Planning & Pure Pursuit Trajectory Tracking
--------------------------------------------------------------------------------
 Hotkeys & Controls:
   - Click anywhere on the 2D Radar to dispatch immediate tactical waypoints!
   - Drive manually with WASD or Arrow Keys | Spacebar to Emergency Stop
   - Toggle Wheel Slip, VSLAM Blackout, or IMU Bias to stress-test the EKF!
   - Export benchmark trajectory data to CSV with 1-click
--------------------------------------------------------------------------------
 Press Ctrl+C in this terminal to shut down the Mission Control server.
================================================================================
"""
    print(banner)

    # Launch browser in a background thread
    threading.Thread(target=open_browser, daemon=True).start()

    # Start Uvicorn Server
    import uvicorn
    uvicorn.run(
        "mission_control.server:app",
        host="127.0.0.1",
        port=8000,
        log_level="warning"
    )

if __name__ == "__main__":
    main()
