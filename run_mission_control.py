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
        "scipy": "scipy",
        "cv2": "opencv-python-headless"
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
    print(f"\n[+] Opening AEGIS-SAR Mission Control Tactical Dashboard: {url}")
    try:
        webbrowser.open(url)
    except Exception as e:
        print(f"[!] Could not open browser automatically: {e}")

def main():
    check_dependencies()

    banner = r"""
================================================================================
    ___    __________________      _____ ___    ____ 
   /   |  / ____/ ____/  _/ ___/     / ___//   |  / __ \
  / /| | / __/ / / __ / / \__ \ ____ \__ \/ /| | / /_/ /
 / ___ |/ /___/ /_/ // / ___/ /_____/__/ / ___ |/ _, _/ 
/_/  |_/_____/\____/___//____/     /____/_/  |_/_/ |_|  
     AI-POWERED SEARCH & RESCUE ROBOT // TACTICAL DISASTER RESPONSE
================================================================================
 [MISSION] Objective: Post-Disaster Survivor Detection & Extraction Triage
 [VISION]  AI Perception: OpenCV Human & Thermal Hotspot Detection (10 FPS)
 [NAV]     Autonomous Nav2 Sector Sweep, Rubble Avoidance, Hazard Monitoring
 [FUSION]  15-State EKF Sensor Fusion (Wheel Odometry + 9-DoF IMU + VSLAM)
 [ALERTS]  Real-Time Location Alerts & Safe Ingress Routing for USAR Teams
--------------------------------------------------------------------------------
 Controls & Tactical Features:
   - Live AI Camera: Toggle AI HUD, Optical RGB, Thermal IR, Night/Smoke
   - Survivor Triage: Red (Critical), Yellow (Delayed), Green (Minor)
   - 1-Click USAR Team Dispatch with automated safe extraction routes
   - Interactive 2D Radar: Click anywhere to command navigation waypoints
   - Disaster Stress Testing: Seismic Aftershocks, Gas Fires, Smoke Plumes
   - Export Casualty Manifest (JSON) and Trajectory Benchmark (CSV)
--------------------------------------------------------------------------------
 Host URL: http://127.0.0.1:8000
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
