#!/usr/bin/env python3
"""
ROS 2 Search & Rescue Master Mission Launch File
================================================
Launches:
  1. GPS-Denied Supervisor Monitor
  2. OpenCV Survivor Detector Node
  3. Thermal & Structural Collapse Hazard Monitor
  4. Autonomous SAR Mission Executive (Nav2 Coordinator)
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. GPS-Denied Watchdog Monitor
        Node(
            package='autonomous_ugv_mission',
            executable='gps_denied_monitor',
            name='gps_denied_supervisor',
            output='screen'
        ),
        # 2. OpenCV Survivor Detector Node
        Node(
            package='autonomous_ugv_mission',
            executable='survivor_detector_node',
            name='survivor_detector',
            output='screen',
            parameters=[{
                'confidence_threshold': 0.70,
                'camera_fov_deg': 72.0,
                'optical_focal_length': 240.0
            }]
        ),
        # 3. Thermal Hazard Monitor
        Node(
            package='autonomous_ugv_mission',
            executable='thermal_hazard_node',
            name='thermal_hazard_monitor',
            output='screen'
        ),
        # 4. Search & Rescue Autonomous Mission Executive
        Node(
            package='autonomous_ugv_mission',
            executable='sar_mission_executive',
            name='sar_mission_executive',
            output='screen'
        ),
    ])
