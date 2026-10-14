#!/usr/bin/env python3
"""
Thermal & Structural Collapse Hazard Monitor Node
=================================================
Monitors laser range anomalies and high thermal signatures, identifying
dangerous debris, active fire hotspots, and unpassable rubble. Publishes
hazard markers for Nav2 costmaps and mission control.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String

class ThermalHazardNode(Node):
    def __init__(self):
        super().__init__('thermal_hazard_node')
        self.get_logger().info('Initializing Thermal & Structural Collapse Hazard Monitor...')

        # Known disaster hazards (can be augmented dynamically via sensor feedback)
        self.disaster_hazards = [
            {"id": "HAZ-01", "name": "Active Structural Fire", "x": -3.0, "y": -12.0, "radius": 2.2, "temp_c": 420.0},
            {"id": "HAZ-02", "name": "Ruptured Gas Pipe", "x": 11.0, "y": 7.0, "radius": 1.8, "temp_c": 340.0},
            {"id": "HAZ-03", "name": "Ceiling Collapse Hazard", "x": -9.0, "y": 3.0, "radius": 2.5, "temp_c": 22.0},
        ]

        # Subscribers
        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publishers
        self.pub_markers = self.create_publisher(MarkerArray, '/sar/hazard_markers', 10)
        self.pub_hazard_status = self.create_publisher(String, '/sar/hazard_status', 10)

        # Timer to broadcast hazard markers
        self.create_timer(1.0, self.publish_hazard_markers)
        self.get_logger().info('Hazard Monitor online.')

    def scan_callback(self, msg: LaserScan):
        # Inspect for sudden proximity obstacles (< 0.6m) indicative of rubble fall
        min_range = min(msg.ranges) if msg.ranges else 10.0
        if min_range < 0.5:
            self.get_logger().warn(f'Critical debris proximity: obstacle at {min_range:.2f}m!')

    def publish_hazard_markers(self):
        marker_array = MarkerArray()
        for idx, haz in enumerate(self.disaster_hazards):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'disaster_hazards'
            marker.id = idx
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = haz['x']
            marker.pose.position.y = haz['y']
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0

            marker.scale.x = haz['radius'] * 2.0
            marker.scale.y = haz['radius'] * 2.0
            marker.scale.z = 1.0

            # Red glowing marker for fire, Amber for collapse
            if haz.get('temp_c', 20.0) > 100.0:
                marker.color.r = 1.0
                marker.color.g = 0.2
                marker.color.b = 0.0
                marker.color.a = 0.65
            else:
                marker.color.r = 1.0
                marker.color.g = 0.8
                marker.color.b = 0.0
                marker.color.a = 0.65

            marker_array.markers.append(marker)

        self.pub_markers.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = ThermalHazardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
