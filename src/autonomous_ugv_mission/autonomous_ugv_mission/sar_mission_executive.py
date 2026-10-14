#!/usr/bin/env python3
"""
Autonomous Search & Rescue Mission Executive (Nav2 Coordinator)
==============================================================
ROS 2 Action Client commanding the Nav2 stack for autonomous disaster
quadrant sweep, survivor priority triage management, and emergency
evacuation routing.
"""

import math
import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import String

class SARMissionExecutive(Node):
    def __init__(self):
        super().__init__('sar_mission_executive')
        self.get_logger().info('Initializing AI Search & Rescue Mission Executive...')

        # Action Client for Nav2 NavigateToPose
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Disaster Sector Search Waypoints
        self.sar_sectors = [
            {'x': 6.0,  'y': 3.0,  'yaw': 0.0,   'name': 'Sector 1: East Ingress & Rubble'},
            {'x': -7.0, 'y': 6.0,  'yaw': 1.57,  'name': 'Sector 2: North Collapse Corridor'},
            {'x': -13.0,'y': -9.0, 'yaw': 3.14,  'name': 'Sector 3: West Void Space'},
            {'x': 12.0, 'y': -8.0, 'yaw': -1.57, 'name': 'Sector 4: South Recovery Area'},
            {'x': 0.0,  'y': -3.0, 'yaw': 0.0,   'name': 'Base Station: Triage Staging'}
        ]

        self.current_sector_idx = 0
        self._goal_handle = None
        self.located_survivors = []

        # Subscriber to survivor alerts
        self.sub_alerts = self.create_subscription(
            String,
            '/sar/alerts',
            self.alert_callback,
            10
        )

        # Connect to Nav2
        self.get_logger().info('Connecting to Nav2 Action Server...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('Nav2 Action Server not yet active. Will retry in background...')
            self.create_timer(3.0, self.check_nav2_connection)
        else:
            self.get_logger().info('Nav2 connected. Initiating Search & Rescue sweep.')
            self.dispatch_next_sector()

    def check_nav2_connection(self):
        if self._action_client.server_is_ready():
            self.get_logger().info('Nav2 Action Server is now online. Initiating SAR sweep.')
            self.dispatch_next_sector()

    def alert_callback(self, msg: String):
        try:
            alert = json.loads(msg.data)
            self.located_survivors.append(alert)
            self.get_logger().info(
                f"[SAR EXECUTIVE] Survivor logged in casualty manifest: "
                f"Incident {alert.get('incident_id')} at ({alert['location']['x']}, {alert['location']['y']}) | "
                f"Priority: {alert.get('triage_priority')}"
            )
        except Exception as e:
            self.get_logger().error(f"Error parsing alert: {e}")

    def dispatch_next_sector(self):
        if self.current_sector_idx >= len(self.sar_sectors):
            self.get_logger().info('========================================================')
            self.get_logger().info('SEARCH & RESCUE MISSION COMPLETED: ALL SECTORS SWEPT!')
            self.get_logger().info(f'Total Survivors Located: {len(self.located_survivors)}')
            self.get_logger().info('========================================================')
            return

        target = self.sar_sectors[self.current_sector_idx]
        self.get_logger().info(
            f"--> Dispatching SAR Robot to [{self.current_sector_idx + 1}/{len(self.sar_sectors)}]: "
            f"{target['name']} at (X: {target['x']:.1f}m, Y: {target['y']:.1f}m)"
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = float(target['x'])
        goal_msg.pose.pose.position.y = float(target['y'])
        goal_msg.pose.pose.position.z = 0.0

        # Yaw to quaternion
        half_yaw = target['yaw'] * 0.5
        goal_msg.pose.pose.orientation.z = math.sin(half_yaw)
        goal_msg.pose.pose.orientation.w = math.cos(half_yaw)

        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error(f"Sector waypoint {self.current_sector_idx + 1} rejected by Nav2!")
            self.current_sector_idx += 1
            self.dispatch_next_sector()
            return

        self.get_logger().info(f"Sector {self.current_sector_idx + 1} accepted by Nav2.")
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def feedback_callback(self, feedback_msg):
        # Navigation in progress feedback
        pass

    def goal_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Sector {self.current_sector_idx + 1} sweep complete!")
        else:
            self.get_logger().warn(f"Sector {self.current_sector_idx + 1} navigation aborted or perturbed (status {status}).")

        self.current_sector_idx += 1
        self.dispatch_next_sector()

def main(args=None):
    rclpy.init(args=args)
    executive = SARMissionExecutive()
    try:
        rclpy.spin(executive)
    except KeyboardInterrupt:
        pass
    finally:
        executive.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
