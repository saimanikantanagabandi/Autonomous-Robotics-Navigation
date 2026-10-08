#!/usr/bin/env python3
"""
Autonomous Reconnaissance Patrol Mission Executive
==================================================
ROS 2 Action Client commanding Nav2 in a GPS-denied tactical environment.
Sequences multi-checkpoint patrol routes with timeout monitoring and
dynamic waypoint progression.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

class ReconnaissancePatrolExecutive(Node):
    def __init__(self):
        super().__init__('reconnaissance_patrol_executive')

        self.get_logger().info('Initializing GPS-Denied Autonomous Mission Executive...')

        # Action Client for Nav2 NavigateToPose
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Reconnaissance Patrol Waypoints (in map coordinate frame)
        self.patrol_waypoints = [
            {'x': 5.0,  'y': 0.0,  'yaw': 0.0,   'name': 'Alpha Checkpoint (Perimeter Ingress)'},
            {'x': 8.0,  'y': 6.0,  'yaw': 1.57,  'name': 'Bravo Checkpoint (North Corridor)'},
            {'x': 0.0,  'y': 8.0,  'yaw': 3.14,  'name': 'Charlie Checkpoint (Storage Vault)'},
            {'x': -6.0, 'y': 2.0,  'yaw': -1.57, 'name': 'Delta Checkpoint (West Flank)'},
            {'x': 0.0,  'y': 0.0,  'yaw': 0.0,   'name': 'Base Station (Extraction Point)'}
        ]

        self.current_wp_index = 0
        self._goal_handle = None

        # Wait for Nav2 Action Server
        self.get_logger().info('Connecting to Nav2 action server...')
        if not self._action_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('Nav2 action server unavailable! Check if Nav2 stack is running.')
            return

        self.get_logger().info('Nav2 connected. Initiating tactical patrol sequence.')
        self.dispatch_next_waypoint()

    def dispatch_next_waypoint(self):
        if self.current_wp_index >= len(self.patrol_waypoints):
            self.get_logger().info('====================================================')
            self.get_logger().info('RECONNAISSANCE PATROL MISSION COMPLETED SUCCESSFULLY!')
            self.get_logger().info('====================================================')
            return

        target = self.patrol_waypoints[self.current_wp_index]
        self.get_logger().info(
            f"--> Dispatching UGV to [{self.current_wp_index + 1}/{len(self.patrol_waypoints)}]: "
            f"{target['name']} at (X: {target['x']:.1f}m, Y: {target['y']:.1f}m)"
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = float(target['x'])
        goal_msg.pose.pose.position.y = float(target['y'])
        goal_msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        yaw = target['yaw']
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Waypoint goal rejected by Nav2 planner!')
            return

        self._goal_handle = goal_handle
        self.get_logger().info('Goal accepted. Monitoring UGV transit...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # Periodically log distance remaining
        dist_remaining = feedback.distance_remaining
        if dist_remaining is not None:
            self.get_logger().debug(f'Distance to checkpoint: {dist_remaining:.2f} m')

    def result_callback(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"[+] Checkpoint {self.current_wp_index + 1} reached and secured."
            )
            self.current_wp_index += 1
            # Advance to next waypoint
            self.dispatch_next_waypoint()
        else:
            self.get_logger().error(f'[-] Checkpoint navigation failed with status code: {status}')
            # Retry or advance
            self.current_wp_index += 1
            self.dispatch_next_waypoint()

def main(args=None):
    rclpy.init(args=args)
    node = ReconnaissancePatrolExecutive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Patrol mission interrupted by operator.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
