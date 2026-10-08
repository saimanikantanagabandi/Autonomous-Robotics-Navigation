from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autonomous_ugv_mission',
            executable='gps_denied_monitor',
            name='gps_denied_supervisor',
            output='screen'
        ),
        Node(
            package='autonomous_ugv_mission',
            executable='patrol_mission_node',
            name='reconnaissance_patrol_executive',
            output='screen'
        ),
    ])
