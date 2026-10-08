import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav = get_package_share_directory('autonomous_ugv_navigation')
    ekf_config_file = os.path.join(pkg_nav, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odometry/filtered')]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        ekf_node
    ])
