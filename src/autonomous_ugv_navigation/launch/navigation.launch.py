import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_nav = get_package_share_directory('autonomous_ugv_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    params_file = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        nav2_launch
    ])
