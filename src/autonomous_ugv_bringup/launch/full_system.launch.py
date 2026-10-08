import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('autonomous_ugv_bringup')
    pkg_gazebo = get_package_share_directory('autonomous_ugv_gazebo')
    pkg_nav = get_package_share_directory('autonomous_ugv_navigation')
    pkg_desc = get_package_share_directory('autonomous_ugv_description')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz_config_file = os.path.join(pkg_desc, 'rviz', 'view_robot.rviz')

    # 1. Gazebo Simulation & Robot State Publisher
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
        )
    )

    # 2. EKF Sensor Fusion (robot_localization)
    launch_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'localization.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 3. RTAB-Map Visual SLAM
    launch_vslam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'vslam.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 4. Nav2 Autonomous Navigation Stack
    launch_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 5. RViz2 Visualization Node
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        launch_gazebo,
        launch_localization,
        launch_vslam,
        launch_navigation,
        node_rviz
    ])
