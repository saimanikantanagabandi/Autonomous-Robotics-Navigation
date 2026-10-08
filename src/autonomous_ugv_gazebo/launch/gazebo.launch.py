import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_ugv_description = get_package_share_directory('autonomous_ugv_description')
    pkg_ugv_gazebo = get_package_share_directory('autonomous_ugv_gazebo')

    # Paths
    world_file = os.path.join(pkg_ugv_gazebo, 'worlds', 'gps_denied_warehouse.world')
    xacro_file = os.path.join(pkg_ugv_description, 'urdf', 'robot.urdf.xacro')

    # Convert xacro to urdf via Command
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': robot_description_content, 'use_sim_time': True}

    # Robot State Publisher Node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Gazebo Server (gzserver)
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items()
    )

    # Gazebo Client GUI (gzclient)
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Spawn Entity Node
    spawn_ugv = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'autonomous_ugv',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-Y', '0.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_server,
        gazebo_client,
        node_robot_state_publisher,
        spawn_ugv
    ])
