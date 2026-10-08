import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Visual Odometry Node from RTAB-Map
    rgbd_odometry_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        output='screen',
        parameters=[{
            'frame_id': 'base_footprint',
            'odom_frame_id': 'vo_odom',
            'publish_tf': false, # Published via robot_localization EKF
            'approx_sync': true,
            'queue_size': 10,
            'use_sim_time': use_sim_time,
            'Vis/FeatureType': '6',       # 6 = FAST / BRIEF (low CPU overhead for edge computing)
            'Vis/MaxFeatures': '600',
            'Odom/Strategy': '0',        # 0 = Frame-to-Map, 1 = Frame-to-Frame
            'Odom/ResetCountdown': '1',   # Automatically re-initialize on lost tracking
        }],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('depth/image', '/camera/depth/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('odom', '/vo/odom')
        ]
    )

    # RTAB-Map SLAM Node (Generates 2D occupancy grid & detects loop closures)
    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'subscribe_depth': true,
            'subscribe_scan': true,
            'approx_sync': true,
            'queue_size': 10,
            'use_sim_time': use_sim_time,
            'RGBD/ProximityBySpace': 'true',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.1',
            'RGBD/OptimizeFromGraphEnd': 'false',
            'Reg/Strategy': '0',          # Visual SLAM loop closure
            'Grid/RayTracing': 'true',
            'Grid/3D': 'false',           # 2D Occupancy Grid for Nav2
            'Grid/NormalsSegmentation': 'false',
            'Grid/RangeMax': '8.0',
            'Mem/IncrementalMemory': 'true'
        }],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('depth/image', '/camera/depth/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('scan', '/scan'),
            ('odom', '/odometry/filtered'), # Consume EKF-fused odometry
            ('grid_map', '/map')
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        rgbd_odometry_node,
        rtabmap_slam_node
    ])
