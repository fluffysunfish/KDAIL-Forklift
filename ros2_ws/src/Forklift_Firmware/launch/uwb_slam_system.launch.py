from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    map_resolution_arg = DeclareLaunchArgument(
        'map_resolution',
        default_value='0.05',
        description='Map resolution in meters per pixel'
    )
    
    map_size_arg = DeclareLaunchArgument(
        'map_size',
        default_value='2000',
        description='Map size in pixels (width and height)'
    )
    
    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value='12.0',
        description='Maximum laser range to consider in meters'
    )
    
    yaw_offset_arg = DeclareLaunchArgument(
        'yaw_offset',
        default_value='0.0',
        description='IMU yaw offset in degrees'
    )
    
    # UWB-IMU Transform Publisher Node
    uwb_imu_transform_node = Node(
        package='Forklift_Firmware',
        executable='uwb_imu_transform_publisher.py',
        name='uwb_imu_transform_publisher',
        output='screen',
        parameters=[{
            'z_position': 0.0,
            'publish_frequency': 50.0,
            'anchor_1_x': 0.0,
            'anchor_1_y': 0.0,
            'yaw_offset': LaunchConfiguration('yaw_offset')
        }]
    )
    
    # UWB Static SLAM Node
    uwb_slam_node = Node(
        package='Forklift_Firmware',
        executable='uwb_static_slam.py',
        name='uwb_static_slam',
        output='screen',
        parameters=[{
            'map_resolution': LaunchConfiguration('map_resolution'),
            'map_width': LaunchConfiguration('map_size'),
            'map_height': LaunchConfiguration('map_size'),
            'update_frequency': 10.0,
            'occupied_threshold': 0.65,
            'free_threshold': 0.35,
            'log_odds_occupied': 0.4,
            'log_odds_free': 0.2,
            'max_range': LaunchConfiguration('max_range')
        }]
    )
    
    # RViz2 Node for visualization
    rviz_config_file = os.path.join(
        get_package_share_directory('Forklift_Firmware'),
        'rviz',
        'uwb_slam.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )
    
    return LaunchDescription([
        map_resolution_arg,
        map_size_arg,
        max_range_arg,
        yaw_offset_arg,
        uwb_imu_transform_node,
        uwb_slam_node,
        rviz_node
    ])