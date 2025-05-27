#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('firmware_2')
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'config.rviz')

    return LaunchDescription([
        # Launch the transform publisher
        Node(
            package='firmware_2',
            executable='transform_publisher.py',
            name='transform_publisher',
            output='screen',
            parameters=[],
        ),

        # Launch the SLAM mapper
        Node(
            package='firmware_2',
            executable='slam_mapper.py',
            name='slam_mapper',
            output='screen',
            parameters=[],
        ),

        # Launch your existing LiDAR publisher
        # Node(
        #     package='firmware_2',
        #     executable='lidar_publisher.py',  # Your existing LiDAR code
        #     name='lidar_publisher',
        #     output='screen',
        #     parameters=[],
        # ),

        # Optional: Launch RViz2 with a custom config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
        ),
    ])