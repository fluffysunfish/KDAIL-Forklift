from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Include the RPLidar launch file
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rplidar.launch.py']),
    )
    
    # Get the RViz configuration file
    rviz_config_dir = os.path.join(get_package_share_directory('rplidar_ros'), 'rviz', 'slam.rviz')
    
    # Launch the sensor data simulator
    sensor_simulator_node = Node(
        package='rplidar_ros',
        executable='sensor_data_simulator.py',
        name='sensor_data_simulator',
        output='screen',
        parameters=[{
            'movement_pattern': 'circle',
            'speed': 0.2,
            'radius': 3.0,
            'update_frequency': 10.0
        }]
    )
    
    # Launch the UWB publisher node (using real data)
    uwb_publisher_node = Node(
        package='rplidar_ros',
        executable='uwb_publisher.py',
        name='uwb_publisher',
        output='screen',
        parameters=[{
            'use_fake_data': False,  # Use real data from topics
            'publish_frequency': 20.0
        }]
    )
    
    # Launch our static SLAM node
    static_slam_node = Node(
        package='rplidar_ros',
        executable='static_slam.py',
        name='static_slam',
        output='screen'
    )
    
    # Launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )
    
    return LaunchDescription([
        rplidar_launch,
        sensor_simulator_node,  # Start the sensor simulator first
        uwb_publisher_node,     # Then the UWB publisher
        static_slam_node,       # Then the SLAM node
        rviz_node               # Finally RViz
    ])