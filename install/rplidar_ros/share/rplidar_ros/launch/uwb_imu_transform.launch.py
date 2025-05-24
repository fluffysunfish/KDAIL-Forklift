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
    
    # Launch the UWB position publisher
    uwb_publisher_node = Node(
        package='rplidar_ros',
        executable='uwb_publisher.py',
        name='uwb_publisher',
        output='screen'
    )
    
    # Launch the IMU yaw publisher
    imu_publisher_node = Node(
        package='rplidar_ros',
        executable='imu_publisher.py',
        name='imu_publisher',
        output='screen'
    )
    
    # Launch the UWB transform publisher
    uwb_transform_node = Node(
        package='rplidar_ros',
        executable='uwb_transform_publisher.py',
        name='uwb_transform_publisher',
        output='screen'
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
        uwb_publisher_node,    # Publish UWB coordinates
        imu_publisher_node,    # Publish IMU yaw angle
        uwb_transform_node,    # Create transform from UWB and IMU data
        static_slam_node,      # Run SLAM
        rviz_node              # Visualization
    ])