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
    
    # Launch the UWB positioning node
    uwb_node = Node(
        package='rplidar_ros',
        executable='uwb_xy.py',
        name='uwb_positioning',
        output='screen',
        parameters=[{
            'x_position': 1.0,
            'y_position': 1.0,
            'z_position': 0.0,
            'yaw_angle': 0.0,
            'publish_frequency': 20.0
        }]
    )
    
    # Launch the UWB demo node
    uwb_demo_node = Node(
        package='rplidar_ros',
        executable='uwb_demo.py',
        name='uwb_demo',
        output='screen',
        parameters=[{
            'demo_type': 'circle',
            'speed': 0.5,
            'radius': 2.0,
            'update_frequency': 10.0
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
        uwb_node,  # Run UWB node before SLAM node to ensure transform is available
        uwb_demo_node,  # Run the demo to update the UWB position
        static_slam_node,
        rviz_node
    ])