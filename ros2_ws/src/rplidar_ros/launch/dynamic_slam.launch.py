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

    # Launch our static SLAM node
    static_slam_node = Node(
        package='rplidar_ros',
        executable='static_slam.py',
        name='static_slam',
        output='screen'
    )

    # Launch the dynamic UWB positioning node
    dynamic_uwb_node = Node(
        package='Forklift_Firmware',
        executable='dynamic_uwb_positioning.py',
        name='dynamic_uwb_positioning',
        output='screen',
        parameters=[{
            'z_position': 0.0,
            'publish_frequency': 20.0
        }]
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
        dynamic_uwb_node,  # Run dynamic UWB node before SLAM node to ensure transform is available
        static_slam_node,
        rviz_node
    ])