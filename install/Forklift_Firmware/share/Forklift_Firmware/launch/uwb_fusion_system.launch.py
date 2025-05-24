#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for UWB sensor'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baudrate for UWB sensor'
    )

    # UWB Raw Data Publisher Node
    uwb_raw_publisher = Node(
        package='your_package_name',
        executable='uwb_raw_publisher.py',
        name='uwb_raw_publisher',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate')
        }]
    )

    # UWB-IMU Fusion Node
    uwb_imu_fusion = Node(
        package='your_package_name',
        executable='uwb_imu_fusion.py',
        name='uwb_imu_fusion',
        output='screen'
    )

    # Optional: Test Monitor Node
    uwb_fusion_test = Node(
        package='your_package_name',
        executable='uwb_fusion_test.py',
        name='uwb_fusion_test',
        output='screen'
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        uwb_raw_publisher,
        uwb_imu_fusion,
        # uwb_fusion_test,  # Uncomment to run test monitor
    ])