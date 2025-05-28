cmake_minimum_required(VERSION 3.8)
project(firmware_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Install Python scripts
install(
  PROGRAMS
    scripts/slam_mapper.py
    scripts/transform_publisher.py
    scripts/static_slam.py
    scripts/updated_static_slam.py
    scripts/testing/revised_updated_static_slam.py
    scripts/testing/pose_publisher.py
    scripts/testing/pose_through_lidar.py
    # scripts/testing/pose_through_lidar_w_mir_fix.py
    scripts/testing/box_finding_and_nav.py
    scripts/testing/motor_speed_calc.py
    scripts/testing/new_box_nav.py
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install RViz config files
install(
  DIRECTORY
    rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # Skip copyright checks since they haven't been added yet
    set(ament_cmake_copyright_FOUND TRUE)
    # Skip cpplint since this is not in a git repo yet
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
