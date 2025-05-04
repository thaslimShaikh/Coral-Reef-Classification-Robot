cmake_minimum_required(VERSION 3.8)
project(remus100_sim)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(gazebo_ros REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/coral_reef_navigator.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install models
install(DIRECTORY
  models/
  DESTINATION share/${PROJECT_NAME}/models
)

ament_package()
