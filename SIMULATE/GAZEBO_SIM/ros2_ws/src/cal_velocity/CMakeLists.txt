cmake_minimum_required(VERSION 3.8)
project(cal_velocity)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Executable
add_executable(cal_velocity_node src/cal_velocity_node.cpp)

# Link libraries
ament_target_dependencies(cal_velocity_node
  rclcpp
  std_msgs
  geometry_msgs
)

# Install targets
install(TARGETS
  cal_velocity_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
