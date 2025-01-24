cmake_minimum_required(VERSION 3.5)
project(path_saver)

 

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Build the executable
add_executable(path_saver_node src/path_saver_node.cpp)
ament_target_dependencies(path_saver_node rclcpp geometry_msgs)

# Install the executable
install(TARGETS
  path_saver_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
