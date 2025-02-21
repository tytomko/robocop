cmake_minimum_required(VERSION 3.5)
project(middle_teleop)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(middle_teleop_node src/middle_teleop_node.cpp)
ament_target_dependencies(middle_teleop_node rclcpp std_msgs geometry_msgs)

install(TARGETS
  middle_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
