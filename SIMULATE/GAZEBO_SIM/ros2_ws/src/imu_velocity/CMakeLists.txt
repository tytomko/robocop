cmake_minimum_required(VERSION 3.8)
project(imu_velocity)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(imu_velocity_node src/imu_velocity_node.cpp)
ament_target_dependencies(imu_velocity_node rclcpp sensor_msgs geometry_msgs)

install(TARGETS
  imu_velocity_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
