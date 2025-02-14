cmake_minimum_required(VERSION 3.8)
project(robot_status_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_custom_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
# GeographicLib 추가 
# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

# 파일 추가시 CMakelist.txt에 추가

add_executable(robot_status_publisher src/one_robot_status_pub.cpp)
ament_target_dependencies(robot_status_publisher
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  robot_custom_interfaces
  tf2
  # tf2_geometry_msgs 등
)
target_link_libraries(robot_status_publisher
  /lib/x86_64-linux-gnu/libGeographic.so
)

add_executable(imu_velocity_node src/imu_velocity_node.cpp)
ament_target_dependencies(imu_velocity_node rclcpp sensor_msgs geometry_msgs)

install(TARGETS
  robot_status_publisher
  imu_velocity_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
