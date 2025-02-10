cmake_minimum_required(VERSION 3.8)
project(robot_control_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(robot_custom_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)

# 실행파일
add_executable(robot_patrol src/robot_patrol.cpp)
ament_target_dependencies(robot_patrol 
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
  tf2_eigen
  robot_custom_interfaces
)

add_executable(middle_teleop_node src/middle_teleop_node.cpp)
ament_target_dependencies(middle_teleop_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  robot_custom_interfaces
  tf2
  # tf2_geometry_msgs 등
)

# 실행 파일 추가
add_executable(key_publisher src/key_publisher.cpp)
# 실행 파일에 필요한 의존성 추가
ament_target_dependencies(key_publisher rclcpp std_msgs)

# 실행 파일 추가
add_executable(joystick_control src/joystick_control.cpp)
# 실행 파일에 필요한 의존성 추가
ament_target_dependencies(joystick_control 
  rclcpp 
  std_msgs
  sensor_msgs
  geometry_msgs
  robot_custom_interfaces
  tf2
  tf2_geometry_msgs
  tf2_eigen
)


install(TARGETS
  robot_patrol
  middle_teleop_node
  key_publisher
  joystick_control
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
