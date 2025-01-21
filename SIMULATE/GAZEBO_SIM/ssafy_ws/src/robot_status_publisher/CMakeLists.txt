cmake_minimum_required(VERSION 3.8)
project(robot_status_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_custom_interfaces REQUIRED)

# 파일 추가시 CMakelist.txt에 추가
add_executable(robot_status_pub src/robot_status_pub.cpp)
add_executable(one_robot_stat src/one_robot_status_pub.cpp)
ament_target_dependencies(one_robot_stat robot_status_pub rclcpp std_msgs robot_custom_interfaces)
install(TARGETS
  one_robot_stat
  robot_status_pub
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
