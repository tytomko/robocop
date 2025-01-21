cmake_minimum_required(VERSION 3.5)
project(keyboard_teleop)

# ROS 2 패키지 의존성 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)

# 실행 파일 생성
add_executable(keyboard_teleop_node src/keyboard_teleop_node.cpp)

# 의존성 설정
ament_target_dependencies(keyboard_teleop_node rclcpp geometry_msgs std_msgs gazebo_ros)

# 설치 위치 설정
install(TARGETS keyboard_teleop_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
