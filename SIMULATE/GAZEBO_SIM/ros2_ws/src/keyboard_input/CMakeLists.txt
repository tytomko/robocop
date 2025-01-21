cmake_minimum_required(VERSION 3.8)
project(keyboard_input)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 의존성 포함
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 실행 파일 추가
add_executable(key_publisher src/key_publisher.cpp)

# 실행 파일에 필요한 의존성 추가
ament_target_dependencies(key_publisher rclcpp std_msgs)

# 실행 파일 설치
install(TARGETS
  key_publisher
  DESTINATION lib/${PROJECT_NAME})

# 패키지 매크로 호출
ament_package()
