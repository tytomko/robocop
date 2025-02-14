cmake_minimum_required(VERSION 3.8)
project(ukf_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# to use GeographicLib library
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

# 생성할 실행 파일에 소스 파일 추가 (ukf_node와 ukf.cpp)
add_executable(
  ukf_node
  src/ukf_node.cpp
  src/ukf.cpp
)

# ament_target_dependencies를 사용해 ROS2 의존 패키지 지정
ament_target_dependencies(ukf_node
  rclcpp
  angles
  diagnostic_msgs
  diagnostic_updater
  geographic_msgs
  geometry_msgs
  message_filters
  nav_msgs
  sensor_msgs
  std_srvs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  yaml_cpp_vendor
)

# GeographicLib와 Boost 라이브러리 링크 (Eigen3는 header-only라 링크 필요 없음)
target_link_libraries(ukf_node
  ${GeographicLib_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS
  ukf_node
  DESTINATION lib/${PROJECT_NAME}
)

# include 폴더 추가 (헤더 파일 경로)
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# 설치: include 폴더 내 헤더 파일과 params, launch 파일들을 share 폴더로 설치
install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY
  params
  launch
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 아래 두 줄은 테스트 시 cpplint, 라이선스 체크를 건너뛰기 위해 설정 (필요 시 주석 해제)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
