# 실행 가이드

## 스태이터스 실행 파라미터 포함 명령어

```bash
ros2 run robot_status_publisher one_robot_stat --ros-args -p robot_name:=ssafy -p robot_number:=1
```

## 맵 생성기(점 개수, 점좌표 입력하면 직선으로 잇는 csv파일 맵 생성)
```bash
ros2 run map_maker_pkg map_maker
```

## 터틀봇 실행
```bash
ros2 launch turtlebot3_gazebo tb3_imu_lidar_gps_burger.launch.py
```


가제보 SIM 폴더 안에 SSAFY_WS가 있음.

그 안에 `robot_custom_interfaces` 패키지에 커스텀 메시지를 정의함.

커스텀 메시지를 사용하려면:

### `CMakeLists.txt`에 아래 executable 추가

```cmake
find_package(robot_custom_interfaces REQUIRED)

```

### `package.xml`에 아래 추가

```xml
<depend>robot_custom_interfaces</depend>
```

c++ 파일 만들고 실행하려면 파이썬setup.py 수정하는것처럼 아래추가해야함
### CMakeLists.txt아래추가
```cmake
    add_executable(robot_status_pub src/robot_status_pub.cpp)
    ament_target_dependencies(robot_status_pub rclcpp std_msgs robot_custom_interfaces)
    install(TARGETS
        robot_status_pub
        DESTINATION lib/${PROJECT_NAME})
```


# 설치해야하는거

```
sudo apt-get update
sudo apt-get install libgeographic-dev
```