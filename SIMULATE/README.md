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
```
    add_executable(robot_status_pub src/robot_status_pub.cpp)
    ament_target_dependencies(robot_status_pub rclcpp std_msgs robot_custom_interfaces)
    install(TARGETS
        robot_status_pub
        DESTINATION lib/${PROJECT_NAME})
```

# 파라미터 포함 명령어

```
ros2 run robot_status_publisher one_robot_stat --ros-args -p robot_name:=ssafy -p robot_number:=1
```

# 설치해야하는거

```
sudo apt-get update
sudo apt-get install libgeographic-dev
```