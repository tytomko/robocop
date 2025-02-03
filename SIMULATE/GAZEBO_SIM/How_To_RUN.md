# 1. 시뮬레이션 및 status발행코드 실행하기

## ssafy_ws 진입

빌드 후 아래명령어 실행


### 터틀봇 실행
```bash
ros2 launch turtlebot3_gazebo tb3_imu_lidar_gps_burger.launch.py
```

### status발행 코드실행
```bash
ros2 run robot_status_publisher robot_status_publisher --ros-args -p robot_name:=ssafy -p robot_number:=1
```

# 2. 로봇 조종 코드 실행하기

## ros2_ws 진입

빌드 후 아래명령어 실행

### 이동 코드
```bash
ros2 run middle_teleop middle_teleop_node --ros-args -p robot_name:=ssafy -p robot_number:=1
```

### 키보드 입력을 이동명령어로 바꾸는 코드
```bash
ros2 run keyboard_input key_publisher --ros-args -p robot_name:=ssafy -p robot_number:=1
```

# 3. 플래닝 패키지 실행


```bash
ros2 run robot_planning_pkg global_path_planner --ros-args -p robot_name:=ssafy -p robot_number:=1
```


# 4. 서비스 콜 하기

## ssafy_ws 진입

빌드 후 아래 명령어 실행

### 호밍서비스
```bash
ros2 service call /robot_1/homing robot_custom_interfaces/srv/Homing
```

### 네비게이트서비스
```bash
ros2 service call /robot_1/navigate robot_custom_interfaces/srv/Navigate "{goal: {x: 304412.94040598295, y: 3892840.5655467883, theta: 0.0}}"
```

### 패트롤 서비스
```bash
ros2 service call /robot_1/patrol robot_custom_interfaces/srv/Patrol "{goals: [{x: 304410.5991357809, y: 3892836.2283385303, theta: 0.0}, {x: 304411.54326689156, y: 3892845.3441660292, theta: 0.0}, {x: 304413.21575799957, y: 3892843.983522342, theta: 0.0}]}"
```
### Estop 서비스
```bash
ros2 service call /robot_1/stop robot_custom_interfaces/srv/Estop
```

### Resume 서비스
```bash
ros2 service call /robot_1/resume robot_custom_interfaces/srv/Resume
```
