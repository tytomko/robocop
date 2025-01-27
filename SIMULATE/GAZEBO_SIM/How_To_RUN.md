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
ros2 run keyboard_input key_publisher  --ros-args -p robot_name:=ssafy -p robot_number:=1
```