# GAZEBO_SIM 프로젝트 README

## 0. 개요
이 문서는 GAZEBO_SIM 프로젝트의 사전설치 라이브러리, 파일 구조, 그리고 각 노드들의 역할에 대해 설명합니다.

---

## 1. 사전설치 라이브러리

프로젝트에서 사용되는 주요 라이브러리는 다음과 같습니다:

1. **libgeographic-dev**  
   - UTM 좌표 변환용 C++ 라이브러리

2. **networkx**  
   - 그래프 구조 파이썬 라이브러리

3. **ros-humble-gazebo-***  
   - GAZEBO 시뮬레이션 관련 라이브러리

4. **ros-humble-velodyne***  
   - velodyne 관련 라이브러리

5. **mrpt-apps**  
   - velodyne 관련 라이브러리

6. **Eigen3**
   - ???
---

## 2. 파일 구조

프로젝트의 파일 구조는 아래와 같습니다:

```plaintext
…/SIMULATE/GAZEBO_SIM
└── ssafy_ws
    ├── src
    │   ├── ~~map_maker_pkg~~
    │   │   └── map_maker.cpp
    │   ├── robot_control_pkg
    │   │   ├── joystick_control.cpp
    │   │   ├── key_publisher.cpp
    │   │   ├── middle_teleop_node.cpp
    │   │   └── robot_patrol.cpp
    │   ├── robot_custom_inerfaces
    │   │   ├── msg
    │   │   │   └── Status.msg
    │   │   └── srv
    │   │       ├── Estop.srv
    │   │       ├── Homing.srv
    │   │       ├── Manual.srv
    │   │       ├── Navigate.srv
    │   │       ├── Patrol.srv
    │   │       └── Waiting.srv
    │   ├── robot_planning_pkg
    │   │   └── global_path_planner.py
    │   ├── robot_status_publisher
    │   │   └── one_robot_status_pub.cpp
    │   ├── robot_vision_pkg
    │   │   └── velodyne_detection.cpp
    │   ├── total_launch_pkg
    │   │   └── launch
    │   │       └── ssafy_robot_launch.py
    │   ├── rosbridge_suite
    │   ├── turtlebot3_description
    │   ├── turtlebot3_simulations
    │   └── velodyne_simulator
    └── maps
        ├── global_map_maker.py
        ├── global_map.json
        ├── json_viewer.py
        ├── path_to_png.py
        └── robot_plot.py
```
---

## 3. 노드 설명
각 노드들의 역할은 아래와 같습니다.

1. **robot_control_pkg**
   1) **joystick_control.cpp**  
      - `/joy` 토픽을 받아 `/cmd_vel`을 발행하는 노드로, 조이스틱으로 로봇 조작을 구현합니다.
   2) **key_publisher.cpp**  
      - 키보드 입력을 string 타입의 명령으로 변환하여 토픽을 발행하는 노드입니다.
   3) **middle_teleop_node.cpp**  
      - string 타입의 명령 토픽을 받아 `/cmd_vel`을 발행하는 노드로, 원격 조작용으로 사용됩니다.
   4) **robot_patrol.cpp**  
      - 로봇이 전달받은 경로를 따라 이동하도록 Pure-pursuit 알고리즘을 구현한 노드입니다.

2. **robot_custom_inerfaces**
   1) **msg**
      1) **Status.msg**  
         - 로봇의 상태 정보를 담는 커스텀 메시지입니다.
   2) **srv**
      1) **Estop.srv**  
         - 로봇 정지를 위한 서비스입니다.
      2) **Homing.srv**  
         - 로봇 귀환을 위한 서비스입니다.
      3) **Manual.srv**  
         - 메뉴얼 조작을 위한 서비스입니다.
      4) **Navigate.srv**  
         - 네비게이션을 위한 서비스입니다.
      5) **Patrol.srv**  
         - 순찰을 위한 서비스입니다.
      6) **Waiting.srv**  
         - 대기를 위한 서비스입니다.

3. **robot_planning_pkg**
   1) **global_path_planner.py**  
      - A* 알고리즘을 사용해 목적지까지의 최단 경로를 계산하는 노드입니다.  
      - 순찰(Patrol) 모드일 경우, 순찰 경로까지의 Approach_path와 Global_path를 계산합니다.

4. **robot_status_publisher**
   1) **one_robot_status_pub.cpp**  
      - 로봇의 상태를 발행하며, 서비스를 직접 입력받아 로봇의 상태 변경 및 다른 노드 실행을 수행합니다.

5. **robot_vision_pkg**
   1) **velodyne_detection.cpp**  
      - 3D 라이다 센서를 이용해 전방의 물체를 탐지하고, 정지 명령을 내리는 노드입니다.

6. **total_launch_pkg**
   1) **launch**
      1) **ssafy_robot_launch.py**  
         - 로봇별 파라미터를 입력받아 전체 노드를 실행하는 런치 파일입니다.

7. **rosbridge_suite**  
   - ROS를 웹소켓을 통해 백엔드와 연결하는 패키지입니다.

8. **turtlebot3_description**  
   - 터틀봇3을 GAZEBO에서 실행하기 위한 패키지입니다.

9. **turtlebot3_simulations**  
   - 터틀봇3 GAZEBO 시뮬레이션에 사용되는 패키지입니다.

10. **velodyne_simulator**  
    - 벨로다인을 GAZEBO 상에서 실행하기 위한 패키지입니다.

11. **maps**
    1) **global_map_maker.py**  
       - 로봇이 이동 가능한 경로를 Graph 형태의 JSON 파일로 저장하는 노드입니다.
    2) **global_map.json**  
       - 노드 간의 엣지 구조를 나타내는 JSON 파일입니다.
    3) **json_viewer.py**  
       - JSON 파일을 matplotlib을 이용해 시각화하는 노드입니다.
    4) **path_to_png.py**  
       - 경로 정보를 PNG 파일로 변환하는 노드입니다.
    5) **robot_plot.py**  
       - 로봇의 현재 위치, 목표점, 바라보는 방향 등을 시각화하는 노드입니다.
       
---

## 4. 실행 방법

### 가제보 실행
```bash
ros2 launch turtlebot3_gazebo tb3_imu_lidar_gps_burger.launch.py
```

### 런치파일실행 로봇별로실행
```bash
ros2 launch total_launch_pkg ssafy_robot_launch.py robot_name:=ssafy robot_number:=1
```
```bash
ros2 launch total_launch_pkg ssafy_robot_launch.py robot_name:=samsung robot_number:=2
```
```bash
ros2 launch total_launch_pkg ssafy_robot_launch.py robot_name:=charging robot_number:=3
```
```bash
ros2 launch total_launch_pkg ssafy_robot_launch.py robot_name:=champ robot_number:=4
```
### 키보드입력(먼저 메뉴얼모드로변경)
```bash
ros2 run robot_control_pkg key_publisher --ros-args -p robot_name:=ssafy -p robot_number:=1
```

### 호밍서비스
```bash
ros2 service call /robot_1/homing robot_custom_interfaces/srv/Homing
```

### 네비게이트서비스
```bash
ros2 service call /robot_1/navigate robot_custom_interfaces/srv/Navigate "{goal: {x: 304401.2780476108, y: 3892837.505477577, theta: 0.0}}"
```
                
### 패트롤 서비스
```bash
ros2 service call /robot_1/patrol robot_custom_interfaces/srv/Patrol "{goals: [{x: 304401.2780476108, y: 3892837.505477577, theta: 0.0}, {x: 304417.68651993107, y: 3892838.6313477717, theta: 0.0}, {x: 304417.91822496144, y: 3892850.079599839, theta: 0.0}]}"
```
### Estop 서비스
```bash
ros2 service call /robot_1/stop robot_custom_interfaces/srv/Estop
```

### Temp_stop 서비스
```bash
ros2 service call /robot_1/temp_stop robot_custom_interfaces/srv/Estop
```

### Resume 서비스
```bash
ros2 service call /robot_1/resume robot_custom_interfaces/srv/Estop
```

### waiting 서비스
```bash
ros2 service call /robot_1/waiting robot_custom_interfaces/srv/Waiting
```

### Manual 서비스(모드변경)
```bash
ros2 service call /robot_1/manual robot_custom_interfaces/srv/Manual 
```

### 로봇 여러 대 동시 주행
```bash
ros2 service call /robot_2/patrol robot_custom_interfaces/srv/Patrol "{goals: [{x: 304396.57, y: 3892844.46, theta: 0.0}, {x: 304401.15, y: 3892849.86, theta: 0.0}]}"
```

```bash
ros2 service call /robot_1/patrol robot_custom_interfaces/srv/Patrol "{goals: [{x: 304417.70, y: 3892839.59, theta: 0.0}, {x: 304417.77, y: 3892845.46, theta: 0.0}, {x: 304408.34, y: 3892851.10, theta: 0.0}]}"
```

```bash
ros2 service call /robot_3/patrol robot_custom_interfaces/srv/Patrol "{goals: [{x: 304401.85, y: 3892835.85, theta: 0.0}, {x: 304417.43, y: 3892837.16, theta: 0.0}]}"
```

### [수정된 사항]
1. **섹션 제목 추가**  
   - 기존 내용을 섹션 5로 구성하여, 커스텀 메시지 사용 방법 및 필수 설치 항목을 한 눈에 확인할 수 있도록 구성했습니다.

2. **구조화된 내용 정리**  
   - 커스텀 메시지 사용 방법을 CMakeLists.txt와 package.xml 부분으로 나누어 설명했습니다.
   - C++ 실행 파일 추가 부분도 별도로 구분하여 명시했습니다.
   - 설치해야 하는 항목은 별도의 하위 섹션으로 정리하여, 설치 명령어들을 코드 블록으로 표시했습니다.

---

## 5. 커스텀 메시지 사용 및 필수 설치 항목

가제보 SIM 폴더 안에 SSAFY_WS가 있으며, 그 안의 `robot_custom_interfaces` 패키지에서 커스텀 메시지를 정의합니다.

### 5.1. 커스텀 메시지 사용

커스텀 메시지를 사용하려면 아래와 같이 설정해야 합니다.

### CMakeLists.txt

먼저, `robot_custom_interfaces` 패키지를 찾기 위해 아래 코드를 추가합니다.
```

```cmake
find_package(robot_custom_interfaces REQUIRED)
```

C++ 파일을 작성하고 실행하려면, Python의 `setup.py`를 수정하는 것처럼 아래 내용을 추가해야 합니다.

```cmake
add_executable(robot_status_pub src/robot_status_pub.cpp)
ament_target_dependencies(robot_status_pub rclcpp std_msgs robot_custom_interfaces)
install(TARGETS
    robot_status_pub
    DESTINATION lib/${PROJECT_NAME})
```

#### package.xml

`package.xml` 파일에 아래 내용을 추가하여, `robot_custom_interfaces` 패키지에 대한 의존성을 명시합니다.

```xml
<depend>robot_custom_interfaces</depend>
```

### 5.2. 설치해야 하는 항목

프로젝트 실행을 위해 아래 명령어들로 필수 라이브러리 및 패키지를 설치합니다.

```bash
sudo apt-get update
sudo apt-get install libgeographic-dev
```

```bash
sudo pip3 install scikit-learn
sudo pip3 install networkx
pip install "numpy<1.25.0"
```

```bash
sudo apt-get update
sudo apt-get install libeigen3-dev
```

```bash
sudo apt-get update
sudo apt-get install ros-humble-geographic-msgs
```

---
