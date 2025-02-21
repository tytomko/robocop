# GAZEBO_SIM 프로젝트 README

## 0. 개요
이 문서는 **GAZEBO_SIM** 프로젝트의 사전 설치 라이브러리, 파일 구조, 각 노드의 역할, 실행 명령어, 그리고 커스텀 메시지 사용 및 필수 설치 항목에 대해 상세히 설명합니다.

---

## 1. 사전 설치 라이브러리
프로젝트에서 사용되는 주요 라이브러리는 다음과 같습니다:

1. **libgeographic-dev**  
   - gps_common 패키지는 ROS 1용으로 개발되었으며, ROS 2용 공식 버전은 제공되지 않습니다. 
   - UTM 좌표 변환용 C++ 라이브러리

2. **networkx**  
   - 그래프 구조 파이썬 라이브러리

3. **ros-humble-gazebo-\***  
   - GAZEBO 시뮬레이션 관련 라이브러리

4. **ros-humble-velodyne\***  
   - Velodyne 관련 라이브러리

5. **mrpt-apps**  
   - Velodyne 관련 라이브러리

6. **Eigen3**  
   - ...
7. **pcl**
   - Polint cloud 관련 라이브러리

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
    │   ├── robot_ai_pkg
    │   │   └── ai_process.cpp
    │   ├── gps_hz_pkg
    │   │   └── gps.py   
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

### 3.1. robot_control_pkg
- **joystick_control.cpp**  
  `/joy` 토픽을 수신하여 `/cmd_vel`로 명령을 발행하며, 조이스틱을 통해 로봇 제어를 구현합니다.
- **key_publisher.cpp**  
  키보드 입력을 string 타입의 명령으로 변환하여 해당 토픽을 발행합니다.
- **middle_teleop_node.cpp**  
  string 타입의 명령을 수신하여 `/cmd_vel`을 발행, 원격 조작에 사용됩니다.
- **robot_patrol.cpp**  
  Pure-pursuit 알고리즘을 이용해 로봇이 지정된 경로를 따라 이동하도록 제어합니다.

### 3.2. robot_custom_inerfaces
- **msg**
  - **Status.msg**  
    로봇 상태 정보를 담는 커스텀 메시지
- **srv**
  - **Estop.srv**: 로봇 정지 서비스  
  - **Homing.srv**: 로봇 귀환 서비스  
  - **Manual.srv**: 메뉴얼 조작 서비스  
  - **Navigate.srv**: 네비게이션 서비스  
  - **Patrol.srv**: 순찰 서비스  
  - **Waiting.srv**: 대기 서비스

### 3.3. robot_planning_pkg
- **global_path_planner.py**  
  A* 알고리즘을 이용해 목적지까지의 최단 경로를 계산합니다. 순찰(Patrol) 모드일 경우, 순찰 경로까지의 Approach_path와 Global_path를 함께 계산합니다.

### 3.4. robot_status_publisher
- **one_robot_status_pub.cpp**  
  로봇의 상태를 발행하며, 서비스 호출을 통해 로봇 상태 변경 및 다른 노드 실행을 수행합니다.

### 3.5. robot_vision_pkg
- **velodyne_detection.cpp**  
  3D 라이다 센서를 이용해 전방 물체를 탐지하고, 정지 명령을 발행합니다.

### 3.6. total_launch_pkg
- **launch/ssafy_robot_launch.py**  
  로봇별 파라미터를 입력받아 전체 노드를 실행하는 런치 파일입니다.

### 3.7. 기타 패키지
- **rosbridge_suite**  
  ROS와 웹소켓을 통해 백엔드를 연결합니다.
- **turtlebot3_description**  
  터틀봇3 모델을 GAZEBO에서 실행하기 위한 패키지입니다.
- **turtlebot3_simulations**  
  터틀봇3 GAZEBO 시뮬레이션 관련 패키지입니다.
- **velodyne_simulator**  
  GAZEBO에서 벨로다인 시뮬레이터를 실행하기 위한 패키지입니다.

### 3.8. maps
- **global_map_maker.py**  
  로봇이 이동 가능한 경로를 Graph 형태의 JSON 파일로 저장합니다.
- **global_map.json**  
  노드 간 엣지 구조를 나타내는 JSON 파일입니다.
- **json_viewer.py**  
  matplotlib을 이용하여 JSON 파일을 시각화합니다.
- **path_to_png.py**  
  경로 정보를 PNG 파일로 변환합니다.
- **robot_plot.py**  
  로봇의 현재 위치, 목표점, 및 진행 방향을 시각화합니다.

---

## 4. 모델 경로 설정
```bash
export ROS_DOMAIN_ID=30
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/ubuntu/S12P11C101/SIMULATE/GAZEBO_SIM/ssafy_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/
```
위 내용에서 경로를 자신의 위치로 맞춰서 `~/.bashrc`에 추가하세요. 아래 예시

### 예시
```bash
echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/ubuntu/S12P11C101/SIMULATE/GAZEBO_SIM/ssafy_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/' >> ~/.bashrc
source ~/.bashrc
```

## 5. 실행 명령어

### 5.1. 가제보 실행

모델불러오는데 오래걸림.
```bash
ros2 launch turtlebot3_gazebo ssafy_office.launch.py 
```

### 5.2. 런치 파일 실행 (로봇별)
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

### 5.3. 키보드 입력 (메뉴얼 모드 변경 후)
```bash
ros2 run robot_control_pkg key_publisher --ros-args -p robot_name:=ssafy -p robot_number:=1
```

### 5.4. 서비스 호출 명령어

- **호밍 서비스**
  ```bash
  ros2 service call /robot_1/homing robot_custom_interfaces/srv/Homing
  ```
- **네비게이트 서비스**
  ```bash
  ros2 service call /robot_1/navigate robot_custom_interfaces/srv/Navigate "{goal: {x: 304401.2780476108, y: 3892837.505477577, theta: 0.0}}"
  ```
- **패트롤 서비스**
  ```bash
  ros2 service call /robot_1/patrol robot_custom_interfaces/srv/Patrol "{goals: [{x: 304401.2780476108, y: 3892837.505477577, theta: 0.0}, {x: 304417.68651993107, y: 3892838.6313477717, theta: 0.0}, {x: 304417.91822496144, y: 3892850.079599839, theta: 0.0}]}"
  ```
- **Estop 서비스**
  ```bash
  ros2 service call /robot_1/stop robot_custom_interfaces/srv/Estop
  ```
- **Temp_stop 서비스**
  ```bash
  ros2 service call /robot_1/temp_stop robot_custom_interfaces/srv/Estop
  ```
- **Resume 서비스**
  ```bash
  ros2 service call /robot_1/resume robot_custom_interfaces/srv/Estop
  ```
- **Waiting 서비스**
  ```bash
  ros2 service call /robot_1/waiting robot_custom_interfaces/srv/Waiting
  ```
- **Manual 서비스 (모드 변경)**
  ```bash
  ros2 service call /robot_1/manual robot_custom_interfaces/srv/Manual
  ```

### 5.5. 여러 대의 로봇 동시 주행 예시

- **예시 1:**
  ```bash
  ros2 service call /robot_2/patrol robot_custom_interfaces/srv/Patrol "{goals: [{x: 304396.57, y: 3892844.46, theta: 0.0}, {x: 304401.15, y: 3892849.86, theta: 0.0}]}"
  ```
- **예시 2:**
  ```bash
  ros2 service call /robot_1/patrol robot_custom_interfaces/srv/Patrol "{goals: [{x: 304417.70, y: 3892839.59, theta: 0.0}, {x: 304417.77, y: 3892845.46, theta: 0.0}, {x: 304408.34, y: 3892851.10, theta: 0.0}]}"
  ```
- **예시 3:**
  ```bash
  ros2 service call /robot_3/patrol robot_custom_interfaces/srv/Patrol "{goals: [{x: 304401.85, y: 3892835.85, theta: 0.0}, {x: 304417.43, y: 3892837.16, theta: 0.0}]}"
  ```

---

## 6. 커스텀 메시지 사용 및 필수 설치 항목

### 6.1. 커스텀 메시지 사용

GAZEBO_SIM 프로젝트 내 `ssafy_ws` 폴더 안에 위치한 `robot_custom_interfaces` 패키지에서 커스텀 메시지를 정의합니다.

#### CMakeLists.txt 수정 사항
- **패키지 찾기**  
  `robot_custom_interfaces` 패키지를 찾기 위해 아래 코드를 추가합니다.
  ```cmake
  find_package(robot_custom_interfaces REQUIRED)
  ```

- **실행 파일 추가 및 의존성 설정**  
  C++ 파일을 작성 후 실행하기 위해 아래와 같이 설정합니다.
  ```cmake
  add_executable(robot_status_pub src/robot_status_pub.cpp)
  ament_target_dependencies(robot_status_pub rclcpp std_msgs robot_custom_interfaces)
  install(TARGETS
      robot_status_pub
      DESTINATION lib/${PROJECT_NAME})
  ```

#### package.xml 수정 사항
- **의존성 추가**  
  `package.xml` 파일에 아래 내용을 추가하여 `robot_custom_interfaces` 패키지에 대한 의존성을 명시합니다.
  ```xml
  <depend>robot_custom_interfaces</depend>
  ```

### 6.2. 필수 설치 항목
프로젝트 실행을 위해 다음 명령어들을 사용하여 필수 라이브러리 및 패키지를 설치합니다.

ssafy_ws/ 디렉토리에서 아래명령어 수행
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

```bash
sudo pip3 install scikit-learn
sudo pip3 install networkx
sudo pip3 install catkin_pkg
sudo pip3 install "numpy<1.25.0"
```

```bash
sudo apt-get update
sudo apt-get install mrpt-apps
sudo apt-get install libeigen3-dev
sudo apt-get install ros-humble-geographic-msgs
sudo apt-get install libgeographic-dev
sudo apt-get install libpcl-dev
sudo apt-get install ros-hubmle-pcl-conversions
sudo apt-get install ros-hubmle-pcl-ros
```