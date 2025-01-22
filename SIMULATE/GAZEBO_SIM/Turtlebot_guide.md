# GAZEBO_SIM

## 1. 클론
```bash
git clone https://github.com/Carpediem324/GAZEBO_SIM.git
```

## 2. 빌드
```bash
cd ~/GAZEBO_SIM/ssafy_ws && colcon build
```

## 3. 소싱
```bash
source install/setup.bash
```

## 4. 모델 경로 설정
```bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/ubuntu/GAZEBO_SIM/ssafy_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/
```
위 내용에서 경로를 자신의 위치로 맞춰서 `~/.bashrc`에 추가하세요. 아래 예시

### 예시
```bash
echo 'export ROS_DOMAIN_ID=30 # TURTLEBOT3' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/ubuntu/GAZEBO_SIM/ssafy_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/' >> ~/.bashrc
source ~/.bashrc
```

## 5. 실행
```bash
ros2 launch turtlebot3_gazebo tb3_imu_lidar_gps_burger.launch.py
```


# Trouble Shooting

## 1. No module named 'catkin_pkg'

### 1) Conda를 사용하는 경우
```bash
conda install -c auto catkin_pkg
```

### 2) 일반 환경
```bash
pip install catkin_pkg
```

## 2. Failed   <<< velodyne_gazebo_plugins [33.2s, exited with code 2]

### conda랑 충돌남
~/.bashrc에 콘다 관련 전부 주석처리 후 재부팅


### 벨로다인 패키지 설치
```bash
sudo apt install ros-humble-velodyne*
```

```bash
sudo apt install mrpt-apps
```
```bash
velodyne-view
```

### 테스트
```bash
ros2 launch velodyne_description example.launch.py
```

### 가제보 추가파일 설치
```bash
sudo apt-get install ros-humble-gazebo-*
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

```bash
sudo apt install ros-humble-dynamixel-sdk
sudo apt install ros-humble-turtlebot3-msgs
sudo apt install ros-humble-turtlebot3
```

### 가제보설치
```bash
sudo apt-get install ros-humble-ros-gz
```

왜 ?