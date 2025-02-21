FROM ros:humble

# 시스템 업데이트 및 필요 패키지 설치
RUN apt-get update && apt-get install -y \
    ros-humble-rosbridge-server \
    ros-humble-rmw-fastrtps-cpp \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# 작업 디렉토리 설정
WORKDIR /ros2_ws

# 커스텀 메시지 패키지 복사
COPY src/robot_custom_interfaces src/robot_custom_interfaces

# ROS2 환경 설정 및 빌드
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \
    source install/setup.bash"

# 노드와 런치 파일 복사
COPY robot_publisher.py .
COPY cmd_vel_subscriber.py .
COPY robot_bridge.launch.xml .
RUN chmod +x robot_publisher.py cmd_vel_subscriber.py

# ROS2 환경 설정
COPY ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

# 환경변수 설정
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# entrypoint 수정
RUN echo 'source "/ros2_ws/install/setup.bash"' >> /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "robot_bridge.launch.xml"]

