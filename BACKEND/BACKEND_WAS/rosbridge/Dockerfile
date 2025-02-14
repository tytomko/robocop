FROM ros:humble

# 시스템 업데이트 및 필요 패키지 설치
RUN apt-get update && apt-get install -y \
    ros-humble-rosbridge-server \
    ros-humble-rmw-fastrtps-cpp \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

COPY test_publisher.py /ros2_ws/
RUN chmod +x /ros2_ws/test_publisher.py



# ROS2 환경 설정
COPY ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

# 작업 디렉토리 설정
WORKDIR /ros2_ws

# 환경변수 설정
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

