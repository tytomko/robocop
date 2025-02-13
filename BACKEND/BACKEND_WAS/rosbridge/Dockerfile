FROM ros:humble

# rosbridge 서버 및 필요한 패키지 설치
RUN apt-get update && apt-get install -y \
    ros-humble-rosbridge-server \
    ros-humble-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
# 환경변수 설정
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

