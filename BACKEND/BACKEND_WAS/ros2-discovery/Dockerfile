FROM ros:humble

# 필요한 패키지 설치
RUN apt-get update && apt-get install -y \
    ros-humble-fastrtps \
    ros-humble-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

# 환경변수 설정
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

