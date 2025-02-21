FROM ros:humble

# FastDDS 도구 설치
RUN apt-get update && apt-get install -y \
    ros-humble-fastrtps \
    ros-humble-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

# 환경변수 설정
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Discovery Server 실행
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && fastdds discovery -i 0 -l 52.79.51.253 -p 11811"]
