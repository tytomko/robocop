#!/bin/bash
set -e

# ROS2 환경 설정
source /opt/ros/humble/setup.bash

exec "$@"

