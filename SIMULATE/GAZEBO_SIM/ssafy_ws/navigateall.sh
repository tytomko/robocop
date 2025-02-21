#!/bin/bash
# 각 로봇이 목표 좌표로 이동하도록 ros2 service call을 실행하는 스크립트입니다.
# 각 명령은 해당 로봇의 /navigate 서비스를 호출하여 goal 좌표를 전달합니다.
# 모든 명령을 동시에 실행하도록 각 명령어 끝에 '&'를 추가하고, 마지막에 'wait'로 모든 작업이 완료될 때까지 대기합니다.

ros2 service call /robot_8/navigate robot_custom_interfaces/srv/Navigate "{goal: {x: 304412.42, y: 3892854.98, theta: 0.0}}" &
ros2 service call /robot_7/navigate robot_custom_interfaces/srv/Navigate "{goal: {x: 304420.68, y: 3892850.80, theta: 0.0}}" &
ros2 service call /robot_6/navigate robot_custom_interfaces/srv/Navigate "{goal: {x: 304423.03, y: 3892842.65, theta: 0.0}}" &
ros2 service call /robot_5/navigate robot_custom_interfaces/srv/Navigate "{goal: {x: 304419.08, y: 3892835.34, theta: 0.0}}" &
ros2 service call /robot_4/navigate robot_custom_interfaces/srv/Navigate "{goal: {x: 304410.12, y: 3892832.35, theta: 0.0}}" &
ros2 service call /robot_3/navigate robot_custom_interfaces/srv/Navigate "{goal: {x: 304404.22, y: 3892835.48, theta: 0.0}}" &
ros2 service call /robot_2/navigate robot_custom_interfaces/srv/Navigate "{goal: {x: 304401.31, y: 3892840.63, theta: 0.0}}" &
ros2 service call /robot_1/navigate robot_custom_interfaces/srv/Navigate "{goal: {x: 304405.67, y: 3892849.96, theta: 0.0}}" &

wait

