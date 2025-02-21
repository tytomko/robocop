#!/usr/bin/env python3
#
# Copyright 2018 Open Source Robotics Foundation,
# Copyright 2019 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#/**********************************
#    Created on : 10th Jan  2024
#    Ported to ROS2 by: Shin Hyeon-hak 
#    Github : Carepediem324
#**********************************/

from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='ukf_ros2',
            executable='ukf_node',
            name='ukf_filter_node',
            output='screen',
            parameters=[{
                'init_state_provided': False,
                'imu_topic': '/ssafy/imu',
                'gnss_topic': '/ssafy/gps',
                'odom_topic': '/ukf/odometry',
                'vel_mps_topic': '/ukf/velocity_mps',
                'vel_kmph_topic': '/ukf/velocity_kmph',
                'heading_topic': '/ukf/heading',
                'utm_topic': '/ukf/utm_pose'
            }]
        )
    ])
