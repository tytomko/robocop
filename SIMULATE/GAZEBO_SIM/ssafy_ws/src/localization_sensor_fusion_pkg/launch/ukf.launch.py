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

from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='localization_sensor_fusion_pkg',
            executable='ukf_node',
            name='ukf_filter_node',
            output='screen',
            parameters=[{
                'init_state_provided': False,
                'imu_topic': '/ssafy/imu',
                'gnss_topic': '/ssafy/gps',
                'odom_topic': '/ssafy/odometry'
            }]
        )
    ])
