#!/usr/bin/env python3
"""
The launch file performs the following tasks:
1. Sets the TurtleBot3 model to "burger".
2. Configures the simulation to use simulated time.
3. Specifies the world file to be used in the Gazebo simulation.
4. Defines the path to the TurtleBot3 URDF file with the Velodyne sensor attached.
5. Launches the Gazebo simulator with the specified world file.
6. Starts the robot_state_publisher node to publish the state of the robot to the ROS ecosystem.
This setup allows for a comprehensive simulation environment where the TurtleBot3 is equipped with IMU, GPS, and 3D Lidar sensors, enabling advanced navigation and perception capabilities.
"""
# This file was created by github:Carpediem324
# For any inquiries, please contact imur.navigator@gmail.com

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command


TURTLEBOT3_MODEL = "burger"

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    #world_file_name = 'ssafy_map.world'
    world_file_name = 'turtlebot3_velodyne_robots.world'
    world = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'worlds', world_file_name)
    robot_desc_path = os.path.join(get_package_share_directory("turtlebot3_description"), "urdf", "turtlebot3_velodyne_burger.urdf")

    entity_name_0=""

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so'],
            output='screen'),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=entity_name_0,
            parameters=[{'frame_prefix': entity_name_0+'/', 'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', robot_desc_path, ' robot_name:=', entity_name_0])}],
            output="screen"
        ),

    ])