import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch Argument 선언 (default 값 설정)
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='ssafy',
        description='Robot name parameter'
    )
    robot_number_arg = DeclareLaunchArgument(
        'robot_number',
        default_value='1',
        description='Robot number parameter'
    )
    
    # LaunchConfiguration 사용
    robot_name = LaunchConfiguration('robot_name')
    robot_number = LaunchConfiguration('robot_number')
    
    # turtlebot3_gazebo 패키지에서 launch 파일 경로를 가져옴
    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    tb3_launch_file = os.path.join(turtlebot3_gazebo_share, 'launch', 'tb3_imu_lidar_gps_burger.launch.py')
    
    turtlebot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_launch_file)
    )
    
    # robot_status_publisher 노드
    robot_status_node = Node(
        package='robot_status_publisher',
        executable='robot_status_publisher',
        name='robot_status_publisher',
        output='screen',
        parameters=[{'robot_name': robot_name, 'robot_number': robot_number}]
    )
    
    # global_path_planner 노드
    global_path_planner_node = Node(
        package='robot_planning_pkg',
        executable='global_path_planner',
        name='global_path_planner',
        output='screen',
        parameters=[{'robot_name': robot_name, 'robot_number': robot_number}]
    )
    
    # robot_patrol 노드
    robot_patrol_node = Node(
        package='robot_patrol_pkg',
        executable='robot_patrol',
        name='robot_patrol',
        output='screen',
        parameters=[{'robot_name': robot_name, 'robot_number': robot_number}]
    )
    
    robot_vision_node = Node(
        package='robot_vision_pkg',
        executable='velodyne_detection',
        name='velodyne_detection',
        output='screen',
        parameters=[{'robot_name': robot_name, 'robot_number': robot_number}]
    )


    return LaunchDescription([
        robot_name_arg,
        robot_number_arg,
        # 나중엔 turtlebot_launch빼고 Isaac sim으로 대체
        turtlebot_launch,
        robot_status_node,
        global_path_planner_node,
        robot_patrol_node,
        robot_vision_node
    ])
