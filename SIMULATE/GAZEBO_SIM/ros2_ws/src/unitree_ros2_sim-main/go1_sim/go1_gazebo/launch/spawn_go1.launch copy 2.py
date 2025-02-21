import os

# ROS2에서 패키지의 공유 디렉토리 경로를 가져오기 위한 모듈
from ament_index_python.packages import get_package_share_directory

# ROS2 Launch 관련 액션 및 서브스티튜션 임포트
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource

# ROS2 launch 시스템이 호출하는 진입점 함수
def generate_launch_description():
    
    # 런치 인자로부터 world 파일명과 URDF 파일명을 받을 수 있도록 LaunchConfiguration 사용
    world_file_name = LaunchConfiguration('world_file_name')
    urdf_file = LaunchConfiguration('urdf_file')
    
    # 런치 인자를 선언 (world_file_name 인자, 기본값은 'test_latest.world')
    world_file_name_arg = DeclareLaunchArgument(
        'world_file_name',
        default_value='test_latest.world'
    )

    # 런치 인자를 선언 (urdf_file 인자, 기본값은 'robot.xacro')
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='robot.xacro'
    )

    # 로봇의 초기 위치 (X, Y, Z 좌표)
    position = [0, 0.0, 0.6]
    # 로봇의 초기 오리엔테이션 (롤, 피치, 요)
    orientation = [0.0, 0.0, 0.0]
    # 로봇의 이름 (예: "GO1")
    robot_name = "GO1"
    
    # Gazebo world를 시작하기 위한 별도의 launch 파일을 포함시킴
    # 이 경우 'go1_gazebo' 패키지의 'launch/start_world.launch.py' 파일을 사용함
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('go1_gazebo'), 'launch'),
            '/start_world.launch.py']),
        # 위 launch 파일에 world_file_name 인자를 전달
        launch_arguments={'world_file_name': world_file_name}.items(),
    )
    
    # Gazebo에서 로봇을 스폰(생성)하기 위한 노드 설정
    # gazebo_ros 패키지의 spawn_entity.py를 사용하여 로봇을 Gazebo에 생성함
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-entity', robot_name,                      # 생성될 엔티티(로봇)의 이름
            '-x', str(position[0]),                       # x 좌표
            '-y', str(position[1]),                       # y 좌표
            '-z', str(position[2]),                       # z 좌표
            '-R', str(orientation[0]),                    # roll 값
            '-P', str(orientation[1]),                    # pitch 값
            '-Y', str(orientation[2]),                    # yaw 값
            '-topic', '/robot_description'                # 로봇 URDF 정보가 게시되는 토픽
        ]
    )

    # 지도와 오도메트리 좌표계를 연결하는 TF 변환 정보를 퍼블리시하는 노드 설정
    map_odom_tf_publisher_node = Node(
        package='go1_navigation',
        executable='nav_tf_publisher',
        name='map_odom_transform_publisher',
        output='screen'
    )

    # 정적(static) 맵 데이터를 퍼블리시하는 노드 설정
    static_map_publisher_node = Node(
        package='go1_navigation',
        executable='static_map_publisher',
        name='static_map_publisher',
        output='screen'
    )

    # 내비게이션을 위한 맵 파일 경로 설정 (go1_navigation 패키지 내 map 디렉토리의 map.yaml)
    map_file_path = os.path.join(get_package_share_directory('go1_navigation'), 'map', 'map.yaml')

    # (주석 처리된 코드)
    # 원래 nav2_map_server를 사용하여 맵을 퍼블리시하는 노드 설정 코드가 있었으나 주석 처리됨.
    # map_publisher_node = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[
    #         {'yaml_filename' : map_file_path}
    #     ]
    # )

    # nav2_map_server를 사용하여 맵 서버 노드를 설정
    map_publisher_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file_path}],  # YAML 파일을 파라미터로 전달
        remappings=[('map', 'map')]
    )
    
    # 내비게이션 라이프사이클 매니저 노드 설정
    # 이 노드는 맵 서버를 자동으로 라이프사이클 상태로 전환하는 역할을 수행함
    manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[
            {'use_sim_time': False},         # 시뮬레이션 시간을 사용하지 않음
            {'autostart': True},             # 자동 시작 설정
            {'node_names': ['map_server']}   # 라이프사이클 관리할 노드 이름 목록
        ]
    )

    # ROS2 컨트롤러 관련 launch 파일 포함 (go1_gazebo 패키지의 controllers_go1.launch.py)
    launch_ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('go1_gazebo'), 'launch'),
            '/controllers_go1.launch.py'])
    )

    # 로봇 모델을 시각화하는 RViz 또는 다른 툴을 위한 launch 파일 포함
    # go1_description 패키지의 go1_visualize.launch.py 파일을 사용함
    visualize_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('go1_description'), 'launch', 'go1_visualize.launch.py')
            ]),
        launch_arguments={
            'use_joint_state_publisher': 'False',  # joint state publisher 사용 여부
            'use_sim_time': "True",                  # 시뮬레이션 시간 사용 여부
            'urdf_file': urdf_file                   # URDF 파일 경로 전달
        }.items(),
    )
    
    # (주석 처리된 코드)
    # 조이스틱 텔레옵 노드와 내비게이션 노드를 포함하는 launch 파일들이 있었으나 현재는 주석 처리됨.
    # joystick = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('go2_teleop'), 'launch', 'joystick.launch.py')
    #     ]), launch_arguments={'use_sim_true': 'true'}.items()
    # )
    
    # navigation = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('go2_navigation'), 'launch', 'nav_core.launch.py')
    #     ]), launch_arguments={'use_sim_true': 'true'}.items()
    # )

    # 최종적으로 LaunchDescription 객체를 생성하여 위에서 설정한 모든 액션과 노드를 포함시킴
    return LaunchDescription(
        [
            world_file_name_arg,          # world_file_name 런치 인자 선언
            urdf_file_arg,                # urdf_file 런치 인자 선언
            start_world,                  # Gazebo world 시작 launch 파일 포함
            spawn_robot,                  # Gazebo에 로봇 스폰 노드
            launch_ros2_control,          # ROS2 컨트롤러 launch 파일 포함
            visualize_robot,              # 로봇 시각화 launch 파일 포함
            map_odom_tf_publisher_node,   # 맵-오도메트리 TF 퍼블리셔 노드
            static_map_publisher_node     # 정적 맵 퍼블리셔 노드
            # 아래는 필요에 따라 주석 해제하여 사용할 수 있는 노드들임.
            # map_publisher_node,
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #       target_action=map_publisher_node,
            #       on_exit=[manager_node],
            #     )
            # )
            # joystick, 
            # navigation
        ]
    )
