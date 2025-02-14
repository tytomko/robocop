import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace, Node

def generate_launch_description():
    # 기본 인자 선언
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_gazebo_world = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(
            FindPackageShare("champ_config").find("champ_config"), "worlds/outdoor.world"
        ),
        description="Gazebo world file",
    )
    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Use gui"
    )
    
    # 패키지 경로 설정
    config_pkg_share = FindPackageShare("champ_config").find("champ_config")
    descr_pkg_share = FindPackageShare("champ_description").find("champ_description")
    joints_config = os.path.join(config_pkg_share, "config/joints/joints.yaml")
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")
    gait_config = os.path.join(config_pkg_share, "config/gait/gait.yaml")
    default_model_path = os.path.join(descr_pkg_share, "urdf/champ.urdf.xacro")

    # 소환할 로봇 이름과 초기 위치 지정
    robot_names = ["kim", "lee"]
    initial_positions = {
        "kim": {"world_init_x": "0.0", "world_init_y": "0.0"},
        "lee": {"world_init_x": "2.0", "world_init_y": "0.0"},
    }
    
    # Gazebo 시뮬레이터는 한 번만 실행 (전체 시스템)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare("champ_gazebo").find("champ_gazebo"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "world": LaunchConfiguration("world"),
            "gui": LaunchConfiguration("gui"),
        }.items(),
    )
    
    # 각 로봇의 Bringup 및 스폰 그룹 구성
    robot_groups = []
    for name in robot_names:
        group = GroupAction(
            actions=[
                # 각 로봇을 별도의 네임스페이스로 묶음
                PushRosNamespace(name),
                
                # 챔프 로봇 bringup 런치 파일 포함 (내부에서 로봇의 노드들 및 컨트롤러 실행)
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            FindPackageShare("champ_bringup").find("champ_bringup"),
                            "launch",
                            "bringup.launch.py",
                        )
                    ),
                    launch_arguments={
                        "description_path": default_model_path,
                        "joints_map_path": joints_config,
                        "links_map_path": links_config,
                        "gait_config_path": gait_config,
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "robot_name": name,
                        # Gazebo 관련 인자는 여기서 더 이상 포함하지 않아도 됩니다.
                        "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
                        "hardware_connected": "false",
                        "publish_foot_contacts": "false",
                    }.items(),
                ),
                
                # spawn_entity 노드를 사용해 Gazebo에 로봇을 소환
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', name,
                        '-topic', 'robot_description',  # 또는 -file로 URDF 파일 지정 가능
                        '-x', initial_positions[name]["world_init_x"],
                        '-y', initial_positions[name]["world_init_y"],
                        '-Y', "0.6",  # yaw (회전) 값, 필요에 따라 변경
                    ],
                    output='screen'
                )
            ]
        )
        robot_groups.append(group)
    
    # 최종 LaunchDescription 구성: Gazebo는 단 한 번 실행, 이후 각 로봇 그룹 실행
    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_gazebo_world,
            declare_gui,
            gazebo_launch,
        ] + robot_groups
    )
