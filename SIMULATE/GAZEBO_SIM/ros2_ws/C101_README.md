*설치 방법*

로봇 가동 전 설치, 등록해야할 것들

    1.  lcm 설치

        (1) cd ~/Downloads
            git clone https://github.com/lcm-proj/lcm.git
            cd lcm

        (2) mkdir build && cd build
            cmake ..
            make -j$(nproc)
            sudo make install
            sudo ldconfig

    2. bashrc 파일에 모델 경로 설정하기

        (1) export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/ubuntu/S12P11C101/SIMULATE/GAZEBO_SIM/ros2_ws/src/unitree_ros2_sim-main/go1_sim/go1_gazebo/models/
        
        ==> 자신의 우분투 path에 맞게 고쳐 넣어야 합니다. 수정해야됨!!

        (2) source /usr/share/gazebo/setup.sh 


==> 워크스페이스에서 colcon build 실행해줍니다. 보라색 에러같은게 뜨긴 하는데 가볍게 무시해주면 됩니다.


*실행 방법*


 cmd에 차례대로 쳐야 됨 (하나 당 하나의 창. 각 창마다 ". install/setup.bash" 입력)

    1. ros2 launch go1_gazebo spawn_go1.launch.py world_file_name:=warehouse.world
            로딩이 완료된 다음, 2번을 실행해야함. 로딩이 아주 오래걸리므로 인내심을 가지고 기다릴것. 
            사람들이 뛰어다니기 시작하면 로딩이 완료된 것임

    2. ros2 run unitree_guide2 junior_ctrl --ros-args --remap cmd_vel:=/ssafy/cmd_vel

            (1) cmd 창에 2를 입력하면 로봇이 일어납니다.
            (2) cmd 창에 5를 입력하면 로봇이 키보드 입력모드로 바뀌게 됩니다.

    3. ros2 run middle_teleop middle_teleop_node --ros-args --param robot_name:=ssafy --param robot_number:=1
    4. ros2 run keyboard_input key_publisher --ros-args --param robot_name:=ssafy --param robot_number:=1

    4번이 틀어져 있는 창에서 키보드를 입력하게 되면 로봇이 움직입니다.