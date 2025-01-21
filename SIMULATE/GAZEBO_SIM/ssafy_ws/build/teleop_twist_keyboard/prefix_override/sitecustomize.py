import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ubuntu/S12P11C101/SIMULATE/GAZEBO_SIM/ssafy_ws/install/teleop_twist_keyboard'
