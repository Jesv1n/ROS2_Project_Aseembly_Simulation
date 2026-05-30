import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jesvin/robot_pick_place_project/ros2_ws/install/robot_pick_place_bringup'
