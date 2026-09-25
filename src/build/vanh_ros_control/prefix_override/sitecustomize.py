import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/v005101/Documents/quadruped_robotics/src/install/vanh_ros_control'
