import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/daxterjak/RoboticAlt/phase2/ros2_ws/install/diff_drive_l298n'
