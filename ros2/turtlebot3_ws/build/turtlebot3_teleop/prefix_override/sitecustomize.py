import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/simpson/turtlebot3_ws/install/turtlebot3_teleop'
