import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/uhreed/turtlebot3_line_follower/src/turtlebot3_line_follower/install/turtlebot3_line_follower'
