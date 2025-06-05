import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/victorn65/ros2_ws/src/servo_sirvo_fisico/install/servo_sirvo_fisico'
