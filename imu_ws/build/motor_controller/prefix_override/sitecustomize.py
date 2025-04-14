import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mete/ros2_project/ros2bookcode/chapt6/imu_ws/install/motor_controller'
