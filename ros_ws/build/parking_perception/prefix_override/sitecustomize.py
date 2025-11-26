import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/apurv/Projects/Parking Bot Project/ros_ws/install/parking_perception'
