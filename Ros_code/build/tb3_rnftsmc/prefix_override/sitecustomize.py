import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tomtom/Projet-sensor-based-controll/Ros_code/install/tb3_rnftsmc'
