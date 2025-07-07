import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/giszterlab/USING/unitree_ros2_sim/install/go1_navigation'
