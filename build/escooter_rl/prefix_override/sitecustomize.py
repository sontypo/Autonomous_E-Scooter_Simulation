import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hong-son/Autonomous_E-Scooter_Simulation/install/escooter_rl'
