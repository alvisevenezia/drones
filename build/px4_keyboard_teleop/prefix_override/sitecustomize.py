import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/antoine/Documents/drones/install/px4_keyboard_teleop'
