import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/martin/SCARA-RI20252/install/trayectory_planer_scara'
