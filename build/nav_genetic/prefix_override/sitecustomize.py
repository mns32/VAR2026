import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/marina/Escritorio/var/VAR2026/install/nav_genetic'
