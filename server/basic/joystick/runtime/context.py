"""进程内 joystick 配置单例：进程启动时加载一次。"""

from .config import load_joystick_config

JOYSTICK_CONFIG = load_joystick_config()
