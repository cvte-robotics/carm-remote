"""Quest 输入模块的导入路径装配与模块根目录选择。"""

import logging
import os
from pathlib import Path
import sys


PRODUCTION_DEPENDENCIES_PATH = Path("/var/www/backend/carm_backend/dependencies")
REPOSITORY_INCLUDE_PATH = Path(__file__).resolve().parents[4] / "include"
QUEST_ONLY_PROVIDERS_ENV = "CARM_JOYSTICK_PROVIDERS"
QUEST_REQUIRED_FILES = (
    "__init__.py", "adb.py", "config.py", "constants.py", "device_status.py",
    "module.py", "parser.py", "policy.py", "receiver.py",
)


def quest_only_providers_enabled() -> bool:
    """当前进程是否显式选择了桌面 Quest profile。"""
    return os.environ.get(QUEST_ONLY_PROVIDERS_ENV, "").strip().lower() == "quest"


def _repository_quest_root() -> Path:
    """返回仓库内 Quest 模块根目录，并校验必需文件齐全。

    Raises: ImportError 任一必需模块文件缺失。
    """
    root = (REPOSITORY_INCLUDE_PATH / "quest").resolve()
    missing = [name for name in QUEST_REQUIRED_FILES if not (root / name).is_file()]
    if missing:
        raise ImportError("Desktop Quest 模块缺失: %s" % ", ".join(missing))
    return root


def _validate_quest_module_source() -> None:
    """校验已导入的 quest 模块确实来自仓库 Quest 根目录。

    Raises: ImportError 模块来源与仓库根不一致。
    """
    root = _repository_quest_root()
    import importlib
    module = importlib.import_module("quest")
    module_file = getattr(module, "__file__", None)
    if not module_file or root not in Path(module_file).resolve().parents:
        raise ImportError("Desktop Quest 模块来源冲突: %s" % module_file)


def canonical_input_module_root() -> Path:
    """选择当前运行时使用的后端自有 Quest 模块根。"""
    if quest_only_providers_enabled():
        return REPOSITORY_INCLUDE_PATH
    if PRODUCTION_DEPENDENCIES_PATH.is_dir():
        return PRODUCTION_DEPENDENCIES_PATH
    return REPOSITORY_INCLUDE_PATH


def setup_import_paths() -> None:
    """把 Quest 输入模块根加入 sys.path；私有 provider 由 Board 组合负责加载。

    Raises: Exception 显式选择 Quest profile 时，路径装配与来源校验失败。
    """
    try:
        module_root = canonical_input_module_root()
        module_root_text = str(module_root)
        if module_root.is_dir():
            while module_root_text in sys.path:
                sys.path.remove(module_root_text)
            sys.path.insert(0, module_root_text)
            logging.debug("[Joystick] Added canonical input module root: %s", module_root)
        if quest_only_providers_enabled():
            _validate_quest_module_source()
    except Exception as exc:
        if quest_only_providers_enabled():
            raise
        logging.error("[Joystick] Error setting up paths: %s", exc)
