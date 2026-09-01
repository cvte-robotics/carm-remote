from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

from PyQt6.QtWidgets import QApplication, QMessageBox


def main(argv=None) -> int:
    qt_app_dir = Path(__file__).resolve().parent
    server_dir = Path(__file__).resolve().parents[1] / "server"
    for module_dir in (qt_app_dir, server_dir):
        while str(module_dir) in sys.path:
            sys.path.remove(str(module_dir))
    sys.path.insert(0, str(qt_app_dir))
    sys.path.insert(1, str(server_dir))
    os.environ["CARM_JOYSTICK_PROVIDERS"] = "quest"

    from basic.joystick.end_effectors import EndEffectorBase

    EndEffectorBase.load_all()
    parser = argparse.ArgumentParser(description="CARM Quest desktop")
    parser.add_argument(
        "--end-effector",
        default="carm",
        help="包内末端实现名称（按名称动态导入）",
    )
    args, qt_args = parser.parse_known_args(argv)

    app = QApplication([sys.argv[0], *qt_args])
    app.setOrganizationName("CVTE")
    app.setApplicationName("CARMQuestDesktop")
    app.setApplicationDisplayName("CARM 手柄遥操")

    from app.logging_config import configure_ui_logging
    from app.runtime_paths import ui_log_path

    configure_ui_logging(ui_log_path())

    try:
        from preflight import validate
        validate()
    except RuntimeError as exc:
        QMessageBox.critical(window=None, title="CARM 启动检查失败", text=str(exc))
        return 1

    from app.main_window import MainWindow

    window = MainWindow(end_effector=args.end_effector)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
