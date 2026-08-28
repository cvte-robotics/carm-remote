from __future__ import annotations

import sys

from PyQt6.QtWidgets import QApplication, QMessageBox


def main() -> int:
    app = QApplication(sys.argv)
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

    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
