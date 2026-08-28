from __future__ import annotations

import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import sys
import tempfile


LOG_FORMAT = "%(asctime)s %(levelname)s %(name)s %(message)s"


def configure_ui_logging(log_path: Path) -> Path:
    handlers: list[logging.Handler] = []
    actual_path = log_path
    for candidate in (
        log_path,
        Path(tempfile.gettempdir())
        / "carm-quest-desktop"
        / "logs"
        / "desktop-ui.log",
    ):
        try:
            candidate.parent.mkdir(parents=True, exist_ok=True)
            file_handler = RotatingFileHandler(
                candidate,
                maxBytes=5 * 1024 * 1024,
                backupCount=3,
                encoding="utf-8",
            )
            file_handler.setFormatter(logging.Formatter(LOG_FORMAT))
            handlers.append(file_handler)
            actual_path = candidate
            break
        except OSError:
            continue

    console_handler = logging.StreamHandler()
    console_handler.setFormatter(logging.Formatter(LOG_FORMAT))
    handlers.append(console_handler)

    logging.basicConfig(
        level=logging.INFO,
        handlers=handlers,
        force=True,
    )
    os.environ["CARM_DESKTOP_LOG_DIR"] = str(actual_path.parent)

    def log_uncaught(exc_type, exc_value, traceback) -> None:
        logging.getLogger("desktop.ui").critical(
            "Uncaught UI exception",
            exc_info=(exc_type, exc_value, traceback),
        )
        sys.__excepthook__(exc_type, exc_value, traceback)

    sys.excepthook = log_uncaught
    logging.getLogger("desktop.ui").info("UI logging initialized: %s", actual_path)
    return actual_path
