from __future__ import annotations

from pathlib import Path
import os
import tempfile

from PyQt6.QtCore import QStandardPaths


def backend_root() -> Path:
    return Path(__file__).resolve().parents[2]


def log_dir() -> Path:
    standard = QStandardPaths.writableLocation(
        QStandardPaths.StandardLocation.AppLocalDataLocation
    )
    candidates = []
    configured = os.environ.get("CARM_DESKTOP_LOG_DIR")
    if configured:
        candidates.append(Path(configured))
    if standard:
        candidates.append(Path(standard) / "logs")
    candidates.extend(
        [
            Path.home() / ".carm-quest-desktop" / "logs",
            Path(tempfile.gettempdir()) / "carm-quest-desktop" / "logs",
        ]
    )
    for candidate in candidates:
        try:
            candidate.mkdir(parents=True, exist_ok=True)
            probe = candidate / ".write-test"
            probe.write_text("ok", encoding="ascii")
            probe.unlink()
            return candidate
        except OSError:
            continue
    raise RuntimeError("No writable desktop log directory")


def ui_log_path() -> Path:
    return log_dir() / "desktop-ui.log"


def backend_log_path() -> Path:
    return log_dir() / "desktop-backend.log"
