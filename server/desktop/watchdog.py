"""Heartbeat ownership for the desktop backend process."""

from __future__ import annotations

import threading
import time
from typing import Callable


class HeartbeatWatchdog:
    def __init__(
        self,
        on_timeout: Callable[[], None],
        *,
        heartbeat_timeout: float = 3.0,
        startup_timeout: float = 10.0,
        poll_interval: float = 0.25,
    ) -> None:
        self._on_timeout = on_timeout
        self._heartbeat_timeout = heartbeat_timeout
        self._poll_interval = poll_interval
        self._deadline = time.monotonic() + startup_timeout
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            daemon=True,
            name="DesktopHeartbeatWatchdog",
        )

    def start(self) -> None:
        self._thread.start()

    def touch(self) -> None:
        with self._lock:
            self._deadline = time.monotonic() + self._heartbeat_timeout

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread.is_alive() and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)

    def _run(self) -> None:
        while not self._stop_event.wait(self._poll_interval):
            with self._lock:
                expired = time.monotonic() >= self._deadline
            if expired:
                self._stop_event.set()
                self._on_timeout()
                return
