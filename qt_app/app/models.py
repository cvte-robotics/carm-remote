from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class ShutdownState(str, Enum):
    NONE = "NONE"
    STOPPING_SESSION = "STOPPING_SESSION"
    REQUESTING_BACKEND_SHUTDOWN = "REQUESTING_BACKEND_SHUTDOWN"
    WAITING_PROCESS_EXIT = "WAITING_PROCESS_EXIT"
    COMPLETE = "COMPLETE"


@dataclass(frozen=True)
class Device:
    name: str
    address: str


@dataclass(frozen=True)
class ArmProbe:
    ip: str
    arm_name: str = ""
    arm_dof: int = 0
    family: str = ""


@dataclass
class Snapshot:
    status: dict[str, Any] = field(default_factory=dict)
    pose: Any = None
    button: Any = None
    sequence: int = -1


@dataclass
class UiState:
    name: str = "BackendStarting"
    message: str = "正在启动后端..."
    devices: list[Device] = field(default_factory=list)
    allowed_profiles: list[str] = field(default_factory=list)
    probe: ArmProbe | None = None

    @property
    def active(self) -> bool:
        return self.name in {"Connecting", "Connected", "Initializing", "Teleoperating"}
