"""Host-owned fixed input source registry and binding identities."""

from dataclasses import dataclass, replace
from enum import Enum
import itertools
import re
from typing import Callable, Dict, Optional, Tuple

from carm_teleop_input_contracts import (
    DeviceCandidate,
    DeviceDescriptor,
    InputSourceModule,
    SourceCapabilities,
    SourceLease,
)


PROVIDER_QUEST = "quest"
PROVIDER_SURREAL = "surreal"
_PROVIDER_ORDER = (PROVIDER_QUEST, PROVIDER_SURREAL)
_BLE_ADDRESS_RE = re.compile(r"^(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$")


class SourceScanError(RuntimeError):
    pass


class BindingPhase(Enum):
    PROVISIONAL = "provisional"
    ACTIVE = "active"
    CLOSED = "closed"


@dataclass(frozen=True)
class ActiveSourceBinding:
    lease: SourceLease
    public_source_type: str
    capabilities: SourceCapabilities
    phase: BindingPhase
    input_ready: bool

    def activate(self) -> "ActiveSourceBinding":
        if self.phase is not BindingPhase.PROVISIONAL:
            raise ValueError("only provisional bindings can become active")
        return replace(self, phase=BindingPhase.ACTIVE, input_ready=True)

    def with_input_ready(self, ready: bool) -> "ActiveSourceBinding":
        if self.phase is not BindingPhase.ACTIVE:
            raise ValueError("only active bindings expose input readiness")
        if not isinstance(ready, bool):
            raise ValueError("input readiness must be a boolean")
        return replace(self, input_ready=ready)

    def close(self) -> "ActiveSourceBinding":
        return replace(self, phase=BindingPhase.CLOSED, input_ready=False)


class SourceLeaseAllocator:
    """Allocate module identities that cannot survive module retirement."""

    def __init__(self) -> None:
        self._module_instance_ids: Dict[str, int] = {}
        self._next_module_instance_id = itertools.count(1)

    def allocate(self, provider_id: str, input_generation: int) -> SourceLease:
        if provider_id not in _PROVIDER_ORDER:
            raise ValueError(f"unsupported provider: {provider_id}")
        if input_generation <= 0:
            raise ValueError("input_generation must be positive")
        instance_id = self._module_instance_ids.get(provider_id)
        if instance_id is None:
            instance_id = next(self._next_module_instance_id)
            self._module_instance_ids[provider_id] = instance_id
        return SourceLease(
            provider_id=provider_id,
            module_instance_id=instance_id,
            input_generation=input_generation,
        )

    def retire(self, provider_id: str) -> None:
        self._module_instance_ids.pop(provider_id, None)


ModuleFactory = Callable[[], InputSourceModule]


class SourceRegistry:
    """Fixed Quest-first registry for scan identity and module routing."""

    def __init__(
        self,
        *,
        quest_factory: Optional[ModuleFactory] = None,
        surreal_factory: Optional[ModuleFactory] = None,
        lease_allocator: Optional[SourceLeaseAllocator] = None,
    ) -> None:
        self._factories = {
            PROVIDER_QUEST: quest_factory,
            PROVIDER_SURREAL: surreal_factory,
        }
        self._modules: Dict[str, InputSourceModule] = {}
        self._lease_allocator = lease_allocator or SourceLeaseAllocator()
        self._scan_generation = 0
        self._next_descriptor_id = itertools.count(1)
        self._scanned_descriptors: Dict[str, DeviceDescriptor] = {}
        self._active_lease: Optional[SourceLease] = None
        self._closing_lease: Optional[SourceLease] = None

    @staticmethod
    def _device_key(address: str) -> str:
        normalized = (address or "").strip()
        return normalized.upper() if _BLE_ADDRESS_RE.match(normalized) else normalized

    @property
    def provider_ids(self) -> Tuple[str, ...]:
        return tuple(
            provider_id
            for provider_id in _PROVIDER_ORDER
            if self._factories[provider_id] is not None
        )

    @property
    def scan_generation(self) -> int:
        return self._scan_generation

    def allocate_lease(self, provider_id: str, input_generation: int) -> SourceLease:
        return self._lease_allocator.allocate(provider_id, input_generation)

    def get_or_create(self, provider_id: str) -> InputSourceModule:
        if provider_id not in _PROVIDER_ORDER:
            raise KeyError(provider_id)
        module = self._modules.get(provider_id)
        if module is not None:
            return module
        factory = self._factories[provider_id]
        if factory is None:
            raise KeyError(provider_id)
        module = factory()
        if module.module_id != provider_id:
            raise ValueError(
                f"module id mismatch: expected {provider_id}, got {module.module_id}"
            )
        self._modules[provider_id] = module
        return module

    def existing(self, provider_id: str) -> Optional[InputSourceModule]:
        return self._modules.get(provider_id)

    def _new_descriptor(
        self,
        candidate: DeviceCandidate,
        scan_generation: Optional[int] = None,
    ) -> DeviceDescriptor:
        return DeviceDescriptor(
            candidate=candidate,
            scan_generation=(
                self._scan_generation if scan_generation is None else scan_generation
            ),
            descriptor_id=next(self._next_descriptor_id),
        )

    def begin_scan(self) -> int:
        self._scan_generation += 1
        self._scanned_descriptors = {}
        return self._scan_generation

    def scan_provider(
        self,
        provider_id: str,
        scan_generation: int,
    ) -> Tuple[DeviceDescriptor, ...]:
        if scan_generation != self._scan_generation:
            return ()
        module = self.get_or_create(provider_id)
        descriptors = tuple(
            self._new_descriptor(candidate, scan_generation)
            for candidate in module.scan()
        )
        self._scanned_descriptors.update({
            self._device_key(descriptor.candidate.address): descriptor
            for descriptor in descriptors
        })
        return descriptors

    def scan_quest_first(self) -> Tuple[DeviceDescriptor, ...]:
        generation = self.begin_scan()
        descriptors = self.scan_provider(PROVIDER_QUEST, generation)
        if descriptors or PROVIDER_SURREAL not in self.provider_ids:
            return descriptors
        module = self.get_or_create(PROVIDER_SURREAL)
        availability = module.availability()
        if not availability.available:
            message_factory = getattr(module, "scan_unavailable_message", None)
            message = (
                message_factory()
                if callable(message_factory)
                else availability.reason or "输入设备扫描不可用"
            )
            raise SourceScanError(message)
        return self.scan_provider(PROVIDER_SURREAL, generation)

    def resolve_device(self, address: str, name: str) -> Optional[DeviceDescriptor]:
        scanned = self._scanned_descriptors.get(self._device_key(address))
        if scanned is not None:
            module = self.get_or_create(scanned.candidate.provider_id)
            candidate = module.resolve_device(address, name, scanned.candidate)
            return None if candidate is None else self._new_descriptor(
                candidate,
                scanned.scan_generation,
            )

        for provider_id in self.provider_ids:
            module = self.get_or_create(provider_id)
            candidate = module.resolve_device(address, name, None)
            if candidate is not None:
                # Surreal module deliberately returns None for unscanned ANS.
                return self._new_descriptor(candidate)
        return None

    def resolution_error(self, address: str, name: str) -> Optional[str]:
        for provider_id in self.provider_ids:
            module = self.get_or_create(provider_id)
            resolver = getattr(module, "resolution_error", None)
            if callable(resolver):
                message = resolver(address, name)
                if message:
                    return message
        return None

    def activate(self, lease: SourceLease) -> InputSourceModule:
        if self._active_lease is not None and self._active_lease != lease:
            raise RuntimeError("previous source lease has not finished closing")
        module = self.get_or_create(lease.provider_id)
        current_lease = getattr(module, "current_lease", lambda: None)()
        if current_lease != lease:
            raise RuntimeError("module did not accept the active source lease")
        self._active_lease = lease
        self._closing_lease = None
        return module

    def begin_close(self, lease: SourceLease) -> bool:
        if self._active_lease != lease:
            return False
        self._closing_lease = lease
        return True

    def complete_close(self, lease: SourceLease) -> bool:
        if self._active_lease != lease or self._closing_lease != lease:
            return False
        self._active_lease = None
        self._closing_lease = None
        return True

    def finish(self, lease: SourceLease) -> bool:
        """Compatibility alias for a fully completed close."""
        if self._active_lease != lease:
            return False
        self._closing_lease = lease
        return self.complete_close(lease)

    def active_module(self) -> Optional[InputSourceModule]:
        lease = self._active_lease
        return None if lease is None else self._modules.get(lease.provider_id)

    def retire_module(self, provider_id: str, reason: str) -> bool:
        if (
            self._active_lease is not None
            and self._active_lease.provider_id == provider_id
        ):
            return False
        module = self._modules.get(provider_id)
        if module is None:
            return False
        if not bool(module.shutdown(reason)):
            return False
        if self._modules.get(provider_id) is module:
            self._modules.pop(provider_id, None)
            self._lease_allocator.retire(provider_id)
        return True

    def shutdown(self, reason: str) -> bool:
        success = True
        for provider_id in _PROVIDER_ORDER:
            module = self._modules.get(provider_id)
            if module is not None:
                success = self.retire_module(provider_id, reason) and success
        if success:
            self._active_lease = None
            self._closing_lease = None
        return success
