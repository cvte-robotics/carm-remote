"""Host 持有的固定输入源注册表与绑定（binding）身份。"""

from dataclasses import dataclass, replace
from enum import Enum
import itertools
import logging
import re
from typing import Callable, Dict, Iterable, Optional, Sequence, Tuple

from carm_teleop_input_contracts import (
    DeviceCandidate,
    InputSourceModule,
    SourceCapabilities,
    SourceLease,
)


_BLE_ADDRESS_RE = re.compile(r"^(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$")


class SourceScanError(RuntimeError):
    """全部可用 provider 扫描均失败时抛出的聚合错误。"""


@dataclass(frozen=True)
class DeviceDescriptor:
    """一次扫描命中的设备描述符：候选信息 + 扫描世代 + 全局唯一 id。"""

    candidate: DeviceCandidate
    scan_generation: int
    descriptor_id: int

    def __post_init__(self) -> None:
        """校验 candidate 类型，以及世代/id 为非负整数。"""
        if not isinstance(self.candidate, DeviceCandidate):
            raise ValueError("descriptor candidate is required")
        for name, value in (
            ("scan_generation", self.scan_generation),
            ("descriptor_id", self.descriptor_id),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(
                    f"descriptor {name} must be a non-negative integer"
                )


class BindingPhase(Enum):
    """binding 生命周期阶段：provisional → active → closed。"""

    PROVISIONAL = "provisional"
    ACTIVE = "active"
    CLOSED = "closed"


@dataclass(frozen=True)
class ActiveSourceBinding:
    """Host 自有的 active binding 身份：lease、会话凭据与输入就绪状态。"""

    lease: SourceLease
    session_id: int
    public_source_type: str
    capabilities: SourceCapabilities
    phase: BindingPhase
    input_ready: bool

    def __post_init__(self) -> None:
        """校验 Host 会话凭据 session_id 为正整数。

        Host 每次激活分配一个单调递增的 session id，并把同一个值复用为
        ``lease.input_generation``，以保持 provider 边界的冻结 lease 契约。
        ``session_id`` 是 Host 的会话凭据：已关闭的 binding 绝不复用其 id。
        """
        if (
            isinstance(self.session_id, bool)
            or not isinstance(self.session_id, int)
            or self.session_id <= 0
        ):
            raise ValueError("binding session_id must be a positive integer")

    def activate(self) -> "ActiveSourceBinding":
        """provisional → active，并把输入就绪标记置为 True。

        Raises: ValueError 当前不是 provisional 阶段。
        """
        if self.phase is not BindingPhase.PROVISIONAL:
            raise ValueError("only provisional bindings can become active")
        return replace(self, phase=BindingPhase.ACTIVE, input_ready=True)

    def with_input_ready(self, ready: bool) -> "ActiveSourceBinding":
        """在 active 阶段更新输入就绪标记。

        Raises: ValueError 非 active 阶段，或 ready 非布尔值。
        """
        if self.phase is not BindingPhase.ACTIVE:
            raise ValueError("only active bindings expose input readiness")
        if not isinstance(ready, bool):
            raise ValueError("input readiness must be a boolean")
        return replace(self, input_ready=ready)

    def close(self) -> "ActiveSourceBinding":
        """转为 closed，并清除输入就绪标记。"""
        return replace(self, phase=BindingPhase.CLOSED, input_ready=False)


class SourceLeaseAllocator:
    """分配模块身份（module identity），使其无法在模块停用后继续存活。"""

    def __init__(self) -> None:
        """初始化 provider → module_instance_id 映射与单调计数器。"""
        self._module_instance_ids: Dict[str, int] = {}
        self._next_module_instance_id = itertools.count(1)

    def allocate(self, provider_id: str, input_generation: int) -> SourceLease:
        """为 provider 分配（或复用）稳定的模块实例 id 并生成 lease。

        Args: provider_id 目标 provider；input_generation 正整数世代。
        Returns: 冻结的 SourceLease。
        Raises: ValueError provider_id 非法，或 input_generation 非正。
        """
        if not provider_id or not isinstance(provider_id, str):
            raise ValueError("unsupported provider: %s" % (provider_id,))
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
        """停用 provider 的模块实例身份（下次激活重新分配 id）。"""
        self._module_instance_ids.pop(provider_id, None)


ModuleFactory = Callable[[], InputSourceModule]


class SourceRegistry:
    """由组合根注入的输入源注册表，负责扫描身份与模块路由。"""

    def __init__(
        self,
        providers: Sequence[Tuple[str, Optional[ModuleFactory]]] = (),
        *,
        lease_allocator: Optional[SourceLeaseAllocator] = None,
    ) -> None:
        """按配置顺序登记 provider 工厂，并初始化扫描与 lease 状态。"""
        ordered = tuple((provider_id, factory) for provider_id, factory in providers if factory is not None)
        self._order = tuple(provider_id for provider_id, _factory in ordered)
        self._factories = {provider_id: factory for provider_id, factory in ordered}
        self._modules: Dict[str, InputSourceModule] = {}
        self._lease_allocator = lease_allocator or SourceLeaseAllocator()
        self._scan_generation = 0
        self._next_descriptor_id = itertools.count(1)
        self._scanned_descriptors: Dict[str, DeviceDescriptor] = {}
        self._active_lease: Optional[SourceLease] = None
        self._closing_lease: Optional[SourceLease] = None

    @staticmethod
    def _device_key(address: str) -> str:
        """归一化设备地址：BLE MAC 转大写，其余保持原样。"""
        normalized = (address or "").strip()
        return normalized.upper() if _BLE_ADDRESS_RE.match(normalized) else normalized

    @property
    def provider_ids(self) -> Tuple[str, ...]:
        """按配置顺序返回 provider id 元组。"""
        return self._order

    @property
    def scan_generation(self) -> int:
        """返回当前扫描世代号。"""
        return self._scan_generation

    def allocate_lease(self, provider_id: str, input_generation: int) -> SourceLease:
        """委托 lease 分配器生成 provider lease。"""
        return self._lease_allocator.allocate(provider_id, input_generation)

    def get_or_create(self, provider_id: str) -> InputSourceModule:
        """按 provider id 取用或惰性创建模块实例。

        Raises: KeyError provider 未登记；ValueError 模块自报 id 与登记不一致。
        """
        if provider_id not in self._factories:
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
        """返回已创建的模块实例；未创建时为 None。"""
        return self._modules.get(provider_id)

    def _new_descriptor(
        self,
        candidate: DeviceCandidate,
        scan_generation: Optional[int] = None,
    ) -> DeviceDescriptor:
        """为候选设备生成描述符（默认取当前扫描世代）。"""
        return DeviceDescriptor(
            candidate=candidate,
            scan_generation=(
                self._scan_generation if scan_generation is None else scan_generation
            ),
            descriptor_id=next(self._next_descriptor_id),
        )

    def begin_scan(self) -> int:
        """开启新扫描世代并清空上一世代的命中缓存。"""
        self._scan_generation += 1
        self._scanned_descriptors = {}
        return self._scan_generation

    def was_scanned(self, address: str) -> bool:
        """判断地址是否在当前扫描世代中被命中。"""
        descriptor = self._scanned_descriptors.get(self._device_key(address))
        return bool(descriptor and descriptor.scan_generation == self._scan_generation)

    def scan_provider(
        self,
        provider_id: str,
        scan_generation: int,
    ) -> Tuple[DeviceDescriptor, ...]:
        """在指定世代扫描单个 provider，并缓存命中的描述符。"""
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

    def _provider_failure(self, provider_id: str, exc: BaseException, action: str) -> str:
        """记录 provider 异常，并返回统一的错误消息。"""
        logging.exception(
            "[SourceRegistry] Provider %s %s failed", provider_id, action
        )
        return "%s: %s" % (provider_id, exc)

    def scan_in_order(self) -> Tuple[DeviceDescriptor, ...]:
        """按配置顺序扫描各 provider，命中第一个非空结果即返回。

        Returns: 命中的设备描述符元组；无命中且无错误时返回空元组。
        Raises: SourceScanError 存在不可用/扫描异常的 provider 且全无命中。
        """
        generation = self.begin_scan()
        unavailable = []
        for provider_id in self.provider_ids:
            try:
                module = self.get_or_create(provider_id)
                availability = module.availability()
            except Exception as exc:
                unavailable.append(self._provider_failure(provider_id, exc, "scan"))
                continue
            if not availability.available:
                message_factory = getattr(module, "scan_unavailable_message", None)
                message = (
                    message_factory()
                    if callable(message_factory)
                    else availability.reason or "输入设备扫描不可用"
                )
                unavailable.append("%s: %s" % (provider_id, message))
                continue
            try:
                descriptors = self.scan_provider(provider_id, generation)
            except Exception as exc:
                unavailable.append(self._provider_failure(provider_id, exc, "scan"))
                continue
            if descriptors:
                return descriptors
        if unavailable:
            raise SourceScanError("; ".join(unavailable))
        return ()

    def resolve_device(self, address: str, name: str) -> Optional[DeviceDescriptor]:
        """把地址/名称解析为设备描述符。

        优先使用当前扫描世代命中的描述符（交回原 provider 复核），否则按顺序
        询问各 provider；仅在候选非空时生成新描述符。
        Returns: DeviceDescriptor，无法解析时为 None。
        """
        scanned = self._scanned_descriptors.get(self._device_key(address))
        if scanned is not None:
            try:
                module = self.get_or_create(scanned.candidate.provider_id)
                candidate = module.resolve_device(address, name, scanned.candidate)
            except Exception as exc:
                self._provider_failure(scanned.candidate.provider_id, exc, "resolve")
                return None
            return None if candidate is None else self._new_descriptor(
                candidate,
                scanned.scan_generation,
            )

        for provider_id in self.provider_ids:
            try:
                module = self.get_or_create(provider_id)
                candidate = module.resolve_device(address, name, None)
            except Exception as exc:
                self._provider_failure(provider_id, exc, "resolve")
                continue
            if candidate is not None:
                # Surreal 模块对未扫描的 ANS 会刻意返回 None。
                return self._new_descriptor(candidate)
        return None

    def resolution_error(self, address: str, name: str) -> Optional[str]:
        """按顺序询问各 provider 的解析失败原因，返回首个非空消息。"""
        for provider_id in self.provider_ids:
            try:
                module = self.get_or_create(provider_id)
                resolver = getattr(module, "resolution_error", None)
                if not callable(resolver):
                    continue
                message = resolver(address, name)
            except Exception as exc:
                self._provider_failure(provider_id, exc, "resolve")
                continue
            if message:
                return message
        return None

    def activate(self, lease: SourceLease) -> InputSourceModule:
        """登记 active lease，并要求目标模块接受该 lease。

        Returns: 接受该 lease 的模块实例。
        Raises: RuntimeError 上一 lease 尚未完成关闭，或模块未接受该 lease。
        """
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
        """开始关闭指定 lease（进入 closing 过渡态）。

        Returns: lease 匹配且不处于其他关闭流程时为 True。
        """
        if self._active_lease != lease:
            return False
        if self._closing_lease not in (None, lease):
            return False
        self._closing_lease = lease
        return True

    def cleanup_lease(self) -> Optional[SourceLease]:
        """返回当前待清理的 lease（closing 优先于 active）。"""
        return self._closing_lease or self._active_lease

    def complete_close(self, lease: SourceLease) -> bool:
        """完成关闭：清空 active 与 closing lease。

        Returns: active 与 closing 均等于该 lease 时为 True。
        """
        if self._active_lease != lease or self._closing_lease != lease:
            return False
        self._active_lease = None
        self._closing_lease = None
        return True

    def active_module(self) -> Optional[InputSourceModule]:
        """返回 active lease 对应的模块实例（无 active 或未创建时为 None）。"""
        lease = self._active_lease
        return None if lease is None else self._modules.get(lease.provider_id)

    def cleanup_idle_resources(self) -> bool:
        """清理不属于当前关闭 lease 的固定 provider owner。

        跳过正在关闭的那个 provider；仅对 ``current_lease() is None`` 的空闲模块
        调用其清理接口，任一失败即返回 False。
        """
        success = True
        cleanup_lease = self.cleanup_lease()
        for provider_id in self.provider_ids:
            if (
                cleanup_lease is not None
                and cleanup_lease.provider_id == provider_id
            ):
                continue
            module = self._modules.get(provider_id)
            if module is None or module.current_lease() is not None:
                continue
            try:
                cleaned = bool(module.cleanup_idle_resources())
            except Exception:
                logging.exception(
                    "[SourceRegistry] Idle Provider cleanup failed: %s",
                    provider_id,
                )
                cleaned = False
            success = cleaned and success
        return success

    def retire_module(self, provider_id: str, reason: str) -> bool:
        """停用单个 provider 模块：active 中则拒绝，成功后清除实例与 lease 身份。"""
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
        """逐个停用全部 provider 模块；全部成功才清空 active/closing lease。"""
        success = True
        for provider_id in self.provider_ids:
            module = self._modules.get(provider_id)
            if module is None:
                continue
            try:
                retired = self.retire_module(provider_id, reason)
            except Exception:
                logging.exception(
                    "[SourceRegistry] Provider shutdown failed: %s",
                    provider_id,
                )
                retired = False
            success = retired and success
        if success:
            self._active_lease = None
            self._closing_lease = None
        return success
