"""joystick 机械臂运行时可动态装载的 end-effector 实现。"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
import importlib
import re
from typing import Callable, ClassVar, Dict, Tuple, Type


@dataclass(frozen=True)
class EndEffectorContext:
    """单个 end-effector 实例的构造上下文（carm 句柄、臂侧与角色）。"""

    carm: object
    side: str
    role: str


class EndEffectorBase(ABC):
    """end-effector 基类，同时充当包内本地动态注册表。"""

    NAME: ClassVar[str] = ""
    _registered: Dict[str, Type["EndEffectorBase"]] = {}

    def __init__(self, context: EndEffectorContext, **options) -> None:
        """绑定上下文，并解包 carm/side/role 与额外选项。"""
        self.context = context
        self.carm = context.carm
        self.side = context.side
        self.role = context.role
        self.options = dict(options)

    def __init_subclass__(cls, **kwargs) -> None:
        """子类定义时校验 NAME 并登记到本地注册表。"""
        super().__init_subclass__(**kwargs)
        name = str(getattr(cls, "NAME", "") or "").strip().lower()
        if not re.fullmatch(r"[a-z][a-z0-9_]*", name):
            raise TypeError("end-effector subclass must define a valid NAME")
        cls.NAME = name
        EndEffectorBase._registered[name] = cls

    @classmethod
    def _normalize_name(cls, name: str) -> str:
        """把 end-effector 名称规范化为小写并校验命名格式。"""
        normalized = (name or "").strip().lower()
        if not re.fullmatch(r"[a-z][a-z0-9_]*", normalized):
            raise ValueError("invalid end-effector name: %s" % name)
        return normalized

    @classmethod
    def _load_module(cls, name: str) -> None:
        """尝试导入包内同名实现模块（模块不存在时静默返回）。"""
        package = __name__
        module_name = "%s.%s" % (package, name)
        try:
            importlib.import_module(module_name)
        except ModuleNotFoundError as exc:
            if exc.name == module_name:
                return
            raise

    @classmethod
    def load_all(cls) -> Tuple[str, ...]:
        """加载包内全部实现模块，供 CLI 发现可用名称。"""
        import pkgutil

        for module in pkgutil.iter_modules(__path__):
            if module.name.startswith("_"):
                continue
            cls._load_module(module.name)
        return cls.available_names()

    @classmethod
    def available_names(cls) -> Tuple[str, ...]:
        """返回已注册的 end-effector 名称（字典序）。"""
        return tuple(sorted(cls._registered))

    @classmethod
    def create(cls, name: str, context: EndEffectorContext, **options):
        """按名称创建 end-effector 实例，并通过驱动契约校验。

        Raises: ValueError 名称非法或实现未注册。
        """
        normalized = cls._normalize_name(name)
        implementation = cls._registered.get(normalized)
        if implementation is None:
            cls._load_module(normalized)
            implementation = cls._registered.get(normalized)
        if implementation is None:
            raise ValueError(
                "unsupported end-effector implementation: %s" % name
            )
        instance = implementation(context, **options)
        return validate_end_effector_driver(instance)

    @abstractmethod
    def set_eff(self, trigger: int) -> bool:
        """解释并提交一次 provider 归一化后的 trigger 值。

        Args: trigger 为 provider 归一化后的触发值。
        Returns: 下发成功时为 True。
        Raises: ValueError trigger 非法。
        """


EndEffectorDriver = EndEffectorBase
EndEffectorFactory = Callable[..., EndEffectorDriver]


def validate_end_effector_driver(driver) -> EndEffectorDriver:
    """校验对象继承 EndEffectorBase 且 set_eff 可调用。

    Returns: 原对象（便于链式调用）。
    Raises: TypeError 不满足驱动契约。
    """
    sender = getattr(driver, "set_eff", None)
    if not isinstance(driver, EndEffectorBase):
        raise TypeError("end-effector driver must inherit EndEffectorBase")
    if not callable(sender):
        raise TypeError("end-effector driver must provide callable set_eff")
    return driver


def resolve_end_effector_factory(name: str) -> EndEffectorFactory:
    """按名称解析出 end-effector 工厂（延迟创建实例）。

    Args: name 配置中的实现名（大小写不敏感）。
    Returns: 接受 carm/side/role 关键字的工厂可调用对象。
    Raises: ValueError 名称非法或实现未注册。
    """
    normalized = EndEffectorBase._normalize_name(name)
    EndEffectorBase._load_module(normalized)

    def factory(**context):
        """按已解析名称延迟创建 end-effector 实例。"""
        return EndEffectorBase.create(
            normalized,
            EndEffectorContext(
                carm=context["carm"],
                side=context["side"],
                role=context.get("role", "master"),
            ),
        )

    return factory


from . import carm as _carm  # noqa: F401,E402
from . import passthrough as _passthrough  # noqa: F401,E402


__all__ = [
    "EndEffectorBase",
    "EndEffectorContext",
    "EndEffectorDriver",
    "EndEffectorFactory",
    "resolve_end_effector_factory",
    "validate_end_effector_driver",
]
