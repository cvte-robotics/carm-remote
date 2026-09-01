"""Dynamic end-effector implementations for joystick arm runtimes."""

from abc import ABC, abstractmethod
from dataclasses import dataclass
import importlib
import re
from typing import Callable, ClassVar, Dict, Tuple, Type


@dataclass(frozen=True)
class EndEffectorContext:
    carm: object
    side: str
    role: str


class EndEffectorBase(ABC):
    """Base class and package-local dynamic registry for end effectors."""

    NAME: ClassVar[str] = ""
    _registered: Dict[str, Type["EndEffectorBase"]] = {}

    def __init__(self, context: EndEffectorContext, **options) -> None:
        self.context = context
        self.carm = context.carm
        self.side = context.side
        self.role = context.role
        self.options = dict(options)

    def __init_subclass__(cls, **kwargs) -> None:
        super().__init_subclass__(**kwargs)
        name = str(getattr(cls, "NAME", "") or "").strip().lower()
        if not re.fullmatch(r"[a-z][a-z0-9_]*", name):
            raise TypeError("end-effector subclass must define a valid NAME")
        cls.NAME = name
        EndEffectorBase._registered[name] = cls

    @classmethod
    def _normalize_name(cls, name: str) -> str:
        normalized = (name or "").strip().lower()
        if not re.fullmatch(r"[a-z][a-z0-9_]*", normalized):
            raise ValueError("invalid end-effector name: %s" % name)
        return normalized

    @classmethod
    def _load_module(cls, name: str) -> None:
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
        """Load all package-local implementation modules for CLI discovery."""
        import pkgutil

        for module in pkgutil.iter_modules(__path__):
            if module.name.startswith("_"):
                continue
            cls._load_module(module.name)
        return cls.available_names()

    @classmethod
    def available_names(cls) -> Tuple[str, ...]:
        return tuple(sorted(cls._registered))

    @classmethod
    def create(cls, name: str, context: EndEffectorContext, **options):
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
        """Interpret and submit one provider-normalized trigger value."""


EndEffectorDriver = EndEffectorBase
EndEffectorFactory = Callable[..., EndEffectorDriver]


def validate_end_effector_driver(driver) -> EndEffectorDriver:
    sender = getattr(driver, "set_eff", None)
    if not isinstance(driver, EndEffectorBase):
        raise TypeError("end-effector driver must inherit EndEffectorBase")
    if not callable(sender):
        raise TypeError("end-effector driver must provide callable set_eff")
    return driver


def resolve_end_effector_factory(name: str) -> EndEffectorFactory:
    normalized = EndEffectorBase._normalize_name(name)
    EndEffectorBase._load_module(normalized)

    def factory(**context):
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
