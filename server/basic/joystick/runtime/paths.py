import importlib
import importlib.util
import logging
import os
from pathlib import Path
import sys
from typing import Optional


PRODUCTION_DEPENDENCIES_PATH = Path("/var/www/backend/carm_backend/dependencies")
REPOSITORY_INCLUDE_PATH = Path(__file__).resolve().parents[4] / "include"
DEFAULT_SURREAL_DRIVER_PATH = Path("/home/cvte/carm_surreal")
QUEST_ONLY_PROVIDERS_ENV = "CARM_JOYSTICK_PROVIDERS"
QUEST_REQUIRED_FILES = (
    "__init__.py", "adb.py", "config.py", "constants.py", "module.py",
    "parser.py", "policy.py", "receiver.py",
)


def quest_only_providers_enabled() -> bool:
    """Return whether this process explicitly selected the desktop Quest profile."""
    return os.environ.get(QUEST_ONLY_PROVIDERS_ENV, "").strip().lower() == "quest"


def _repository_quest_root() -> Path:
    root = (REPOSITORY_INCLUDE_PATH / "quest").resolve()
    missing = [name for name in QUEST_REQUIRED_FILES if not (root / name).is_file()]
    if missing:
        raise ImportError(f"Desktop Quest 模块缺失: {', '.join(missing)}")
    return root


def _validate_quest_module_source() -> None:
    root = _repository_quest_root()
    module = importlib.import_module("quest")
    module_file = getattr(module, "__file__", None)
    if not module_file or root not in Path(module_file).resolve().parents:
        raise ImportError(f"Desktop Quest 模块来源冲突: {module_file}")


def canonical_input_module_root() -> Path:
    """Select the backend-owned provider root for this runtime."""
    if quest_only_providers_enabled():
        return REPOSITORY_INCLUDE_PATH
    if PRODUCTION_DEPENDENCIES_PATH.is_dir():
        return PRODUCTION_DEPENDENCIES_PATH
    return REPOSITORY_INCLUDE_PATH


def resolve_surreal_driver_root() -> Optional[Path]:
    """Resolve the legacy Surreal package directory without mutating sys.path."""
    configured = Path(
        os.environ.get("CARM_SURREAL_PATH", str(DEFAULT_SURREAL_DRIVER_PATH))
    ).expanduser()
    candidates = (configured / "surreal", configured)
    for candidate in candidates:
        if (candidate / "surreal_sdk.py").is_file():
            return candidate.resolve()
    return None


def _surreal_modules_match(
    package_root: Path,
    driver_root: Optional[Path],
) -> bool:
    expected_sdk = (
        (driver_root / "surreal_sdk.py").resolve()
        if driver_root is not None
        else (package_root / "surreal_sdk.py").resolve()
    )
    for name, module in tuple(sys.modules.items()):
        if name != "surreal" and not name.startswith("surreal."):
            continue
        module_file = getattr(module, "__file__", None)
        if not module_file:
            return False
        module_path = Path(module_file).resolve()
        if name == "surreal.surreal_sdk":
            if module_path != expected_sdk:
                return False
        elif module_path != package_root / "__init__.py" and package_root not in module_path.parents:
            return False
    return True


def _ensure_canonical_surreal_package(
    module_root: Path,
    driver_root: Optional[Path],
):
    package_root = (module_root / "surreal").resolve()
    package_init = package_root / "__init__.py"
    if not package_init.is_file():
        raise ImportError(f"Surreal canonical package 缺失: {package_init}")

    if not _surreal_modules_match(package_root, driver_root):
        for name in tuple(sys.modules):
            if name == "surreal" or name.startswith("surreal."):
                sys.modules.pop(name, None)

    package = importlib.import_module("surreal")
    package_file = getattr(package, "__file__", None)
    if not package_file or Path(package_file).resolve() != package_init:
        raise ImportError("Surreal canonical package identity 校验失败")
    return package


def _configure_surreal_driver_override(driver_root: Path, surreal) -> None:
    """Load only the validated SDK leaf while preserving the canonical package."""
    module_name = "surreal.surreal_sdk"
    module_path = (driver_root / "surreal_sdk.py").resolve()
    existing = sys.modules.get(module_name)
    if existing is not None:
        existing_path = getattr(existing, "__file__", None)
        if existing_path and Path(existing_path).resolve() == module_path:
            surreal.surreal_sdk = existing
            return
        raise RuntimeError(
            "Surreal SDK override must be configured before the provider is imported"
        )

    spec = importlib.util.spec_from_file_location(module_name, module_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"无法加载 Surreal SDK override: {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
        if not hasattr(module, "BLEDevice") or not hasattr(module, "SurrealSdk"):
            raise ImportError("Surreal SDK override 缺少 BLEDevice 或 SurrealSdk")
    except Exception:
        sys.modules.pop(module_name, None)
        raise
    surreal.surreal_sdk = module


def setup_import_paths() -> None:
    """Expose one canonical root for the built-in Quest and Surreal modules."""
    try:
        module_root = canonical_input_module_root()
        module_root_text = str(module_root)
        if module_root.is_dir():
            while module_root_text in sys.path:
                sys.path.remove(module_root_text)
            sys.path.insert(0, module_root_text)
            logging.debug("[Joystick] Added canonical input module root: %s", module_root)
        if quest_only_providers_enabled():
            _validate_quest_module_source()
            return

        if not module_root.is_dir():
            return

        driver_root = resolve_surreal_driver_root()
        surreal = _ensure_canonical_surreal_package(module_root, driver_root)
        if driver_root is not None:
            _configure_surreal_driver_override(driver_root, surreal)
            logging.debug(
                "[Joystick] Added validated Surreal driver override: %s",
                driver_root,
            )
    except Exception as exc:
        if quest_only_providers_enabled():
            raise
        logging.error("[Joystick] Error setting up paths: %s", exc)
