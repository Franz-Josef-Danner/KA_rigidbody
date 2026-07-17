"""Load the bundled Culverin/Jolt binary for the active Blender Python platform."""

from __future__ import annotations

import importlib
import os
import platform
import sys
from pathlib import Path
from types import ModuleType

BUNDLED_CULVERIN_VERSION = "0.13.2"


class CulverinLoadError(RuntimeError):
    pass


def _vendor_directory() -> Path:
    root = Path(__file__).resolve().parent.parent
    machine = platform.machine().lower()
    py_tag = f"cp{sys.version_info.major}{sys.version_info.minor}"

    if py_tag != "cp313":
        raise CulverinLoadError(
            f"Bundled Jolt requires CPython 3.13; Blender is using {sys.version_info.major}.{sys.version_info.minor}."
        )

    if sys.platform == "win32" and machine in {"amd64", "x86_64"}:
        folder = "win_amd64_cp313"
    elif sys.platform.startswith("linux") and machine in {"amd64", "x86_64"}:
        folder = "linux_x86_64_cp313"
    else:
        raise CulverinLoadError(f"No bundled Jolt binary for platform {sys.platform}/{machine}/{py_tag}.")

    path = root / "vendor" / "culverin" / folder
    if not path.is_dir():
        raise CulverinLoadError(f"Bundled Jolt runtime is missing: {path}")
    return path


def load_culverin() -> ModuleType:
    existing = sys.modules.get("culverin")
    if existing is not None:
        version = str(getattr(existing, "__version__", "unknown")).split()[0]
        if version != BUNDLED_CULVERIN_VERSION:
            raise CulverinLoadError(
                f"Culverin {version} is already loaded; KA Rigid Dynamics requires {BUNDLED_CULVERIN_VERSION}."
            )
        return existing

    vendor = _vendor_directory()
    vendor_text = str(vendor)
    if vendor_text not in sys.path:
        sys.path.insert(0, vendor_text)
    importlib.invalidate_caches()

    try:
        module = importlib.import_module("culverin")
    except Exception as exc:
        raise CulverinLoadError(f"Bundled Jolt runtime could not be loaded: {exc}") from exc

    version = str(getattr(module, "__version__", "unknown")).split()[0]
    if version != BUNDLED_CULVERIN_VERSION:
        raise CulverinLoadError(
            f"Loaded Culverin {version}; expected {BUNDLED_CULVERIN_VERSION}."
        )
    return module


def culverin_status() -> tuple[bool, str]:
    try:
        module = load_culverin()
    except CulverinLoadError as exc:
        return False, str(exc)
    version = str(getattr(module, "__version__", BUNDLED_CULVERIN_VERSION))
    return True, f"Bundled native Jolt runtime loaded via Culverin {version}."
