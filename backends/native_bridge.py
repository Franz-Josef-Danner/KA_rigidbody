"""ctypes loader for future native Jolt/PhysX bridge libraries."""

from __future__ import annotations

import ctypes
import os
from typing import Optional

EXPECTED_ABI_VERSION = 1


class NativeBridgeLoadError(RuntimeError):
    pass


def load_bridge(path: str) -> ctypes.CDLL:
    path = os.path.abspath(os.path.expanduser(path))
    if not os.path.isfile(path):
        raise NativeBridgeLoadError(f"Native bridge not found: {path}")
    try:
        library = ctypes.CDLL(path)
    except OSError as exc:
        raise NativeBridgeLoadError(str(exc)) from exc

    if not hasattr(library, "ka_physics_abi_version"):
        raise NativeBridgeLoadError("Library does not expose ka_physics_abi_version")
    library.ka_physics_abi_version.restype = ctypes.c_int
    abi_version = int(library.ka_physics_abi_version())
    if abi_version != EXPECTED_ABI_VERSION:
        raise NativeBridgeLoadError(
            f"Native bridge ABI {abi_version} does not match expected ABI {EXPECTED_ABI_VERSION}"
        )
    return library


def bridge_status(path: str) -> tuple[bool, str]:
    if not path:
        return False, "No native bridge path configured."
    try:
        load_bridge(path)
    except NativeBridgeLoadError as exc:
        return False, str(exc)
    return True, "Native bridge loaded and ABI validated."
