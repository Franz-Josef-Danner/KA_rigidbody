"""Shared ctypes loader for native KA physics bridge libraries."""

from __future__ import annotations

import ctypes
import os

EXPECTED_ABI_VERSION = 2


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
        library = load_bridge(path)
        if hasattr(library, "ka_physics_backend_name"):
            library.ka_physics_backend_name.restype = ctypes.c_char_p
            raw_name = library.ka_physics_backend_name()
            name = raw_name.decode("utf-8", "replace") if raw_name else "Native bridge"
        else:
            name = "Native bridge"
        if hasattr(library, "ka_physics_backend_version"):
            library.ka_physics_backend_version.restype = ctypes.c_char_p
            raw_version = library.ka_physics_backend_version()
            version = raw_version.decode("utf-8", "replace") if raw_version else "unknown"
        else:
            version = "unknown"
    except NativeBridgeLoadError as exc:
        return False, str(exc)
    return True, f"{name} {version} loaded; ABI {EXPECTED_ABI_VERSION} validated."
