"""Standalone worker for crash-isolated CoACD decomposition."""

from __future__ import annotations

import ctypes
import json
import os
import sys
from pathlib import Path

import numpy as np

from coacd_bridge import _decompose_in_process


def _suppress_windows_crt_dialogs() -> None:
    """Route native assertion text to stderr instead of a modal dialog."""
    if os.name != "nt":
        return
    try:
        # _OUT_TO_STDERR. This affects normal release-CRT assertions.
        msvcrt = ctypes.CDLL("msvcrt")
        setter = getattr(msvcrt, "_set_error_mode", None)
        if setter is not None:
            setter.argtypes = [ctypes.c_int]
            setter.restype = ctypes.c_int
            setter(2)
    except Exception:
        pass
    try:
        # Suppress OS-level critical-error and fault message boxes as well.
        ctypes.windll.kernel32.SetErrorMode(0x0001 | 0x0002 | 0x8000)
    except Exception:
        pass


def main(argv) -> int:
    _suppress_windows_crt_dialogs()
    if len(argv) != 3:
        return 64
    input_path = Path(argv[1])
    output_path = Path(argv[2])
    temporary = output_path.with_suffix(output_path.suffix + ".tmp")
    try:
        with np.load(input_path, allow_pickle=False) as payload:
            vertices = np.asarray(payload["vertices"], dtype=np.float64)
            triangles = np.asarray(payload["triangles"], dtype=np.int32)
            settings = json.loads(str(payload["settings_json"].item()))
        parts = _decompose_in_process(vertices, triangles, settings)
        result = {"ok": True, "parts": parts}
    except BaseException as exc:
        result = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
    try:
        temporary.write_text(json.dumps(result, separators=(",", ":")), encoding="utf-8")
        os.replace(temporary, output_path)
    except Exception:
        return 74
    return 0 if result.get("ok") else 70


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
