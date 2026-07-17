"""Minimal bundled CoACD bridge used for convex decomposition.

The official CoACD Python package is a small ctypes wrapper around one native
library.  Keeping the wrapper here avoids installing packages into Blender and
lets the add-on ship matching Windows/Linux binaries.
"""

from __future__ import annotations

import ctypes
import os
import platform
from ctypes import POINTER, Structure, c_bool, c_char_p, c_double, c_int, c_uint, c_uint64
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

try:
    import numpy as np
except Exception:  # Blender normally includes NumPy.
    np = None

COACD_VERSION = "1.0.11"


class CoACDError(RuntimeError):
    pass


class _CoACDMesh(Structure):
    _fields_ = [
        ("vertices_ptr", POINTER(c_double)),
        ("vertices_count", c_uint64),
        ("triangles_ptr", POINTER(c_int)),
        ("triangles_count", c_uint64),
    ]


class _CoACDMeshArray(Structure):
    _fields_ = [("meshes_ptr", POINTER(_CoACDMesh)), ("meshes_count", c_uint64)]


_LIBRARY = None
_LIBRARY_PATH: str | None = None


def _candidate_library() -> Path:
    root = Path(__file__).resolve().parents[1] / "vendor" / "coacd"
    system = platform.system().lower()
    machine = platform.machine().lower()
    if machine not in {"x86_64", "amd64"}:
        raise CoACDError(f"CoACD is bundled only for x86-64, not {machine or 'unknown architecture'}.")
    if system == "windows":
        return root / "win_amd64" / "lib_coacd.dll"
    if system == "linux":
        return root / "linux_x86_64" / "lib_coacd.so"
    raise CoACDError(f"CoACD is not bundled for {platform.system() or 'this platform'}.")


def load_coacd():
    global _LIBRARY, _LIBRARY_PATH
    if _LIBRARY is not None:
        return _LIBRARY
    if np is None:
        raise CoACDError("CoACD requires NumPy, but NumPy is unavailable in this Blender build.")
    path = _candidate_library()
    if not path.is_file():
        raise CoACDError(f"Bundled CoACD library is missing: {path}")
    try:
        library = ctypes.CDLL(str(path))
    except OSError as exc:
        raise CoACDError(f"Bundled CoACD could not be loaded: {exc}") from exc

    library.CoACD_setLogLevel.argtypes = [c_char_p]
    library.CoACD_setLogLevel.restype = None
    library.CoACD_freeMeshArray.argtypes = [_CoACDMeshArray]
    library.CoACD_freeMeshArray.restype = None
    library.CoACD_run.argtypes = [
        POINTER(_CoACDMesh), c_double, c_int, c_int, c_int, c_int, c_int, c_int,
        c_int, c_bool, c_bool, c_bool, c_int, c_bool, c_double, c_int, c_uint, c_bool,
    ]
    library.CoACD_run.restype = _CoACDMeshArray
    library.CoACD_setLogLevel(b"off")
    _LIBRARY = library
    _LIBRARY_PATH = str(path)
    return library


def coacd_status() -> Tuple[bool, str]:
    try:
        load_coacd()
        return True, f"CoACD {COACD_VERSION} ({_LIBRARY_PATH})"
    except Exception as exc:
        return False, str(exc)


def decompose(
    vertices: Sequence[Sequence[float]],
    triangles: Sequence[Sequence[int]] | Sequence[int],
    settings: Dict[str, object],
) -> List[Dict[str, object]]:
    """Return convex pieces as local vertices/triangle indices.

    ``threshold`` is interpreted in scene units because real_metric is enabled.
    The input arrays remain alive until the native call returns.
    """
    library = load_coacd()
    vertex_array = np.ascontiguousarray(vertices, dtype=np.float64)
    if vertex_array.ndim != 2 or vertex_array.shape[1] != 3 or vertex_array.shape[0] < 4:
        raise CoACDError("CoACD needs at least four 3D vertices.")
    triangle_array = np.ascontiguousarray(triangles, dtype=np.int32)
    if triangle_array.ndim == 1:
        if triangle_array.size % 3:
            raise CoACDError("Triangle index count is not divisible by three.")
        triangle_array = triangle_array.reshape((-1, 3))
    if triangle_array.ndim != 2 or triangle_array.shape[1] != 3 or triangle_array.shape[0] < 4:
        raise CoACDError("CoACD needs a closed triangle mesh with at least four faces.")

    mesh = _CoACDMesh()
    mesh.vertices_ptr = ctypes.cast(vertex_array.ctypes.data, POINTER(c_double))
    mesh.vertices_count = int(vertex_array.shape[0])
    mesh.triangles_ptr = ctypes.cast(triangle_array.ctypes.data, POINTER(c_int))
    mesh.triangles_count = int(triangle_array.shape[0])

    threshold = max(1.0e-7, float(settings.get("threshold", 0.003)))
    max_parts = max(1, int(settings.get("max_parts", 8)))
    preprocess_mode = str(settings.get("preprocess_mode", "AUTO")).upper()
    preprocess_value = 1 if preprocess_mode == "ON" else 2 if preprocess_mode == "OFF" else 0
    result = library.CoACD_run(
        ctypes.byref(mesh),
        threshold,
        max_parts,
        preprocess_value,
        max(10, int(settings.get("preprocess_resolution", 50))),
        max(100, int(settings.get("resolution", 2000))),
        max(1, int(settings.get("mcts_nodes", 20))),
        max(1, int(settings.get("mcts_iterations", 150))),
        max(1, int(settings.get("mcts_max_depth", 3))),
        bool(settings.get("pca", False)),
        bool(settings.get("merge", True)),
        bool(settings.get("decimate", False)),
        max(4, int(settings.get("max_hull_vertices", 96))),
        bool(settings.get("extrude", False)),
        max(0.0, float(settings.get("extrude_margin", 0.0))),
        0,  # convex-hull approximation, not boxes
        max(0, int(settings.get("seed", 0))),
        True,  # real metric / Blender scene units
    )

    pieces: List[Dict[str, object]] = []
    try:
        for index in range(int(result.meshes_count)):
            native_mesh = result.meshes_ptr[index]
            piece_vertices = np.ctypeslib.as_array(
                native_mesh.vertices_ptr, (int(native_mesh.vertices_count), 3)
            ).copy()
            piece_triangles = np.ctypeslib.as_array(
                native_mesh.triangles_ptr, (int(native_mesh.triangles_count), 3)
            ).copy()
            if len(piece_vertices) < 4 or len(piece_triangles) < 4:
                continue
            pieces.append({
                "vertices": piece_vertices.tolist(),
                "indices": piece_triangles.reshape(-1).astype(np.int32).tolist(),
            })
    finally:
        library.CoACD_freeMeshArray(result)
    if not pieces:
        raise CoACDError("CoACD returned no valid convex pieces.")
    return pieces
