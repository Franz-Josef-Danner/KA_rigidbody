"""Crash-isolated bundled CoACD bridge used for convex decomposition.

The official CoACD Python package is a small ctypes wrapper around one native
library. Calling that library in Blender's process is unsafe for production:
a malformed or numerically difficult mesh can terminate Blender before Python
can raise an exception. KA Rigid Dynamics therefore executes each native CoACD job in Blender's bundled Python interpreter. On Windows,
where the bundled DLL can open a blocking MSVC assertion dialog, the default
path is a deterministic native-free conservative interior-box decomposition and only imports validated
results back into the Blender process.
"""

from __future__ import annotations

import ctypes
import json
import os
import platform
import subprocess
import sys
import tempfile
from ctypes import POINTER, Structure, c_bool, c_char_p, c_double, c_int, c_uint, c_uint64
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

try:
    import numpy as np
except Exception:  # Blender normally includes NumPy.
    np = None

COACD_VERSION = "1.0.11"
COACD_EXECUTION_MODE = "SAFE_INTERIOR_BOXES_V2" if os.name == "nt" else "ISOLATED_SUBPROCESS_V2"


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
_WORKER_PYTHON: str | None = None


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
    """Load CoACD in the current process.

    Production decomposition does not call this in Blender itself; it is used by
    the isolated worker and by the lightweight availability probe only.
    """
    global _LIBRARY, _LIBRARY_PATH
    if _LIBRARY is not None:
        return _LIBRARY
    if np is None:
        raise CoACDError("CoACD requires NumPy, but NumPy is unavailable in this Python build.")
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


def _worker_python_candidates() -> List[Path]:
    override = str(os.environ.get("KA_COACD_PYTHON", "") or "").strip()
    candidates: List[Path] = []
    if override:
        candidates.append(Path(override))

    executable = Path(sys.executable)
    if executable.stem.lower().startswith("python"):
        candidates.append(executable)

    prefixes = []
    for value in (sys.prefix, sys.base_prefix):
        if value and value not in prefixes:
            prefixes.append(value)
    version = f"{sys.version_info.major}.{sys.version_info.minor}"
    for prefix_value in prefixes:
        prefix = Path(prefix_value)
        candidates.extend((
            prefix / "bin" / "python.exe",
            prefix / "bin" / f"python{version}.exe",
            prefix / "bin" / "python3.exe",
            prefix / "python.exe",
            prefix / "bin" / f"python{version}",
            prefix / "bin" / "python3",
            prefix / "bin" / "python",
        ))
        bin_dir = prefix / "bin"
        if bin_dir.is_dir():
            candidates.extend(sorted(bin_dir.glob("python*")))

    unique: List[Path] = []
    seen = set()
    for candidate in candidates:
        try:
            resolved = candidate.resolve()
        except OSError:
            resolved = candidate
        key = os.path.normcase(str(resolved))
        if key in seen:
            continue
        seen.add(key)
        if resolved.is_file():
            unique.append(resolved)
    return unique


def worker_python() -> str:
    global _WORKER_PYTHON
    if _WORKER_PYTHON and Path(_WORKER_PYTHON).is_file():
        return _WORKER_PYTHON
    candidates = _worker_python_candidates()
    if not candidates:
        raise CoACDError(
            "No standalone Python interpreter was found for crash-isolated CoACD. "
            "Set KA_COACD_PYTHON to Blender's bundled python executable."
        )
    _WORKER_PYTHON = str(candidates[0])
    return _WORKER_PYTHON


def coacd_status() -> Tuple[bool, str]:
    try:
        if np is None:
            raise CoACDError("Compound decomposition requires NumPy, but NumPy is unavailable in this Blender build.")
        native_windows_opt_in = str(os.environ.get("KA_COACD_NATIVE_WINDOWS", "0")).strip().lower() in {"1", "true", "yes", "on"}
        if os.name == "nt" and not native_windows_opt_in:
            return True, "Safe conservative interior-box Compound Convex decomposition (native CoACD disabled on Windows to prevent blocking CRT assertions and collider overfill)"
        library = _candidate_library()
        if not library.is_file():
            raise CoACDError(f"Bundled CoACD library is missing: {library}")
        python = worker_python()
        return True, f"CoACD {COACD_VERSION} ({COACD_EXECUTION_MODE}, worker={python})"
    except Exception as exc:
        return False, str(exc)


def _normalize_input(
    vertices: Sequence[Sequence[float]],
    triangles: Sequence[Sequence[int]] | Sequence[int],
):
    if np is None:
        raise CoACDError("CoACD requires NumPy, but NumPy is unavailable in this Python build.")
    vertex_array = np.ascontiguousarray(vertices, dtype=np.float64)
    if vertex_array.ndim != 2 or vertex_array.shape[1] != 3 or vertex_array.shape[0] < 4:
        raise CoACDError("CoACD needs at least four 3D vertices.")
    if not bool(np.isfinite(vertex_array).all()):
        raise CoACDError("CoACD input contains non-finite vertex coordinates.")

    triangle_array = np.ascontiguousarray(triangles, dtype=np.int32)
    if triangle_array.ndim == 1:
        if triangle_array.size % 3:
            raise CoACDError("Triangle index count is not divisible by three.")
        triangle_array = triangle_array.reshape((-1, 3))
    if triangle_array.ndim != 2 or triangle_array.shape[1] != 3 or triangle_array.shape[0] < 4:
        raise CoACDError("CoACD needs a closed triangle mesh with at least four faces.")
    if int(triangle_array.min(initial=0)) < 0 or int(triangle_array.max(initial=0)) >= int(vertex_array.shape[0]):
        raise CoACDError("CoACD triangle indices reference vertices outside the input array.")
    return vertex_array, triangle_array




def _sanitize_mesh_arrays(vertex_array, triangle_array):
    """Remove invalid, duplicate and zero-area triangles before decomposition."""
    if np is None:
        raise CoACDError("CoACD requires NumPy, but NumPy is unavailable in this Python build.")
    vertices = np.ascontiguousarray(vertex_array, dtype=np.float64)
    triangles = np.ascontiguousarray(triangle_array, dtype=np.int32)
    if len(vertices) < 4 or len(triangles) < 4:
        raise CoACDError("Compound decomposition needs at least four vertices and four triangles.")

    extent = np.ptp(vertices, axis=0)
    characteristic = max(1.0, float(np.linalg.norm(extent)))
    area_epsilon_sq = max(1.0e-30, (characteristic * characteristic * 1.0e-12) ** 2)
    kept = []
    seen = set()
    for triangle in triangles:
        a, b, c = map(int, triangle)
        if a == b or b == c or c == a:
            continue
        key = tuple(sorted((a, b, c)))
        if key in seen:
            continue
        pa, pb, pc = vertices[a], vertices[b], vertices[c]
        cross = np.cross(pb - pa, pc - pa)
        if float(np.dot(cross, cross)) <= area_epsilon_sq:
            continue
        seen.add(key)
        kept.append((a, b, c))
    if len(kept) < 4:
        raise CoACDError("Mesh cleanup left fewer than four usable triangles.")

    triangles = np.asarray(kept, dtype=np.int32)
    used = np.unique(triangles.reshape(-1))
    if len(used) < 4:
        raise CoACDError("Mesh cleanup left fewer than four usable vertices.")
    remap = np.full(len(vertices), -1, dtype=np.int32)
    remap[used] = np.arange(len(used), dtype=np.int32)
    vertices = np.ascontiguousarray(vertices[used], dtype=np.float64)
    triangles = np.ascontiguousarray(remap[triangles], dtype=np.int32)
    return vertices, triangles


def _points_inside_mesh(vertex_array, triangle_array, points):
    """Classify points with a deterministic non-axis-aligned parity ray.

    The ray direction deliberately avoids the principal axes so shared edges and
    vertices are very unlikely to be hit exactly. Work is batched to keep the
    temporary NumPy arrays bounded for high-detail fracture meshes.
    """
    if np is None:
        raise CoACDError("Safe compound decomposition requires NumPy.")
    points = np.ascontiguousarray(points, dtype=np.float64)
    if not len(points):
        return np.zeros(0, dtype=np.bool_)
    tri = vertex_array[triangle_array]
    v0 = tri[:, 0]
    edge1 = tri[:, 1] - v0
    edge2 = tri[:, 2] - v0
    direction = np.asarray((1.0, 0.3713906763541037, 0.17320508075688773), dtype=np.float64)
    direction /= max(float(np.linalg.norm(direction)), 1.0e-18)
    h = np.cross(np.broadcast_to(direction, edge2.shape), edge2)
    determinant = np.einsum("ij,ij->i", edge1, h)
    extent = np.ptp(vertex_array, axis=0)
    characteristic = max(1.0e-9, float(np.linalg.norm(extent)))
    epsilon = max(1.0e-12, characteristic * 1.0e-10)
    valid_triangle = np.abs(determinant) > epsilon
    inverse = np.zeros_like(determinant)
    inverse[valid_triangle] = 1.0 / determinant[valid_triangle]
    result = np.zeros(len(points), dtype=np.bool_)
    batch_size = 64
    for offset in range(0, len(points), batch_size):
        origins = points[offset:offset + batch_size].copy()
        # Deterministic sub-epsilon jitter further avoids edge coincidences.
        ids = np.arange(offset, offset + len(origins), dtype=np.float64)
        origins[:, 1] += (((ids * 0.6180339887498948) % 1.0) - 0.5) * epsilon * 7.0
        origins[:, 2] += (((ids * 0.4142135623730950) % 1.0) - 0.5) * epsilon * 11.0
        s = origins[:, None, :] - v0[None, :, :]
        u = np.einsum("bti,ti->bt", s, h) * inverse[None, :]
        q = np.cross(s, edge1[None, :, :])
        v = np.einsum("i,bti->bt", direction, q) * inverse[None, :]
        distance = np.einsum("ti,bti->bt", edge2, q) * inverse[None, :]
        hit = (
            valid_triangle[None, :]
            & (u >= -epsilon)
            & (v >= -epsilon)
            & (u + v <= 1.0 + epsilon)
            & (distance > epsilon)
        )
        result[offset:offset + len(origins)] = (np.count_nonzero(hit, axis=1) % 2) == 1
    return result


def _box_mesh(center, half):
    cx, cy, cz = map(float, center)
    hx, hy, hz = map(float, half)
    vertices = [
        [cx + sx * hx, cy + sy * hy, cz + sz * hz]
        for sx, sy, sz in (
            (-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
            (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1),
        )
    ]
    triangles = (
        (0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
        (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
        (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7),
    )
    return {"vertices": vertices, "indices": [value for triangle in triangles for value in triangle]}


def _safe_cell_components(mask):
    """Return deterministic 6-connected components of a small 3D boolean grid."""
    shape = mask.shape
    visited = np.zeros(shape, dtype=np.bool_)
    components = []
    for index in np.argwhere(mask):
        start = tuple(int(value) for value in index)
        if visited[start]:
            continue
        visited[start] = True
        stack = [start]
        cells = []
        while stack:
            cell = stack.pop()
            cells.append(cell)
            x, y, z = cell
            for neighbour in ((x - 1, y, z), (x + 1, y, z), (x, y - 1, z), (x, y + 1, z), (x, y, z - 1), (x, y, z + 1)):
                nx, ny, nz = neighbour
                if nx < 0 or ny < 0 or nz < 0 or nx >= shape[0] or ny >= shape[1] or nz >= shape[2]:
                    continue
                if visited[neighbour] or not bool(mask[neighbour]):
                    continue
                visited[neighbour] = True
                stack.append(neighbour)
        components.append(cells)
    return components


def _rectangularize_safe_cells(cells, safe_mask):
    """Split one connected cell component into all-safe axis-aligned cuboids."""
    result = []

    def recurse(group):
        if not group:
            return
        coordinates = np.asarray(group, dtype=np.int32)
        minimum = coordinates.min(axis=0)
        maximum = coordinates.max(axis=0)
        slices = tuple(slice(int(minimum[axis]), int(maximum[axis]) + 1) for axis in range(3))
        block = safe_mask[slices]
        expected = int(np.prod(maximum - minimum + 1))
        if len(group) == expected and bool(block.all()):
            result.append((tuple(map(int, minimum)), tuple(map(int, maximum))))
            return
        spans = maximum - minimum
        axes = list(np.argsort(spans)[::-1])
        for axis in axes:
            if spans[axis] <= 0:
                continue
            values = coordinates[:, axis]
            ordered = np.sort(values, kind="mergesort")
            split_value = int(ordered[len(ordered) // 2 - 1])
            left = [cell for cell in group if cell[axis] <= split_value]
            right = [cell for cell in group if cell[axis] > split_value]
            if left and right:
                recurse(left)
                recurse(right)
                return
        # Degenerate fallback: individual cells are always safe cuboids.
        for cell in sorted(group):
            result.append((cell, cell))

    recurse(cells)
    return result


def _safe_spatial_decompose(vertex_array, triangle_array, settings):
    """Build conservative interior-cell compounds without native CoACD.

    The mesh is sampled once on a regular lattice. A cell is accepted only when
    its center and all eight corners classify as interior. Connected safe cells
    are then split into non-overlapping all-safe cuboids. This is substantially
    faster and more conservative than repeatedly fitting OBBs to open surface
    clusters, and it cannot produce the many-times-overfilled proxies that caused
    the initial explosive separation in 0.6.3.
    """
    vertices, triangles = _sanitize_mesh_arrays(vertex_array, triangle_array)
    max_parts = max(1, min(32, int(settings.get("max_parts", 8))))
    minimum = vertices.min(axis=0)
    maximum = vertices.max(axis=0)
    extent = maximum - minimum
    positive = extent[extent > 1.0e-12]
    if len(positive) < 3:
        raise CoACDError("Safe compound decomposition requires a volumetric closed mesh.")

    requested_resolution = int(settings.get("preprocess_resolution", 50))
    grid_resolution = max(6, min(10, int(round(requested_resolution / 10.0)) + 3))
    node_axes = [
        np.linspace(minimum[axis], maximum[axis], grid_resolution + 1, dtype=np.float64)
        for axis in range(3)
    ]
    center_axes = [(axis[:-1] + axis[1:]) * 0.5 for axis in node_axes]
    nodes = np.asarray(
        [(x, y, z) for x in node_axes[0] for y in node_axes[1] for z in node_axes[2]],
        dtype=np.float64,
    )
    centers = np.asarray(
        [(x, y, z) for x in center_axes[0] for y in center_axes[1] for z in center_axes[2]],
        dtype=np.float64,
    )
    classifications = _points_inside_mesh(vertices, triangles, np.concatenate((nodes, centers), axis=0))
    node_inside = classifications[:len(nodes)].reshape((grid_resolution + 1,) * 3)
    center_inside = classifications[len(nodes):].reshape((grid_resolution,) * 3)

    safe = center_inside.copy()
    for dx in (0, 1):
        for dy in (0, 1):
            for dz in (0, 1):
                safe &= node_inside[
                    dx:dx + grid_resolution,
                    dy:dy + grid_resolution,
                    dz:dz + grid_resolution,
                ]
    if not bool(safe.any()):
        raise CoACDError("Safe interior cell sampling found no fully contained cells; use the single convex hull fallback.")

    cuboids = []
    for component in _safe_cell_components(safe):
        cuboids.extend(_rectangularize_safe_cells(component, safe))

    cell_size = extent / float(grid_resolution)
    requested_inset = max(0.0, float(settings.get("inset", 0.0005)))
    records = []
    for cell_min, cell_max in cuboids:
        lower = np.asarray([node_axes[axis][cell_min[axis]] for axis in range(3)], dtype=np.float64)
        upper = np.asarray([node_axes[axis][cell_max[axis] + 1] for axis in range(3)], dtype=np.float64)
        center = (lower + upper) * 0.5
        half = (upper - lower) * 0.5
        # Pull every face inward. The grid already guarantees sampled
        # containment; this margin protects against parity and float noise.
        margin = np.minimum(half * 0.08, requested_inset + cell_size * 0.035)
        half = half - margin
        if bool(np.any(half <= 1.0e-7)):
            continue
        volume = float(8.0 * np.prod(half))
        part = _box_mesh(center, half)
        part["safe_proxy"] = "INTERIOR_CELL_CUBOID"
        records.append((volume, tuple(cell_min), tuple(cell_max), part))

    if not records:
        raise CoACDError("Safe interior cell fitting returned no usable boxes; use the single convex hull fallback.")
    records.sort(key=lambda item: (-item[0], item[1], item[2]))
    return [item[3] for item in records[:max_parts]]

def _decompose_in_process(
    vertices: Sequence[Sequence[float]],
    triangles: Sequence[Sequence[int]] | Sequence[int],
    settings: Dict[str, object],
) -> List[Dict[str, object]]:
    """Run the native library in the current process.

    This function is intentionally private and is called only by
    ``coacd_worker.py``. A native access violation is therefore contained in the
    worker process instead of terminating Blender.
    """
    library = load_coacd()
    vertex_array, triangle_array = _normalize_input(vertices, triangles)
    vertex_array, triangle_array = _sanitize_mesh_arrays(vertex_array, triangle_array)

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
        count = int(result.meshes_count)
        if count < 0 or count > max(4096, max_parts * 16):
            raise CoACDError(f"CoACD returned an invalid piece count: {count}")
        if count and not bool(result.meshes_ptr):
            raise CoACDError("CoACD returned a null mesh array.")
        for index in range(count):
            native_mesh = result.meshes_ptr[index]
            vertex_count = int(native_mesh.vertices_count)
            triangle_count = int(native_mesh.triangles_count)
            if vertex_count < 4 or triangle_count < 4:
                continue
            if vertex_count > 10_000_000 or triangle_count > 20_000_000:
                raise CoACDError("CoACD returned an implausibly large mesh.")
            piece_vertices = np.ctypeslib.as_array(native_mesh.vertices_ptr, (vertex_count, 3)).copy()
            piece_triangles = np.ctypeslib.as_array(native_mesh.triangles_ptr, (triangle_count, 3)).copy()
            if not bool(np.isfinite(piece_vertices).all()):
                continue
            if int(piece_triangles.min(initial=0)) < 0 or int(piece_triangles.max(initial=0)) >= vertex_count:
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


def decompose(
    vertices: Sequence[Sequence[float]],
    triangles: Sequence[Sequence[int]] | Sequence[int],
    settings: Dict[str, object],
) -> List[Dict[str, object]]:
    """Return convex pieces while isolating native crashes from Blender."""
    vertex_array, triangle_array = _normalize_input(vertices, triangles)
    requested_mode = str(settings.get("execution_mode", "") or "").strip().upper()
    native_windows_opt_in = str(os.environ.get("KA_COACD_NATIVE_WINDOWS", "0")).strip().lower() in {"1", "true", "yes", "on"}
    if requested_mode in {"SAFE_SPATIAL", "SAFE_INTERIOR", "SAFE_INTERIOR_BOXES"} or (os.name == "nt" and not native_windows_opt_in):
        return _safe_spatial_decompose(vertex_array, triangle_array, settings)
    vertex_array, triangle_array = _sanitize_mesh_arrays(vertex_array, triangle_array)
    python = worker_python()
    worker = Path(__file__).with_name("coacd_worker.py")
    if not worker.is_file():
        raise CoACDError(f"CoACD worker is missing: {worker}")

    timeout = max(5.0, min(120.0, float(settings.get("worker_timeout", os.environ.get("KA_COACD_TIMEOUT", 45.0)))))
    with tempfile.TemporaryDirectory(prefix="ka_coacd_") as directory:
        input_path = Path(directory) / "input.npz"
        output_path = Path(directory) / "output.json"
        np.savez_compressed(
            input_path,
            vertices=vertex_array,
            triangles=triangle_array,
            settings_json=np.asarray(json.dumps(settings, sort_keys=True, separators=(",", ":"))),
        )
        command = [python, str(worker), str(input_path), str(output_path)]
        environment = os.environ.copy()
        environment["PYTHONNOUSERSITE"] = "1"
        creationflags = 0
        startupinfo = None
        if os.name == "nt":
            creationflags = int(getattr(subprocess, "CREATE_NO_WINDOW", 0))
            startupinfo = subprocess.STARTUPINFO()
            startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        try:
            completed = subprocess.run(
                command,
                cwd=str(worker.parent),
                env=environment,
                capture_output=True,
                text=True,
                timeout=timeout,
                check=False,
                creationflags=creationflags,
                startupinfo=startupinfo,
            )
        except subprocess.TimeoutExpired as exc:
            raise CoACDError(f"CoACD worker exceeded {timeout:.1f} seconds and was terminated.") from exc
        except OSError as exc:
            raise CoACDError(f"CoACD worker could not be started: {exc}") from exc

        if completed.returncode != 0:
            detail = ""
            if output_path.is_file():
                try:
                    failed_payload = json.loads(output_path.read_text(encoding="utf-8"))
                    detail = str(failed_payload.get("error") or "")
                except Exception:
                    detail = ""
            if not detail:
                detail = (completed.stderr or completed.stdout or "native worker terminated without diagnostics").strip()
            if len(detail) > 1200:
                detail = detail[-1200:]
            raise CoACDError(f"CoACD worker failed with exit code {completed.returncode}: {detail}")
        if not output_path.is_file():
            raise CoACDError("CoACD worker completed without producing an output file.")
        try:
            payload = json.loads(output_path.read_text(encoding="utf-8"))
        except Exception as exc:
            raise CoACDError(f"CoACD worker output is invalid: {exc}") from exc

    if not bool(payload.get("ok", False)):
        raise CoACDError(str(payload.get("error") or "CoACD worker reported an unknown error."))
    pieces = payload.get("parts")
    if not isinstance(pieces, list) or not pieces:
        raise CoACDError("CoACD worker returned no convex pieces.")
    validated: List[Dict[str, object]] = []
    for piece in pieces:
        if not isinstance(piece, dict):
            continue
        points = piece.get("vertices")
        indices = piece.get("indices")
        if not isinstance(points, list) or len(points) < 4 or not isinstance(indices, list) or len(indices) < 12:
            continue
        validated.append({"vertices": points, "indices": indices})
    if not validated:
        raise CoACDError("CoACD worker returned no usable convex pieces after validation.")
    return validated
