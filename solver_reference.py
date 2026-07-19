"""Blender scene extraction, preflight validation and transform application."""

from __future__ import annotations

import array
import gzip
import hashlib
import json
import math
import os
import re
import struct
import sys
import tempfile
import time
import zlib
from collections import OrderedDict
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import bmesh
import bpy
from mathutils import Matrix, Quaternion, Vector
from mathutils.bvhtree import BVHTree

try:
    import numpy as _np
except Exception:  # Blender builds normally bundle NumPy; keep a pure-Python fallback.
    _np = None

from .cache import CACHE_VERSION, cache_file_path
from .bonds import bonds_for_enabled_bodies
from ..backends.culverin_loader import BUNDLED_CULVERIN_VERSION
from .coacd_bridge import COACD_EXECUTION_MODE, COACD_VERSION, CoACDError, decompose as coacd_decompose
from .simulation_scene import (
    BODY_ID_PROPERTY,
    SCENE_ID_PROPERTY,
    SIMULATION_SCENE_VERSION,
    build_simulation_scene,
    canonical_scene_digest,
    ensure_scene_body_ids,
    ensure_stable_id,
)



_GEOMETRY_CACHE_MAX_ENTRIES = 512
_GEOMETRY_CACHE: "OrderedDict[str, Dict]" = OrderedDict()
_GEOMETRY_CACHE_TOTAL_HITS = 0
_GEOMETRY_CACHE_TOTAL_MISSES = 0
_HULL_CACHE_TOTAL_HITS = 0
_HULL_CACHE_TOTAL_MISSES = 0
ADDON_VERSION = "0.7.1"
SIGNATURE_SCHEMA = 19
_SUPPORT_DIRECTION_CACHE: Dict[int, List[Vector]] = {}
_PERSISTENT_HULL_CACHE_VERSION = 7
_PERSISTENT_HULL_CACHE_MAX_ENTRIES = 2048
_PERSISTENT_HULL_CACHE: "OrderedDict[str, Dict]" = OrderedDict()
_PERSISTENT_HULL_CACHE_PATH: Optional[str] = None
_PERSISTENT_HULL_CACHE_DIRTY = False
_PERSISTENT_HULL_CACHE_LOAD_SECONDS = 0.0
_PERSISTENT_HULL_CACHE_SAVE_SECONDS = 0.0
_PERSISTENT_HULL_CACHE_FILE_SIZE = 0
_HULL_CACHE_MAGIC = b"KACL064\0"
_HULL_CACHE_HEADER = struct.Struct("<8sIII")


def _vector_list(value: Iterable[float]) -> List[float]:
    """Return a JSON-safe float list for mathutils vectors and numeric sequences."""
    return [float(component) for component in value]


def _persistent_hull_cache_file(directory: str) -> str:
    return os.path.join(directory, "ka_rigid_colliders_v7.kahc")


def _legacy_persistent_hull_cache_file(directory: str) -> str:
    return os.path.join(directory, "ka_rigid_hulls_v2.json.gz")


def _encode_persistent_hulls() -> Tuple[bytes, bytes]:
    """Serialize persistent convex hull and CoACD compound proxies."""
    values = array.array("d")
    entries = []
    for geometry_key, entry in _PERSISTENT_HULL_CACHE.items():
        if not isinstance(entry, dict):
            continue
        hull_records = []
        hulls = entry.get("hulls", {})
        if isinstance(hulls, dict):
            for quality_key, hull in hulls.items():
                if not isinstance(hull, dict):
                    continue
                points = list(hull.get("points", []))
                point_offset = len(values)
                for point in points:
                    values.extend(float(component) for component in point[:3])
                center_offset = len(values)
                values.extend(float(component) for component in hull.get("center", (0.0, 0.0, 0.0))[:3])
                hull_records.append({
                    "key": str(quality_key),
                    "point_offset": point_offset,
                    "point_count": len(points),
                    "center_offset": center_offset,
                    "raw_count": int(hull.get("raw_count", len(points))),
                    "quality": dict(hull.get("quality", {})),
                })
        compound_records = []
        compounds = entry.get("compounds", {})
        if isinstance(compounds, dict):
            for quality_key, compound in compounds.items():
                if not isinstance(compound, dict):
                    continue
                parts = []
                for part in compound.get("parts", []):
                    if not isinstance(part, dict):
                        continue
                    part_points = list(part.get("vertices", []))
                    point_offset = len(values)
                    for point in part_points:
                        values.extend(float(component) for component in point[:3])
                    parts.append({
                        "point_offset": point_offset,
                        "point_count": len(part_points),
                        "indices": [int(value) for value in part.get("indices", [])],
                        "center": [float(value) for value in part.get("center", (0.0, 0.0, 0.0))[:3]],
                        "volume": float(part.get("volume", 0.0)),
                        "radius": float(part.get("radius", 0.0)),
                        "raw_vertex_count": int(part.get("raw_vertex_count", len(part_points))),
                        "selected_vertex_count": int(part.get("selected_vertex_count", len(part_points))),
                        "hull_quality": dict(part.get("hull_quality", {})),
                        "box_center": [float(value) for value in part.get("box_center", part.get("center", (0.0, 0.0, 0.0)))[:3]],
                        "box_half_extents": [float(value) for value in part.get("box_half_extents", (0.0, 0.0, 0.0))[:3]],
                        "box_rotation": [float(value) for value in part.get("box_rotation", (1.0, 0.0, 0.0, 0.0))[:4]],
                    })
                if parts:
                    compound_records.append({
                        "key": str(quality_key),
                        "parts": parts,
                        "quality": dict(compound.get("quality", {})),
                    })
        if hull_records or compound_records:
            entries.append({"key": str(geometry_key), "hulls": hull_records, "compounds": compound_records})
    if sys.byteorder != "little":
        values.byteswap()
    metadata = json.dumps({"entries": entries}, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    return zlib.compress(metadata, level=1), zlib.compress(values.tobytes(), level=1)

def _decode_persistent_hulls(metadata_blob: bytes, values_blob: bytes) -> "OrderedDict[str, Dict]":
    metadata = json.loads(zlib.decompress(metadata_blob).decode("utf-8"))
    values = array.array("d")
    values.frombytes(zlib.decompress(values_blob))
    if sys.byteorder != "little":
        values.byteswap()
    result: "OrderedDict[str, Dict]" = OrderedDict()
    for entry in metadata.get("entries", [])[-_PERSISTENT_HULL_CACHE_MAX_ENTRIES:]:
        if not isinstance(entry, dict):
            continue
        hulls: Dict[str, Dict] = {}
        for record in entry.get("hulls", []):
            if not isinstance(record, dict):
                continue
            point_offset = int(record.get("point_offset", 0))
            point_count = int(record.get("point_count", 0))
            center_offset = int(record.get("center_offset", point_offset + point_count * 3))
            points = [
                [float(values[index]), float(values[index + 1]), float(values[index + 2])]
                for index in range(point_offset, point_offset + point_count * 3, 3)
            ]
            center = [float(values[center_offset + index]) for index in range(3)]
            hulls[str(record.get("key", ""))] = {
                "points": points,
                "center": center,
                "raw_count": int(record.get("raw_count", point_count)),
                "quality": dict(record.get("quality", {})),
            }
        compounds: Dict[str, Dict] = {}
        for record in entry.get("compounds", []):
            if not isinstance(record, dict):
                continue
            parts = []
            for part_record in record.get("parts", []):
                if not isinstance(part_record, dict):
                    continue
                point_offset = int(part_record.get("point_offset", 0))
                point_count = int(part_record.get("point_count", 0))
                points = [
                    [float(values[index]), float(values[index + 1]), float(values[index + 2])]
                    for index in range(point_offset, point_offset + point_count * 3, 3)
                ]
                if len(points) < 4:
                    continue
                parts.append({
                    "vertices": points,
                    "indices": [int(value) for value in part_record.get("indices", [])],
                    "center": [float(value) for value in part_record.get("center", (0.0, 0.0, 0.0))[:3]],
                    "volume": float(part_record.get("volume", 0.0)),
                    "radius": float(part_record.get("radius", 0.0)),
                    "raw_vertex_count": int(part_record.get("raw_vertex_count", point_count)),
                    "selected_vertex_count": int(part_record.get("selected_vertex_count", point_count)),
                    "hull_quality": dict(part_record.get("hull_quality", {})),
                    "box_center": [float(value) for value in part_record.get("box_center", part_record.get("center", (0.0, 0.0, 0.0)))[:3]],
                    "box_half_extents": [float(value) for value in part_record.get("box_half_extents", (0.0, 0.0, 0.0))[:3]],
                    "box_rotation": [float(value) for value in part_record.get("box_rotation", (1.0, 0.0, 0.0, 0.0))[:4]],
                })
            if parts:
                compounds[str(record.get("key", ""))] = {
                    "parts": parts,
                    "quality": dict(record.get("quality", {})),
                }
        if hulls or compounds:
            result[str(entry.get("key", ""))] = {"hulls": hulls, "compounds": compounds}
    return result

def _configure_persistent_hull_cache(directory: str) -> None:
    """Load the disk-backed convex-proxy cache for the active scene cache directory."""
    global _PERSISTENT_HULL_CACHE_PATH, _PERSISTENT_HULL_CACHE, _PERSISTENT_HULL_CACHE_DIRTY
    global _PERSISTENT_HULL_CACHE_LOAD_SECONDS, _PERSISTENT_HULL_CACHE_FILE_SIZE
    path = _persistent_hull_cache_file(directory)
    if path == _PERSISTENT_HULL_CACHE_PATH:
        return
    started = time.perf_counter()
    _PERSISTENT_HULL_CACHE_PATH = path
    _PERSISTENT_HULL_CACHE = OrderedDict()
    _PERSISTENT_HULL_CACHE_DIRTY = False
    _PERSISTENT_HULL_CACHE_FILE_SIZE = 0
    try:
        if os.path.isfile(path):
            with open(path, "rb") as handle:
                header = handle.read(_HULL_CACHE_HEADER.size)
                if len(header) != _HULL_CACHE_HEADER.size:
                    raise ValueError("Truncated persistent hull cache")
                magic, version, metadata_size, values_size = _HULL_CACHE_HEADER.unpack(header)
                if magic != _HULL_CACHE_MAGIC or int(version) != _PERSISTENT_HULL_CACHE_VERSION:
                    raise ValueError("Unsupported persistent hull cache")
                metadata_blob = handle.read(metadata_size)
                values_blob = handle.read(values_size)
            _PERSISTENT_HULL_CACHE = _decode_persistent_hulls(metadata_blob, values_blob)
            _PERSISTENT_HULL_CACHE_FILE_SIZE = os.path.getsize(path)
        else:
            legacy = _legacy_persistent_hull_cache_file(directory)
            if os.path.isfile(legacy):
                with gzip.open(legacy, "rt", encoding="utf-8") as handle:
                    payload = json.load(handle)
                entries = payload.get("entries", {}) if int(payload.get("version", 0)) == 2 else {}
                if isinstance(entries, dict):
                    for key, value in list(entries.items())[-_PERSISTENT_HULL_CACHE_MAX_ENTRIES:]:
                        if isinstance(value, dict):
                            _PERSISTENT_HULL_CACHE[str(key)] = value
                    _PERSISTENT_HULL_CACHE_DIRTY = bool(_PERSISTENT_HULL_CACHE)
    except Exception:
        _PERSISTENT_HULL_CACHE = OrderedDict()
        _PERSISTENT_HULL_CACHE_DIRTY = False
    finally:
        _PERSISTENT_HULL_CACHE_LOAD_SECONDS = time.perf_counter() - started


def _save_persistent_hull_cache() -> None:
    global _PERSISTENT_HULL_CACHE_DIRTY, _PERSISTENT_HULL_CACHE_SAVE_SECONDS, _PERSISTENT_HULL_CACHE_FILE_SIZE
    if not _PERSISTENT_HULL_CACHE_DIRTY or not _PERSISTENT_HULL_CACHE_PATH:
        _PERSISTENT_HULL_CACHE_SAVE_SECONDS = 0.0
        return
    while len(_PERSISTENT_HULL_CACHE) > _PERSISTENT_HULL_CACHE_MAX_ENTRIES:
        _PERSISTENT_HULL_CACHE.popitem(last=False)
    started = time.perf_counter()
    try:
        os.makedirs(os.path.dirname(_PERSISTENT_HULL_CACHE_PATH), exist_ok=True)
        metadata_blob, values_blob = _encode_persistent_hulls()
        temporary = _PERSISTENT_HULL_CACHE_PATH + ".tmp"
        with open(temporary, "wb") as handle:
            handle.write(_HULL_CACHE_HEADER.pack(
                _HULL_CACHE_MAGIC,
                _PERSISTENT_HULL_CACHE_VERSION,
                len(metadata_blob),
                len(values_blob),
            ))
            handle.write(metadata_blob)
            handle.write(values_blob)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, _PERSISTENT_HULL_CACHE_PATH)
        _PERSISTENT_HULL_CACHE_FILE_SIZE = os.path.getsize(_PERSISTENT_HULL_CACHE_PATH)
        legacy = _legacy_persistent_hull_cache_file(os.path.dirname(_PERSISTENT_HULL_CACHE_PATH))
        if os.path.isfile(legacy):
            os.remove(legacy)
        _PERSISTENT_HULL_CACHE_DIRTY = False
    except Exception:
        pass
    finally:
        _PERSISTENT_HULL_CACHE_SAVE_SECONDS = time.perf_counter() - started


def _persistent_hulls_for_geometry(key: Optional[str]) -> Dict:
    if not key:
        return {}
    entry = _PERSISTENT_HULL_CACHE.get(key)
    if not isinstance(entry, dict):
        return {}
    _PERSISTENT_HULL_CACHE.move_to_end(key)
    hulls = entry.get("hulls", {})
    return dict(hulls) if isinstance(hulls, dict) else {}

def _persistent_compounds_for_geometry(key: Optional[str]) -> Dict:
    if not key:
        return {}
    entry = _PERSISTENT_HULL_CACHE.get(key)
    if not isinstance(entry, dict):
        return {}
    _PERSISTENT_HULL_CACHE.move_to_end(key)
    compounds = entry.get("compounds", {})
    return dict(compounds) if isinstance(compounds, dict) else {}


def _store_persistent_hulls(geometry: Dict) -> None:
    global _PERSISTENT_HULL_CACHE_DIRTY
    key = geometry.get("cache_key")
    hulls = geometry.get("hulls")
    compounds = geometry.get("compounds")
    if not key:
        return
    stored: Dict[str, Dict] = {}
    if isinstance(hulls, dict) and hulls:
        stored["hulls"] = hulls
    if isinstance(compounds, dict) and compounds:
        stored["compounds"] = compounds
    if not stored:
        return
    previous = _PERSISTENT_HULL_CACHE.get(str(key), {})
    if isinstance(previous, dict):
        if "hulls" not in stored and isinstance(previous.get("hulls"), dict):
            stored["hulls"] = previous["hulls"]
        if "compounds" not in stored and isinstance(previous.get("compounds"), dict):
            stored["compounds"] = previous["compounds"]
    _PERSISTENT_HULL_CACHE[str(key)] = stored
    _PERSISTENT_HULL_CACHE.move_to_end(str(key))
    _PERSISTENT_HULL_CACHE_DIRTY = True

def clear_geometry_cache(*, clear_persistent: bool = False, persistent_directory: Optional[str] = None) -> int:
    """Clear evaluated geometry and optionally the disk-backed convex proxy cache."""
    global _GEOMETRY_CACHE_TOTAL_HITS, _GEOMETRY_CACHE_TOTAL_MISSES
    global _HULL_CACHE_TOTAL_HITS, _HULL_CACHE_TOTAL_MISSES
    global _PERSISTENT_HULL_CACHE, _PERSISTENT_HULL_CACHE_DIRTY
    global _PERSISTENT_HULL_CACHE_LOAD_SECONDS, _PERSISTENT_HULL_CACHE_SAVE_SECONDS, _PERSISTENT_HULL_CACHE_FILE_SIZE
    count = len(_GEOMETRY_CACHE)
    _GEOMETRY_CACHE.clear()
    _GEOMETRY_CACHE_TOTAL_HITS = 0
    _GEOMETRY_CACHE_TOTAL_MISSES = 0
    _HULL_CACHE_TOTAL_HITS = 0
    _HULL_CACHE_TOTAL_MISSES = 0
    if clear_persistent:
        if persistent_directory:
            _configure_persistent_hull_cache(persistent_directory)
        count += len(_PERSISTENT_HULL_CACHE)
        _PERSISTENT_HULL_CACHE = OrderedDict()
        _PERSISTENT_HULL_CACHE_DIRTY = False
        _PERSISTENT_HULL_CACHE_LOAD_SECONDS = 0.0
        _PERSISTENT_HULL_CACHE_SAVE_SECONDS = 0.0
        _PERSISTENT_HULL_CACHE_FILE_SIZE = 0
        if _PERSISTENT_HULL_CACHE_PATH:
            for path in (
                _PERSISTENT_HULL_CACHE_PATH,
                _legacy_persistent_hull_cache_file(os.path.dirname(_PERSISTENT_HULL_CACHE_PATH)),
            ):
                if os.path.isfile(path):
                    try:
                        os.remove(path)
                    except OSError:
                        pass
    return count


def geometry_cache_stats() -> Dict[str, object]:
    return {
        "entries": len(_GEOMETRY_CACHE),
        "max_entries": _GEOMETRY_CACHE_MAX_ENTRIES,
        "geometry_hits_total": _GEOMETRY_CACHE_TOTAL_HITS,
        "geometry_misses_total": _GEOMETRY_CACHE_TOTAL_MISSES,
        "hull_hits_total": _HULL_CACHE_TOTAL_HITS,
        "hull_misses_total": _HULL_CACHE_TOTAL_MISSES,
        "persistent_hull_entries": len(_PERSISTENT_HULL_CACHE),
        "persistent_hull_load_seconds": float(_PERSISTENT_HULL_CACHE_LOAD_SECONDS),
        "persistent_hull_save_seconds": float(_PERSISTENT_HULL_CACHE_SAVE_SECONDS),
        "persistent_hull_file_size": int(_PERSISTENT_HULL_CACHE_FILE_SIZE),
        "persistent_hull_format": "KACL7-float64-zlib1",
    }


def _profile_add(profile: Optional[Dict], key: str, value) -> None:
    if profile is None:
        return
    profile[key] = profile.get(key, 0) + value


def _geometry_cache_store(key: str, value: Dict) -> Dict:
    _GEOMETRY_CACHE[key] = value
    _GEOMETRY_CACHE.move_to_end(key)
    while len(_GEOMETRY_CACHE) > _GEOMETRY_CACHE_MAX_ENTRIES:
        _GEOMETRY_CACHE.popitem(last=False)
    return value

FRACTURE_TAGS = (
    "ka_fracture_final_piece",
    "ka_fracture_break_piece",
    "ka_fracture_prepared_piece",
)


def is_ka_fracture_piece(obj: bpy.types.Object) -> bool:
    """Return whether an object is a KA Fracture fragment."""
    return bool(
        obj.name.startswith("KA_Fracture_Piece_")
        or any(bool(obj.get(tag, False)) for tag in FRACTURE_TAGS)
    )


GROUND_OBJECT_NAME = "KA_Physics_Ground"
GROUND_OBJECT_TAG = "ka_rigid_ground"


def is_ka_ground(obj: bpy.types.Object) -> bool:
    """Return whether an object belongs to the managed KA ground singleton."""
    return bool(obj.get(GROUND_OBJECT_TAG, False)) or obj.name.startswith(GROUND_OBJECT_NAME)


def ground_objects(scene: bpy.types.Scene, *, enabled_only: bool = False) -> List[bpy.types.Object]:
    objects = [obj for obj in scene.objects if is_ka_ground(obj)]
    if enabled_only:
        objects = [
            obj for obj in objects
            if hasattr(obj, "ka_rigid_body") and obj.ka_rigid_body.enabled
        ]
    return sorted(
        objects,
        key=lambda obj: (
            0 if obj.name == GROUND_OBJECT_NAME else 1,
            0 if bool(obj.get(GROUND_OBJECT_TAG, False)) else 1,
            obj.name_full.casefold(),
            obj.name_full,
        ),
    )


def _canonical_quaternion_values(rotation: Quaternion) -> Tuple[float, float, float, float]:
    values = [float(rotation.w), float(rotation.x), float(rotation.y), float(rotation.z)]
    # q and -q describe the same rotation. Canonicalize the sign so duplicate
    # detection does not depend on Blender's decomposition sign choice.
    if values[0] < 0.0 or (abs(values[0]) <= 1.0e-12 and tuple(values[1:]) < (0.0, 0.0, 0.0)):
        values = [-value for value in values]
    return tuple(round(value, 7) for value in values)


def _static_collider_fingerprint(
    obj: bpy.types.Object,
    depsgraph: bpy.types.Depsgraph,
) -> Tuple:
    """Describe the actual static collision surface closely enough to find overlaps."""
    settings = obj.ka_rigid_body
    location, rotation, _scale = obj.matrix_world.decompose()
    transform = (
        tuple(round(float(value), 7) for value in location),
        _canonical_quaternion_values(rotation),
    )
    material = (
        round(float(settings.friction), 7),
        round(float(settings.restitution), 7),
        int(settings.collision_layer),
        int(settings.collision_mask),
    )
    shape = str(settings.collision_shape)
    if shape == "PLANE":
        # Jolt planes are infinite in local XY. Mesh size and object scale do
        # not change the collision surface, only origin and orientation do.
        geometry = ("INFINITE_PLANE",)
    else:
        entry = _evaluated_geometry_entry(obj, depsgraph)
        geometry = (
            entry.get("cache_key"),
            tuple(round(float(value), 7) for value in entry.get("half_extents", ())),
            int(entry.get("source_vertex_count", 0)),
            int(entry.get("triangle_count", 0)),
        )
    return (shape, transform, geometry, material)


def _preferred_static_keeper(objects: Sequence[bpy.types.Object]) -> bpy.types.Object:
    return sorted(
        objects,
        key=lambda obj: (
            0 if obj.name == GROUND_OBJECT_NAME else 1,
            0 if bool(obj.get(GROUND_OBJECT_TAG, False)) else 1,
            obj.name_full.casefold(),
            obj.name_full,
        ),
    )[0]


def duplicate_static_collider_groups(
    scene: bpy.types.Scene,
    *,
    bodies: Optional[Sequence[bpy.types.Object]] = None,
    depsgraph: Optional[bpy.types.Depsgraph] = None,
    include_ground_groups: bool = False,
) -> List[Dict[str, object]]:
    """Find enabled static bodies that create the same collision surface."""
    active = list(bodies) if bodies is not None else enabled_body_objects(scene)
    graph = depsgraph or bpy.context.evaluated_depsgraph_get()
    buckets: Dict[Tuple, List[bpy.types.Object]] = {}
    for obj in active:
        settings = obj.ka_rigid_body
        if settings.body_type != "STATIC":
            continue
        if not include_ground_groups and is_ka_ground(obj):
            continue
        fingerprint = _static_collider_fingerprint(obj, graph)
        buckets.setdefault(fingerprint, []).append(obj)

    groups: List[Dict[str, object]] = []
    for fingerprint, objects in buckets.items():
        if len(objects) < 2:
            continue
        keeper = _preferred_static_keeper(objects)
        duplicates = [obj for obj in objects if obj != keeper]
        groups.append({
            "fingerprint": fingerprint,
            "keeper": keeper,
            "duplicates": duplicates,
            "objects": list(objects),
            "ground_group": all(is_ka_ground(obj) for obj in objects),
        })
    groups.sort(key=lambda group: str(group["keeper"].name_full).casefold())
    return groups


def _apply_duplicate_policy(
    scene: bpy.types.Scene,
    keeper: bpy.types.Object,
    duplicates: Sequence[bpy.types.Object],
    policy: str,
) -> Dict[str, List[str]]:
    result = {"excluded": [], "deleted": []}
    for obj in duplicates:
        name = obj.name_full
        if policy == "DELETE":
            bpy.data.objects.remove(obj, do_unlink=True)
            result["deleted"].append(name)
        elif policy == "EXCLUDE":
            if hasattr(obj, "ka_rigid_body"):
                obj.ka_rigid_body.enabled = False
            result["excluded"].append(name)
    return result


def sanitize_name(value: str) -> str:
    value = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip())
    return value or "Scene"


def resolve_cache_directory(scene: bpy.types.Scene) -> str:
    settings = scene.ka_rigid_world
    configured = settings.cache_directory.strip()
    if configured:
        return os.path.abspath(bpy.path.abspath(configured))

    if bpy.data.filepath:
        blend_dir = os.path.dirname(bpy.data.filepath)
        return os.path.join(blend_dir, "ka_rigid_cache", sanitize_name(scene.name))

    return os.path.join(tempfile.gettempdir(), "ka_rigid_cache", sanitize_name(scene.name))


def store_rest_transform(obj: bpy.types.Object, force: bool = False) -> None:
    settings = obj.ka_rigid_body
    if settings.rest_transform_stored and not force:
        return
    matrix = obj.matrix_world
    location, rotation, scale = matrix.decompose()
    settings.rest_location = location
    settings.rest_rotation = rotation
    settings.rest_scale = scale
    settings.rest_transform_stored = True


def repair_managed_ground(obj: bpy.types.Object, *, store_rest: bool = False) -> List[str]:
    """Restore the invariant required by the managed KA ground.

    The managed ground is an infinite, two-sided Jolt plane. It must never be
    converted to a dynamic body or a finite triangle mesh by bulk assignment.
    Returning the changed fields lets operators and Preflight report the repair.
    """
    if not is_ka_ground(obj) or not hasattr(obj, "ka_rigid_body"):
        return []
    settings = obj.ka_rigid_body
    changed: List[str] = []
    if not bool(obj.get(GROUND_OBJECT_TAG, False)):
        obj[GROUND_OBJECT_TAG] = True
        changed.append("tag")
    if not settings.enabled:
        settings.enabled = True
        changed.append("enabled")
    if settings.body_type != "STATIC":
        settings.body_type = "STATIC"
        changed.append("body_type")
    if settings.collision_shape != "PLANE":
        settings.collision_shape = "PLANE"
        changed.append("collision_shape")
    if store_rest:
        store_rest_transform(obj, force=True)
    return changed


def restore_rest_transform(obj: bpy.types.Object) -> bool:
    settings = obj.ka_rigid_body
    if not settings.rest_transform_stored:
        return False
    obj.matrix_world = Matrix.LocRotScale(
        Vector(settings.rest_location),
        Quaternion(settings.rest_rotation),
        Vector(settings.rest_scale),
    )
    return True


def evaluated_dimensions(obj: bpy.types.Object, depsgraph: bpy.types.Depsgraph) -> Vector:
    evaluated = obj.evaluated_get(depsgraph)
    return Vector(evaluated.dimensions)


def evaluated_world_bounds(obj: bpy.types.Object, depsgraph: bpy.types.Depsgraph) -> Tuple[Vector, Vector]:
    evaluated = obj.evaluated_get(depsgraph)
    points = [evaluated.matrix_world @ Vector(corner) for corner in evaluated.bound_box]
    minimum = Vector((min(point.x for point in points), min(point.y for point in points), min(point.z for point in points)))
    maximum = Vector((max(point.x for point in points), max(point.y for point in points), max(point.z for point in points)))
    return minimum, maximum


def _mesh_fingerprint(
    coords: array.array,
    triangle_indices: array.array,
    local_matrix,
    vertex_count: int,
    triangle_count: int,
) -> str:
    digest = hashlib.blake2b(digest_size=20)
    digest.update(struct.pack("<II", int(vertex_count), int(triangle_count)))
    digest.update(coords.tobytes())
    digest.update(triangle_indices.tobytes())
    for row in local_matrix:
        digest.update(struct.pack("<3d", float(row[0]), float(row[1]), float(row[2])))
    return digest.hexdigest()


def _evaluated_geometry_entry(
    obj: bpy.types.Object,
    depsgraph: bpy.types.Depsgraph,
    profile: Optional[Dict] = None,
) -> Dict:
    """Read an evaluated mesh once and reuse its bounds, volume and generated hulls."""
    global _GEOMETRY_CACHE_TOTAL_HITS, _GEOMETRY_CACHE_TOTAL_MISSES

    started = time.perf_counter()
    if obj.type != "MESH":
        dimensions = evaluated_dimensions(obj, depsgraph)
        half = dimensions * 0.5
        vertices = [
            (float(x), float(y), float(z))
            for x in (-half.x, half.x)
            for y in (-half.y, half.y)
            for z in (-half.z, half.z)
        ]
        center, half_extents = _bounds_center_and_half_extents([Vector(v) for v in vertices])
        _profile_add(profile, "mesh_read_seconds", time.perf_counter() - started)
        _profile_add(profile, "geometry_cache_misses", 1)
        return {
            "cache_key": None,
            "vertices": vertices,
            "indices": [],
            "source_vertex_count": len(vertices),
            "triangle_count": 0,
            "bounds_center": _vector_list(center),
            "half_extents": _vector_list(half_extents),
            "volume_world": max(1.0e-9, dimensions.x * dimensions.y * dimensions.z),
            "hulls": {},
            "compounds": {},
        }

    evaluated = obj.evaluated_get(depsgraph)
    mesh = evaluated.to_mesh()
    if mesh is None:
        _profile_add(profile, "mesh_read_seconds", time.perf_counter() - started)
        _profile_add(profile, "geometry_cache_misses", 1)
        return {
            "cache_key": None, "vertices": [], "indices": [], "source_vertex_count": 0,
            "triangle_count": 0, "bounds_center": [0.0, 0.0, 0.0],
            "half_extents": [1.0e-5, 1.0e-5, 1.0e-5], "volume_world": 0.0, "hulls": {}, "compounds": {},
        }

    try:
        vertex_count = len(mesh.vertices)
        coords = array.array("f", [0.0]) * (vertex_count * 3)
        if vertex_count:
            mesh.vertices.foreach_get("co", coords)

        mesh.calc_loop_triangles()
        triangle_count = len(mesh.loop_triangles)
        triangle_indices = array.array("i", [0]) * (triangle_count * 3)
        if triangle_count:
            mesh.loop_triangles.foreach_get("vertices", triangle_indices)

        _location, rotation, _scale = evaluated.matrix_world.decompose()
        local_matrix = rotation.inverted().to_matrix() @ evaluated.matrix_world.to_3x3()
        key = _mesh_fingerprint(coords, triangle_indices, local_matrix, vertex_count, triangle_count)
        cached = _GEOMETRY_CACHE.get(key)
        if cached is not None:
            _GEOMETRY_CACHE.move_to_end(key)
            _GEOMETRY_CACHE_TOTAL_HITS += 1
            _profile_add(profile, "geometry_cache_hits", 1)
            _profile_add(profile, "mesh_read_seconds", time.perf_counter() - started)
            return cached

        _GEOMETRY_CACHE_TOTAL_MISSES += 1
        _profile_add(profile, "geometry_cache_misses", 1)
        _profile_add(profile, "mesh_read_seconds", time.perf_counter() - started)

        transform_started = time.perf_counter()
        vertices: List[Tuple[float, float, float]] = []
        append_vertex = vertices.append
        for index in range(0, len(coords), 3):
            transformed = local_matrix @ Vector((coords[index], coords[index + 1], coords[index + 2]))
            append_vertex((float(transformed.x), float(transformed.y), float(transformed.z)))
        vectors = [Vector(value) for value in vertices]
        bounds_center, half_extents = _bounds_center_and_half_extents(vectors)
        _profile_add(profile, "vertex_transform_seconds", time.perf_counter() - transform_started)

        volume_started = time.perf_counter()
        volume_world = 0.0
        bm = bmesh.new()
        try:
            bm.from_mesh(mesh)
            if bm.faces:
                local_volume = abs(float(bm.calc_volume(signed=True)))
                volume_world = local_volume * abs(float(local_matrix.determinant()))
        except Exception:
            volume_world = 0.0
        finally:
            bm.free()
        _profile_add(profile, "volume_seconds", time.perf_counter() - volume_started)

        entry = {
            "cache_key": key,
            "vertices": vertices,
            "indices": [int(value) for value in triangle_indices],
            "source_vertex_count": vertex_count,
            "triangle_count": triangle_count,
            "bounds_center": _vector_list(bounds_center),
            "half_extents": _vector_list(half_extents),
            "volume_world": float(volume_world),
            "hulls": _persistent_hulls_for_geometry(key),
            "compounds": _persistent_compounds_for_geometry(key),
        }
        return _geometry_cache_store(key, entry)
    finally:
        evaluated.to_mesh_clear()


def _resolve_collision_proxy(obj: bpy.types.Object, settings) -> Optional[bpy.types.Object]:
    proxy = getattr(settings, "collision_proxy", None)
    if proxy is None:
        proxy_name = obj.get("ka_rigid_collision_proxy")
        if isinstance(proxy_name, str) and proxy_name:
            proxy = bpy.data.objects.get(proxy_name)
    if proxy is None or proxy == obj or getattr(proxy, "type", None) != "MESH":
        return None
    return proxy


def _collision_proxy_geometry_entry(
    body_obj: bpy.types.Object,
    proxy_obj: bpy.types.Object,
    depsgraph: bpy.types.Depsgraph,
    profile: Optional[Dict] = None,
) -> Dict:
    """Transform an evaluated proxy mesh into the body's collision-local frame."""
    started = time.perf_counter()
    source = _evaluated_geometry_entry(proxy_obj, depsgraph, profile)
    body_location, body_rotation, _body_scale = _world_transform(body_obj)
    proxy_location, proxy_rotation, _proxy_scale = _world_transform(proxy_obj)
    relative_rotation = body_rotation.inverted() @ proxy_rotation
    relative_offset = body_rotation.inverted() @ (proxy_location - body_location)
    source_key = str(source.get("cache_key") or proxy_obj.name_full)
    digest = hashlib.blake2b(digest_size=20)
    digest.update(b"KA_COLLISION_PROXY_V1\0")
    digest.update(source_key.encode("utf-8", errors="replace"))
    for row in relative_rotation.to_matrix():
        digest.update(struct.pack("<3d", float(row[0]), float(row[1]), float(row[2])))
    digest.update(struct.pack("<3d", float(relative_offset.x), float(relative_offset.y), float(relative_offset.z)))
    key = digest.hexdigest()
    cached = _GEOMETRY_CACHE.get(key)
    if cached is not None:
        _GEOMETRY_CACHE.move_to_end(key)
        _profile_add(profile, "collision_proxy_cache_hits", 1)
        _profile_add(profile, "collision_proxy_seconds", time.perf_counter() - started)
        return cached

    vertices = [relative_offset + relative_rotation @ Vector(value) for value in source.get("vertices", [])]
    bounds_center, half_extents = _bounds_center_and_half_extents(vertices)
    entry = {
        "cache_key": key,
        "vertices": [_vector_list(vertex) for vertex in vertices],
        "indices": list(source.get("indices", [])),
        "source_vertex_count": int(source.get("source_vertex_count", len(vertices))),
        "triangle_count": int(source.get("triangle_count", 0)),
        "bounds_center": _vector_list(bounds_center),
        "half_extents": _vector_list(half_extents),
        "volume_world": float(source.get("volume_world", 0.0)),
        "hulls": _persistent_hulls_for_geometry(key),
        "compounds": _persistent_compounds_for_geometry(key),
        "proxy_object": proxy_obj.name_full,
    }
    _profile_add(profile, "collision_proxy_cache_misses", 1)
    _profile_add(profile, "collision_proxy_seconds", time.perf_counter() - started)
    return _geometry_cache_store(key, entry)


def mesh_volume_world(
    obj: bpy.types.Object,
    depsgraph: bpy.types.Depsgraph,
    profile: Optional[Dict] = None,
) -> float:
    return float(_evaluated_geometry_entry(obj, depsgraph, profile).get("volume_world", 0.0))


def calculate_mass(
    obj: bpy.types.Object,
    depsgraph: bpy.types.Depsgraph,
    profile: Optional[Dict] = None,
    geometry: Optional[Dict] = None,
) -> float:
    started = time.perf_counter()
    settings = obj.ka_rigid_body
    if settings.mass_mode == "MASS":
        result = max(1.0e-6, float(settings.mass))
    else:
        entry = geometry or _evaluated_geometry_entry(obj, depsgraph, profile)
        volume = float(entry.get("volume_world", 0.0))
        if volume <= 1.0e-9:
            half = Vector(entry.get("half_extents", (0.0, 0.0, 0.0)))
            volume = max(1.0e-9, 8.0 * half.x * half.y * half.z)
        result = max(1.0e-6, volume * max(1.0e-6, float(settings.density)))
    _profile_add(profile, "mass_seconds", time.perf_counter() - started)
    return result


def _world_transform(obj: bpy.types.Object) -> Tuple[Vector, Quaternion, Vector]:
    location, rotation, scale = obj.matrix_world.decompose()
    return location, rotation, scale


def _evaluated_local_mesh_data(
    obj: bpy.types.Object,
    depsgraph: bpy.types.Depsgraph,
    *,
    include_triangles: bool = False,
) -> Tuple[List[Vector], List[int]]:
    entry = _evaluated_geometry_entry(obj, depsgraph)
    vertices = [Vector(value) for value in entry.get("vertices", [])]
    indices = list(entry.get("indices", [])) if include_triangles else []
    return vertices, indices

def _bounds_center_and_half_extents(vertices: Sequence[Vector]) -> Tuple[Vector, Vector]:
    if not vertices:
        return Vector((0.0, 0.0, 0.0)), Vector((1.0e-5, 1.0e-5, 1.0e-5))
    minimum = Vector((min(v.x for v in vertices), min(v.y for v in vertices), min(v.z for v in vertices)))
    maximum = Vector((max(v.x for v in vertices), max(v.y for v in vertices), max(v.z for v in vertices)))
    center = (minimum + maximum) * 0.5
    half = (maximum - minimum) * 0.5
    return center, Vector(tuple(max(1.0e-5, abs(value)) for value in half))


def _polyhedron_centroid(faces: Sequence[Sequence[Vector]], fallback: Vector) -> Vector:
    volume6_sum = 0.0
    weighted = Vector((0.0, 0.0, 0.0))
    for face in faces:
        if len(face) < 3:
            continue
        a = face[0]
        for index in range(1, len(face) - 1):
            b = face[index]
            c = face[index + 1]
            volume6 = float(a.dot(b.cross(c)))
            volume6_sum += volume6
            weighted += (a + b + c) * volume6
    if abs(volume6_sum) <= 1.0e-12:
        return fallback.copy()
    return weighted / (4.0 * volume6_sum)


def _compute_convex_hull(vertices: Sequence[Vector]) -> Tuple[List[Vector], Vector]:
    if len(vertices) < 4:
        center, _half = _bounds_center_and_half_extents(vertices)
        return [vertex.copy() for vertex in vertices], center

    bm = bmesh.new()
    try:
        source = [bm.verts.new(vertex) for vertex in vertices]
        bm.verts.ensure_lookup_table()
        result = bmesh.ops.convex_hull(bm, input=source, use_existing_faces=False)
        hull_faces = [element for element in result.get("geom", []) if isinstance(element, bmesh.types.BMFace)]
        hull_verts = {vertex for face in hull_faces for vertex in face.verts}
        if not hull_verts:
            hull_verts = {
                element for element in result.get("geom", []) if isinstance(element, bmesh.types.BMVert)
            }
        if not hull_verts:
            center, _half = _bounds_center_and_half_extents(vertices)
            return [vertex.copy() for vertex in vertices], center

        ordered = sorted(hull_verts, key=lambda vertex: (vertex.co.x, vertex.co.y, vertex.co.z))
        points = [vertex.co.copy() for vertex in ordered]
        fallback = sum(points, Vector((0.0, 0.0, 0.0))) / max(1, len(points))
        faces = [[vertex.co.copy() for vertex in face.verts] for face in hull_faces]
        center = _polyhedron_centroid(faces, fallback)
        return points, center
    except Exception:
        center, _half = _bounds_center_and_half_extents(vertices)
        return [vertex.copy() for vertex in vertices], center
    finally:
        bm.free()


def _point_key(point: Vector) -> Tuple[int, int, int]:
    return tuple(int(round(float(component) * 1.0e9)) for component in point)


def _support_sample_points(points: Sequence[Vector], target: int) -> List[Vector]:
    """Deterministic incremental farthest-point sampling in O(N * target)."""
    count = len(points)
    if target <= 0 or count <= target:
        return [point.copy() for point in points]
    target = max(4, min(int(target), count))

    if _np is not None:
        coords = _np.asarray([(float(p.x), float(p.y), float(p.z)) for p in points], dtype=_np.float64)
        selected: List[int] = []
        selected_mask = _np.zeros(count, dtype=_np.bool_)
        # Start from deterministic axis extrema. NumPy argmax returns the first tie.
        for axis in range(3):
            for index in (int(_np.argmax(coords[:, axis])), int(_np.argmin(coords[:, axis]))):
                if not bool(selected_mask[index]):
                    selected.append(index)
                    selected_mask[index] = True
                    if len(selected) >= target:
                        return [points[i].copy() for i in selected[:target]]
        minimum_distance_sq = _np.full(count, _np.inf, dtype=_np.float64)
        for index in selected:
            delta = coords - coords[index]
            minimum_distance_sq = _np.minimum(minimum_distance_sq, _np.einsum("ij,ij->i", delta, delta))
        minimum_distance_sq[selected_mask] = -1.0
        while len(selected) < target:
            index = int(_np.argmax(minimum_distance_sq))
            if minimum_distance_sq[index] < 0.0:
                break
            selected.append(index)
            selected_mask[index] = True
            delta = coords - coords[index]
            minimum_distance_sq = _np.minimum(minimum_distance_sq, _np.einsum("ij,ij->i", delta, delta))
            minimum_distance_sq[selected_mask] = -1.0
        return [points[i].copy() for i in selected[:target]]

    selected: List[int] = []
    selected_set = set()
    axes = (
        lambda p: p.x, lambda p: -p.x,
        lambda p: p.y, lambda p: -p.y,
        lambda p: p.z, lambda p: -p.z,
    )
    for key in axes:
        index = max(range(count), key=lambda i: key(points[i]))
        if index not in selected_set:
            selected.append(index)
            selected_set.add(index)
            if len(selected) >= target:
                return [points[i].copy() for i in selected[:target]]
    minimum_distance_sq = [float("inf")] * count
    for chosen in selected:
        point = points[chosen]
        for index, candidate in enumerate(points):
            distance = (candidate - point).length_squared
            if distance < minimum_distance_sq[index]:
                minimum_distance_sq[index] = distance
    for chosen in selected:
        minimum_distance_sq[chosen] = -1.0
    while len(selected) < target:
        index = max(range(count), key=minimum_distance_sq.__getitem__)
        if minimum_distance_sq[index] < 0.0:
            break
        selected.append(index)
        selected_set.add(index)
        point = points[index]
        for candidate_index, candidate in enumerate(points):
            if candidate_index in selected_set:
                minimum_distance_sq[candidate_index] = -1.0
                continue
            distance = (candidate - point).length_squared
            if distance < minimum_distance_sq[candidate_index]:
                minimum_distance_sq[candidate_index] = distance
    return [points[i].copy() for i in selected[:target]]

def _convex_hull_data(vertices: Sequence[Vector], max_vertices: int = 0) -> Tuple[List[Vector], Vector, int]:
    complete_points, complete_center = _compute_convex_hull(vertices)
    raw_count = len(complete_points)
    if max_vertices <= 0 or raw_count <= max_vertices or max_vertices < 4:
        return complete_points, complete_center, raw_count

    sampled = _support_error_sample_points(complete_points, max_vertices)
    simplified_points, simplified_center = _compute_convex_hull(sampled)
    if len(simplified_points) < 4:
        return complete_points, complete_center, raw_count
    return simplified_points, simplified_center, raw_count


def _support_directions(sample_count: int = 256) -> List[Vector]:
    cached = _SUPPORT_DIRECTION_CACHE.get(sample_count)
    if cached is not None:
        return cached
    directions = [
        Vector((1.0, 0.0, 0.0)), Vector((-1.0, 0.0, 0.0)),
        Vector((0.0, 1.0, 0.0)), Vector((0.0, -1.0, 0.0)),
        Vector((0.0, 0.0, 1.0)), Vector((0.0, 0.0, -1.0)),
    ]
    golden_angle = math.pi * (3.0 - math.sqrt(5.0))
    for index in range(max(16, int(sample_count))):
        y = 1.0 - (2.0 * (index + 0.5) / sample_count)
        radial = math.sqrt(max(0.0, 1.0 - y * y))
        angle = golden_angle * index
        directions.append(Vector((math.cos(angle) * radial, y, math.sin(angle) * radial)))
    _SUPPORT_DIRECTION_CACHE[sample_count] = directions
    return directions


def _support_error_sample_points(points: Sequence[Vector], target: int) -> List[Vector]:
    """Select hull points by the directional support error they actually remove.

    Farthest-point sampling distributes points evenly but often misses the small
    extreme features that define a convex collider. This deterministic greedy
    selector starts with axis extrema and repeatedly adds the source point behind
    the currently largest support-plane error.
    """
    count = len(points)
    if target <= 0 or count <= target:
        return [point.copy() for point in points]
    target = max(4, min(int(target), count))
    directions = _support_directions()

    if _np is not None:
        coords = _np.asarray([(float(p.x), float(p.y), float(p.z)) for p in points], dtype=_np.float64)
        direction_array = _np.asarray([(float(d.x), float(d.y), float(d.z)) for d in directions], dtype=_np.float64)
        projections = coords @ direction_array.T
        full_indices = _np.argmax(projections, axis=0)
        full_support = projections[full_indices, _np.arange(projections.shape[1])]
        selected: List[int] = []
        selected_mask = _np.zeros(count, dtype=_np.bool_)
        for axis in range(3):
            for index in (int(_np.argmax(coords[:, axis])), int(_np.argmin(coords[:, axis]))):
                if not bool(selected_mask[index]):
                    selected.append(index)
                    selected_mask[index] = True
                    if len(selected) >= target:
                        return [points[i].copy() for i in selected[:target]]
        proxy_support = _np.max(projections[selected, :], axis=0)
        blocked_directions = _np.zeros(len(directions), dtype=_np.bool_)
        while len(selected) < target:
            errors = full_support - proxy_support
            errors[blocked_directions] = -1.0
            direction_index = int(_np.argmax(errors))
            if errors[direction_index] <= 1.0e-12:
                break
            point_index = int(full_indices[direction_index])
            if bool(selected_mask[point_index]):
                blocked_directions[direction_index] = True
                continue
            selected.append(point_index)
            selected_mask[point_index] = True
            proxy_support = _np.maximum(proxy_support, projections[point_index, :])

        # A finite direction set can expose fewer unique extrema than the budget.
        # Fill the remainder with deterministic farthest points without replacing
        # any support-critical selections.
        if len(selected) < target:
            minimum_distance_sq = _np.full(count, _np.inf, dtype=_np.float64)
            for index in selected:
                delta = coords - coords[index]
                minimum_distance_sq = _np.minimum(minimum_distance_sq, _np.einsum("ij,ij->i", delta, delta))
            minimum_distance_sq[selected_mask] = -1.0
            while len(selected) < target:
                index = int(_np.argmax(minimum_distance_sq))
                if minimum_distance_sq[index] < 0.0:
                    break
                selected.append(index)
                selected_mask[index] = True
                delta = coords - coords[index]
                minimum_distance_sq = _np.minimum(minimum_distance_sq, _np.einsum("ij,ij->i", delta, delta))
                minimum_distance_sq[selected_mask] = -1.0
        return [points[i].copy() for i in selected[:target]]

    selected: List[int] = []
    selected_set = set()
    axes = (
        lambda p: p.x, lambda p: -p.x,
        lambda p: p.y, lambda p: -p.y,
        lambda p: p.z, lambda p: -p.z,
    )
    for key in axes:
        index = max(range(count), key=lambda i: key(points[i]))
        if index not in selected_set:
            selected.append(index)
            selected_set.add(index)
            if len(selected) >= target:
                return [points[i].copy() for i in selected[:target]]
    full_indices = [max(range(count), key=lambda i, d=direction: points[i].dot(d)) for direction in directions]
    full_support = [float(points[index].dot(direction)) for index, direction in zip(full_indices, directions)]
    proxy_support = [max(float(points[index].dot(direction)) for index in selected) for direction in directions]
    blocked = set()
    while len(selected) < target:
        choices = [
            (full_support[i] - proxy_support[i], i)
            for i in range(len(directions))
            if i not in blocked
        ]
        if not choices:
            break
        error, direction_index = max(choices)
        if error <= 1.0e-12:
            break
        point_index = full_indices[direction_index]
        if point_index in selected_set:
            blocked.add(direction_index)
            continue
        selected.append(point_index)
        selected_set.add(point_index)
        point = points[point_index]
        proxy_support = [max(value, float(point.dot(direction))) for value, direction in zip(proxy_support, directions)]
    if len(selected) < target:
        for point in _support_sample_points(points, target):
            key = _point_key(point)
            index = next((i for i, source in enumerate(points) if _point_key(source) == key), None)
            if index is not None and index not in selected_set:
                selected.append(index)
                selected_set.add(index)
                if len(selected) >= target:
                    break
    return [points[i].copy() for i in selected[:target]]


def _hull_characteristic_length(points: Sequence[Vector]) -> float:
    _center, half = _bounds_center_and_half_extents(points)
    return max(1.0e-8, 2.0 * float(half.length))


def _effective_hull_tolerance(points: Sequence[Vector], absolute: float, relative: float) -> Tuple[float, float]:
    characteristic = _hull_characteristic_length(points)
    return max(max(1.0e-8, float(absolute)), max(0.0, float(relative)) * characteristic), characteristic


def _directional_hull_error(complete: Sequence[Vector], simplified: Sequence[Vector]) -> Tuple[float, float]:
    if not complete or not simplified:
        return 0.0, 0.0
    directions = _support_directions()
    if _np is not None:
        complete_array = _np.asarray([(p.x, p.y, p.z) for p in complete], dtype=_np.float64)
        simplified_array = _np.asarray([(p.x, p.y, p.z) for p in simplified], dtype=_np.float64)
        direction_array = _np.asarray([(d.x, d.y, d.z) for d in directions], dtype=_np.float64)
        full_support = _np.max(complete_array @ direction_array.T, axis=0)
        proxy_support = _np.max(simplified_array @ direction_array.T, axis=0)
        errors = _np.maximum(0.0, full_support - proxy_support)
        return float(errors.max()) if errors.size else 0.0, float(_np.sqrt(_np.mean(errors * errors))) if errors.size else 0.0
    errors = []
    for direction in directions:
        full_support = max(float(point.dot(direction)) for point in complete)
        proxy_support = max(float(point.dot(direction)) for point in simplified)
        errors.append(max(0.0, full_support - proxy_support))
    maximum = max(errors, default=0.0)
    rms = math.sqrt(sum(error * error for error in errors) / max(1, len(errors)))
    return maximum, rms


def _hull_quality_settings(world, *, fracture_piece: bool = False) -> Dict[str, object]:
    preset = str(getattr(world, "hull_quality_preset", "BALANCED")) if world else "BALANCED"
    adaptive = bool(getattr(world, "adaptive_hull_accuracy", True)) if world else True
    separation_inset = (
        max(0.0, float(getattr(world, "fracture_hull_inset", 0.001)))
        if fracture_piece else 0.0
    )
    common = {
        "adaptive": adaptive,
        "algorithm": "SUPPORT_ERROR_V3_FRACTURE_INSET",
        "separation_inset": separation_inset,
        "fracture_piece": bool(fracture_piece),
    }
    if preset == "FAST":
        return {**common, "preset": preset, "minimum": 24, "maximum": 40, "rescue_maximum": 96, "absolute_tolerance": 0.0015, "relative_tolerance": 0.012, "precision_rescue": True}
    if preset == "ACCURATE":
        return {**common, "preset": preset, "minimum": 64, "maximum": 128, "rescue_maximum": 512, "absolute_tolerance": 0.00025, "relative_tolerance": 0.001, "precision_rescue": True}
    if preset == "CUSTOM":
        maximum = int(getattr(world, "convex_hull_max_vertices", 64))
        return {
            **common,
            "preset": preset,
            "adaptive": bool(getattr(world, "adaptive_hull_accuracy", True)),
            "minimum": max(4, int(getattr(world, "hull_min_vertices", 24))),
            "maximum": max(0, maximum),
            "rescue_maximum": max(4, int(getattr(world, "hull_rescue_max_vertices", 256))),
            "absolute_tolerance": max(1.0e-8, float(getattr(world, "hull_error_tolerance", 0.00075))),
            "relative_tolerance": max(0.0, float(getattr(world, "hull_relative_error_tolerance", 0.005))),
            "precision_rescue": True,
        }
    return {**common, "preset": "BALANCED", "minimum": 32, "maximum": 64, "rescue_maximum": 256, "absolute_tolerance": 0.00075, "relative_tolerance": 0.005, "precision_rescue": True}


def _adaptive_convex_hull_data(
    vertices: Sequence[Vector],
    minimum: int,
    maximum: int,
    rescue_maximum: int,
    absolute_tolerance: float,
    relative_tolerance: float,
) -> Tuple[List[Vector], Vector, int, Dict[str, object]]:
    complete_points, complete_center = _compute_convex_hull(vertices)
    raw_count = len(complete_points)
    effective_tolerance, characteristic_length = _effective_hull_tolerance(
        complete_points, absolute_tolerance, relative_tolerance
    )
    if maximum > 0:
        maximum = max(4, min(int(maximum), raw_count))
        minimum = max(4, min(int(minimum), maximum))
    rescue_maximum = max(maximum, min(max(4, int(rescue_maximum)), raw_count))
    if maximum <= 0 or raw_count <= max(4, minimum):
        return complete_points, complete_center, raw_count, {
            "selected_vertices": raw_count, "max_error": 0.0, "rms_error": 0.0, "target_met": True,
            "effective_tolerance": effective_tolerance, "characteristic_length": characteristic_length,
            "precision_rescue": False, "rescue_mode": "not_required",
        }

    targets: List[int] = []
    target = minimum
    while target < maximum:
        targets.append(target)
        target = min(maximum, max(target + 8, target * 2))
    targets.append(maximum)
    rescue_target = max(maximum + 16, maximum * 2)
    while rescue_target < rescue_maximum:
        targets.append(rescue_target)
        rescue_target = min(rescue_maximum, max(rescue_target + 32, int(rescue_target * 1.5)))
    if rescue_maximum > maximum:
        targets.append(rescue_maximum)

    best_points = complete_points
    best_center = complete_center
    best_error = float("inf")
    best_rms = float("inf")
    selected = raw_count
    target_met = False
    used_rescue = False
    for target in sorted(set(min(raw_count, value) for value in targets if value >= 4)):
        sampled = _support_error_sample_points(complete_points, target)
        points, center = _compute_convex_hull(sampled)
        if len(points) < 4:
            continue
        max_error, rms_error = _directional_hull_error(complete_points, points)
        if max_error < best_error or (math.isclose(max_error, best_error) and len(points) < len(best_points)):
            best_points, best_center = points, center
            best_error, best_rms = max_error, rms_error
            selected = len(points)
        target_met = max_error <= effective_tolerance
        if target_met:
            best_points, best_center = points, center
            best_error, best_rms = max_error, rms_error
            selected = len(points)
            used_rescue = target > maximum
            break

    if not target_met:
        return complete_points, complete_center, raw_count, {
            "selected_vertices": raw_count,
            "max_error": 0.0,
            "rms_error": 0.0,
            "target_met": True,
            "precision_rescue": True,
            "rescue_mode": "complete_hull",
            "pre_rescue_selected_vertices": selected,
            "pre_rescue_max_error": float(best_error if math.isfinite(best_error) else 0.0),
            "pre_rescue_rms_error": float(best_rms if math.isfinite(best_rms) else 0.0),
            "effective_tolerance": effective_tolerance,
            "characteristic_length": characteristic_length,
        }
    return best_points, best_center, raw_count, {
        "selected_vertices": selected,
        "max_error": float(best_error),
        "rms_error": float(best_rms),
        "target_met": True,
        "precision_rescue": bool(used_rescue),
        "rescue_mode": "budget_escalation" if used_rescue else "within_primary_budget",
        "effective_tolerance": effective_tolerance,
        "characteristic_length": characteristic_length,
    }


def _inset_hull_points(
    points: Sequence[Vector],
    center: Vector,
    requested_inset: float,
) -> Tuple[List[Vector], float]:
    """Shrink a hull around its physical center to separate touching fragments."""
    amount = max(0.0, float(requested_inset))
    if amount <= 0.0 or not points:
        return [point.copy() for point in points], 0.0
    result: List[Vector] = []
    maximum_applied = 0.0
    for point in points:
        delta = point - center
        length = delta.length
        if length <= 1.0e-12:
            result.append(point.copy())
            continue
        applied = min(amount, length * 0.12)
        maximum_applied = max(maximum_applied, applied)
        result.append(center + delta * ((length - applied) / length))
    return result, maximum_applied


def _cached_convex_hull_data(
    geometry: Dict,
    quality: Dict[str, object],
    profile: Optional[Dict] = None,
) -> Tuple[List[Vector], Vector, int, Dict[str, object]]:
    global _HULL_CACHE_TOTAL_HITS, _HULL_CACHE_TOTAL_MISSES
    hulls = geometry.setdefault("hulls", {})
    cache_key = json.dumps(quality, sort_keys=True, separators=(",", ":"))
    cached = hulls.get(cache_key)
    if cached is not None:
        _HULL_CACHE_TOTAL_HITS += 1
        _profile_add(profile, "hull_cache_hits", 1)
        return (
            [Vector(value) for value in cached["points"]],
            Vector(cached["center"]),
            int(cached["raw_count"]),
            dict(cached.get("quality", {})),
        )

    _HULL_CACHE_TOTAL_MISSES += 1
    _profile_add(profile, "hull_cache_misses", 1)
    started = time.perf_counter()
    vertices = [Vector(value) for value in geometry.get("vertices", [])]
    if bool(quality.get("adaptive", True)):
        points, center, raw_count, metrics = _adaptive_convex_hull_data(
            vertices,
            int(quality.get("minimum", 24)),
            int(quality.get("maximum", 64)),
            int(quality.get("rescue_maximum", 256)),
            float(quality.get("absolute_tolerance", 0.00075)),
            float(quality.get("relative_tolerance", 0.005)),
        )
    else:
        points, center, raw_count = _convex_hull_data(vertices, int(quality.get("maximum", 64)))
        complete = _compute_convex_hull(vertices)[0]
        max_error, rms_error = _directional_hull_error(complete, points)
        effective_tolerance, characteristic_length = _effective_hull_tolerance(
            complete,
            float(quality.get("absolute_tolerance", 0.00075)),
            float(quality.get("relative_tolerance", 0.005)),
        )
        metrics = {
            "selected_vertices": len(points), "max_error": max_error, "rms_error": rms_error,
            "target_met": max_error <= effective_tolerance, "effective_tolerance": effective_tolerance,
            "characteristic_length": characteristic_length, "precision_rescue": False,
            "rescue_mode": "disabled",
        }
    requested_inset = max(0.0, float(quality.get("separation_inset", 0.0)))
    points, applied_inset = _inset_hull_points(points, center, requested_inset)
    metrics["separation_inset_requested"] = requested_inset
    metrics["separation_inset_applied"] = float(applied_inset)
    metrics["fracture_piece"] = bool(quality.get("fracture_piece", False))
    metrics["preset"] = quality.get("preset", "CUSTOM")
    metrics["algorithm"] = quality.get("algorithm", "SUPPORT_ERROR_V3_FRACTURE_INSET")
    metrics["absolute_tolerance"] = float(quality.get("absolute_tolerance", 0.0))
    metrics["relative_tolerance"] = float(quality.get("relative_tolerance", 0.0))
    metrics["tolerance"] = float(metrics.get("effective_tolerance", quality.get("absolute_tolerance", 0.0)))
    hulls[cache_key] = {
        "points": [_vector_list(point) for point in points],
        "center": _vector_list(center),
        "raw_count": int(raw_count),
        "quality": metrics,
    }
    _store_persistent_hulls(geometry)
    _profile_add(profile, "hull_seconds", time.perf_counter() - started)
    return points, center, raw_count, metrics



def _point_in_projected_triangle_yz(y: float, z: float, a: Vector, b: Vector, c: Vector) -> Optional[Tuple[float, float, float]]:
    """Return barycentric weights for a YZ projection, or None when outside/degenerate."""
    v0y, v0z = b.y - a.y, b.z - a.z
    v1y, v1z = c.y - a.y, c.z - a.z
    v2y, v2z = y - a.y, z - a.z
    denominator = v0y * v1z - v1y * v0z
    if abs(denominator) <= 1.0e-12:
        return None
    inv = 1.0 / denominator
    u = (v2y * v1z - v1y * v2z) * inv
    v = (v0y * v2z - v2y * v0z) * inv
    w = 1.0 - u - v
    epsilon = -1.0e-7
    if u < epsilon or v < epsilon or w < epsilon:
        return None
    return w, u, v


def _voxel_occupancy(
    vertices: Sequence[Vector],
    indices: Sequence[int],
    resolution: int,
) -> Tuple[set, Vector, Vector, Tuple[int, int, int]]:
    """Voxelize a closed mesh with deterministic X-axis parity rays.

    The grid is intentionally modest. It is used only to generate a stable
    primitive compound proxy, not to reproduce render geometry.
    """
    if len(vertices) < 4:
        return set(), Vector((0.0, 0.0, 0.0)), Vector((1.0, 1.0, 1.0)), (0, 0, 0)
    minimum = Vector((min(v.x for v in vertices), min(v.y for v in vertices), min(v.z for v in vertices)))
    maximum = Vector((max(v.x for v in vertices), max(v.y for v in vertices), max(v.z for v in vertices)))
    extent = maximum - minimum
    longest = max(float(extent.x), float(extent.y), float(extent.z), 1.0e-6)
    base = max(3, int(resolution))
    dims = tuple(max(2, min(24, int(round(base * max(float(axis), longest / base) / longest)))) for axis in extent)
    # The expression above can over-compress thin axes. Preserve at least two
    # samples and cap all axes to keep payload creation bounded.
    nx, ny, nz = dims
    cell = Vector((extent.x / nx if nx else 1.0, extent.y / ny if ny else 1.0, extent.z / nz if nz else 1.0))
    occupied = set()

    triangles = []
    for offset in range(0, len(indices) - 2, 3):
        try:
            a, b, c = vertices[int(indices[offset])], vertices[int(indices[offset + 1])], vertices[int(indices[offset + 2])]
        except (IndexError, ValueError):
            continue
        triangles.append((a, b, c))

    # Interior sampling. For each YZ column, pair sorted ray intersections.
    for iy in range(ny):
        y = minimum.y + (iy + 0.5) * cell.y
        for iz in range(nz):
            z = minimum.z + (iz + 0.5) * cell.z
            crossings: List[float] = []
            for a, b, c in triangles:
                if y < min(a.y, b.y, c.y) - 1.0e-9 or y > max(a.y, b.y, c.y) + 1.0e-9:
                    continue
                if z < min(a.z, b.z, c.z) - 1.0e-9 or z > max(a.z, b.z, c.z) + 1.0e-9:
                    continue
                weights = _point_in_projected_triangle_yz(y, z, a, b, c)
                if weights is None:
                    continue
                wa, wb, wc = weights
                crossings.append(wa * a.x + wb * b.x + wc * c.x)
            if len(crossings) < 2:
                continue
            crossings.sort()
            unique: List[float] = []
            tolerance = max(1.0e-8, abs(cell.x) * 1.0e-5)
            for value in crossings:
                if not unique or abs(value - unique[-1]) > tolerance:
                    unique.append(value)
            for pair_index in range(0, len(unique) - 1, 2):
                low, high = unique[pair_index], unique[pair_index + 1]
                if high < low:
                    low, high = high, low
                for ix in range(nx):
                    x = minimum.x + (ix + 0.5) * cell.x
                    if low - tolerance <= x <= high + tolerance:
                        occupied.add((ix, iy, iz))

    # Closed-volume calculation can fail on imperfect fragments. Always seed
    # surface samples so a thin but valid piece still receives a proxy.
    def mark_point(point: Vector) -> None:
        ix = min(nx - 1, max(0, int((point.x - minimum.x) / max(cell.x, 1.0e-12))))
        iy = min(ny - 1, max(0, int((point.y - minimum.y) / max(cell.y, 1.0e-12))))
        iz = min(nz - 1, max(0, int((point.z - minimum.z) / max(cell.z, 1.0e-12))))
        occupied.add((ix, iy, iz))

    for vertex in vertices:
        mark_point(vertex)
    for a, b, c in triangles:
        mark_point((a + b + c) / 3.0)

    return occupied, minimum, cell, (nx, ny, nz)


def _box_voxel_volume(box: Tuple[int, int, int, int, int, int]) -> int:
    return (box[3] - box[0] + 1) * (box[4] - box[1] + 1) * (box[5] - box[2] + 1)


def _extract_voxel_boxes(occupied: set, max_parts: int) -> List[Tuple[int, int, int, int, int, int]]:
    """Create deterministic rectangular prisms and merge them to a part budget."""
    remaining = set(occupied)
    boxes: List[Tuple[int, int, int, int, int, int]] = []
    while remaining:
        x0, y0, z0 = min(remaining, key=lambda value: (value[2], value[1], value[0]))
        x1 = x0
        while (x1 + 1, y0, z0) in remaining:
            x1 += 1
        y1 = y0
        while all((x, y1 + 1, z0) in remaining for x in range(x0, x1 + 1)):
            y1 += 1
        z1 = z0
        while all(
            (x, y, z1 + 1) in remaining
            for x in range(x0, x1 + 1)
            for y in range(y0, y1 + 1)
        ):
            z1 += 1
        box = (x0, y0, z0, x1, y1, z1)
        boxes.append(box)
        for x in range(x0, x1 + 1):
            for y in range(y0, y1 + 1):
                for z in range(z0, z1 + 1):
                    remaining.discard((x, y, z))

    limit = max(2, int(max_parts))
    if len(boxes) <= limit:
        return boxes

    # A full pairwise merge becomes cubic when a fragmented surface seeds many
    # voxels. Cluster occupied cell centers directly instead; the fixed
    # initialization and iteration count keep the result deterministic.
    points = sorted(occupied, key=lambda value: (value[2], value[1], value[0]))
    cluster_count = min(limit, len(points))
    centers = [tuple(float(value) for value in points[0])]
    while len(centers) < cluster_count:
        candidate = max(
            points,
            key=lambda point: min(
                (point[0] - center[0]) ** 2 + (point[1] - center[1]) ** 2 + (point[2] - center[2]) ** 2
                for center in centers
            ),
        )
        centers.append(tuple(float(value) for value in candidate))

    assignments = [0] * len(points)
    for _iteration in range(12):
        changed = False
        groups = [[] for _ in centers]
        for index, point in enumerate(points):
            cluster = min(
                range(len(centers)),
                key=lambda item: (
                    (point[0] - centers[item][0]) ** 2
                    + (point[1] - centers[item][1]) ** 2
                    + (point[2] - centers[item][2]) ** 2,
                    item,
                ),
            )
            changed = changed or assignments[index] != cluster
            assignments[index] = cluster
            groups[cluster].append(point)
        new_centers = []
        for center, group in zip(centers, groups):
            if not group:
                new_centers.append(center)
                continue
            count = float(len(group))
            new_centers.append((
                sum(point[0] for point in group) / count,
                sum(point[1] for point in group) / count,
                sum(point[2] for point in group) / count,
            ))
        centers = new_centers
        if not changed:
            break

    clustered = []
    for cluster in range(len(centers)):
        group = [point for point, assignment in zip(points, assignments) if assignment == cluster]
        if not group:
            continue
        clustered.append((
            min(point[0] for point in group),
            min(point[1] for point in group),
            min(point[2] for point in group),
            max(point[0] for point in group),
            max(point[1] for point in group),
            max(point[2] for point in group),
        ))
    return clustered



def _triangle_records(
    vertices: Sequence[Vector], indices: Sequence[int]
) -> Tuple[List[Tuple[Vector, Vector, Vector, float, float, float, float]], List[Tuple[int, int, int]]]:
    """Build triangle records for deterministic inside tests and BVH polygons."""
    records: List[Tuple[Vector, Vector, Vector, float, float, float, float]] = []
    polygons: List[Tuple[int, int, int]] = []
    for offset in range(0, len(indices) - 2, 3):
        try:
            ia, ib, ic = int(indices[offset]), int(indices[offset + 1]), int(indices[offset + 2])
            a, b, c = vertices[ia], vertices[ib], vertices[ic]
        except (IndexError, TypeError, ValueError):
            continue
        records.append((a, b, c, min(a.y, b.y, c.y), max(a.y, b.y, c.y), min(a.z, b.z, c.z), max(a.z, b.z, c.z)))
        polygons.append((ia, ib, ic))
    return records, polygons


def _point_inside_triangle_mesh_x(
    point: Vector,
    triangles: Sequence[Tuple[Vector, Vector, Vector, float, float, float, float]],
    tolerance: float,
) -> bool:
    """Classify a point with a deterministic positive-X parity ray."""
    crossings: List[float] = []
    for a, b, c, min_y, max_y, min_z, max_z in triangles:
        if point.y < min_y - tolerance or point.y > max_y + tolerance:
            continue
        if point.z < min_z - tolerance or point.z > max_z + tolerance:
            continue
        weights = _point_in_projected_triangle_yz(point.y, point.z, a, b, c)
        if weights is None:
            continue
        wa, wb, wc = weights
        x = wa * a.x + wb * b.x + wc * c.x
        if x >= point.x - tolerance:
            crossings.append(float(x))
    if not crossings:
        return False
    crossings.sort()
    unique: List[float] = []
    for value in crossings:
        if not unique or abs(value - unique[-1]) > tolerance:
            unique.append(value)
    return bool(len(unique) % 2)


def _compound_surface_metrics(
    parts: Sequence[Dict[str, object]],
    vertices: Sequence[Vector],
    indices: Sequence[int],
    cell: Vector,
) -> Dict[str, object]:
    """Estimate box volume outside the source mesh and its maximum protrusion.

    Each box is sampled on a deterministic 3x3x3 lattice. Samples are weighted
    by box volume, so large proxy parts cannot hide behind many small parts.
    A BVH supplies nearest-surface distances while parity rays classify volume.
    """
    triangles, polygons = _triangle_records(vertices, indices)
    try:
        bvh = BVHTree.FromPolygons(vertices, polygons, all_triangles=True) if polygons else None
    except Exception:
        bvh = None

    positive_cells = [abs(float(value)) for value in cell if abs(float(value)) > 1.0e-12]
    surface_tolerance = max(1.0e-6, (min(positive_cells) if positive_cells else 1.0e-5) * 0.04)
    fractions = (0.06, 0.5, 0.94)
    total_weight = 0.0
    outside_weight = 0.0
    outside_distance_sq_weight = 0.0
    maximum_deviation = 0.0
    outside_samples = 0
    sample_count = 0

    for part in parts:
        center = Vector(part.get("center", (0.0, 0.0, 0.0)))
        half = Vector(part.get("half_extents", (0.0, 0.0, 0.0)))
        box_volume = max(0.0, 8.0 * float(half.x) * float(half.y) * float(half.z))
        point_weight = box_volume / 27.0 if box_volume > 0.0 else 0.0
        for fx in fractions:
            for fy in fractions:
                for fz in fractions:
                    point = Vector((
                        center.x + (2.0 * fx - 1.0) * half.x,
                        center.y + (2.0 * fy - 1.0) * half.y,
                        center.z + (2.0 * fz - 1.0) * half.z,
                    ))
                    sample_count += 1
                    total_weight += point_weight
                    nearest_distance = float("inf")
                    if bvh is not None:
                        try:
                            nearest = bvh.find_nearest(point)
                            if nearest is not None and nearest[3] is not None:
                                nearest_distance = max(0.0, float(nearest[3]))
                        except Exception:
                            nearest_distance = float("inf")
                    inside = (
                        nearest_distance <= surface_tolerance
                        or _point_inside_triangle_mesh_x(point, triangles, surface_tolerance)
                    )
                    if inside:
                        continue
                    outside_samples += 1
                    outside_weight += point_weight
                    if math.isfinite(nearest_distance):
                        maximum_deviation = max(maximum_deviation, nearest_distance)
                        outside_distance_sq_weight += nearest_distance * nearest_distance * point_weight

    outside_ratio = outside_weight / max(total_weight, 1.0e-18)
    rms_deviation = math.sqrt(outside_distance_sq_weight / max(outside_weight, 1.0e-18)) if outside_weight > 0.0 else 0.0
    return {
        "surface_distance_available": bool(bvh is not None),
        "sample_count": int(sample_count),
        "outside_samples": int(outside_samples),
        "sampled_outside_volume_ratio": float(outside_ratio),
        "sampled_inside_volume_ratio": float(max(0.0, 1.0 - outside_ratio)),
        "maximum_surface_deviation": float(maximum_deviation),
        "rms_surface_deviation": float(rms_deviation),
        "surface_tolerance": float(surface_tolerance),
    }

def _compound_settings(world) -> Dict[str, object]:
    preset = str(getattr(world, "compound_quality_preset", "BALANCED") if world is not None else "BALANCED").upper()
    presets = {
        "FAST": {
            "max_parts": 4, "absolute_tolerance": 0.010, "relative_tolerance": 0.010,
            "max_hull_vertices": 64, "preprocess_resolution": 30, "resolution": 1000,
            "mcts_nodes": 10, "mcts_iterations": 80,
        },
        "BALANCED": {
            "max_parts": 8, "absolute_tolerance": 0.003, "relative_tolerance": 0.005,
            "max_hull_vertices": 96, "preprocess_resolution": 50, "resolution": 2000,
            "mcts_nodes": 20, "mcts_iterations": 150,
        },
        "ACCURATE": {
            "max_parts": 16, "absolute_tolerance": 0.001, "relative_tolerance": 0.002,
            "max_hull_vertices": 128, "preprocess_resolution": 80, "resolution": 4000,
            "mcts_nodes": 30, "mcts_iterations": 250,
        },
    }
    if preset in presets:
        result = dict(presets[preset])
    else:
        preset = "CUSTOM"
        result = {
            "max_parts": max(2, int(getattr(world, "compound_max_parts", 8) if world is not None else 8)),
            "absolute_tolerance": max(1.0e-7, float(getattr(world, "compound_error_tolerance", 0.003) if world is not None else 0.003)),
            "relative_tolerance": max(0.0, float(getattr(world, "compound_relative_error_tolerance", 0.005) if world is not None else 0.005)),
            "max_hull_vertices": max(8, int(getattr(world, "compound_max_hull_vertices", 96) if world is not None else 96)),
            "preprocess_resolution": max(10, int(getattr(world, "compound_preprocess_resolution", 50) if world is not None else 50)),
            "resolution": max(100, int(getattr(world, "compound_resolution", 2000) if world is not None else 2000)),
            "mcts_nodes": 20,
            "mcts_iterations": max(10, int(getattr(world, "compound_mcts_iterations", 150) if world is not None else 150)),
        }
    result.update({
        "preset": preset,
        "inset": max(0.0, float(getattr(world, "compound_inset", 0.0005) if world is not None else 0.0005)),
        "preprocess_mode": "AUTO",
        "mcts_max_depth": 3,
        "merge": True,
        "decimate": False,
        "pca": False,
        "extrude": False,
        "seed": 0,
        "algorithm": f"COMPOUND_{COACD_EXECUTION_MODE}_{COACD_VERSION}_V4",
    })
    return result


def _mesh_volume_from_flat_indices(vertices: Sequence[Vector], indices: Sequence[int]) -> float:
    volume = 0.0
    for index in range(0, len(indices) - 2, 3):
        try:
            a = vertices[int(indices[index])]
            b = vertices[int(indices[index + 1])]
            c = vertices[int(indices[index + 2])]
        except (IndexError, TypeError, ValueError):
            continue
        volume += float(a.dot(b.cross(c))) / 6.0
    return abs(volume)


def _inset_convex_part(vertices: Sequence[Vector], inset: float) -> Tuple[List[Vector], Vector]:
    if not vertices:
        return [], Vector((0.0, 0.0, 0.0))
    center = sum(vertices, Vector((0.0, 0.0, 0.0))) / len(vertices)
    if inset <= 0.0:
        return [vertex.copy() for vertex in vertices], center
    result: List[Vector] = []
    for vertex in vertices:
        delta = vertex - center
        length = delta.length
        if length <= 1.0e-12:
            result.append(vertex.copy())
            continue
        # Never collapse a small part. The inset is primarily a sibling-contact
        # margin for the fixed-cluster fallback used by Culverin 0.13.2.
        amount = min(float(inset), length * 0.15)
        result.append(center + delta * ((length - amount) / length))
    return result, center


def _oriented_box_from_points(
    vertices: Sequence[Vector],
) -> Tuple[Vector, Vector, Quaternion]:
    """Return a deterministic local oriented box for a convex child hull."""
    if not vertices:
        return Vector((0.0, 0.0, 0.0)), Vector((1.0e-5, 1.0e-5, 1.0e-5)), Quaternion((1.0, 0.0, 0.0, 0.0))

    def axis_aligned():
        center, half = _bounds_center_and_half_extents(vertices)
        return center, half, Quaternion((1.0, 0.0, 0.0, 0.0))

    if _np is None or len(vertices) < 4:
        return axis_aligned()
    try:
        coordinates = _np.asarray([(float(v.x), float(v.y), float(v.z)) for v in vertices], dtype=_np.float64)
        mean = coordinates.mean(axis=0)
        centered = coordinates - mean
        covariance = (centered.T @ centered) / max(1, len(coordinates))
        eigenvalues, axes = _np.linalg.eigh(covariance)
        order = _np.argsort(eigenvalues)[::-1]
        eigenvalues = eigenvalues[order]
        axes = axes[:, order]
        largest = max(1.0e-18, float(abs(eigenvalues[0])))
        # Nearly equal eigenvalues make PCA axes arbitrary. Axis-aligned boxes
        # are deterministic and safer for those approximately spherical parts.
        if any(abs(float(eigenvalues[i] - eigenvalues[i + 1])) <= largest * 1.0e-7 for i in range(2)):
            return axis_aligned()
        for column in range(3):
            axis = axes[:, column]
            dominant = int(_np.argmax(_np.abs(axis)))
            if axis[dominant] < 0.0:
                axes[:, column] *= -1.0
        if float(_np.linalg.det(axes)) < 0.0:
            axes[:, 2] *= -1.0
        projected = centered @ axes
        minimum = projected.min(axis=0)
        maximum = projected.max(axis=0)
        local_center = (minimum + maximum) * 0.5
        box_center = mean + axes @ local_center
        half = _np.maximum((maximum - minimum) * 0.5, 1.0e-5)
        rotation_matrix = Matrix(tuple(
            tuple(float(axes[row, column]) for column in range(3))
            for row in range(3)
        ))
        rotation = rotation_matrix.to_quaternion().normalized()
        if rotation.w < 0.0:
            rotation = Quaternion((-rotation.w, -rotation.x, -rotation.y, -rotation.z))
        return Vector(tuple(map(float, box_center))), Vector(tuple(map(float, half))), rotation
    except Exception:
        return axis_aligned()


def _cached_compound_data(
    geometry: Dict,
    settings: Dict[str, object],
    profile: Optional[Dict] = None,
) -> Tuple[List[Dict[str, object]], Dict[str, object]]:
    """Create and cache a true convex decomposition using bundled CoACD."""
    compounds = geometry.setdefault("compounds", {})
    half = Vector(geometry.get("half_extents", (0.0, 0.0, 0.0)))
    characteristic_length = max(1.0e-8, float(half.length * 2.0))
    resolved = dict(settings)
    resolved["threshold"] = max(
        float(settings.get("absolute_tolerance", 0.003)),
        characteristic_length * float(settings.get("relative_tolerance", 0.005)),
    )
    resolved["characteristic_length"] = characteristic_length
    cache_key = json.dumps(resolved, sort_keys=True, separators=(",", ":"))
    cached = compounds.get(cache_key)
    if cached is not None:
        _profile_add(profile, "compound_cache_hits", 1)
        return [dict(part) for part in cached.get("parts", [])], dict(cached.get("quality", {}))

    _profile_add(profile, "compound_cache_misses", 1)
    started = time.perf_counter()
    raw_vertices = list(geometry.get("vertices", []))
    raw_indices = [int(value) for value in geometry.get("indices", [])]
    parts: List[Dict[str, object]] = []
    error: Optional[str] = None
    try:
        native_parts = coacd_decompose(raw_vertices, raw_indices, resolved)
        inset = float(resolved.get("inset", 0.0005))
        for native in native_parts[: int(resolved.get("max_parts", 8))]:
            source_points = [Vector(value) for value in native.get("vertices", [])]
            source_indices = [int(value) for value in native.get("indices", [])]
            if len(source_points) < 4:
                continue
            inset_points, center = _inset_convex_part(source_points, inset)
            # Re-hull and cap each child collider. CoACD can return dense convex
            # surfaces; Jolt only needs their support points for collision.
            part_vertex_limit = max(8, int(resolved.get("max_hull_vertices", 96)))
            hull_points, _hull_center, _raw_part_vertices, part_hull_quality = _adaptive_convex_hull_data(
                inset_points,
                min(24, part_vertex_limit),
                part_vertex_limit,
                part_vertex_limit,
                max(1.0e-7, float(resolved.get("threshold", 0.003)) * 0.25),
                0.0,
            )
            if len(hull_points) < 4:
                continue
            volume = _mesh_volume_from_flat_indices(source_points, source_indices)
            radius = max(((point - center).length for point in hull_points), default=1.0e-5)
            box_center, box_half_extents, box_rotation = _oriented_box_from_points(hull_points)
            parts.append({
                "vertices": [_vector_list(point) for point in hull_points],
                "indices": [],
                "center": _vector_list(center),
                "volume": float(max(volume, 1.0e-12)),
                "radius": float(max(radius, 1.0e-5)),
                "raw_vertex_count": int(_raw_part_vertices),
                "selected_vertex_count": int(len(hull_points)),
                "hull_quality": part_hull_quality,
                "box_center": _vector_list(box_center),
                "box_half_extents": _vector_list(box_half_extents),
                "box_rotation": [float(box_rotation.w), float(box_rotation.x), float(box_rotation.y), float(box_rotation.z)],
            })
        parts.sort(key=lambda part: (
            -float(part.get("volume", 0.0)),
            tuple(round(float(value), 9) for value in part.get("center", (0.0, 0.0, 0.0))),
        ))
        if not parts:
            raise CoACDError("CoACD returned no usable convex hulls after validation.")
    except Exception as exc:
        error = str(exc)
        parts = []

    elapsed = time.perf_counter() - started
    source_volume = max(0.0, float(geometry.get("volume_world", 0.0)))
    part_volume = sum(float(part.get("volume", 0.0)) for part in parts)
    runtime_proxy_volume = sum(
        max(0.0, 8.0 * float(part.get("box_half_extents", (0.0, 0.0, 0.0))[0])
            * float(part.get("box_half_extents", (0.0, 0.0, 0.0))[1])
            * float(part.get("box_half_extents", (0.0, 0.0, 0.0))[2]))
        for part in parts
    )
    runtime_proxy_ratio = runtime_proxy_volume / source_volume if source_volume > 1.0e-12 else None
    safe_interior_mode = str(COACD_EXECUTION_MODE).startswith("SAFE_INTERIOR")
    quality_reasons: List[str] = []
    if error:
        quality_reasons.append(error)
    if safe_interior_mode and runtime_proxy_ratio is not None and runtime_proxy_ratio > 1.02:
        quality_reasons.append(f"safe_proxy_overfill:{runtime_proxy_ratio:.6g}")
        parts = []
    quality = {
        "algorithm": resolved.get("algorithm"),
        "coacd_version": COACD_VERSION,
        "preset": resolved.get("preset", "BALANCED"),
        "part_count": len(parts),
        "max_parts": int(resolved.get("max_parts", 8)),
        "max_hull_vertices": int(resolved.get("max_hull_vertices", 96)),
        "threshold": float(resolved.get("threshold", 0.003)),
        "absolute_tolerance": float(resolved.get("absolute_tolerance", 0.003)),
        "relative_tolerance": float(resolved.get("relative_tolerance", 0.005)),
        "characteristic_length": characteristic_length,
        "source_volume": source_volume,
        "part_volume": part_volume,
        "part_to_source_volume_ratio": (part_volume / source_volume if source_volume > 1.0e-12 else None),
        "runtime_proxy_volume": float(runtime_proxy_volume),
        "runtime_proxy_to_source_volume_ratio": runtime_proxy_ratio,
        "conservative_interior_proxy": bool(safe_interior_mode),
        "inset": float(resolved.get("inset", 0.0)),
        "accepted": bool(parts),
        "fallback_reason": (quality_reasons[0] if quality_reasons else None),
        "fallback_reasons": quality_reasons,
        "seconds": float(elapsed),
        "runtime_representation": "NATIVE_CONVEX_OR_CONSERVATIVE_INTERIOR_BOX_COMPOUND",
        "native_compound_pending": True,
        "coacd_execution": COACD_EXECUTION_MODE,
    }
    compounds[cache_key] = {"parts": parts, "quality": quality}
    if parts:
        _store_persistent_hulls(geometry)
    _profile_add(profile, "compound_seconds", elapsed)
    return parts, quality


def _initial_speed(settings) -> float:
    return Vector(settings.initial_linear_velocity).length


def object_to_body_dict(
    obj: bpy.types.Object,
    depsgraph: bpy.types.Depsgraph,
    world_settings=None,
    profile: Optional[Dict] = None,
) -> Dict:
    settings = obj.ka_rigid_body
    world = world_settings
    managed_ground = is_ka_ground(obj)
    body_type = "STATIC" if managed_ground else str(settings.body_type)
    location, rotation, scale = _world_transform(obj)
    collision_shape = "PLANE" if managed_ground else str(settings.collision_shape)
    requested_collision_shape = collision_shape
    need_triangles = collision_shape in {"MESH", "COMPOUND_CONVEX"}
    source_geometry = _evaluated_geometry_entry(obj, depsgraph, profile)
    collision_proxy = (
        _resolve_collision_proxy(obj, settings)
        if collision_shape in {"CONVEX_HULL", "COMPOUND_CONVEX", "MESH"}
        else None
    )
    geometry = (
        _collision_proxy_geometry_entry(obj, collision_proxy, depsgraph, profile)
        if collision_proxy is not None
        else source_geometry
    )
    if collision_proxy is not None and profile is not None:
        profile["collision_proxy_bodies"] = int(profile.get("collision_proxy_bodies", 0)) + 1

    local_vertices = [Vector(value) for value in geometry.get("vertices", [])]
    mesh_indices = list(geometry.get("indices", [])) if need_triangles else []
    bounds_center = Vector(geometry.get("bounds_center", (0.0, 0.0, 0.0)))
    half_extents_vector = Vector(geometry.get("half_extents", (1.0e-5, 1.0e-5, 1.0e-5)))

    shape_center = Vector((0.0, 0.0, 0.0)) if managed_ground else bounds_center
    convex_vertices: List[Vector] = []
    compound_parts: List[Dict[str, object]] = []
    compound_quality_metrics: Dict[str, object] = {}
    raw_hull_vertex_count = 0
    hull_quality_metrics: Dict[str, object] = {}
    hull_quality = _hull_quality_settings(world, fracture_piece=is_ka_fracture_piece(obj))

    # Compound Convex always keeps a single-hull fallback in the payload. This
    # makes a failed decomposition safe and also gives diagnostics a direct
    # precision/performance comparison for the same render mesh.
    if collision_shape in {"CONVEX_HULL", "COMPOUND_CONVEX"}:
        convex_vertices, shape_center, raw_hull_vertex_count, hull_quality_metrics = _cached_convex_hull_data(
            geometry, hull_quality, profile
        )
        if profile is not None:
            profile["hull_selected_vertices"] = int(profile.get("hull_selected_vertices", 0)) + len(convex_vertices)
            profile["hull_raw_vertices"] = int(profile.get("hull_raw_vertices", 0)) + int(raw_hull_vertex_count)
            profile["hull_error_max"] = max(
                float(profile.get("hull_error_max", 0.0)),
                float(hull_quality_metrics.get("max_error", 0.0)),
            )
            profile["hull_targets_missed"] = int(profile.get("hull_targets_missed", 0)) + int(
                hull_quality_metrics.get("target_met") is False
            )
            profile["hull_precision_rescues"] = int(profile.get("hull_precision_rescues", 0)) + int(
                bool(hull_quality_metrics.get("precision_rescue"))
            )

    if collision_shape == "COMPOUND_CONVEX":
        if len(mesh_indices) < 3 or len(local_vertices) < 4:
            compound_quality_metrics = {
                "accepted": False,
                "fallback_reason": "Compound Convex source contains no usable triangle mesh.",
                "fallback_reasons": ["missing_triangle_mesh"],
                "runtime_representation": "SINGLE_HULL_FALLBACK",
            }
            collision_shape = "CONVEX_HULL"
        else:
            compound_parts, compound_quality_metrics = _cached_compound_data(
                geometry, _compound_settings(world), profile
            )
            if compound_quality_metrics.get("accepted") and compound_parts:
                shape_center = bounds_center
                if profile is not None:
                    profile["compound_bodies"] = int(profile.get("compound_bodies", 0)) + 1
                    profile["compound_parts"] = int(profile.get("compound_parts", 0)) + len(compound_parts)
                    profile["compound_native_bodies"] = int(profile.get("compound_native_bodies", 0)) + len(compound_parts)
            else:
                collision_shape = "CONVEX_HULL"
                if profile is not None:
                    profile["compound_fallbacks"] = int(profile.get("compound_fallbacks", 0)) + 1
                    reasons = profile.setdefault("compound_fallback_reasons", {})
                    rejected = list(compound_quality_metrics.get("fallback_reasons") or [])
                    if not rejected:
                        rejected = [str(compound_quality_metrics.get("fallback_reason") or "unknown")]
                    for reason in rejected:
                        reason = str(reason)
                        reasons[reason] = int(reasons.get(reason, 0)) + 1

    if collision_shape == "SPHERE":
        radius = max(
            1.0e-5,
            max(((vertex - shape_center).length for vertex in local_vertices), default=max(half_extents_vector)),
        )
    else:
        radius = max(1.0e-5, math.sqrt(sum(component * component for component in half_extents_vector)))

    raw_mass = calculate_mass(obj, depsgraph, profile, source_geometry) if body_type == "DYNAMIC" else 0.0
    effective_mass = raw_mass
    stability_adjustments: List[str] = []
    skip_simulation = False

    if body_type == "DYNAMIC" and world is not None:
        minimum_mass = max(1.0e-6, float(world.minimum_dynamic_mass))
        minimum_radius = max(1.0e-5, float(world.minimum_body_radius))
        too_light = raw_mass < minimum_mass
        too_small = radius < minimum_radius
        policy = str(world.small_body_policy)
        if policy == "SKIP" and (too_light or too_small):
            skip_simulation = True
            stability_adjustments.append("excluded_small_body")
        elif policy == "STABILIZE":
            if too_light:
                effective_mass = minimum_mass
                stability_adjustments.append("mass_clamped")
            if too_small and collision_shape not in {"SPHERE", "BOX"}:
                collision_shape = "BOX"
                shape_center = bounds_center
                convex_vertices = []
                compound_parts = []
                compound_quality_metrics = {}
                mesh_indices = []
                stability_adjustments.append("box_proxy")

    ccd_requested = bool(settings.use_ccd)
    ccd_effective = ccd_requested
    ccd_reason = "manual"
    if body_type != "DYNAMIC":
        ccd_effective = False
        ccd_reason = "non_dynamic"
    elif world is not None and bool(world.adaptive_ccd) and ccd_requested:
        speed = _initial_speed(settings)
        small_enough = radius <= max(1.0e-5, float(world.ccd_max_radius))
        fast_enough = speed >= max(0.0, float(world.ccd_speed_threshold))
        ccd_effective = small_enough or fast_enough
        if small_enough:
            ccd_reason = "adaptive_small_body"
        elif fast_enough:
            ccd_reason = "adaptive_initial_speed"
        else:
            ccd_reason = "adaptive_not_required"
    elif not ccd_requested:
        ccd_reason = "disabled_on_body"

    body = {
        "stable_id": ensure_stable_id(obj, BODY_ID_PROPERTY),
        "name": obj.name_full,
        "body_type": body_type,
        "collision_shape": collision_shape,
        "source_collision_shape": requested_collision_shape,
        "managed_ground": bool(managed_ground),
        "location": list(location),
        "rotation": [rotation.w, rotation.x, rotation.y, rotation.z],
        "scale": list(scale),
        "shape_center": _vector_list(shape_center),
        "half_extents": _vector_list(half_extents_vector),
        "radius": float(radius),
        "mass": float(effective_mass),
        "raw_mass": float(raw_mass),
        "mass_mode": settings.mass_mode,
        "density": float(settings.density),
        "friction": float(settings.friction),
        "restitution": float(settings.restitution),
        "linear_damping": float(settings.linear_damping),
        "angular_damping": float(settings.angular_damping),
        "linear_velocity": list(settings.initial_linear_velocity),
        "angular_velocity": list(settings.initial_angular_velocity),
        "ccd": bool(ccd_effective),
        "ccd_requested": ccd_requested,
        "ccd_reason": ccd_reason,
        "collision_layer": 1 << max(0, min(15, int(settings.collision_layer))),
        "collision_mask": int(settings.collision_mask) & 0xFFFF,
        "source_vertex_count": int(geometry.get("source_vertex_count", len(local_vertices))),
        "render_source_vertex_count": int(source_geometry.get("source_vertex_count", 0)),
        "collision_proxy": collision_proxy.name_full if collision_proxy is not None else None,
        "stability_adjustments": stability_adjustments,
        "skip_simulation": skip_simulation,
    }
    if collision_shape == "CONVEX_HULL":
        body["convex_vertices"] = [_vector_list(vertex) for vertex in convex_vertices]
        body["convex_vertex_count"] = len(convex_vertices)
        body["convex_vertex_count_raw"] = raw_hull_vertex_count
        body["collider_quality"] = hull_quality_metrics
        if requested_collision_shape == "COMPOUND_CONVEX":
            body["compound_parts"] = []
            body["compound_part_count"] = 0
            body["compound_quality"] = compound_quality_metrics
            body["compound_fallback"] = True
    elif collision_shape == "COMPOUND_CONVEX":
        body["convex_vertices"] = [_vector_list(vertex) for vertex in convex_vertices]
        body["compound_parts"] = compound_parts
        body["compound_part_count"] = len(compound_parts)
        body["compound_quality"] = compound_quality_metrics
        body["convex_vertex_count"] = len(convex_vertices)
        body["convex_vertex_count_raw"] = raw_hull_vertex_count
        body["collider_quality"] = hull_quality_metrics
        body["compound_fallback"] = False
    elif collision_shape == "MESH":
        body["mesh_vertices"] = [_vector_list(vertex) for vertex in local_vertices]
        body["mesh_indices"] = mesh_indices
        body["triangle_count"] = len(mesh_indices) // 3
    return body

def enabled_body_objects(scene: bpy.types.Scene) -> List[bpy.types.Object]:
    objects = [obj for obj in scene.objects if hasattr(obj, "ka_rigid_body") and obj.ka_rigid_body.enabled]
    stable_ids = ensure_scene_body_ids(objects)
    # Stable UUIDs, not names, determine solver insertion order. Renaming an
    # object therefore no longer changes contact ordering or cache identity.
    body_priority = {"STATIC": 0, "KINEMATIC": 1, "DYNAMIC": 2}
    return sorted(
        objects,
        key=lambda obj: (
            body_priority.get(str(obj.ka_rigid_body.body_type), 3),
            stable_ids[id(obj)],
        ),
    )


def preflight_scene(scene: bpy.types.Scene, *, auto_fix: bool = False) -> Dict:
    """Validate backend-critical input and optionally repair safe collider mistakes."""
    world = scene.ka_rigid_world
    bodies = enabled_body_objects(scene)
    result: Dict[str, object] = {
        "errors": [],
        "warnings": [],
        "fixed": [],
        "small_bodies": [],
        "body_count_before": len(bodies),
        "body_count": len(bodies),
        "dynamic_count": 0,
        "static_count": 0,
        "mass_ratio": None,
        "mass_ratio_before": None,
        "mass_conditioning_floor": None,
        "ground_objects": [],
        "duplicate_static_groups": [],
        "excluded_static_duplicates": [],
        "deleted_static_duplicates": [],
    }
    errors: List[str] = result["errors"]  # type: ignore[assignment]
    warnings: List[str] = result["warnings"]  # type: ignore[assignment]
    fixed: List[str] = result["fixed"]  # type: ignore[assignment]
    small_bodies: List[str] = result["small_bodies"]  # type: ignore[assignment]
    duplicate_reports: List[Dict[str, object]] = result["duplicate_static_groups"]  # type: ignore[assignment]
    excluded_duplicates: List[str] = result["excluded_static_duplicates"]  # type: ignore[assignment]
    deleted_duplicates: List[str] = result["deleted_static_duplicates"]  # type: ignore[assignment]

    if not bodies:
        errors.append("No KA Rigid Dynamics bodies are enabled.")
        return result

    if world.backend == "REFERENCE" and any(
        obj.ka_rigid_body.collision_shape in {"CONVEX_HULL", "COMPOUND_CONVEX", "MESH"}
        for obj in bodies
    ):
        warnings.append("Reference cannot accurately simulate Convex Hull, Compound Convex or Mesh bodies; Jolt will be selected for Bake.")

    depsgraph = bpy.context.evaluated_depsgraph_get()
    duplicate_policy = str(getattr(world, "duplicate_static_policy", "EXCLUDE"))

    # A KA ground is a managed singleton and must remain a static infinite
    # plane. Bulk body assignment used to turn it into a dynamic convex body
    # and a later Static assignment converted it into a one-sided mesh, which
    # allowed selected fragments to fall through. Repair this before any other
    # collider validation or duplicate detection.
    all_grounds = ground_objects(scene, enabled_only=True)
    for ground in all_grounds:
        settings = ground.ka_rigid_body
        invalid = (settings.body_type != "STATIC" or settings.collision_shape != "PLANE")
        if not invalid:
            continue
        message = f"{ground.name}: managed ground must be an enabled Static Plane collider."
        if auto_fix:
            changed = repair_managed_ground(ground)
            fixed.append(f"{ground.name}: restored managed ground invariant ({', '.join(changed)})")
        else:
            errors.append(message)

    # A KA ground is a managed singleton. Multiple enabled KA grounds are
    # always suspicious, even if one was moved after creation.
    enabled_grounds = [obj for obj in ground_objects(scene, enabled_only=True)]
    result["ground_objects"] = [obj.name_full for obj in enabled_grounds]
    if len(enabled_grounds) > 1:
        keeper = _preferred_static_keeper(enabled_grounds)
        duplicates = [obj for obj in enabled_grounds if obj != keeper]
        duplicate_reports.append({
            "type": "GROUND_SINGLETON",
            "keeper": keeper.name_full,
            "duplicates": [obj.name_full for obj in duplicates],
        })
        message = (
            f"Multiple KA ground objects are enabled. Keeping {keeper.name_full}; "
            f"duplicates: {', '.join(obj.name_full for obj in duplicates)}."
        )
        if auto_fix and duplicate_policy in {"EXCLUDE", "DELETE"}:
            changed = _apply_duplicate_policy(scene, keeper, duplicates, duplicate_policy)
            excluded_duplicates.extend(changed["excluded"])
            deleted_duplicates.extend(changed["deleted"])
            action = "excluded from Bake" if duplicate_policy == "EXCLUDE" else "deleted"
            for name in changed["excluded"] + changed["deleted"]:
                fixed.append(f"{name}: duplicate KA ground {action}; keeper {keeper.name_full}")
        else:
            warnings.append(message)

    # Refresh after ground cleanup before looking for exact overlapping static
    # colliders. Groups consisting only of managed grounds are skipped because
    # the singleton check above already reports them; mixed ground/non-ground
    # overlaps are still detected.
    bodies = enabled_body_objects(scene)
    static_groups = duplicate_static_collider_groups(
        scene, bodies=bodies, depsgraph=depsgraph, include_ground_groups=True
    )
    for group in static_groups:
        if bool(group.get("ground_group", False)):
            continue
        keeper = group["keeper"]
        duplicates = list(group["duplicates"])
        duplicate_reports.append({
            "type": "OVERLAPPING_STATIC",
            "keeper": keeper.name_full,
            "duplicates": [obj.name_full for obj in duplicates],
        })
        message = (
            f"Overlapping static colliders detected. Keeping {keeper.name_full}; "
            f"duplicates: {', '.join(obj.name_full for obj in duplicates)}."
        )
        if auto_fix and duplicate_policy in {"EXCLUDE", "DELETE"}:
            changed = _apply_duplicate_policy(scene, keeper, duplicates, duplicate_policy)
            excluded_duplicates.extend(changed["excluded"])
            deleted_duplicates.extend(changed["deleted"])
            action = "excluded from Bake" if duplicate_policy == "EXCLUDE" else "deleted"
            for name in changed["excluded"] + changed["deleted"]:
                fixed.append(f"{name}: overlapping static collider {action}; keeper {keeper.name_full}")
        else:
            warnings.append(message)

    bodies = enabled_body_objects(scene)
    result["body_count"] = len(bodies)
    result["ground_objects"] = [obj.name_full for obj in ground_objects(scene, enabled_only=True)]

    dynamic_masses: List[float] = []
    names = set()
    for obj in bodies:
        settings = obj.ka_rigid_body
        if obj.name_full in names:
            errors.append(f"Duplicate object name: {obj.name_full}")
        names.add(obj.name_full)

        if settings.body_type == "DYNAMIC":
            result["dynamic_count"] = int(result["dynamic_count"]) + 1
        else:
            result["static_count"] = int(result["static_count"]) + 1

        if settings.collision_shape == "MESH" and settings.body_type != "STATIC":
            message = f"{obj.name}: Mesh colliders are static-only; use Compound Convex for precise moving bodies."
            if auto_fix:
                settings.collision_shape = "CONVEX_HULL"
                fixed.append(f"{obj.name}: Mesh -> Convex Hull")
            else:
                errors.append(message)

        if settings.collision_shape == "PLANE" and settings.body_type == "DYNAMIC":
            message = f"{obj.name}: a Plane collider cannot be dynamic."
            if auto_fix:
                settings.collision_shape = "CONVEX_HULL"
                fixed.append(f"{obj.name}: Plane -> Convex Hull")
            else:
                errors.append(message)

        if settings.body_type != "DYNAMIC":
            continue

        geometry = _evaluated_geometry_entry(obj, depsgraph)
        raw_mass = calculate_mass(obj, depsgraph, geometry=geometry)
        half_extents = Vector(geometry.get("half_extents", (0.0, 0.0, 0.0)))
        approx_radius = max(1.0e-5, half_extents.length)
        minimum_mass = max(1.0e-6, float(world.minimum_dynamic_mass))
        minimum_radius = max(1.0e-5, float(world.minimum_body_radius))
        is_small = raw_mass < minimum_mass or approx_radius < minimum_radius
        if is_small:
            small_bodies.append(obj.name_full)
            warnings.append(
                f"{obj.name}: small dynamic body (mass {raw_mass:.6g} kg, radius ~{approx_radius:.6g} m); "
                f"policy {world.small_body_policy} will be applied."
            )

        if world.small_body_policy == "SKIP" and is_small:
            continue
        effective_mass = max(raw_mass, minimum_mass) if world.small_body_policy == "STABILIZE" else raw_mass
        dynamic_masses.append(effective_mass)

        if settings.mass_mode == "DENSITY":
            volume = float(geometry.get("volume_world", 0.0))
            if volume <= 1.0e-9:
                warnings.append(f"{obj.name}: closed mesh volume could not be calculated; bounding-box mass fallback will be used.")

    if int(result["dynamic_count"]) == 0:
        errors.append("No dynamic body is enabled.")
    if int(result["static_count"]) == 0:
        warnings.append("No static or kinematic collider is enabled.")

    if len(dynamic_masses) >= 2:
        smallest = min(dynamic_masses)
        largest = max(dynamic_masses)
        ratio_before = largest / max(1.0e-12, smallest)
        ratio = ratio_before
        result["mass_ratio_before"] = ratio_before
        limit = max(10.0, float(world.max_mass_ratio))
        conditioning = (
            str(world.small_body_policy) == "STABILIZE"
            and bool(getattr(world, "enforce_mass_ratio_limit", True))
        )
        if conditioning and ratio_before > limit:
            floor = max(float(world.minimum_dynamic_mass), largest / limit)
            result["mass_conditioning_floor"] = floor
            ratio = largest / max(floor, smallest)
            warnings.append(
                f"Dynamic mass ratio {ratio_before:.1f}:1 will be conditioned to approximately "
                f"{ratio:.1f}:1 using a solver-only mass floor of {floor:.6g} kg."
            )
        elif ratio_before > limit:
            warnings.append(
                f"Dynamic mass ratio is {ratio_before:.1f}:1 ({smallest:.6g} to {largest:.6g} kg), "
                f"above the configured limit {limit:.1f}:1."
            )
        result["mass_ratio"] = ratio

    return result


def _condition_dynamic_mass_ratios(bodies: Sequence[Dict], world, profile: Optional[Dict] = None) -> Dict[str, float]:
    """Condition solver-only masses without changing source object settings.

    Extremely small fragments coupled to heavy bodies produce poorly conditioned
    contact islands. In STABILIZE mode, raise only the simulation mass floor so
    the global dynamic ratio remains bounded.
    """
    dynamic = [
        body for body in bodies
        if body.get("body_type") == "DYNAMIC" and not body.get("skip_simulation")
    ]
    summary = {"enabled": False, "largest_mass": 0.0, "mass_floor": 0.0, "adjusted_bodies": 0, "ratio_before": 0.0, "ratio_after": 0.0}
    if not dynamic:
        return summary
    masses = [max(1.0e-12, float(body.get("mass", 0.0))) for body in dynamic]
    largest = max(masses)
    smallest = min(masses)
    ratio_before = largest / smallest
    summary.update({"largest_mass": largest, "ratio_before": ratio_before, "ratio_after": ratio_before})
    enabled = (
        str(getattr(world, "small_body_policy", "SIMULATE")) == "STABILIZE"
        and bool(getattr(world, "enforce_mass_ratio_limit", True))
    )
    if not enabled:
        return summary
    limit = max(10.0, float(getattr(world, "max_mass_ratio", 5000.0)))
    absolute_floor = max(1.0e-6, float(getattr(world, "minimum_dynamic_mass", 0.001)))
    mass_floor = max(absolute_floor, largest / limit)
    adjusted = 0
    for body in dynamic:
        mass = max(0.0, float(body.get("mass", 0.0)))
        if mass + 1.0e-12 >= mass_floor:
            continue
        body["mass"] = float(mass_floor)
        adjustments = body.setdefault("stability_adjustments", [])
        if "mass_ratio_clamped" not in adjustments:
            adjustments.append("mass_ratio_clamped")
        adjusted += 1
    summary.update({
        "enabled": True,
        "mass_floor": mass_floor,
        "adjusted_bodies": adjusted,
        "ratio_after": largest / max(mass_floor, smallest),
    })
    if profile is not None:
        profile["mass_conditioning"] = dict(summary)
    return summary


def build_scene_payload(scene: bpy.types.Scene) -> Dict:
    total_started = time.perf_counter()
    _configure_persistent_hull_cache(resolve_cache_directory(scene))
    depsgraph = bpy.context.evaluated_depsgraph_get()
    world = scene.ka_rigid_world
    profile: Dict[str, object] = {
        "geometry_cache_hits": 0,
        "geometry_cache_misses": 0,
        "hull_cache_hits": 0,
        "hull_cache_misses": 0,
        "hull_precision_rescues": 0,
        "compound_cache_hits": 0,
        "compound_cache_misses": 0,
        "compound_bodies": 0,
        "compound_parts": 0,
        "compound_native_bodies": 0,
        "compound_fallbacks": 0,
        "compound_fallback_reasons": {},
        "mesh_read_seconds": 0.0,
        "vertex_transform_seconds": 0.0,
        "volume_seconds": 0.0,
        "hull_seconds": 0.0,
        "compound_seconds": 0.0,
        "mass_seconds": 0.0,
        "collision_proxy_bodies": 0,
        "collision_proxy_cache_hits": 0,
        "collision_proxy_cache_misses": 0,
        "collision_proxy_seconds": 0.0,
    }
    objects = enabled_body_objects(scene)
    extraction_started = time.perf_counter()
    extracted = [object_to_body_dict(obj, depsgraph, world, profile) for obj in objects]
    profile["body_extraction_seconds"] = time.perf_counter() - extraction_started
    mass_conditioning = _condition_dynamic_mass_ratios(extracted, world, profile)
    skipped = [
        {
            "name": body["name"],
            "raw_mass": body.get("raw_mass"),
            "radius": body.get("radius"),
            "reasons": body.get("stability_adjustments", []),
        }
        for body in extracted
        if body.get("skip_simulation")
    ]
    bodies = [body for body in extracted if not body.get("skip_simulation")]
    constraints = (
        bonds_for_enabled_bodies(scene, (str(body.get("stable_id", "")) for body in bodies))
        if bool(getattr(world, "bond_enabled", True))
        else []
    )
    gravity = scene.gravity if world.use_scene_gravity else world.gravity
    scene_id = ensure_stable_id(scene, SCENE_ID_PROPERTY)
    payload = {
        "scene_id": scene_id,
        "scene_name": scene.name,
        "frame_start": int(world.frame_start),
        "frame_end": int(world.frame_end),
        "fps": float(scene.render.fps) / max(1.0e-6, float(scene.render.fps_base)),
        "gravity": list(gravity),
        "substeps": int(world.substeps),
        "adaptive_substeps": bool(world.adaptive_substeps),
        "minimum_substeps": int(world.minimum_substeps),
        "solver_iterations": int(world.solver_iterations),
        "sleep_enabled": bool(world.sleep_enabled),
        "sleep_mode": str(world.sleep_mode),
        "sleep_linear_threshold": float(world.sleep_linear_threshold),
        "sleep_angular_threshold": float(world.sleep_angular_threshold),
        "sleep_time": float(world.sleep_time),
        "jolt_threads_requested": int(world.jolt_threads),
        "jolt_threads": 1 if str(world.reproducibility_mode) == "STRICT" else int(world.jolt_threads),
        "reproducibility_mode": str(world.reproducibility_mode),
        "deterministic_mode": str(world.reproducibility_mode) != "PERFORMANCE",
        "early_sleep_termination": bool(world.early_sleep_termination),
        "early_sleep_frames": int(world.early_sleep_frames),
        "determinism_tolerance": float(world.determinism_tolerance),
        "penetration_slop": float(world.penetration_slop),
        "backend": world.backend,
        # Normal Blender bakes always use the direct binary Float32 frame path.
        # Python frame dictionaries are reserved for internal regression fixtures.
        "store_python_frames": False,
        "stability": {
            "small_body_policy": str(world.small_body_policy),
            "minimum_dynamic_mass": float(world.minimum_dynamic_mass),
            "minimum_body_radius": float(world.minimum_body_radius),
            "enforce_mass_ratio_limit": bool(world.enforce_mass_ratio_limit),
            "max_mass_ratio": float(world.max_mass_ratio),
            "mass_conditioning": mass_conditioning,
            "convex_hull_max_vertices": int(world.convex_hull_max_vertices),
            "fracture_hull_inset": float(world.fracture_hull_inset),
            "fracture_friction": float(world.fracture_friction),
            "bond_stability_mode": str(world.bond_stability_mode),
            "adaptive_hull_accuracy": bool(world.adaptive_hull_accuracy),
            "hull_quality_preset": str(world.hull_quality_preset),
            "hull_error_tolerance": float(world.hull_error_tolerance),
            "hull_relative_error_tolerance": float(world.hull_relative_error_tolerance),
            "hull_rescue_max_vertices": int(world.hull_rescue_max_vertices),
            "hull_min_vertices": int(world.hull_min_vertices),
            "compound_quality_preset": str(world.compound_quality_preset),
            "compound_max_parts": int(world.compound_max_parts),
            "compound_error_tolerance": float(world.compound_error_tolerance),
            "compound_relative_error_tolerance": float(world.compound_relative_error_tolerance),
            "compound_max_hull_vertices": int(world.compound_max_hull_vertices),
            "compound_preprocess_resolution": int(world.compound_preprocess_resolution),
            "compound_resolution": int(world.compound_resolution),
            "compound_mcts_iterations": int(world.compound_mcts_iterations),
            "compound_inset": float(world.compound_inset),
            "compound_algorithm": f"{COACD_EXECUTION_MODE} / CoACD {COACD_VERSION}",
            "compound_runtime_representation": "NATIVE_CONVEX_OR_SINGLE_BODY_OBB_COMPOUND",
            "adaptive_ccd": bool(world.adaptive_ccd),
            "ccd_max_radius": float(world.ccd_max_radius),
            "ccd_speed_threshold": float(world.ccd_speed_threshold),
        },
        "runtime": {
            "addon_version": ADDON_VERSION,
            "coacd_version": COACD_VERSION,
            "signature_schema": SIGNATURE_SCHEMA,
            "cache_version": CACHE_VERSION,
            "culverin_version": BUNDLED_CULVERIN_VERSION,
        },
        "skipped_bodies": skipped,
        "bodies": bodies,
        "constraints": constraints,
    }
    payload["simulation_scene"] = build_simulation_scene(payload, scene_id=scene_id)
    payload["runtime"]["simulation_scene_schema"] = SIMULATION_SCENE_VERSION
    signature_started = time.perf_counter()
    payload["signature"] = scene_signature(payload)
    profile["signature_seconds"] = time.perf_counter() - signature_started
    _save_persistent_hull_cache()
    profile["payload_total_seconds"] = time.perf_counter() - total_started
    profile["cache"] = geometry_cache_stats()
    profile["persistent_hull_cache_entries"] = len(_PERSISTENT_HULL_CACHE)
    payload["build_profile"] = profile
    return payload


def scene_signature(payload: Dict) -> str:
    scene = payload.get("simulation_scene")
    if isinstance(scene, dict):
        return canonical_scene_digest(scene)
    filtered = {
        key: value
        for key, value in payload.items()
        if key not in {"signature", "build_profile", "diagnostics"}
    }
    encoded = json.dumps(filtered, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def validate_scene(scene: bpy.types.Scene) -> List[str]:
    report = preflight_scene(scene, auto_fix=False)
    messages: List[str] = []
    messages.extend(f"ERROR: {message}" for message in report["errors"])
    messages.extend(f"WARNING: {message}" for message in report["warnings"])
    messages.extend(f"FIXED: {message}" for message in report["fixed"])
    messages.append(
        f"Bodies: {report['body_count']} ({report['dynamic_count']} dynamic, {report['static_count']} static/kinematic)."
    )
    cache_path = cache_file_path(resolve_cache_directory(scene))
    messages.append(f"Cache: {cache_path}")
    return messages


def apply_snapshot(snapshot: Dict[str, Dict]) -> int:
    changed = 0
    for object_name, transform in snapshot.items():
        obj = bpy.data.objects.get(object_name)
        if obj is None:
            continue
        location = Vector(transform["location"])
        rotation = Quaternion(transform["rotation"])
        scale = Vector(transform["scale"])
        obj.matrix_world = Matrix.LocRotScale(location, rotation, scale)
        try:
            obj.update_tag(refresh={"OBJECT"})
        except (AttributeError, TypeError):
            try:
                obj.update_tag()
            except AttributeError:
                pass
        changed += 1
    return changed


def fracture_candidates(context: bpy.types.Context) -> List[bpy.types.Object]:
    selected = [obj for obj in context.selected_objects if obj.type == "MESH"]
    tagged = [obj for obj in context.scene.objects if obj.type == "MESH" and any(bool(obj.get(tag, False)) for tag in FRACTURE_TAGS)]
    if tagged:
        return tagged
    if selected:
        return selected
    active_collection = context.collection
    return [obj for obj in active_collection.all_objects if obj.type == "MESH"]
