"""Solver-neutral SimulationScene v1 schema and compatibility adapters.

The Blender extractor still exposes the legacy payload during the 0.6 transition,
but backends consume this module's canonical scene representation. The adapter is
pure Python and deliberately independent from Blender so it can be regression
and tooling tested outside Blender.
"""

from __future__ import annotations

import hashlib
import json
import uuid
from typing import Any, Dict, Iterable, List, Mapping, MutableMapping, Optional, Sequence, Tuple

SIMULATION_SCENE_SCHEMA = "ka.simulation_scene"
SIMULATION_SCENE_VERSION = 1
BODY_ID_PROPERTY = "ka_rigid_stable_id"
CONSTRAINT_ID_PROPERTY = "ka_rigid_constraint_id"
SCENE_ID_PROPERTY = "ka_rigid_scene_id"
_ID_NAMESPACE = uuid.UUID("4f9e5b0a-7272-4ec4-96dc-46fb92d7fc7e")


class SimulationSceneError(ValueError):
    """Raised when a SimulationScene cannot be validated or converted."""


def _valid_uuid(value: object) -> bool:
    try:
        uuid.UUID(str(value))
        return True
    except (TypeError, ValueError, AttributeError):
        return False


def _stored_value(owner: object, key: str) -> Optional[str]:
    try:
        value = owner.get(key)  # Blender ID datablocks and dicts.
    except Exception:
        value = None
    return str(value) if value is not None else None


def _store_value(owner: object, key: str, value: str) -> None:
    try:
        owner[key] = value  # type: ignore[index]
    except Exception as exc:
        raise SimulationSceneError(f"Cannot persist stable ID {key}: {exc}") from exc


def ensure_stable_id(owner: object, key: str, *, seen: Optional[set[str]] = None) -> str:
    """Return a persistent UUID and repair copied/duplicate Blender custom IDs."""
    current = _stored_value(owner, key)
    if current and _valid_uuid(current) and (seen is None or current not in seen):
        stable_id = str(uuid.UUID(current))
    else:
        stable_id = str(uuid.uuid4())
        _store_value(owner, key, stable_id)
    if seen is not None:
        seen.add(stable_id)
    return stable_id


def ensure_scene_body_ids(objects: Iterable[object]) -> Dict[int, str]:
    """Assign unique body IDs while preserving existing IDs whenever possible.

    Blender duplicates custom properties. Sorting by object name makes the repair
    deterministic: the first copy keeps the old ID and later copies receive new
    IDs. The returned mapping uses ``id(obj)`` because Blender objects are not
    guaranteed to be hashable across all supported versions.
    """
    ordered = sorted(
        list(objects),
        key=lambda obj: (
            str(getattr(obj, "name_full", getattr(obj, "name", ""))).casefold(),
            str(getattr(obj, "name_full", getattr(obj, "name", ""))),
        ),
    )
    seen: set[str] = set()
    result: Dict[int, str] = {}
    for obj in ordered:
        result[id(obj)] = ensure_stable_id(obj, BODY_ID_PROPERTY, seen=seen)
    return result


def ensure_scene_constraint_ids(objects: Iterable[object]) -> Dict[int, str]:
    """Assign unique persistent IDs to authored Blender constraint objects."""
    ordered = sorted(
        list(objects),
        key=lambda obj: (
            str(getattr(obj, "name_full", getattr(obj, "name", ""))).casefold(),
            str(getattr(obj, "name_full", getattr(obj, "name", ""))),
        ),
    )
    seen: set[str] = set()
    result: Dict[int, str] = {}
    for obj in ordered:
        result[id(obj)] = ensure_stable_id(obj, CONSTRAINT_ID_PROPERTY, seen=seen)
    return result


def deterministic_child_id(parent_id: str, role: str, index: int, content: object = None) -> str:
    digest = ""
    if content is not None:
        encoded = json.dumps(content, sort_keys=True, separators=(",", ":"), default=str).encode("utf-8")
        digest = hashlib.blake2b(encoded, digest_size=10).hexdigest()
    return str(uuid.uuid5(_ID_NAMESPACE, f"{parent_id}:{role}:{int(index)}:{digest}"))


def _material_key(body: Mapping[str, Any]) -> Tuple[float, float]:
    return (round(float(body.get("friction", 0.5)), 9), round(float(body.get("restitution", 0.0)), 9))


def _material_id(scene_id: str, key: Tuple[float, float]) -> str:
    return str(uuid.uuid5(_ID_NAMESPACE, f"{scene_id}:material:{key[0]:.9g}:{key[1]:.9g}"))


def _collider_geometry(body: Mapping[str, Any], shape: str) -> Dict[str, Any]:
    if shape == "SPHERE":
        return {"radius": float(body.get("radius", 0.5))}
    if shape == "BOX":
        return {"half_extents": list(body.get("half_extents", (0.5, 0.5, 0.5)))}
    if shape == "PLANE":
        return {"infinite": True}
    if shape == "CONVEX_HULL":
        return {
            "vertices": body.get("convex_vertices", []),
            "vertex_count": int(body.get("convex_vertex_count", len(body.get("convex_vertices", [])))),
            "raw_vertex_count": int(body.get("convex_vertex_count_raw", 0)),
        }
    if shape == "COMPOUND_CONVEX":
        parts: List[Dict[str, Any]] = []
        body_id = str(body["stable_id"])
        for index, part in enumerate(body.get("compound_parts", []) or []):
            record = dict(part)
            record["stable_id"] = str(
                part.get("stable_id")
                or deterministic_child_id(body_id, "compound_child", index, part.get("vertices", []))
            )
            parts.append(record)
        return {
            "children": parts,
            "child_count": len(parts),
            "fallback_vertices": body.get("convex_vertices", []),
        }
    if shape == "MESH":
        return {
            "vertices": body.get("mesh_vertices", []),
            "indices": body.get("mesh_indices", []),
            "triangle_count": int(body.get("triangle_count", 0)),
        }
    raise SimulationSceneError(f"Unsupported collider shape in SimulationScene: {shape}")


def legacy_body_to_scene_body(body: Mapping[str, Any], material_id: str) -> Dict[str, Any]:
    stable_id = str(body.get("stable_id") or uuid.uuid4())
    shape = str(body.get("collision_shape", "BOX"))
    collider_id = deterministic_child_id(stable_id, "collider", 0, shape)
    collider = {
        "stable_id": collider_id,
        "shape_type": shape,
        "local_transform": {
            "translation": list(body.get("shape_center", (0.0, 0.0, 0.0))),
            "rotation": [1.0, 0.0, 0.0, 0.0],
        },
        "material_id": material_id,
        "geometry": _collider_geometry(body, shape),
        "quality": body.get("collider_quality", {}),
        "compound_quality": body.get("compound_quality", {}),
        "source_shape_type": str(body.get("source_collision_shape", shape)),
        "fallback": bool(body.get("compound_fallback", False)),
    }
    return {
        "stable_id": stable_id,
        "display_name": str(body.get("name", stable_id)),
        "body_type": str(body.get("body_type", "DYNAMIC")),
        "transform": {
            "translation": list(body.get("location", (0.0, 0.0, 0.0))),
            "rotation": list(body.get("rotation", (1.0, 0.0, 0.0, 0.0))),
            "scale": list(body.get("scale", (1.0, 1.0, 1.0))),
        },
        "mass_properties": {
            "mass": float(body.get("mass", 1.0)),
            "raw_mass": float(body.get("raw_mass", body.get("mass", 1.0))),
            "mode": str(body.get("mass_mode", "MASS")),
            "density": float(body.get("density", 1000.0)),
            "center_of_mass_local": list(body.get("shape_center", (0.0, 0.0, 0.0))),
        },
        "dynamics": {
            "linear_velocity": list(body.get("linear_velocity", (0.0, 0.0, 0.0))),
            "angular_velocity": list(body.get("angular_velocity", (0.0, 0.0, 0.0))),
            "linear_damping": float(body.get("linear_damping", 0.0)),
            "angular_damping": float(body.get("angular_damping", 0.0)),
            "ccd": bool(body.get("ccd", False)),
            "ccd_requested": bool(body.get("ccd_requested", body.get("ccd", False))),
            "ccd_reason": str(body.get("ccd_reason", "manual")),
        },
        "collision_filter": {
            "layer": int(body.get("collision_layer", 1)),
            "mask": int(body.get("collision_mask", 0xFFFF)),
        },
        "colliders": [collider],
        "metadata": {
            "managed_ground": bool(body.get("managed_ground", False)),
            "source_vertex_count": int(body.get("source_vertex_count", 0)),
            "render_source_vertex_count": int(body.get("render_source_vertex_count", 0)),
            "collision_proxy": body.get("collision_proxy"),
            "radius": float(body.get("radius", 0.5)),
            "minimum_feature_length": float(body.get("minimum_feature_length", 0.0) or 0.0),
            "half_extents": list(body.get("half_extents", (0.5, 0.5, 0.5))),
            "stability_adjustments": list(body.get("stability_adjustments", []) or []),
        },
    }


def build_simulation_scene(payload: Mapping[str, Any], *, scene_id: Optional[str] = None) -> Dict[str, Any]:
    scene_id = str(scene_id or payload.get("scene_id") or uuid.uuid4())
    materials: List[Dict[str, Any]] = []
    material_ids: Dict[Tuple[float, float], str] = {}
    scene_bodies: List[Dict[str, Any]] = []
    for source_body in payload.get("bodies", []) or []:
        body = dict(source_body)
        body.setdefault("stable_id", str(uuid.uuid4()))
        key = _material_key(body)
        if key not in material_ids:
            identifier = _material_id(scene_id, key)
            material_ids[key] = identifier
            materials.append({
                "stable_id": identifier,
                "friction": key[0],
                "restitution": key[1],
            })
        scene_bodies.append(legacy_body_to_scene_body(body, material_ids[key]))

    settings_keys = (
        "frame_start", "frame_end", "fps", "gravity", "substeps", "adaptive_substeps",
        "minimum_substeps", "solver_iterations", "sleep_enabled", "sleep_mode",
        "sleep_linear_threshold", "sleep_angular_threshold", "sleep_time",
        "jolt_threads_requested", "jolt_threads", "reproducibility_mode", "deterministic_mode",
        "early_sleep_termination", "early_sleep_frames", "determinism_tolerance",
        "penetration_slop", "backend", "store_python_frames",
    )
    settings = {key: payload[key] for key in settings_keys if key in payload}
    scene = {
        "schema": SIMULATION_SCENE_SCHEMA,
        "schema_version": SIMULATION_SCENE_VERSION,
        "stable_id": scene_id,
        "display_name": str(payload.get("scene_name", "Scene")),
        "coordinate_system": {
            "source": "BLENDER_Z_UP_RIGHT_HANDED",
            "solver_contract": "Y_UP_RIGHT_HANDED",
            "linear_unit": "meter",
            "angular_unit": "radian",
        },
        "settings": settings,
        "materials": materials,
        "bodies": scene_bodies,
        "constraints": list(payload.get("constraints", []) or []),
        "stability": payload.get("stability", {}),
        "runtime": payload.get("runtime", {}),
        "skipped_bodies": payload.get("skipped_bodies", []),
    }
    validate_simulation_scene(scene)
    return scene


def scene_body_to_legacy(body: Mapping[str, Any], materials: Mapping[str, Mapping[str, Any]]) -> Dict[str, Any]:
    colliders = list(body.get("colliders", []) or [])
    if len(colliders) != 1:
        raise SimulationSceneError(
            f"Body {body.get('display_name', body.get('stable_id'))!r} must contain exactly one collider in schema v1"
        )
    collider = colliders[0]
    shape = str(collider.get("shape_type", "BOX"))
    geometry = dict(collider.get("geometry", {}) or {})
    transform = dict(body.get("transform", {}) or {})
    mass = dict(body.get("mass_properties", {}) or {})
    dynamics = dict(body.get("dynamics", {}) or {})
    collision_filter = dict(body.get("collision_filter", {}) or {})
    metadata = dict(body.get("metadata", {}) or {})
    material = materials.get(str(collider.get("material_id")), {})
    result: Dict[str, Any] = {
        "stable_id": str(body["stable_id"]),
        "name": str(body.get("display_name", body["stable_id"])),
        "body_type": str(body.get("body_type", "DYNAMIC")),
        "collision_shape": shape,
        "source_collision_shape": str(collider.get("source_shape_type", shape)),
        "managed_ground": bool(metadata.get("managed_ground", False)),
        "location": list(transform.get("translation", (0.0, 0.0, 0.0))),
        "rotation": list(transform.get("rotation", (1.0, 0.0, 0.0, 0.0))),
        "scale": list(transform.get("scale", (1.0, 1.0, 1.0))),
        "shape_center": list(collider.get("local_transform", {}).get("translation", (0.0, 0.0, 0.0))),
        "half_extents": list(metadata.get("half_extents", geometry.get("half_extents", (0.5, 0.5, 0.5)))),
        "radius": float(metadata.get("radius", geometry.get("radius", 0.5))),
        "minimum_feature_length": float(metadata.get("minimum_feature_length", 0.0) or 0.0),
        "mass": float(mass.get("mass", 1.0)),
        "raw_mass": float(mass.get("raw_mass", mass.get("mass", 1.0))),
        "mass_mode": str(mass.get("mode", "MASS")),
        "density": float(mass.get("density", 1000.0)),
        "friction": float(material.get("friction", 0.5)),
        "restitution": float(material.get("restitution", 0.0)),
        "linear_damping": float(dynamics.get("linear_damping", 0.0)),
        "angular_damping": float(dynamics.get("angular_damping", 0.0)),
        "linear_velocity": list(dynamics.get("linear_velocity", (0.0, 0.0, 0.0))),
        "angular_velocity": list(dynamics.get("angular_velocity", (0.0, 0.0, 0.0))),
        "ccd": bool(dynamics.get("ccd", False)),
        "ccd_requested": bool(dynamics.get("ccd_requested", dynamics.get("ccd", False))),
        "ccd_reason": str(dynamics.get("ccd_reason", "manual")),
        "collision_layer": int(collision_filter.get("layer", 1)),
        "collision_mask": int(collision_filter.get("mask", 0xFFFF)),
        "source_vertex_count": int(metadata.get("source_vertex_count", 0)),
        "render_source_vertex_count": int(metadata.get("render_source_vertex_count", 0)),
        "collision_proxy": metadata.get("collision_proxy"),
        "stability_adjustments": list(metadata.get("stability_adjustments", []) or []),
        "skip_simulation": False,
        "collider_quality": collider.get("quality", {}),
    }
    if shape == "CONVEX_HULL":
        result["convex_vertices"] = geometry.get("vertices", [])
        result["convex_vertex_count"] = int(geometry.get("vertex_count", len(result["convex_vertices"])))
        result["convex_vertex_count_raw"] = int(geometry.get("raw_vertex_count", 0))
    elif shape == "COMPOUND_CONVEX":
        result["convex_vertices"] = geometry.get("fallback_vertices", [])
        result["compound_parts"] = geometry.get("children", [])
        result["compound_part_count"] = int(geometry.get("child_count", len(result["compound_parts"])))
        result["compound_quality"] = collider.get("compound_quality", {})
        result["compound_fallback"] = bool(collider.get("fallback", False))
        result["convex_vertex_count"] = len(result["convex_vertices"])
        result["convex_vertex_count_raw"] = 0
    elif shape == "MESH":
        result["mesh_vertices"] = geometry.get("vertices", [])
        result["mesh_indices"] = geometry.get("indices", [])
        result["triangle_count"] = int(geometry.get("triangle_count", len(result["mesh_indices"]) // 3))
    return result



def apply_single_hull_fallback(
    scene: MutableMapping[str, Any],
    legacy_bodies: Sequence[Mapping[str, Any]],
    body_names: Iterable[str],
    *,
    reason: str = "runtime_side_stick_guard",
) -> int:
    """Replace selected compound colliders with their prepared convex fallback.

    The SimulationScene is the solver source of truth. Mutating only the legacy
    compatibility payload would therefore be discarded by ``solver_payload``.
    This helper updates both representations while preserving body/collider IDs.
    """
    names = {str(name) for name in body_names}
    if not names:
        return 0

    legacy_by_id = {
        str(body.get("stable_id")): body
        for body in legacy_bodies
        if body.get("stable_id") is not None
    }
    legacy_by_name = {str(body.get("name")): body for body in legacy_bodies}
    changed = 0

    for scene_body in scene.get("bodies", []) or []:
        if not isinstance(scene_body, MutableMapping):
            continue
        display_name = str(scene_body.get("display_name", ""))
        stable_id = str(scene_body.get("stable_id", ""))
        if display_name not in names and stable_id not in names:
            continue
        legacy = legacy_by_id.get(stable_id) or legacy_by_name.get(display_name)
        colliders = scene_body.get("colliders", []) or []
        if not colliders or not isinstance(colliders[0], MutableMapping):
            continue
        collider = colliders[0]
        if str(collider.get("shape_type")) not in {"COMPOUND", "COMPOUND_CONVEX"}:
            continue

        old_geometry = dict(collider.get("geometry", {}) or {})
        fallback_vertices = []
        raw_vertex_count = 0
        if legacy is not None:
            fallback_vertices = list(legacy.get("convex_vertices", []) or [])
            raw_vertex_count = int(legacy.get("convex_vertex_count_raw", 0) or 0)
        if not fallback_vertices:
            fallback_vertices = list(old_geometry.get("fallback_vertices", []) or [])
        if len(fallback_vertices) < 4:
            continue

        quality = dict(collider.get("compound_quality", {}) or {})
        quality["accepted"] = False
        quality["fallback_reason"] = str(reason)
        reasons = list(quality.get("fallback_reasons", []) or [])
        if str(reason) not in reasons:
            reasons.append(str(reason))
        quality["fallback_reasons"] = reasons

        collider["shape_type"] = "CONVEX_HULL"
        collider["geometry"] = {
            "vertices": fallback_vertices,
            "vertex_count": len(fallback_vertices),
            "raw_vertex_count": raw_vertex_count,
        }
        collider["compound_quality"] = quality
        collider["fallback"] = True

        metadata = scene_body.get("metadata")
        if not isinstance(metadata, MutableMapping):
            metadata = dict(metadata or {})
            scene_body["metadata"] = metadata
        adjustments = list(metadata.get("stability_adjustments", []) or [])
        marker = f"compound_runtime_single_hull:{reason}"
        if marker not in adjustments:
            adjustments.append(marker)
        metadata["stability_adjustments"] = adjustments
        changed += 1

    return changed

def solver_payload(payload: Mapping[str, Any]) -> Dict[str, Any]:
    """Return the legacy-shaped solver view generated from SimulationScene v1.

    Backends call this at their boundary. This proves that the new schema is the
    source of truth while preserving the mature simulation loop during migration.
    """
    scene = payload.get("simulation_scene")
    if not isinstance(scene, Mapping):
        return dict(payload)
    validate_simulation_scene(scene)
    result = dict(payload)
    settings = dict(scene.get("settings", {}) or {})
    result.update(settings)
    result["scene_name"] = str(scene.get("display_name", result.get("scene_name", "Scene")))
    result["scene_id"] = str(scene["stable_id"])
    result["stability"] = scene.get("stability", result.get("stability", {}))
    result["runtime"] = scene.get("runtime", result.get("runtime", {}))
    result["skipped_bodies"] = scene.get("skipped_bodies", result.get("skipped_bodies", []))
    materials = {str(item["stable_id"]): item for item in scene.get("materials", []) or []}
    result["bodies"] = [scene_body_to_legacy(body, materials) for body in scene.get("bodies", []) or []]
    result["constraints"] = list(scene.get("constraints", []) or [])
    return result


def validate_simulation_scene(scene: Mapping[str, Any]) -> None:
    if scene.get("schema") != SIMULATION_SCENE_SCHEMA:
        raise SimulationSceneError(f"Unsupported SimulationScene schema: {scene.get('schema')!r}")
    if int(scene.get("schema_version", 0)) != SIMULATION_SCENE_VERSION:
        raise SimulationSceneError(f"Unsupported SimulationScene version: {scene.get('schema_version')!r}")
    if not _valid_uuid(scene.get("stable_id")):
        raise SimulationSceneError("SimulationScene stable_id must be a UUID")
    materials = list(scene.get("materials", []) or [])
    material_ids = {str(item.get("stable_id")) for item in materials}
    if len(material_ids) != len(materials) or any(not _valid_uuid(value) for value in material_ids):
        raise SimulationSceneError("SimulationScene material IDs must be unique UUIDs")
    seen_bodies: set[str] = set()
    seen_colliders: set[str] = set()
    for body in scene.get("bodies", []) or []:
        body_id = str(body.get("stable_id"))
        if not _valid_uuid(body_id) or body_id in seen_bodies:
            raise SimulationSceneError(f"Duplicate or invalid body stable_id: {body_id!r}")
        seen_bodies.add(body_id)
        colliders = list(body.get("colliders", []) or [])
        if not colliders:
            raise SimulationSceneError(f"Body {body_id} has no collider")
        for collider in colliders:
            collider_id = str(collider.get("stable_id"))
            if not _valid_uuid(collider_id) or collider_id in seen_colliders:
                raise SimulationSceneError(f"Duplicate or invalid collider stable_id: {collider_id!r}")
            seen_colliders.add(collider_id)
            if str(collider.get("material_id")) not in material_ids:
                raise SimulationSceneError(f"Collider {collider_id} references an unknown material")
    seen_constraints: set[str] = set()
    for constraint in scene.get("constraints", []) or []:
        constraint_id = str(constraint.get("stable_id", ""))
        if not _valid_uuid(constraint_id) or constraint_id in seen_constraints:
            raise SimulationSceneError(f"Duplicate or invalid constraint stable_id: {constraint_id!r}")
        seen_constraints.add(constraint_id)
        body_a = str(constraint.get("body_a", ""))
        body_b = str(constraint.get("body_b", ""))
        if body_a not in seen_bodies or body_b not in seen_bodies or body_a == body_b:
            raise SimulationSceneError(
                f"Constraint {constraint_id} references invalid bodies: {body_a!r}, {body_b!r}"
            )
        constraint_type = str(constraint.get("constraint_type", ""))
        if constraint_type not in {"BREAKABLE_FIXED", "FIXED", "DISTANCE"}:
            raise SimulationSceneError(
                f"Constraint {constraint_id} has unsupported type {constraint_type!r}"
            )
        if constraint_type == "DISTANCE":
            minimum = float(constraint.get("min_distance", 0.0))
            maximum = float(constraint.get("max_distance", 0.0))
            if minimum < 0.0 or maximum <= 0.0 or minimum > maximum:
                raise SimulationSceneError(
                    f"Distance constraint {constraint_id} has invalid limits "
                    f"({minimum}, {maximum})"
                )
        if float(constraint.get("break_force", 0.0)) < 0.0:
            raise SimulationSceneError(f"Constraint {constraint_id} break_force must be non-negative")
        if float(constraint.get("break_torque", 0.0)) < 0.0:
            raise SimulationSceneError(f"Constraint {constraint_id} break_torque must be non-negative")


def canonical_scene_digest(scene: Mapping[str, Any]) -> str:
    validate_simulation_scene(scene)
    encoded = json.dumps(scene, sort_keys=True, separators=(",", ":"), default=str).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
