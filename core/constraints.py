"""Authored non-fracture constraints for KA Rigid Dynamics."""

from __future__ import annotations

from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple

import bpy
from mathutils import Quaternion, Vector

from .simulation_scene import (
    BODY_ID_PROPERTY,
    CONSTRAINT_ID_PROPERTY,
    ensure_scene_constraint_ids,
    ensure_stable_id,
)


def enabled_constraint_objects(scene: bpy.types.Scene) -> List[bpy.types.Object]:
    """Return enabled authored constraints without mutating Blender data.

    This helper is also used by validation and UI code. Blender 5.2 forbids
    writing ID properties while a panel is drawing, so persistent IDs are
    assigned only later in the explicit scene-serialization path.
    """
    objects = [
        obj
        for obj in scene.objects
        if hasattr(obj, "ka_rigid_constraint")
        and bool(obj.ka_rigid_constraint.enabled)
    ]
    return sorted(
        objects,
        key=lambda obj: (
            str(getattr(obj, "name_full", getattr(obj, "name", ""))).casefold(),
            str(getattr(obj, "name_full", getattr(obj, "name", ""))),
        ),
    )


def constraint_count(scene: bpy.types.Scene) -> int:
    """Count enabled constraints using a strictly read-only UI-safe path."""
    return sum(
        1
        for obj in scene.objects
        if hasattr(obj, "ka_rigid_constraint")
        and bool(obj.ka_rigid_constraint.enabled)
    )


def _object_in_scene(scene: bpy.types.Scene, obj: bpy.types.Object | None) -> bool:
    if obj is None:
        return False
    try:
        return scene.objects.get(obj.name) is obj
    except Exception:
        return False


def validate_constraints(
    scene: bpy.types.Scene,
    enabled_bodies: Iterable[bpy.types.Object],
) -> Tuple[List[str], List[str]]:
    """Validate authored constraint references before scene extraction."""
    errors: List[str] = []
    warnings: List[str] = []
    bodies = {id(obj): obj for obj in enabled_bodies}
    constraints = enabled_constraint_objects(scene)

    for obj in constraints:
        settings = obj.ka_rigid_constraint
        body_a = settings.body_a
        body_b = settings.body_b
        label = obj.name_full

        if not _object_in_scene(scene, body_a):
            errors.append(f"{label}: Body A is missing or belongs to another scene.")
            continue
        if not _object_in_scene(scene, body_b):
            errors.append(f"{label}: Body B is missing or belongs to another scene.")
            continue
        if body_a == body_b:
            errors.append(f"{label}: Body A and Body B must be different objects.")
            continue
        if id(body_a) not in bodies or not bool(body_a.ka_rigid_body.enabled):
            errors.append(f"{label}: Body A ({body_a.name_full}) is not an enabled KA body.")
        if id(body_b) not in bodies or not bool(body_b.ka_rigid_body.enabled):
            errors.append(f"{label}: Body B ({body_b.name_full}) is not an enabled KA body.")

        types = {
            str(body_a.ka_rigid_body.body_type),
            str(body_b.ka_rigid_body.body_type),
        }
        if "DYNAMIC" not in types:
            errors.append(f"{label}: a Distance constraint requires at least one Dynamic body.")

        if not bool(settings.use_current_distance) and float(settings.distance) <= 0.0:
            errors.append(f"{label}: fixed constraint length must be greater than zero.")

        if str(settings.constraint_mode) == "ROD":
            warnings.append(
                f"{label}: Rod mode transmits compression as well as tension; use Rope for a chain or cable."
            )

    if len(constraints) > 256:
        errors.append(
            f"{len(constraints)} authored constraints exceed Culverin's 256-constraint world limit."
        )
    return errors, warnings


def _solver_center(body: Mapping[str, Any]) -> Vector:
    location = Vector(tuple(float(value) for value in body.get("location", (0.0, 0.0, 0.0))))
    rotation_values = tuple(float(value) for value in body.get("rotation", (1.0, 0.0, 0.0, 0.0)))
    rotation = Quaternion(rotation_values)
    local_center = Vector(tuple(float(value) for value in body.get("shape_center", (0.0, 0.0, 0.0))))
    return location + rotation @ local_center


def _distance_between(body_a: Mapping[str, Any], body_b: Mapping[str, Any]) -> float:
    return float((_solver_center(body_b) - _solver_center(body_a)).length)


def constraints_for_enabled_bodies(
    scene: bpy.types.Scene,
    bodies: Sequence[Mapping[str, Any]],
) -> List[Dict[str, Any]]:
    """Serialize authored Distance constraints using native body-center anchors.

    Culverin 0.13.2 exposes a stable center-to-center Distance constraint. A
    separately positioned Static anchor body therefore gives the user an exact
    world-space suspension point without requiring unsupported local pivots.
    """
    by_id = {str(body.get("stable_id", "")): body for body in bodies}
    result: List[Dict[str, Any]] = []
    constraint_objects = enabled_constraint_objects(scene)
    constraint_ids = ensure_scene_constraint_ids(constraint_objects)

    for obj in constraint_objects:
        settings = obj.ka_rigid_constraint
        body_a_obj = settings.body_a
        body_b_obj = settings.body_b
        if body_a_obj is None or body_b_obj is None:
            continue

        body_a_id = ensure_stable_id(body_a_obj, BODY_ID_PROPERTY)
        body_b_id = ensure_stable_id(body_b_obj, BODY_ID_PROPERTY)
        body_a = by_id.get(body_a_id)
        body_b = by_id.get(body_b_id)
        if body_a is None or body_b is None:
            missing = body_a_obj.name_full if body_a is None else body_b_obj.name_full
            raise ValueError(
                f"Constraint {obj.name_full} references {missing}, but that body is not present in the solver payload."
            )

        current_distance = max(1.0e-5, _distance_between(body_a, body_b))
        maximum = current_distance if bool(settings.use_current_distance) else max(1.0e-5, float(settings.distance))
        mode = str(settings.constraint_mode)
        minimum = maximum if mode == "ROD" else 0.0

        result.append({
            "stable_id": constraint_ids[id(obj)],
            "display_name": obj.name_full,
            "constraint_type": "DISTANCE",
            "distance_mode": mode,
            "body_a": body_a_id,
            "body_b": body_b_id,
            "body_a_name": body_a_obj.name_full,
            "body_b_name": body_b_obj.name_full,
            "min_distance": float(minimum),
            "max_distance": float(maximum),
            "authored_distance": float(settings.distance),
            "use_current_distance": bool(settings.use_current_distance),
            "current_distance": float(current_distance),
            "enabled": True,
            "anchor_model": "BODY_CENTER_TO_BODY_CENTER",
        })

    result.sort(key=lambda item: str(item.get("stable_id", "")))
    return result
