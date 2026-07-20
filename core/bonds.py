"""Breakable bond authoring helpers for KA Rigid Dynamics.

The module stores solver-neutral bond records on the scene and creates a
proximity graph from Blender mesh vertices. Runtime interpretation remains in
individual physics backends.
"""

from __future__ import annotations

import json
import math
import uuid
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple

from mathutils import Vector
from mathutils.kdtree import KDTree

from .simulation_scene import ensure_scene_body_ids

BOND_SCHEMA = "ka.breakable_bond"
BOND_VERSION = 1
_BOND_NAMESPACE = uuid.UUID("24c63583-2c9d-4c96-8318-ebf16ee29884")


@dataclass
class _GeometryRecord:
    obj: Any
    stable_id: str
    name: str
    body_type: str
    vertices: List[Vector]
    samples: List[Vector]
    tree: KDTree
    minimum: Vector
    maximum: Vector
    center: Vector


def load_bonds(scene: Any) -> List[Dict[str, Any]]:
    """Return valid persisted bond dictionaries for a Blender scene."""
    world = getattr(scene, "ka_rigid_world", None)
    raw = str(getattr(world, "bond_data", "") or "") if world is not None else ""
    if not raw:
        return []
    try:
        decoded = json.loads(raw)
    except (TypeError, ValueError, json.JSONDecodeError):
        return []
    if not isinstance(decoded, list):
        return []
    return [dict(item) for item in decoded if isinstance(item, Mapping)]


def save_bonds(scene: Any, bonds: Sequence[Mapping[str, Any]]) -> None:
    """Persist a deterministic compact bond list on the world settings."""
    world = scene.ka_rigid_world
    ordered = sorted(
        (dict(item) for item in bonds),
        key=lambda item: str(item.get("stable_id", "")),
    )
    world.bond_data = json.dumps(ordered, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    world.bond_count = len(ordered)


def clear_bonds(scene: Any) -> int:
    count = len(load_bonds(scene))
    save_bonds(scene, [])
    return count


def bonds_for_enabled_bodies(scene: Any, body_ids: Iterable[str]) -> List[Dict[str, Any]]:
    """Filter persisted bonds to bodies that are part of the current payload."""
    enabled = {str(value) for value in body_ids}
    result: List[Dict[str, Any]] = []
    for bond in load_bonds(scene):
        body_a = str(bond.get("body_a", ""))
        body_b = str(bond.get("body_b", ""))
        if body_a in enabled and body_b in enabled and body_a != body_b:
            result.append(bond)
    return result


def _sample_vertices(vertices: Sequence[Vector], maximum: int = 1024) -> List[Vector]:
    count = len(vertices)
    if count <= maximum:
        return list(vertices)
    if maximum <= 1:
        return [vertices[0]]
    scale = float(count - 1) / float(maximum - 1)
    return [vertices[min(count - 1, int(round(index * scale)))] for index in range(maximum)]


def _mesh_vertices_world(obj: Any, depsgraph: Any) -> List[Vector]:
    evaluated = obj.evaluated_get(depsgraph)
    mesh = evaluated.to_mesh(preserve_all_data_layers=False, depsgraph=depsgraph)
    try:
        matrix = evaluated.matrix_world
        return [matrix @ vertex.co for vertex in mesh.vertices]
    finally:
        evaluated.to_mesh_clear()


def _geometry_record(obj: Any, stable_id: str, depsgraph: Any) -> _GeometryRecord | None:
    vertices = _mesh_vertices_world(obj, depsgraph)
    if not vertices:
        return None
    minimum = Vector((
        min(vertex.x for vertex in vertices),
        min(vertex.y for vertex in vertices),
        min(vertex.z for vertex in vertices),
    ))
    maximum = Vector((
        max(vertex.x for vertex in vertices),
        max(vertex.y for vertex in vertices),
        max(vertex.z for vertex in vertices),
    ))
    tree = KDTree(len(vertices))
    for index, vertex in enumerate(vertices):
        tree.insert(vertex, index)
    tree.balance()
    return _GeometryRecord(
        obj=obj,
        stable_id=str(stable_id),
        name=str(obj.name_full),
        body_type=str(getattr(obj.ka_rigid_body, "body_type", "DYNAMIC")).upper(),
        vertices=vertices,
        samples=_sample_vertices(vertices),
        tree=tree,
        minimum=minimum,
        maximum=maximum,
        center=(minimum + maximum) * 0.5,
    )


def _axis_gap(first_min: float, first_max: float, second_min: float, second_max: float) -> float:
    if first_max < second_min:
        return second_min - first_max
    if second_max < first_min:
        return first_min - second_max
    return 0.0


def _aabb_distance(first: _GeometryRecord, second: _GeometryRecord) -> float:
    dx = _axis_gap(first.minimum.x, first.maximum.x, second.minimum.x, second.maximum.x)
    dy = _axis_gap(first.minimum.y, first.maximum.y, second.minimum.y, second.maximum.y)
    dz = _axis_gap(first.minimum.z, first.maximum.z, second.minimum.z, second.maximum.z)
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _surface_support_area(points: Sequence[Vector]) -> float:
    """Return a conservative non-collinearity area for nearby surface samples."""
    if len(points) < 3:
        return 0.0
    origin = points[0]
    first = max(points[1:], key=lambda point: (point - origin).length_squared)
    axis = first - origin
    if axis.length_squared <= 1.0e-20:
        return 0.0
    maximum_cross = 0.0
    for point in points[1:]:
        maximum_cross = max(maximum_cross, axis.cross(point - origin).length)
    return 0.5 * maximum_cross


def _nearest_points(
    first: _GeometryRecord,
    second: _GeometryRecord,
    distance_limit: float,
) -> Tuple[float, Vector, Vector, int, float]:
    best_distance = float("inf")
    best_first = first.center
    best_second = second.center
    close_groups: List[List[Vector]] = [[], []]
    for group_index, (source, target) in enumerate(((first, second), (second, first))):
        for point in source.samples:
            nearest, _index, distance = target.tree.find(point)
            if nearest is None:
                continue
            numeric_distance = float(distance)
            if numeric_distance <= distance_limit:
                close_groups[group_index].append(point.copy())
            if numeric_distance < best_distance:
                best_distance = numeric_distance
                if source is first:
                    best_first, best_second = point, nearest
                else:
                    best_first, best_second = nearest, point
    close_count = max((len(group) for group in close_groups), default=0)
    support_area = max((_surface_support_area(group) for group in close_groups), default=0.0)
    return best_distance, best_first, best_second, close_count, support_area


def _overlap(first_min: float, first_max: float, second_min: float, second_max: float) -> float:
    return max(0.0, min(first_max, second_max) - max(first_min, second_min))


def _estimated_contact_area(first: _GeometryRecord, second: _GeometryRecord, normal: Vector) -> float:
    axis = max(range(3), key=lambda index: abs(float(normal[index])))
    dimensions = [index for index in range(3) if index != axis]
    overlap_a = _overlap(first.minimum[dimensions[0]], first.maximum[dimensions[0]], second.minimum[dimensions[0]], second.maximum[dimensions[0]])
    overlap_b = _overlap(first.minimum[dimensions[1]], first.maximum[dimensions[1]], second.minimum[dimensions[1]], second.maximum[dimensions[1]])
    area = overlap_a * overlap_b
    if area > 1.0e-12:
        return float(area)
    first_size = first.maximum - first.minimum
    second_size = second.maximum - second.minimum
    fallback = min(
        max(1.0e-8, float(first_size[dimensions[0]] * first_size[dimensions[1]])),
        max(1.0e-8, float(second_size[dimensions[0]] * second_size[dimensions[1]])),
    )
    return float(fallback * 0.05)


def generate_proximity_bonds(
    objects: Sequence[Any],
    depsgraph: Any,
    *,
    maximum_distance: float,
    break_force: float,
    break_torque: float,
    damage_accumulation: float = 0.0,
) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    """Create deterministic breakable Fixed bonds between nearby mesh objects.

    Candidate pairs are culled with AABBs. The final connection test uses
    nearest world-space mesh vertices in both directions, which is particularly
    reliable for complementary fractured surfaces.
    """
    candidates = [
        obj for obj in objects
        if getattr(obj, "type", None) == "MESH"
        and hasattr(obj, "ka_rigid_body")
        and bool(obj.ka_rigid_body.enabled)
    ]
    id_map = ensure_scene_body_ids(candidates)
    records: List[_GeometryRecord] = []
    skipped: List[str] = []
    for obj in sorted(candidates, key=lambda item: (item.name_full.casefold(), item.name_full)):
        try:
            record = _geometry_record(obj, id_map[id(obj)], depsgraph)
        except Exception:
            record = None
        if record is None:
            skipped.append(str(obj.name_full))
        else:
            records.append(record)
    records.sort(key=lambda item: (float(item.minimum.x), item.name.casefold(), item.name))

    distance_limit = max(0.0, float(maximum_distance))
    bonds: List[Dict[str, Any]] = []
    tested_pairs = 0
    candidate_pairs = 0
    rejected_point_or_edge_contacts = 0
    static_static_pairs_skipped = 0
    dynamic_dynamic_bonds = 0
    dynamic_static_bonds = 0
    for first_index, first in enumerate(records):
        for second in records[first_index + 1:]:
            if float(second.minimum.x) > float(first.maximum.x) + distance_limit:
                break
            tested_pairs += 1
            if _aabb_distance(first, second) > distance_limit:
                continue
            candidate_pairs += 1
            first_dynamic = first.body_type == "DYNAMIC"
            second_dynamic = second.body_type == "DYNAMIC"
            if not first_dynamic and not second_dynamic:
                static_static_pairs_skipped += 1
                continue
            distance, point_a, point_b, close_count, support_area = _nearest_points(
                first, second, distance_limit
            )
            if distance > distance_limit:
                continue
            direction = second.center - first.center
            if direction.length_squared <= 1.0e-16:
                direction = point_b - point_a
            normal = direction.normalized() if direction.length_squared > 1.0e-16 else Vector((0.0, 0.0, 1.0))
            estimated_area = _estimated_contact_area(first, second, normal)
            minimum_support_area = max(1.0e-12, distance_limit * distance_limit * 0.01)
            mixed_dynamic_static = first_dynamic != second_dynamic
            # Explicit/static anchors frequently touch a plane or a simple support
            # along only one or two sampled vertices. Accept that mixed pair when
            # the overlapping projected AABBs still provide a finite support area.
            if close_count < 3 or support_area < minimum_support_area:
                if not mixed_dynamic_static or estimated_area < minimum_support_area:
                    rejected_point_or_edge_contacts += 1
                    continue
            anchor = (point_a + point_b) * 0.5
            body_a, body_b = sorted((first.stable_id, second.stable_id))
            stable_id = str(uuid.uuid5(_BOND_NAMESPACE, f"{body_a}:{body_b}"))
            bonds.append({
                "schema": BOND_SCHEMA,
                "schema_version": BOND_VERSION,
                "stable_id": stable_id,
                "constraint_type": "BREAKABLE_FIXED",
                "body_a": first.stable_id,
                "body_b": second.stable_id,
                "body_a_name": first.name,
                "body_b_name": second.name,
                "body_a_type": first.body_type,
                "body_b_type": second.body_type,
                "anchor": [float(value) for value in anchor],
                "normal": [float(value) for value in normal],
                "area": max(support_area, estimated_area),
                "rest_distance": float(distance),
                "break_force": max(0.0, float(break_force)),
                "break_torque": max(0.0, float(break_torque)),
                "damage_accumulation": max(0.0, float(damage_accumulation)),
                "damage": 0.0,
                "status": "INTACT",
                "enabled": True,
                "source": "PROXIMITY",
            })
            if mixed_dynamic_static:
                dynamic_static_bonds += 1
            else:
                dynamic_dynamic_bonds += 1

    bonds.sort(key=lambda item: str(item["stable_id"]))
    return bonds, {
        "object_count": len(records),
        "skipped_objects": skipped,
        "tested_pairs": tested_pairs,
        "candidate_pairs": candidate_pairs,
        "rejected_point_or_edge_contacts": rejected_point_or_edge_contacts,
        "static_static_pairs_skipped": static_static_pairs_skipped,
        "dynamic_dynamic_bonds": dynamic_dynamic_bonds,
        "dynamic_static_bonds": dynamic_static_bonds,
        "bond_count": len(bonds),
        "maximum_distance": distance_limit,
    }
