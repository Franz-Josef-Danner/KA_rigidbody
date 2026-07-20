"""Self-contained Jolt regression tests that do not modify the open Blender scene."""

from __future__ import annotations

import copy
import json
import math
import os
import time
import tempfile
import uuid
from typing import Any, Dict, List

from ..backends.jolt import JoltBackend, recommended_jolt_threads
from .determinism import compare_frames, frames_digest
from .coordinates import quat_rotate_vector_wxyz
from .cache import decode_direct_frame_block, write_cache, read_cache
from .mass_conditioning import condition_dynamic_mass_ratios
from .coacd_bridge import coacd_status, decompose as coacd_decompose
from .stability_defaults import (
    LOW_FRICTION_CONTACT_DEFAULT,
    LEGACY_BODY_FRICTION_DEFAULT,
    PENETRATION_SLOP_DEFAULT,
)
from .simulation_scene import (
    SIMULATION_SCENE_SCHEMA,
    SIMULATION_SCENE_VERSION,
    apply_single_hull_fallback,
    build_simulation_scene,
    canonical_scene_digest,
    solver_payload,
    validate_simulation_scene,
)

REGRESSION_FILENAME = "ka_rigid_regression.json"


def _body(
    name: str,
    body_type: str,
    shape: str,
    location,
    *,
    half_extents=(0.5, 0.5, 0.5),
    radius=0.5,
    mass=1.0,
    friction=0.5,
    restitution=0.0,
    velocity=(0.0, 0.0, 0.0),
    ccd=False,
) -> Dict[str, Any]:
    result = {
        "name": name,
        "body_type": body_type,
        "collision_shape": shape,
        "location": list(location),
        "rotation": [1.0, 0.0, 0.0, 0.0],
        "scale": [1.0, 1.0, 1.0],
        "shape_center": [0.0, 0.0, 0.0],
        "half_extents": list(half_extents),
        "radius": float(radius),
        "mass": float(mass),
        "friction": float(friction),
        "restitution": float(restitution),
        "linear_damping": 0.04,
        "angular_damping": 0.1,
        "linear_velocity": list(velocity),
        "angular_velocity": [0.0, 0.0, 0.0],
        "ccd": bool(ccd),
        "collision_layer": 1,
        "collision_mask": 0xFFFF,
        "stability_adjustments": [],
    }
    if shape == "CONVEX_HULL":
        hx, hy, hz = half_extents
        result["convex_vertices"] = [
            [x, y, z]
            for x in (-hx, hx)
            for y in (-hy, hy)
            for z in (-hz, hz)
        ]
    elif shape == "COMPOUND":
        result["compound_parts"] = [
            {"center": [-0.35, 0.0, 0.0], "half_extents": [0.35, 0.25, 0.25]},
            {"center": [0.35, 0.0, 0.0], "half_extents": [0.20, 0.20, 0.20]},
        ]
        result["convex_vertices"] = [
            [-0.70, -0.25, -0.25], [0.55, -0.25, -0.25],
            [-0.70, 0.25, -0.25], [-0.70, -0.25, 0.25], [0.55, 0.25, 0.25],
        ]
    elif shape == "COMPOUND_CONVEX":
        parts = []
        for center, half in (((-0.36, 0.0, 0.0), (0.34, 0.25, 0.25)), ((0.30, 0.0, 0.0), (0.25, 0.20, 0.20))):
            cx, cy, cz = center
            hx, hy, hz = half
            vertices = [
                [cx + x, cy + y, cz + z]
                for x in (-hx, hx)
                for y in (-hy, hy)
                for z in (-hz, hz)
            ]
            parts.append({
                "center": list(center),
                "vertices": vertices,
                "volume": float(8.0 * hx * hy * hz),
                "radius": float(math.sqrt(hx * hx + hy * hy + hz * hz)),
            })
        result["compound_parts"] = parts
        result["compound_part_count"] = len(parts)
        result["convex_vertices"] = [point for part in parts for point in part["vertices"]]
    return result


def _payload(name: str, bodies: List[Dict[str, Any]], *, frames=120, gravity=(0.0, 0.0, -9.81), substeps=6) -> Dict[str, Any]:
    return {
        "scene_name": f"KA Regression {name}",
        "signature": f"regression-{name}",
        "frame_start": 1,
        "frame_end": int(frames),
        "fps": 60.0,
        "gravity": list(gravity),
        "substeps": int(substeps),
        "adaptive_substeps": True,
        "minimum_substeps": min(4, int(substeps)),
        "solver_iterations": 8,
        "sleep_enabled": True,
        "sleep_mode": "NATIVE",
        "sleep_linear_threshold": 0.05,
        "sleep_angular_threshold": 0.25,
        "sleep_time": 0.5,
        "jolt_threads": 0,
        "jolt_threads_requested": 0,
        "reproducibility_mode": "REPEATABLE",
        "deterministic_mode": True,
        "early_sleep_termination": True,
        "early_sleep_frames": 3,
        "penetration_slop": 0.002,
        # Regression fixtures inspect frame dictionaries directly. Normal Blender
        # scene payloads never enable this internal testing override.
        "store_python_frames": True,
        "diagnostics": {"enabled": False, "contacts": False, "payload": False},
        "bodies": bodies,
    }


def _final_location(result: Dict[str, Any], name: str):
    frame = result["frames"][str(result["frame_end"])]
    return frame[name]["location"]




def _run_simulation_scene_roundtrip(_backend: JoltBackend) -> Dict[str, Any]:
    bodies = [
        _body("NeutralGround", "STATIC", "PLANE", (0.0, 0.0, 0.0)),
        _body("NeutralCompound", "DYNAMIC", "COMPOUND_CONVEX", (0.0, 0.0, 1.0)),
    ]
    bodies[0]["stable_id"] = "dd6ab746-3de8-5d4e-b808-8fd84783f146"
    bodies[1]["stable_id"] = "1df585ef-bf70-51dd-a890-51f81272bbd9"
    payload = _payload("simulation-scene", bodies, frames=12, substeps=2)
    payload["scene_id"] = "b23023cf-f7bd-5cab-bad5-91a65cdf15a2"
    scene = build_simulation_scene(payload, scene_id=payload["scene_id"])
    validate_simulation_scene(scene)
    payload["simulation_scene"] = scene
    restored = solver_payload(payload)
    compound = restored["bodies"][1]
    child_ids = [part.get("stable_id") for part in compound.get("compound_parts", [])]
    passed = (
        scene.get("schema") == SIMULATION_SCENE_SCHEMA
        and int(scene.get("schema_version", 0)) == SIMULATION_SCENE_VERSION
        and len(scene.get("bodies", [])) == 2
        and len(scene.get("materials", [])) == 1
        and compound.get("collision_shape") == "COMPOUND_CONVEX"
        and len(child_ids) == 2
        and len(set(child_ids)) == 2
        and all(child_ids)
        and canonical_scene_digest(scene) == canonical_scene_digest(copy.deepcopy(scene))
    )
    return {
        "name": "SimulationScene v1 roundtrip",
        "passed": bool(passed),
        "metrics": {
            "schema": scene.get("schema"),
            "schema_version": scene.get("schema_version"),
            "body_count": len(scene.get("bodies", [])),
            "material_count": len(scene.get("materials", [])),
            "compound_child_ids": child_ids,
        },
    }



def _run_simulation_scene_runtime_fallback(_backend: JoltBackend) -> Dict[str, Any]:
    body = _body("FallbackCompound", "DYNAMIC", "COMPOUND_CONVEX", (0.0, 0.0, 1.0))
    body["stable_id"] = "1df585ef-bf70-51dd-a890-51f81272bbd9"
    payload = _payload("simulation-scene-fallback", [body], frames=4, substeps=2)
    scene = build_simulation_scene(payload, scene_id="b23023cf-f7bd-5cab-bad5-91a65cdf15a2")
    original_collider_id = scene["bodies"][0]["colliders"][0]["stable_id"]
    payload["simulation_scene"] = scene
    changed = apply_single_hull_fallback(
        scene, payload["bodies"], ["FallbackCompound"], reason="regression_initial_overlap"
    )
    restored = solver_payload(payload)
    restored_body = restored["bodies"][0]
    collider = scene["bodies"][0]["colliders"][0]
    passed = (
        changed == 1
        and collider.get("shape_type") == "CONVEX_HULL"
        and collider.get("stable_id") == original_collider_id
        and bool(collider.get("fallback"))
        and restored_body.get("collision_shape") == "CONVEX_HULL"
        and len(restored_body.get("convex_vertices", [])) >= 4
        and collider.get("compound_quality", {}).get("fallback_reason") == "regression_initial_overlap"
    )
    return {
        "name": "SimulationScene runtime fallback",
        "passed": bool(passed),
        "metrics": {
            "changed": changed,
            "shape": restored_body.get("collision_shape"),
            "collider_id_preserved": collider.get("stable_id") == original_collider_id,
            "vertex_count": len(restored_body.get("convex_vertices", [])),
        },
    }

def _run_simulation_scene_identity(_backend: JoltBackend) -> Dict[str, Any]:
    stable_id = "1df585ef-bf70-51dd-a890-51f81272bbd9"
    scene_id = "b23023cf-f7bd-5cab-bad5-91a65cdf15a2"
    first_body = _body("BeforeRename", "DYNAMIC", "BOX", (0.0, 0.0, 1.0))
    first_body["stable_id"] = stable_id
    second_body = copy.deepcopy(first_body)
    second_body["name"] = "AfterRename"
    first = build_simulation_scene(_payload("identity-a", [first_body]), scene_id=scene_id)
    second = build_simulation_scene(_payload("identity-b", [second_body]), scene_id=scene_id)
    first_record = first["bodies"][0]
    second_record = second["bodies"][0]
    passed = (
        first_record["stable_id"] == second_record["stable_id"] == stable_id
        and first_record["colliders"][0]["stable_id"] == second_record["colliders"][0]["stable_id"]
        and first_record["display_name"] != second_record["display_name"]
    )
    return {
        "name": "Stable body and collider identity",
        "passed": bool(passed),
        "metrics": {
            "body_id": first_record["stable_id"],
            "collider_id": first_record["colliders"][0]["stable_id"],
            "renamed_to": second_record["display_name"],
        },
    }


def _run_coacd_decomposition(_backend: JoltBackend) -> Dict[str, Any]:
    polygon = [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (1.0, 1.0), (1.0, 2.0), (0.0, 2.0)]
    vertices = [[x, y, z] for z in (0.0, 0.5) for x, y in polygon]
    bottom = [(0, 3, 1), (1, 3, 2), (0, 5, 3), (3, 5, 4)]
    top = [(a + 6, b + 6, c + 6) for a, b, c in ((0, 1, 3), (1, 2, 3), (0, 3, 5), (3, 4, 5))]
    sides = []
    for index in range(6):
        nxt = (index + 1) % 6
        sides.extend(((index, nxt, nxt + 6), (index, nxt + 6, index + 6)))
    indices = [value for triangle in (bottom + top + sides) for value in triangle]
    available, detail = coacd_status()
    if not available:
        return {"name": "Safe compound decomposition", "passed": False, "error": detail, "metrics": {}}
    parts = coacd_decompose(vertices, indices, {
        "threshold": 0.01,
        "execution_mode": "SAFE_SPATIAL",
        "max_parts": 4,
        "preprocess_mode": "AUTO",
        "preprocess_resolution": 30,
        "resolution": 1000,
        "mcts_nodes": 10,
        "mcts_iterations": 80,
        "mcts_max_depth": 3,
        "merge": True,
        "decimate": False,
        "max_hull_vertices": 64,
        "seed": 0,
    })
    part_count = len(parts)
    vertex_counts = [len(part.get("vertices", [])) for part in parts]
    proxy_volume = 0.0
    for part in parts:
        points = list(part.get("vertices", []))
        xs = [float(point[0]) for point in points]
        ys = [float(point[1]) for point in points]
        zs = [float(point[2]) for point in points]
        if xs and ys and zs:
            proxy_volume += (max(xs) - min(xs)) * (max(ys) - min(ys)) * (max(zs) - min(zs))
    source_volume = 1.5
    volume_ratio = proxy_volume / source_volume
    passed = (
        2 <= part_count <= 4
        and all(count == 8 for count in vertex_counts)
        and 0.0 < volume_ratio <= 1.02
    )
    return {
        "name": "Safe compound decomposition",
        "passed": passed,
        "metrics": {
            "part_count": part_count,
            "vertex_counts": vertex_counts,
            "proxy_volume": proxy_volume,
            "source_volume": source_volume,
            "proxy_volume_ratio": volume_ratio,
            "status": detail,
        },
    }

def _run_drop(backend: JoltBackend) -> Dict[str, Any]:
    payload = _payload("drop", [
        _body("Ground", "STATIC", "PLANE", (0.0, 0.0, 0.0), half_extents=(10.0, 10.0, 1.0e-5), friction=0.8),
        _body("DropSphere", "DYNAMIC", "SPHERE", (0.0, 0.0, 2.0), radius=0.25, mass=1.0, friction=0.5),
    ], frames=120)
    result = backend.bake(payload)
    z = float(_final_location(result, "DropSphere")[2])
    sleeping = int(result.get("final_state", {}).get("sleeping_bodies", 0))
    passed = 0.20 <= z <= 0.35 and sleeping == 1
    return {"name": "Drop and settle", "passed": passed, "metrics": {"final_z": z, "sleeping": sleeping}}


def _run_restitution(backend: JoltBackend) -> Dict[str, Any]:
    payload = _payload("restitution", [
        _body("Ground", "STATIC", "PLANE", (0.0, 0.0, 0.0), half_extents=(10.0, 10.0, 1.0e-5), restitution=0.8),
        _body("BounceSphere", "DYNAMIC", "SPHERE", (0.0, 0.0, 2.0), radius=0.2, mass=1.0, restitution=0.8),
    ], frames=150, substeps=8)
    result = backend.bake(payload)
    samples = [float(result["frames"][str(frame)]["BounceSphere"]["location"][2]) for frame in range(1, 151)]
    impact_index = min(range(1, min(90, len(samples))), key=lambda index: samples[index])
    rebound = max(samples[impact_index + 1:] or [samples[impact_index]])
    passed = rebound > 0.65
    return {"name": "Restitution rebound", "passed": passed, "metrics": {"impact_z": samples[impact_index], "rebound_z": rebound}}


def _run_stack(backend: JoltBackend) -> Dict[str, Any]:
    bodies = [_body("Ground", "STATIC", "PLANE", (0.0, 0.0, 0.0), half_extents=(10.0, 10.0, 1.0e-5), friction=0.9)]
    for index in range(5):
        bodies.append(_body(f"Box_{index}", "DYNAMIC", "BOX", (0.0, 0.0, 0.251 + index * 0.502), half_extents=(0.25, 0.25, 0.25), mass=1.0, friction=0.8))
    result = backend.bake(_payload("stack", bodies, frames=180, substeps=8))
    final = result["frames"][str(result["frame_end"])]
    zs = [float(final[f"Box_{index}"]["location"][2]) for index in range(5)]
    ordered = all(zs[index] < zs[index + 1] for index in range(4))
    sleeping = int(result.get("final_state", {}).get("sleeping_bodies", 0))
    passed = ordered and sleeping == 5 and max(abs(zs[index] - (0.25 + index * 0.5)) for index in range(5)) < 0.18
    return {"name": "Stack stability", "passed": passed, "metrics": {"final_z": zs, "sleeping": sleeping}}


def _run_friction(backend: JoltBackend) -> Dict[str, Any]:
    bodies = [
        _body("Ground", "STATIC", "PLANE", (0.0, 0.0, 0.0), half_extents=(20.0, 20.0, 1.0e-5), friction=0.8),
        _body("LowFriction", "DYNAMIC", "BOX", (-2.0, -1.0, 0.26), half_extents=(0.25, 0.25, 0.25), friction=0.02, velocity=(3.0, 0.0, 0.0)),
        _body("HighFriction", "DYNAMIC", "BOX", (-2.0, 1.0, 0.26), half_extents=(0.25, 0.25, 0.25), friction=1.0, velocity=(3.0, 0.0, 0.0)),
    ]
    result = backend.bake(_payload("friction", bodies, frames=150))
    low_x = float(_final_location(result, "LowFriction")[0])
    high_x = float(_final_location(result, "HighFriction")[0])
    passed = low_x > high_x + 0.2
    return {"name": "Friction separation", "passed": passed, "metrics": {"low_friction_x": low_x, "high_friction_x": high_x}}


def _run_low_friction_antistick_contact(backend: JoltBackend) -> Dict[str, Any]:
    """Verify the new fracture material can slide out of a sustained side contact."""
    bodies = [
        _body("Wall", "STATIC", "BOX", (0.0, 0.0, 1.5), half_extents=(0.1, 3.0, 3.0), friction=0.8),
        _body(
            "AntiStick", "DYNAMIC", "BOX", (0.31, -0.5, 2.5),
            half_extents=(0.2, 0.2, 0.2), friction=LOW_FRICTION_CONTACT_DEFAULT,
        ),
        _body(
            "LegacyFriction", "DYNAMIC", "BOX", (0.31, 0.5, 2.5),
            half_extents=(0.2, 0.2, 0.2), friction=LEGACY_BODY_FRICTION_DEFAULT,
        ),
    ]
    payload = _payload(
        "fracture-antistick-contact", bodies, frames=45, gravity=(-6.0, 0.0, -9.81), substeps=8
    )
    payload["penetration_slop"] = PENETRATION_SLOP_DEFAULT
    result = backend.bake(payload)
    anti_stick_z = float(_final_location(result, "AntiStick")[2])
    legacy_z = float(_final_location(result, "LegacyFriction")[2])
    release_distance = legacy_z - anti_stick_z
    passed = release_distance > 0.20
    return {
        "name": "Low-friction anti-stick contact",
        "passed": passed,
        "metrics": {
            "anti_stick_friction": LOW_FRICTION_CONTACT_DEFAULT,
            "legacy_friction": LEGACY_BODY_FRICTION_DEFAULT,
            "penetration_slop": PENETRATION_SLOP_DEFAULT,
            "anti_stick_final_z": anti_stick_z,
            "legacy_final_z": legacy_z,
            "release_distance": release_distance,
        },
    }


def _run_rigid_bond_island(backend: JoltBackend) -> Dict[str, Any]:
    bodies = [
        _body("BondA", "DYNAMIC", "BOX", (-1.0, 0.0, 2.0), velocity=(3.0, 0.0, 0.0)),
        _body("BondB", "DYNAMIC", "BOX", (0.0, 0.0, 2.0)),
        _body("BondC", "DYNAMIC", "BOX", (1.0, 0.0, 2.0)),
    ]
    body_ids = [str(uuid.uuid5(uuid.NAMESPACE_DNS, f"ka-rigid-bond-{index}")) for index in range(3)]
    for body, stable_id in zip(bodies, body_ids):
        body["stable_id"] = stable_id
        body["linear_damping"] = 0.0
        body["angular_damping"] = 0.0
    payload = _payload("rigid-bond-island", bodies, frames=30, gravity=(0.0, 0.0, 0.0), substeps=6)
    payload["sleep_enabled"] = False
    payload["stability"] = {"bond_stability_mode": "RIGID"}
    payload["constraints"] = [
        {
            "stable_id": str(uuid.uuid5(uuid.NAMESPACE_DNS, f"ka-rigid-bond-edge-{index}")),
            "constraint_type": "BREAKABLE_FIXED",
            "body_a": body_ids[index],
            "body_b": body_ids[index + 1],
            "body_a_name": bodies[index]["name"],
            "body_b_name": bodies[index + 1]["name"],
            "anchor": [float(index) - 0.5, 0.0, 2.0],
            "normal": [1.0, 0.0, 0.0],
            "area": 1.0,
            "break_force": 1.0e12,
            "break_torque": 1.0e12,
            "damage_accumulation": 0.0,
            "damage": 0.0,
            "enabled": True,
        }
        for index in range(2)
    ]
    result = backend.bake(payload)
    maximum_distance_error = 0.0
    maximum_rotation_error = 0.0
    for snapshot in result["frames"].values():
        locations = [snapshot[name]["location"] for name in ("BondA", "BondB", "BondC")]
        rotations = [snapshot[name]["rotation"] for name in ("BondA", "BondB", "BondC")]
        maximum_distance_error = max(
            maximum_distance_error,
            abs(math.dist(locations[0], locations[1]) - 1.0),
            abs(math.dist(locations[1], locations[2]) - 1.0),
        )
        for rotation in rotations[1:]:
            dot = abs(sum(float(a) * float(b) for a, b in zip(rotations[0], rotation)))
            maximum_rotation_error = max(maximum_rotation_error, 1.0 - min(1.0, dot))
    totals = result.get("diagnostic_totals", {})
    passed = (
        maximum_distance_error <= 1.0e-5
        and maximum_rotation_error <= 1.0e-6
        and int(totals.get("bond_graph_count", 0)) == 2
        and int(totals.get("bond_constraint_count", -1)) == 0
        and int(totals.get("bond_cluster_count", 0)) == 1
        and int(totals.get("bond_clustered_bodies", 0)) == 3
        and int(totals.get("native_dynamic_body_count", 0)) == 1
        and bool(totals.get("bond_rigid_stabilization"))
        and totals.get("bond_stabilization_strategy") == "RIGID_COMPOUND_ISLANDS_WITH_STATIC_ANCHORS"
        and int(totals.get("bond_projection_passes", -1)) == 0
    )
    return {
        "name": "Rigid bond island",
        "passed": bool(passed),
        "metrics": {
            "maximum_distance_error": maximum_distance_error,
            "maximum_rotation_error": maximum_rotation_error,
            "bond_graph_count": totals.get("bond_graph_count"),
            "bond_constraint_count": totals.get("bond_constraint_count"),
            "bond_cluster_count": totals.get("bond_cluster_count"),
            "bond_clustered_bodies": totals.get("bond_clustered_bodies"),
            "native_dynamic_body_count": totals.get("native_dynamic_body_count"),
            "strategy": totals.get("bond_stabilization_strategy"),
            "projection_passes": totals.get("bond_projection_passes"),
        },
    }


def _run_rigid_static_anchor(backend: JoltBackend) -> Dict[str, Any]:
    """Rigid Dynamic-Static bonds must mechanically anchor and still break."""

    def run_case(anchor_force: float) -> Dict[str, Any]:
        bodies = [
            _body("StaticAnchor", "STATIC", "BOX", (-1.0, 0.0, 2.0), half_extents=(0.5, 0.5, 0.5), mass=0.0),
            _body("AnchorA", "DYNAMIC", "BOX", (0.0, 0.0, 2.0), half_extents=(0.5, 0.5, 0.5), mass=2.0),
            _body("AnchorB", "DYNAMIC", "BOX", (1.0, 0.0, 2.0), half_extents=(0.5, 0.5, 0.5), mass=2.0),
            _body("AnchorProjectile", "DYNAMIC", "SPHERE", (4.0, 0.0, 2.0), radius=0.35, mass=20.0, velocity=(-15.0, 0.0, 0.0), ccd=True),
        ]
        ids = {
            body["name"]: str(uuid.uuid5(uuid.NAMESPACE_DNS, f"ka-rigid-static-anchor-{anchor_force}-{body['name']}"))
            for body in bodies
        }
        for body in bodies:
            body["stable_id"] = ids[body["name"]]
            body["linear_damping"] = 0.0
            body["angular_damping"] = 0.0
        payload = _payload(
            f"rigid-static-anchor-{anchor_force}", bodies, frames=120, gravity=(0.0, 0.0, 0.0), substeps=10
        )
        payload["sleep_enabled"] = False
        payload["stability"] = {"bond_stability_mode": "RIGID"}
        payload["constraints"] = [
            {
                "stable_id": str(uuid.uuid5(uuid.NAMESPACE_DNS, f"ka-rigid-static-anchor-internal-{anchor_force}")),
                "constraint_type": "BREAKABLE_FIXED",
                "body_a": ids["AnchorA"],
                "body_b": ids["AnchorB"],
                "body_a_name": "AnchorA",
                "body_b_name": "AnchorB",
                "anchor": [0.5, 0.0, 2.0],
                "normal": [1.0, 0.0, 0.0],
                "area": 1.0,
                "break_force": 1.0e12,
                "break_torque": 1.0e12,
                "damage_accumulation": 0.0,
                "damage": 0.0,
                "enabled": True,
            },
            {
                "stable_id": str(uuid.uuid5(uuid.NAMESPACE_DNS, f"ka-rigid-static-anchor-edge-{anchor_force}")),
                "constraint_type": "BREAKABLE_FIXED",
                "body_a": ids["StaticAnchor"],
                "body_b": ids["AnchorA"],
                "body_a_name": "StaticAnchor",
                "body_b_name": "AnchorA",
                "anchor": [-0.5, 0.0, 2.0],
                "normal": [1.0, 0.0, 0.0],
                "area": 1.0,
                "break_force": float(anchor_force),
                "break_torque": 1.0e12,
                "damage_accumulation": 0.0,
                "damage": 0.0,
                "enabled": True,
            },
        ]
        return backend.bake(payload)

    strong = run_case(1.0e12)
    weak = run_case(100.0)
    strong_totals = strong.get("diagnostic_totals", {})
    weak_totals = weak.get("diagnostic_totals", {})
    strong_a = strong["frames"][str(strong["frame_end"])]["AnchorA"]["location"]
    strong_b = strong["frames"][str(strong["frame_end"])]["AnchorB"]["location"]
    weak_a = weak["frames"][str(weak["frame_end"])]["AnchorA"]["location"]
    strong_displacement = max(math.dist(strong_a, (0.0, 0.0, 2.0)), math.dist(strong_b, (1.0, 0.0, 2.0)))
    weak_displacement = math.dist(weak_a, (0.0, 0.0, 2.0))
    weak_anchor_events = [
        event for event in weak.get("bond_events", [])
        if {event.get("body_a"), event.get("body_b")} == {"StaticAnchor", "AnchorA"}
    ]
    passed = (
        int(strong_totals.get("bond_static_anchor_bonds", 0)) == 1
        and int(strong_totals.get("bond_static_anchor_constraints", 0)) == 1
        and int(strong_totals.get("bond_static_anchor_omitted", -1)) == 0
        and int(strong_totals.get("bond_constraint_count", 0)) == 1
        and strong_totals.get("bond_stabilization_strategy") == "RIGID_COMPOUND_ISLANDS_WITH_STATIC_ANCHORS"
        and int(strong_totals.get("bond_break_events", -1)) == 0
        and strong_displacement <= 1.0e-4
        and int(weak_totals.get("bond_break_events", 0)) >= 1
        and weak_displacement >= 0.05
        and bool(weak_anchor_events)
        and bool(weak_anchor_events[0].get("solver_constraint"))
    )
    return {
        "name": "Rigid Dynamic-Static anchor",
        "passed": bool(passed),
        "metrics": {
            "strong_anchor_constraints": strong_totals.get("bond_static_anchor_constraints"),
            "strong_displacement": strong_displacement,
            "strong_break_events": strong_totals.get("bond_break_events"),
            "weak_displacement": weak_displacement,
            "weak_break_events": weak_totals.get("bond_break_events"),
            "weak_anchor_event_solver_bound": bool(weak_anchor_events and weak_anchor_events[0].get("solver_constraint")),
            "strategy": strong_totals.get("bond_stabilization_strategy"),
        },
    }



def _run_component_mass_conditioning(_backend: JoltBackend) -> Dict[str, Any]:
    """A heavy unbonded impactor must not inflate a bonded target's masses."""
    bodies = [
        {"name": "TargetHeavy", "stable_id": "target-heavy", "body_type": "DYNAMIC", "mass": 5.0},
        {"name": "TargetTiny", "stable_id": "target-tiny", "body_type": "DYNAMIC", "mass": 0.0001},
        {"name": "HeavyProjectile", "stable_id": "projectile", "body_type": "DYNAMIC", "mass": 1.0e6},
    ]
    constraints = [{
        "constraint_type": "BREAKABLE_FIXED",
        "body_a": "target-heavy",
        "body_b": "target-tiny",
        "enabled": True,
    }]
    summary = condition_dynamic_mass_ratios(
        bodies, constraints, enabled=True, limit=5000.0, absolute_floor=0.001,
    )
    by_name = {str(body["name"]): body for body in bodies}
    passed = (
        summary.get("mode") == "BONDED_COMPONENTS"
        and math.isclose(float(by_name["TargetHeavy"]["mass"]), 5.0, abs_tol=1.0e-12)
        and math.isclose(float(by_name["TargetTiny"]["mass"]), 0.001, abs_tol=1.0e-12)
        and math.isclose(float(by_name["HeavyProjectile"]["mass"]), 1.0e6, abs_tol=1.0e-6)
        and int(summary.get("adjusted_bodies", 0)) == 1
        and int(summary.get("component_count", 0)) == 2
        and float(summary.get("max_component_ratio_after", 0.0)) <= 5000.0 + 1.0e-6
    )
    return {
        "name": "Bond-component mass conditioning",
        "passed": bool(passed),
        "metrics": {
            "mode": summary.get("mode"),
            "target_heavy_mass": by_name["TargetHeavy"]["mass"],
            "target_tiny_mass": by_name["TargetTiny"]["mass"],
            "projectile_mass": by_name["HeavyProjectile"]["mass"],
            "adjusted_bodies": summary.get("adjusted_bodies"),
            "component_count": summary.get("component_count"),
            "max_component_ratio_after": summary.get("max_component_ratio_after"),
            "global_ratio_after": summary.get("ratio_after"),
        },
    }


def _run_mass_aware_dense_anchor_release(backend: JoltBackend) -> Dict[str, Any]:
    """A heavy projectile must release a densely anchored rigid bond island.

    The legacy raw contact scalar divided the impact over eight local and eight
    static bonds, leaving every 550 N bond intact and letting the island spring
    back. Pre-step momentum must break the overloaded graph instead.
    """
    bodies = [
        _body("DenseStatic", "STATIC", "BOX", (-1.5, 0.0, 2.0), half_extents=(0.5, 2.0, 2.0), mass=0.0),
        _body("DenseCore", "DYNAMIC", "BOX", (0.0, 0.0, 2.0), half_extents=(0.45, 0.45, 0.45), mass=5.0),
    ]
    for index in range(8):
        angle = 2.0 * math.pi * index / 8.0
        bodies.append(_body(
            f"DenseArm{index}", "DYNAMIC", "BOX",
            (-0.5, 0.75 * math.cos(angle), 2.0 + 0.75 * math.sin(angle)),
            half_extents=(0.35, 0.35, 0.35), mass=5.0,
        ))
    bodies.append(_body(
        "DenseProjectile", "DYNAMIC", "SPHERE", (5.0, 0.0, 2.0),
        radius=0.5, mass=10000.0, velocity=(-12.0, 0.0, 0.0), ccd=True,
    ))
    ids = {}
    for body in bodies:
        body["stable_id"] = str(uuid.uuid5(uuid.NAMESPACE_DNS, f"ka-dense-release-{body['name']}"))
        body["linear_damping"] = 0.0
        body["angular_damping"] = 0.0
        ids[body["name"]] = body["stable_id"]

    constraints = []
    for index in range(8):
        angle = 2.0 * math.pi * index / 8.0
        arm = f"DenseArm{index}"
        constraints.extend([
            {
                "stable_id": str(uuid.uuid5(uuid.NAMESPACE_DNS, f"ka-dense-release-edge-{index}")),
                "constraint_type": "BREAKABLE_FIXED",
                "body_a": ids["DenseCore"],
                "body_b": ids[arm],
                "body_a_name": "DenseCore",
                "body_b_name": arm,
                "anchor": [-0.25, 0.375 * math.cos(angle), 2.0 + 0.375 * math.sin(angle)],
                "normal": [1.0, 0.0, 0.0],
                "area": 1.0,
                "break_force": 550.0,
                "break_torque": 550.0,
                "damage_accumulation": 0.0,
                "damage": 0.0,
                "enabled": True,
            },
            {
                "stable_id": str(uuid.uuid5(uuid.NAMESPACE_DNS, f"ka-dense-release-anchor-{index}")),
                "constraint_type": "BREAKABLE_FIXED",
                "body_a": ids["DenseStatic"],
                "body_b": ids[arm],
                "body_a_name": "DenseStatic",
                "body_b_name": arm,
                "anchor": [-0.75, 0.75 * math.cos(angle), 2.0 + 0.75 * math.sin(angle)],
                "normal": [1.0, 0.0, 0.0],
                "area": 1.0,
                "break_force": 550.0,
                "break_torque": 550.0,
                "damage_accumulation": 0.0,
                "damage": 0.0,
                "enabled": True,
            },
        ])

    payload = _payload(
        "mass-aware-dense-anchor-release", bodies, frames=50,
        gravity=(0.0, 0.0, 0.0), substeps=10,
    )
    payload["sleep_enabled"] = False
    payload["early_sleep_termination"] = False
    payload["stability"] = {"bond_stability_mode": "RIGID"}
    payload["constraints"] = constraints
    result = backend.bake(payload)
    totals = result.get("diagnostic_totals", {})
    events = list(result.get("bond_events", []))
    first_core = result["frames"]["1"]["DenseCore"]["location"]
    final_core = result["frames"][str(result["frame_end"])]["DenseCore"]["location"]
    final_displacement = math.dist(first_core, final_core)
    maximum_displacement = max(
        math.dist(first_core, frame["DenseCore"]["location"])
        for frame in result["frames"].values()
    )
    maximum_effective_impulse = max((float(event.get("effective_impulse", 0.0)) for event in events), default=0.0)
    maximum_pre_step_speed = max((float(event.get("pre_step_relative_normal_speed", 0.0)) for event in events), default=0.0)
    passed = (
        int(totals.get("bond_break_events", 0)) >= 8
        and int(totals.get("broken_bonds_final", 0)) >= 8
        and final_displacement >= 0.2
        and maximum_displacement >= final_displacement
        and maximum_effective_impulse >= 100.0
        and maximum_pre_step_speed >= 5.0
        and result.get("bond_force_model") == "MASS_AWARE_CONTACT_MOMENTUM_V3"
    )
    return {
        "name": "Mass-aware dense anchor release",
        "passed": bool(passed),
        "metrics": {
            "break_events": totals.get("bond_break_events"),
            "broken_bonds_final": totals.get("broken_bonds_final"),
            "final_displacement": final_displacement,
            "maximum_displacement": maximum_displacement,
            "maximum_effective_impulse": maximum_effective_impulse,
            "maximum_pre_step_speed": maximum_pre_step_speed,
            "force_model": result.get("bond_force_model"),
        },
    }

def _run_rigid_static_anchor_authored_pose(backend: JoltBackend) -> Dict[str, Any]:
    """Static-anchor contacts must not depenetrate an authored rigid island.

    The one-body outer hull spans the gap between the two dynamic members and
    therefore encloses the static support. Without pair filtering Jolt resolves
    that overlap on frame 2 even though Fixed anchors already define the exact
    authored relationship.
    """
    bodies = [
        _body(
            "AuthoredStatic", "STATIC", "BOX", (0.0, 0.0, 1.0),
            half_extents=(0.4, 0.4, 0.4), mass=0.0, friction=0.5,
        ),
        _body(
            "AuthoredLeft", "DYNAMIC", "BOX", (-0.8, 0.0, 1.0),
            half_extents=(0.4, 0.4, 0.4), mass=2.0, friction=0.5,
        ),
        _body(
            "AuthoredRight", "DYNAMIC", "BOX", (0.8, 0.0, 1.0),
            half_extents=(0.4, 0.4, 0.4), mass=2.0, friction=0.5,
        ),
    ]
    ids = {
        body["name"]: str(uuid.uuid5(uuid.NAMESPACE_DNS, f"ka-rigid-authored-static-{body['name']}"))
        for body in bodies
    }
    for body in bodies:
        body["stable_id"] = ids[body["name"]]
        body["linear_damping"] = 0.0
        body["angular_damping"] = 0.0
    payload = _payload(
        "rigid-static-anchor-authored-pose", bodies, frames=5,
        gravity=(0.0, 0.0, 0.0), substeps=12,
    )
    payload["sleep_enabled"] = False
    payload["early_sleep_termination"] = False
    payload["stability"] = {"bond_stability_mode": "RIGID"}
    payload["constraints"] = [
        {
            "stable_id": str(uuid.uuid5(uuid.NAMESPACE_DNS, "ka-rigid-authored-static-internal")),
            "constraint_type": "BREAKABLE_FIXED",
            "body_a": ids["AuthoredLeft"],
            "body_b": ids["AuthoredRight"],
            "body_a_name": "AuthoredLeft",
            "body_b_name": "AuthoredRight",
            "anchor": [0.0, 0.0, 1.0],
            "normal": [1.0, 0.0, 0.0],
            "area": 1.0,
            "break_force": 1.0e12,
            "break_torque": 1.0e12,
            "damage_accumulation": 0.0,
            "damage": 0.0,
            "enabled": True,
        },
        {
            "stable_id": str(uuid.uuid5(uuid.NAMESPACE_DNS, "ka-rigid-authored-static-left-anchor")),
            "constraint_type": "BREAKABLE_FIXED",
            "body_a": ids["AuthoredStatic"],
            "body_b": ids["AuthoredLeft"],
            "body_a_name": "AuthoredStatic",
            "body_b_name": "AuthoredLeft",
            "anchor": [-0.4, 0.0, 1.0],
            "normal": [1.0, 0.0, 0.0],
            "area": 1.0,
            "break_force": 1.0e12,
            "break_torque": 1.0e12,
            "damage_accumulation": 0.0,
            "damage": 0.0,
            "enabled": True,
        },
        {
            "stable_id": str(uuid.uuid5(uuid.NAMESPACE_DNS, "ka-rigid-authored-static-right-anchor")),
            "constraint_type": "BREAKABLE_FIXED",
            "body_a": ids["AuthoredStatic"],
            "body_b": ids["AuthoredRight"],
            "body_a_name": "AuthoredStatic",
            "body_b_name": "AuthoredRight",
            "anchor": [0.4, 0.0, 1.0],
            "normal": [-1.0, 0.0, 0.0],
            "area": 1.0,
            "break_force": 1.0e12,
            "break_torque": 1.0e12,
            "damage_accumulation": 0.0,
            "damage": 0.0,
            "enabled": True,
        },
    ]
    result = backend.bake(payload)
    first = result["frames"]["1"]
    maximum_displacement = 0.0
    maximum_rotation_error = 0.0
    for frame in result["frames"].values():
        for name in ("AuthoredLeft", "AuthoredRight"):
            maximum_displacement = max(
                maximum_displacement,
                math.dist(first[name]["location"], frame[name]["location"]),
            )
            dot = abs(sum(
                float(a) * float(b)
                for a, b in zip(first[name]["rotation"], frame[name]["rotation"])
            ))
            maximum_rotation_error = max(
                maximum_rotation_error, 1.0 - min(1.0, dot)
            )
    totals = result.get("diagnostic_totals", {})
    passed = (
        maximum_displacement <= 1.0e-6
        and maximum_rotation_error <= 1.0e-7
        and int(totals.get("bond_static_anchor_constraints", 0)) == 2
        and int(totals.get("bond_static_anchor_collision_filter_pairs", 0)) == 1
        and int(totals.get("bond_static_anchor_collision_filter_dynamic_actors", 0)) == 1
        and int(totals.get("bond_static_anchor_collision_filter_overflow", -1)) == 0
    )
    return {
        "name": "Rigid Static anchor authored pose",
        "passed": bool(passed),
        "metrics": {
            "maximum_displacement": maximum_displacement,
            "maximum_rotation_error": maximum_rotation_error,
            "anchor_constraints": totals.get("bond_static_anchor_constraints"),
            "filtered_pairs": totals.get("bond_static_anchor_collision_filter_pairs"),
            "filtered_dynamic_actors": totals.get(
                "bond_static_anchor_collision_filter_dynamic_actors"
            ),
            "filter_overflow": totals.get("bond_static_anchor_collision_filter_overflow"),
        },
    }



def _run_rigid_static_anchor_neighbor_rest(backend: JoltBackend) -> Dict[str, Any]:
    """An anchored outer hull must ignore authored overlap with an unbonded static neighbour.

    Culverin merges the two dynamic members into one outer convex hull. That
    hull intersects NeighborStatic even though only AnchorStatic has an authored
    bond. The authored pose must remain exact until the static anchor breaks.
    """
    bodies = [
        _body(
            "NeighborAnchorStatic", "STATIC", "BOX", (-1.2, 0.0, 1.0),
            half_extents=(0.4, 0.4, 0.4), mass=0.0, friction=0.5,
        ),
        _body(
            "NeighborStatic", "STATIC", "BOX", (0.0, 0.0, 1.0),
            half_extents=(0.4, 0.4, 0.4), mass=0.0, friction=0.5,
        ),
        _body(
            "NeighborDynamicA", "DYNAMIC", "BOX", (-0.4, 0.0, 1.0),
            half_extents=(0.4, 0.4, 0.4), mass=2.0, friction=0.5,
        ),
        _body(
            "NeighborDynamicB", "DYNAMIC", "BOX", (0.8, 0.0, 1.0),
            half_extents=(0.4, 0.4, 0.4), mass=2.0, friction=0.5,
        ),
    ]
    ids = {
        body["name"]: str(uuid.uuid5(uuid.NAMESPACE_DNS, f"ka-rigid-neighbor-static-{body['name']}"))
        for body in bodies
    }
    for body in bodies:
        body["stable_id"] = ids[body["name"]]
        body["linear_damping"] = 0.0
        body["angular_damping"] = 0.0
    payload = _payload(
        "rigid-static-anchor-neighbor-rest", bodies, frames=5,
        gravity=(0.0, 0.0, 0.0), substeps=12,
    )
    payload["sleep_enabled"] = False
    payload["early_sleep_termination"] = False
    payload["stability"] = {"bond_stability_mode": "RIGID"}
    payload["constraints"] = [
        {
            "stable_id": str(uuid.uuid5(uuid.NAMESPACE_DNS, "ka-rigid-neighbor-static-internal")),
            "constraint_type": "BREAKABLE_FIXED",
            "body_a": ids["NeighborDynamicA"],
            "body_b": ids["NeighborDynamicB"],
            "body_a_name": "NeighborDynamicA",
            "body_b_name": "NeighborDynamicB",
            "anchor": [0.2, 0.0, 1.0],
            "normal": [1.0, 0.0, 0.0],
            "area": 1.0,
            "break_force": 1.0e12,
            "break_torque": 1.0e12,
            "damage_accumulation": 0.0,
            "damage": 0.0,
            "enabled": True,
        },
        {
            "stable_id": str(uuid.uuid5(uuid.NAMESPACE_DNS, "ka-rigid-neighbor-static-anchor")),
            "constraint_type": "BREAKABLE_FIXED",
            "body_a": ids["NeighborAnchorStatic"],
            "body_b": ids["NeighborDynamicA"],
            "body_a_name": "NeighborAnchorStatic",
            "body_b_name": "NeighborDynamicA",
            "anchor": [-0.8, 0.0, 1.0],
            "normal": [1.0, 0.0, 0.0],
            "area": 1.0,
            "break_force": 1.0e12,
            "break_torque": 1.0e12,
            "damage_accumulation": 0.0,
            "damage": 0.0,
            "enabled": True,
        },
    ]
    result = backend.bake(payload)
    first = result["frames"]["1"]
    maximum_displacement = 0.0
    maximum_rotation_error = 0.0
    for frame in result["frames"].values():
        for name in ("NeighborDynamicA", "NeighborDynamicB"):
            maximum_displacement = max(
                maximum_displacement,
                math.dist(first[name]["location"], frame[name]["location"]),
            )
            dot = abs(sum(
                float(a) * float(b)
                for a, b in zip(first[name]["rotation"], frame[name]["rotation"])
            ))
            maximum_rotation_error = max(
                maximum_rotation_error, 1.0 - min(1.0, dot)
            )
    totals = result.get("diagnostic_totals", {})
    passed = (
        maximum_displacement <= 1.0e-6
        and maximum_rotation_error <= 1.0e-7
        and int(totals.get("bond_static_anchor_constraints", 0)) == 1
        and int(totals.get("bond_static_anchor_initial_overlap_pairs", 0)) >= 1
        and int(totals.get("bond_static_anchor_collision_filter_pairs", 0)) >= 2
        and int(totals.get("bond_static_anchor_collision_filter_overflow", -1)) == 0
    )
    return {
        "name": "Rigid Static anchor neighbouring support rest",
        "passed": bool(passed),
        "metrics": {
            "maximum_displacement": maximum_displacement,
            "maximum_rotation_error": maximum_rotation_error,
            "anchor_constraints": totals.get("bond_static_anchor_constraints"),
            "initial_overlap_pairs": totals.get("bond_static_anchor_initial_overlap_pairs"),
            "filtered_pairs": totals.get("bond_static_anchor_collision_filter_pairs"),
            "filter_overflow": totals.get("bond_static_anchor_collision_filter_overflow"),
        },
    }


def _run_rigid_bond_collision_filter(backend: JoltBackend) -> Dict[str, Any]:
    """Intact bond members must not generate internal contact impulses."""
    bodies = [
        _body("FilterA", "DYNAMIC", "BOX", (-0.45, 0.0, 1.0), half_extents=(0.5, 0.5, 0.5), mass=5.0),
        _body("FilterB", "DYNAMIC", "BOX", (0.35, 0.0, 1.0), half_extents=(0.5, 0.5, 0.5), mass=1.0),
        _body("FilterC", "DYNAMIC", "BOX", (1.15, 0.0, 1.0), half_extents=(0.5, 0.5, 0.5), mass=0.5),
    ]
    body_ids = [str(uuid.uuid5(uuid.NAMESPACE_DNS, f"ka-rigid-filter-{index}")) for index in range(3)]
    for body, stable_id in zip(bodies, body_ids):
        body["stable_id"] = stable_id
        body["linear_damping"] = 0.0
        body["angular_damping"] = 0.0
    payload = _payload("rigid-bond-collision-filter", bodies, frames=10, gravity=(0.0, 0.0, 0.0), substeps=4)
    payload["sleep_enabled"] = False
    payload["diagnostics"] = {"enabled": True, "contacts": True, "payload": False}
    payload["stability"] = {"bond_stability_mode": "RIGID"}
    payload["constraints"] = [
        {
            "stable_id": str(uuid.uuid5(uuid.NAMESPACE_DNS, f"ka-rigid-filter-edge-{index}")),
            "constraint_type": "BREAKABLE_FIXED",
            "body_a": body_ids[index],
            "body_b": body_ids[index + 1],
            "body_a_name": bodies[index]["name"],
            "body_b_name": bodies[index + 1]["name"],
            "anchor": [float(index) * 0.8 - 0.05, 0.0, 1.0],
            "normal": [1.0, 0.0, 0.0],
            "area": 1.0,
            "break_force": 1.0e12,
            "break_torque": 1.0e12,
            "damage_accumulation": 0.0,
            "damage": 0.0,
            "enabled": True,
        }
        for index in range(2)
    ]
    result = backend.bake(payload)
    totals = result.get("diagnostic_totals", {})
    maximum_displacement = 0.0
    first = result["frames"]["1"]
    final = result["frames"][str(result["frame_end"])]
    for body in bodies:
        name = body["name"]
        maximum_displacement = max(
            maximum_displacement,
            math.dist(first[name]["location"], final[name]["location"]),
        )
    passed = (
        int(totals.get("contact_events", -1)) == 0
        and not bool(totals.get("bond_internal_collision_filtering"))
        and int(totals.get("bond_cluster_count", 0)) == 1
        and int(totals.get("bond_clustered_bodies", 0)) == 3
        and int(totals.get("native_dynamic_body_count", 0)) == 1
        and totals.get("bond_stabilization_strategy") == "RIGID_COMPOUND_ISLANDS_WITH_STATIC_ANCHORS"
        and maximum_displacement <= 1.0e-6
    )
    return {
        "name": "Rigid bond internal collision filter",
        "passed": bool(passed),
        "metrics": {
            "contact_events": totals.get("contact_events"),
            "cluster_count": totals.get("bond_cluster_count"),
            "clustered_bodies": totals.get("bond_clustered_bodies"),
            "native_dynamic_body_count": totals.get("native_dynamic_body_count"),
            "strategy": totals.get("bond_stabilization_strategy"),
            "maximum_displacement": maximum_displacement,
        },
    }



def _run_rigid_bond_island_sleep(backend: JoltBackend) -> Dict[str, Any]:
    """A supported intact island must settle as one unit without projection drift."""
    bodies = [
        _body("IslandGround", "STATIC", "PLANE", (0.0, 0.0, 0.0), half_extents=(10.0, 10.0, 1.0e-5), friction=0.7),
        _body("IslandA", "DYNAMIC", "BOX", (-0.5, 0.0, 0.5), half_extents=(0.5, 0.5, 0.5), mass=1.0, friction=0.7),
        _body("IslandB", "DYNAMIC", "BOX", (0.5, 0.0, 0.5), half_extents=(0.5, 0.5, 0.5), mass=1.0, friction=0.7),
    ]
    body_ids = [str(uuid.uuid5(uuid.NAMESPACE_DNS, f"ka-rigid-island-sleep-{index}")) for index in range(2)]
    for body, stable_id in zip(bodies[1:], body_ids):
        body["stable_id"] = stable_id
        body["linear_damping"] = 0.1
        body["angular_damping"] = 0.1
    payload = _payload("rigid-bond-island-sleep", bodies, frames=60, substeps=6)
    payload["early_sleep_termination"] = False
    payload["sleep_time"] = 0.25
    payload["stability"] = {"bond_stability_mode": "RIGID"}
    payload["constraints"] = [{
        "stable_id": str(uuid.uuid5(uuid.NAMESPACE_DNS, "ka-rigid-island-sleep-edge")),
        "constraint_type": "BREAKABLE_FIXED",
        "body_a": body_ids[0],
        "body_b": body_ids[1],
        "body_a_name": "IslandA",
        "body_b_name": "IslandB",
        "anchor": [0.0, 0.0, 0.5],
        "normal": [1.0, 0.0, 0.0],
        "area": 1.0,
        "break_force": 1.0e12,
        "break_torque": 1.0e12,
        "damage_accumulation": 0.0,
        "damage": 0.0,
        "enabled": True,
    }]
    result = backend.bake(payload)
    totals = result.get("diagnostic_totals", {})
    first = result["frames"]["1"]
    final = result["frames"][str(result["frame_end"])]
    maximum_displacement = max(
        math.dist(first[name]["location"], final[name]["location"])
        for name in ("IslandA", "IslandB")
    )
    passed = (
        int(totals.get("bond_projection_passes", -1)) == 0
        and float(totals.get("bond_projection_max_correction", -1.0)) == 0.0
        and int(result.get("final_state", {}).get("sleeping_bodies", 0)) == 2
        and float(totals.get("final_motion_energy_proxy", 1.0)) == 0.0
        and maximum_displacement <= 1.0e-5
        and int(totals.get("bond_cluster_count", 0)) == 1
        and int(totals.get("native_dynamic_body_count", 0)) == 1
        and totals.get("bond_stabilization_strategy") == "RIGID_COMPOUND_ISLANDS_WITH_STATIC_ANCHORS"
    )
    return {
        "name": "Rigid bond island coordinated sleep",
        "passed": bool(passed),
        "metrics": {
            "projection_passes": totals.get("bond_projection_passes"),
            "projection_max_correction": totals.get("bond_projection_max_correction"),
            "sleeping_bodies": result.get("final_state", {}).get("sleeping_bodies"),
            "cluster_count": totals.get("bond_cluster_count"),
            "native_dynamic_body_count": totals.get("native_dynamic_body_count"),
            "final_motion_energy_proxy": totals.get("final_motion_energy_proxy"),
            "maximum_displacement": maximum_displacement,
            "strategy": totals.get("bond_stabilization_strategy"),
        },
    }

def _run_rigid_hull_fallback_ground_contact(backend: JoltBackend) -> Dict[str, Any]:
    """A complete outer hull may touch ground but must not depenetrate upward."""
    ground = _body(
        "HullGround", "STATIC", "PLANE", (0.0, 0.0, 0.0),
        half_extents=(10.0, 10.0, 1.0e-5), friction=0.7,
    )
    hull = _body(
        "HullFallback", "DYNAMIC", "CONVEX_HULL", (0.0, 0.0, 0.0),
        half_extents=(0.5, 0.5, 0.45), mass=2.0, friction=0.7,
    )
    hull["stable_id"] = str(uuid.uuid5(uuid.NAMESPACE_DNS, "ka-rigid-hull-fallback"))
    hull["shape_center"] = [0.0, 0.0, 0.28]
    # The true hull is entirely above the plane, while the source-mesh bounds
    # deliberately extend below it. Version 0.7.4 used those stale bounds as a
    # cluster child and depenetrated the complete island by roughly 15 cm.
    hull["convex_vertices"] = [
        [-0.45, -0.25, 0.03], [0.42, -0.28, 0.04],
        [-0.38, 0.30, 0.02], [0.40, 0.27, 0.05],
        [-0.30, -0.22, 0.48], [0.33, -0.20, 0.52],
        [-0.27, 0.24, 0.50], [0.29, 0.22, 0.49],
        [0.0, 0.0, 0.62],
    ]
    top = _body(
        "HullTop", "DYNAMIC", "BOX", (0.0, 0.0, 0.85),
        half_extents=(0.15, 0.15, 0.15), mass=1.0, friction=0.7,
    )
    top["stable_id"] = str(uuid.uuid5(uuid.NAMESPACE_DNS, "ka-rigid-hull-top"))
    payload = _payload(
        "rigid-hull-fallback-ground", [ground, hull, top],
        frames=8, gravity=(0.0, 0.0, 0.0), substeps=4,
    )
    payload["sleep_enabled"] = False
    payload["early_sleep_termination"] = False
    payload["diagnostics"] = {"enabled": True, "contacts": True, "payload": False}
    payload["stability"] = {"bond_stability_mode": "RIGID"}
    payload["constraints"] = [{
        "stable_id": str(uuid.uuid5(uuid.NAMESPACE_DNS, "ka-rigid-hull-fallback-edge")),
        "constraint_type": "BREAKABLE_FIXED",
        "body_a": hull["stable_id"],
        "body_b": top["stable_id"],
        "body_a_name": hull["name"],
        "body_b_name": top["name"],
        "anchor": [0.0, 0.0, 0.6],
        "normal": [0.0, 0.0, 1.0],
        "area": 1.0,
        "break_force": 1.0e12,
        "break_torque": 1.0e12,
        "damage_accumulation": 0.0,
        "damage": 0.0,
        "enabled": True,
    }]
    result = backend.bake(payload)
    first_z = float(result["frames"]["1"][hull["name"]]["location"][2])
    maximum_upward_shift = max(
        float(snapshot[hull["name"]]["location"][2]) - first_z
        for snapshot in result["frames"].values()
    )
    totals = result.get("diagnostic_totals", {})
    passed = (
        maximum_upward_shift <= 1.0e-4
        and int(totals.get("contact_events", 0)) > 0
        and float(totals.get("max_contact_impulse", 0.0)) >= 0.0
        and int(totals.get("bond_cluster_count", 0)) == 1
        and int(totals.get("native_dynamic_body_count", 0)) == 1
        and totals.get("bond_stabilization_strategy") == "RIGID_COMPOUND_ISLANDS_WITH_STATIC_ANCHORS"
    )
    return {
        "name": "Rigid hull fallback ground contact",
        "passed": bool(passed),
        "metrics": {
            "maximum_upward_shift": maximum_upward_shift,
            "contact_events": totals.get("contact_events"),
            "cluster_count": totals.get("bond_cluster_count"),
            "native_dynamic_body_count": totals.get("native_dynamic_body_count"),
            "strategy": totals.get("bond_stabilization_strategy"),
        },
    }



def _run_rigid_authored_ground_rest(backend: JoltBackend) -> Dict[str, Any]:
    """A zero-velocity rigid island authored on managed ground must not settle."""
    ground = _body(
        "RestGround", "STATIC", "PLANE", (0.0, 0.0, 0.0),
        half_extents=(10.0, 10.0, 1.0e-5), friction=0.7,
    )
    ground["managed_ground"] = True
    base = _body(
        "RestBase", "DYNAMIC", "CONVEX_HULL", (0.0, 0.0, 0.0),
        half_extents=(0.35, 0.30, 0.30), mass=8.0, friction=0.7,
    )
    base["stable_id"] = str(uuid.uuid5(uuid.NAMESPACE_DNS, "ka-rigid-authored-rest-base"))
    base["shape_center"] = [0.0, 0.0, 0.24]
    base["convex_vertices"] = [
        [-0.32, -0.26, 0.0006], [0.31, -0.25, 0.0007],
        [-0.30, 0.27, 0.0008], [0.32, 0.26, 0.00065],
        [-0.24, -0.20, 0.48], [0.25, -0.19, 0.50],
        [-0.23, 0.21, 0.49], [0.24, 0.20, 0.51],
    ]
    top = _body(
        "RestTop", "DYNAMIC", "BOX", (0.0, 0.0, 0.72),
        half_extents=(0.16, 0.16, 0.16), mass=2.0, friction=0.7,
    )
    top["stable_id"] = str(uuid.uuid5(uuid.NAMESPACE_DNS, "ka-rigid-authored-rest-top"))
    payload = _payload(
        "rigid-authored-ground-rest", [ground, base, top],
        frames=30, gravity=(0.0, 0.0, -9.81), substeps=6,
    )
    payload["early_sleep_termination"] = False
    payload["stability"] = {"bond_stability_mode": "RIGID"}
    payload["constraints"] = [{
        "stable_id": str(uuid.uuid5(uuid.NAMESPACE_DNS, "ka-rigid-authored-rest-edge")),
        "constraint_type": "BREAKABLE_FIXED",
        "body_a": base["stable_id"],
        "body_b": top["stable_id"],
        "body_a_name": base["name"],
        "body_b_name": top["name"],
        "anchor": [0.0, 0.0, 0.55],
        "normal": [0.0, 0.0, 1.0],
        "area": 1.0,
        "break_force": 1.0e12,
        "break_torque": 1.0e12,
        "damage_accumulation": 0.0,
        "damage": 0.0,
        "enabled": True,
    }]
    result = backend.bake(payload)
    first = result["frames"]["1"]
    maximum_displacement = 0.0
    maximum_rotation_delta = 0.0
    for snapshot in result["frames"].values():
        for name in (base["name"], top["name"]):
            maximum_displacement = max(
                maximum_displacement,
                math.dist(first[name]["location"], snapshot[name]["location"]),
            )
            dot = abs(sum(
                float(first[name]["rotation"][index]) * float(snapshot[name]["rotation"][index])
                for index in range(4)
            ))
            maximum_rotation_delta = max(
                maximum_rotation_delta,
                2.0 * math.acos(max(-1.0, min(1.0, dot))),
            )
    totals = result.get("diagnostic_totals", {})
    passed = (
        maximum_displacement <= 1.0e-7
        and maximum_rotation_delta <= 1.0e-7
        and int(totals.get("bond_supported_cluster_deactivations", 0)) >= 1
        and int(result.get("final_state", {}).get("sleeping_bodies", 0)) == 2
        and int(totals.get("bond_cluster_count", 0)) == 1
    )
    return {
        "name": "Rigid authored ground rest",
        "passed": bool(passed),
        "metrics": {
            "maximum_displacement": maximum_displacement,
            "maximum_rotation_delta": maximum_rotation_delta,
            "supported_cluster_deactivations": totals.get("bond_supported_cluster_deactivations"),
            "sleeping_bodies": result.get("final_state", {}).get("sleeping_bodies"),
            "cluster_count": totals.get("bond_cluster_count"),
        },
    }


def _run_rigid_authored_ground_wake(backend: JoltBackend) -> Dict[str, Any]:
    """An initially sleeping rigid island must wake when an active body hits it."""
    ground = _body(
        "WakeGround", "STATIC", "PLANE", (0.0, 0.0, 0.0),
        half_extents=(10.0, 10.0, 1.0e-5), friction=0.7,
    )
    ground["managed_ground"] = True
    first = _body(
        "WakeA", "DYNAMIC", "CONVEX_HULL", (0.0, 0.0, 0.0),
        half_extents=(0.20, 0.20, 0.12), mass=5.0, friction=0.7,
    )
    second = _body(
        "WakeB", "DYNAMIC", "CONVEX_HULL", (0.0, 0.0, 0.0),
        half_extents=(0.20, 0.20, 0.12), mass=5.0, friction=0.7,
    )
    first["stable_id"] = str(uuid.uuid5(uuid.NAMESPACE_DNS, "ka-rigid-ground-wake-a"))
    second["stable_id"] = str(uuid.uuid5(uuid.NAMESPACE_DNS, "ka-rigid-ground-wake-b"))
    first["shape_center"] = [-0.22, 0.0, 0.12]
    second["shape_center"] = [0.22, 0.0, 0.12]
    first["convex_vertices"] = [
        [-0.22 + x, y, 0.12 + z]
        for x in (-0.20, 0.20) for y in (-0.20, 0.20) for z in (-0.12, 0.12)
    ]
    second["convex_vertices"] = [
        [0.22 + x, y, 0.12 + z]
        for x in (-0.20, 0.20) for y in (-0.20, 0.20) for z in (-0.12, 0.12)
    ]
    projectile = _body(
        "WakeProjectile", "DYNAMIC", "SPHERE", (0.0, 0.0, 1.2),
        radius=0.10, mass=2.0, friction=0.3,
        velocity=(0.0, 0.0, -6.0), ccd=True,
    )
    projectile["stable_id"] = str(uuid.uuid5(uuid.NAMESPACE_DNS, "ka-rigid-ground-wake-projectile"))
    payload = _payload(
        "rigid-authored-ground-wake", [ground, first, second, projectile],
        frames=80, gravity=(0.0, 0.0, -9.81), substeps=8,
    )
    payload["early_sleep_termination"] = False
    payload["stability"] = {"bond_stability_mode": "RIGID"}
    payload["diagnostics"] = {"enabled": True, "contacts": True, "payload": True}
    payload["constraints"] = [{
        "stable_id": str(uuid.uuid5(uuid.NAMESPACE_DNS, "ka-rigid-ground-wake-edge")),
        "constraint_type": "BREAKABLE_FIXED",
        "body_a": first["stable_id"],
        "body_b": second["stable_id"],
        "body_a_name": first["name"],
        "body_b_name": second["name"],
        "anchor": [0.0, 0.0, 0.12],
        "normal": [1.0, 0.0, 0.0],
        "area": 1.0,
        "break_force": 1.0e12,
        "break_torque": 1.0e12,
        "damage_accumulation": 0.0,
        "damage": 0.0,
        "enabled": True,
    }]
    result = backend.bake(payload)
    frame_one = result["frames"]["1"][first["name"]]
    frame_five = result["frames"]["5"][first["name"]]
    frame_forty = result["frames"]["40"][first["name"]]
    before_impact = math.dist(frame_one["location"], frame_five["location"])
    after_impact = math.dist(frame_one["location"], frame_forty["location"])
    totals = result.get("diagnostic_totals", {})
    speed_peaks = {
        str(entry.get("name", "")): entry
        for entry in result.get("body_speed_peaks", [])
        if isinstance(entry, dict)
    }
    wake_linear_speed = max(
        float(speed_peaks.get(first["name"], {}).get("max_linear_speed", 0.0)),
        float(speed_peaks.get(second["name"], {}).get("max_linear_speed", 0.0)),
    )
    wake_angular_speed = max(
        float(speed_peaks.get(first["name"], {}).get("max_angular_speed", 0.0)),
        float(speed_peaks.get(second["name"], {}).get("max_angular_speed", 0.0)),
    )
    passed = (
        before_impact <= 1.0e-7
        and after_impact <= 1.0e-3
        and (wake_linear_speed >= 1.0e-4 or wake_angular_speed >= 1.0e-4)
        and float(totals.get("max_contact_impulse", 0.0)) > 0.0
        and int(totals.get("bond_supported_cluster_deactivations", 0)) >= 1
        and int(totals.get("contact_events", 0)) > 0
        and int(totals.get("bond_cluster_count", 0)) == 1
    )
    return {
        "name": "Rigid authored ground wake",
        "passed": bool(passed),
        "metrics": {
            "before_impact_displacement": before_impact,
            "after_impact_displacement": after_impact,
            "wake_linear_speed": wake_linear_speed,
            "wake_angular_speed": wake_angular_speed,
            "max_contact_impulse": totals.get("max_contact_impulse"),
            "supported_cluster_deactivations": totals.get("bond_supported_cluster_deactivations"),
            "contact_events": totals.get("contact_events"),
            "cluster_count": totals.get("bond_cluster_count"),
        },
    }

def _run_ccd(backend: JoltBackend) -> Dict[str, Any]:
    bodies = [
        _body("Wall", "STATIC", "BOX", (0.0, 0.0, 0.0), half_extents=(0.05, 1.0, 1.0), friction=0.5),
        _body("Projectile", "DYNAMIC", "SPHERE", (-2.0, 0.0, 0.0), radius=0.05, mass=0.02, velocity=(45.0, 0.0, 0.0), ccd=True),
    ]
    result = backend.bake(_payload("ccd", bodies, frames=30, gravity=(0.0, 0.0, 0.0), substeps=2))
    final_x = float(_final_location(result, "Projectile")[0])
    passed = final_x < 0.25
    return {"name": "CCD thin wall", "passed": passed, "metrics": {"final_x": final_x}}



def _run_sharp_hull_ground_contact(backend: JoltBackend) -> Dict[str, Any]:
    """Sharp source-hull vertices must not remain below managed ground."""
    ground = _body(
        "SharpGround", "STATIC", "PLANE", (0.0, 0.0, 0.0),
        half_extents=(10.0, 10.0, 1.0e-5), friction=0.7,
    )
    ground["managed_ground"] = True
    body = _body(
        "SharpHull", "DYNAMIC", "CONVEX_HULL", (0.0, 0.0, 1.8),
        half_extents=(0.7, 0.8, 0.9), mass=3.0, friction=0.55, ccd=True,
    )
    points = [
        [-0.50, -0.20, -0.30],
        [0.70, -0.10, -0.20],
        [0.20, 0.80, -0.10],
        [-0.10, 0.20, 0.90],
    ]
    body["convex_vertices"] = points
    body["shape_center"] = [
        sum(point[axis] for point in points) / len(points)
        for axis in range(3)
    ]
    body["angular_velocity"] = [20.0, 12.0, 17.0]
    body["collider_quality"] = {
        "separation_inset_applied": 0.00025,
        "max_error": 0.0002,
    }
    payload = _payload(
        "sharp-hull-ground-contact", [ground, body], frames=180, substeps=8
    )
    payload["early_sleep_termination"] = False
    result = backend.bake(payload)
    minimum_z = float("inf")
    minimum_frame = None
    # Frame 1 is the exact authored pose and intentionally bypasses output
    # compensation. It begins well above ground in this fixture.
    for frame, snapshot in result["frames"].items():
        transform = snapshot[body["name"]]
        location = transform["location"]
        rotation = transform["rotation"]
        frame_minimum = min(
            float(location[2]) + quat_rotate_vector_wxyz(rotation, point)[2]
            for point in points
        )
        if frame_minimum < minimum_z:
            minimum_z = float(frame_minimum)
            minimum_frame = int(frame)
    totals = result.get("diagnostic_totals", {})
    native_bridge = not bool(totals.get("visual_ground_compensation_enabled", False))
    corrected_frames = int(totals.get("visual_ground_compensation_frames", 0))
    passed = (
        minimum_z >= -1.1e-4
        and (native_bridge or corrected_frames > 0)
    )
    return {
        "name": "Sharp convex hull managed-ground contact",
        "passed": bool(passed),
        "metrics": {
            "minimum_sharp_hull_z": minimum_z,
            "minimum_frame": minimum_frame,
            "visual_ground_compensation_enabled": totals.get("visual_ground_compensation_enabled"),
            "corrected_frames": corrected_frames,
            "corrected_bodies": totals.get("visual_ground_compensation_bodies"),
            "maximum_correction": totals.get("visual_ground_compensation_max_correction"),
        },
    }


def _run_managed_ground_guard(backend: JoltBackend) -> Dict[str, Any]:
    # Reproduce the 0.4.5 failure mode: bulk assignment stored the managed
    # ground as Dynamic/Mesh. The backend must still normalize it to a static
    # infinite Plane before body creation.
    ground = _body(
        "KA_Physics_Ground", "DYNAMIC", "MESH", (0.0, 0.0, 0.0),
        half_extents=(10.0, 10.0, 1.0e-5), friction=0.8, ccd=True,
    )
    ground["managed_ground"] = True
    projectile = _body(
        "FastDrop", "DYNAMIC", "SPHERE", (0.0, 0.0, 2.0),
        radius=0.08, mass=0.02, velocity=(0.0, 0.0, -30.0), ccd=True,
    )
    result = backend.bake(_payload(
        "managed-ground-guard", [ground, projectile], frames=80, substeps=2
    ))
    final_z = float(_final_location(result, "FastDrop")[2])
    final = result.get("final_state", {})
    passed = (
        final_z > -0.02
        and int(final.get("static_bodies", -1)) == 1
        and int(final.get("dynamic_bodies", -1)) == 1
    )
    return {
        "name": "Managed ground plane guard",
        "passed": passed,
        "metrics": {
            "final_z": final_z,
            "static_bodies": int(final.get("static_bodies", -1)),
            "dynamic_bodies": int(final.get("dynamic_bodies", -1)),
            "sleeping_bodies": int(final.get("sleeping_bodies", -1)),
        },
    }


def _run_compound(backend: JoltBackend) -> Dict[str, Any]:
    payload = _payload("compound", [
        _body("Ground", "STATIC", "PLANE", (0.0, 0.0, 0.0), half_extents=(10.0, 10.0, 1.0e-5), friction=0.8),
        _body("CompoundBody", "DYNAMIC", "COMPOUND", (0.0, 0.0, 2.0), half_extents=(0.7, 0.25, 0.25), mass=1.5, friction=0.5),
    ], frames=120, substeps=6)
    result = backend.bake(payload)
    final = result["frames"][str(result["frame_end"])]["CompoundBody"]
    z = float(final["location"][2])
    sleeping = int(result.get("final_state", {}).get("sleeping_bodies", 0))
    passed = 0.18 <= z <= 0.35 and sleeping == 1
    return {"name": "Compound body settle", "passed": passed, "metrics": {"final_z": z, "sleeping": sleeping}}



def _run_compound_convex_cluster(backend: JoltBackend) -> Dict[str, Any]:
    payload = _payload("compound-convex-cluster", [
        _body("Ground", "STATIC", "PLANE", (0.0, 0.0, 0.0), half_extents=(10.0, 10.0, 1.0e-5), friction=0.8),
        _body("CompoundConvex", "DYNAMIC", "COMPOUND_CONVEX", (0.0, 0.0, 2.0), half_extents=(0.70, 0.25, 0.25), mass=1.5, friction=0.5),
    ], frames=150, substeps=8)
    result = backend.bake(payload)
    final = result["frames"][str(result["frame_end"])]["CompoundConvex"]
    z = float(final["location"][2])
    x = float(final["location"][0])
    sleeping = int(result.get("final_state", {}).get("sleeping_bodies", 0))
    totals = result.get("diagnostic_totals", {})
    passed = (
        0.16 <= z <= 0.38
        and abs(x) < 0.35
        and sleeping == 1
        and int(totals.get("native_body_count", 0)) == 2
        and int(totals.get("compound_constraint_count", 0)) == 0
    )
    return {
        "name": "Compound Convex single-body fallback",
        "passed": passed,
        "metrics": {
            "final_location": final["location"],
            "sleeping": sleeping,
            "native_body_count": totals.get("native_body_count"),
            "constraint_count": totals.get("compound_constraint_count"),
        },
    }

def _run_dense_fracture_pile(backend: JoltBackend) -> Dict[str, Any]:
    bodies = [
        _body("Ground", "STATIC", "PLANE", (0.0, 0.0, 0.0), half_extents=(10.0, 10.0, 1.0e-5), friction=0.8)
    ]
    index = 0
    for layer in range(6):
        for row in range(4):
            for column in range(4):
                x = (column - 1.5) * 0.185 + (((index * 17) % 7) - 3) * 0.001
                y = (row - 1.5) * 0.185 + (((index * 13) % 5) - 2) * 0.001
                z = 0.095 + layer * 0.19
                bodies.append(
                    _body(
                        f"Fragment_{index:03d}", "DYNAMIC", "BOX", (x, y, z),
                        half_extents=(0.09, 0.09, 0.09), mass=1.0, friction=0.65,
                    )
                )
                index += 1
    result = backend.bake(_payload("dense-fracture-pile", bodies, frames=180, substeps=8))
    totals = result.get("diagnostic_totals", {})
    final = result.get("final_state", {})
    sleeping = int(final.get("sleeping_bodies", 0))
    active = int(final.get("active_bodies", 0))
    early_frame = totals.get("early_sleep_frame")
    minimum_steps = int(totals.get("minimum_executed_substeps_per_frame") or 0)
    maximum_steps = int(totals.get("maximum_executed_substeps_per_frame") or 0)
    simulated_frames = max(0, int(early_frame or result.get("frame_end", 1)) - int(result.get("frame_start", 1)))
    exact_substep_accounting = int(totals.get("executed_substeps", 0)) == simulated_frames * minimum_steps
    tail_energy = float(totals.get("final_motion_energy_proxy", -1.0))
    direct_values = int(totals.get("binary_frame_values", 0))
    expected_values = len(result.get("frames", {})) * len(bodies) * 7
    passed = (
        sleeping == 96 and active == 0 and early_frame is not None and int(early_frame) <= 120
        and minimum_steps == 4 and maximum_steps == 4 and exact_substep_accounting
        and tail_energy <= 1.0e-12 and direct_values == expected_values
    )
    return {
        "name": "Dense 96-body fracture pile",
        "passed": passed,
        "metrics": {
            "sleeping": sleeping,
            "active": active,
            "early_sleep_frame": early_frame,
            "executed_substeps": int(totals.get("executed_substeps", 0)),
            "minimum_substeps": minimum_steps,
            "maximum_substeps": maximum_steps,
            "exact_substep_accounting": exact_substep_accounting,
            "final_motion_energy_proxy": tail_energy,
            "direct_binary_values": direct_values,
            "expected_binary_values": expected_values,
        },
    }


def _run_confirmed_hybrid_sleep(backend: JoltBackend) -> Dict[str, Any]:
    # A slowly drifting body in zero gravity remains native-active long enough
    # for Hybrid to issue and confirm an explicit queued deactivation.
    payload = _payload("confirmed-hybrid-sleep", [
        _body("Drifter", "DYNAMIC", "BOX", (0.0, 0.0, 0.0),
              half_extents=(0.25, 0.25, 0.25), velocity=(0.01, 0.0, 0.0)),
    ], frames=20, gravity=(0.0, 0.0, 0.0), substeps=4)
    payload["sleep_mode"] = "HYBRID"
    payload["sleep_time"] = 0.0
    result = backend.bake(payload)
    totals = result.get("diagnostic_totals", {})
    final = result.get("final_state", {})
    requests = int(totals.get("sleep_deactivation_requests", 0))
    confirmed = int(totals.get("sleep_deactivation_confirmed", 0))
    rejected = int(totals.get("sleep_deactivation_rejected", 0))
    passed = (
        requests > 0 and confirmed > 0 and confirmed + rejected == requests
        and int(final.get("active_bodies", -1)) == 0
        and int(final.get("sleeping_bodies", -1)) == 1
    )
    return {
        "name": "Confirmed hybrid sleep commands",
        "passed": passed,
        "metrics": {
            "requests": requests, "confirmed": confirmed, "rejected": rejected,
            "active": int(final.get("active_bodies", -1)),
            "sleeping": int(final.get("sleeping_bodies", -1)),
        },
    }


def _run_high_detail_convex_hull(backend: JoltBackend) -> Dict[str, Any]:
    """Verify that precision-rescue hulls with hundreds of vertices remain stable."""
    point_count = 384
    points = []
    golden_angle = math.pi * (3.0 - math.sqrt(5.0))
    for index in range(point_count):
        z = 1.0 - (2.0 * (index + 0.5) / point_count)
        radial = math.sqrt(max(0.0, 1.0 - z * z))
        angle = golden_angle * index
        points.append([
            0.45 * radial * math.cos(angle),
            0.35 * radial * math.sin(angle),
            0.25 * z,
        ])
    body = _body(
        "PrecisionHull", "DYNAMIC", "CONVEX_HULL", (0.0, 0.0, 1.5),
        half_extents=(0.45, 0.35, 0.25), radius=0.6225, mass=8.0,
        friction=0.7,
    )
    body["convex_vertices"] = points
    result = backend.bake(_payload("high-detail-convex-hull", [
        _body("Ground", "STATIC", "PLANE", (0.0, 0.0, 0.0), half_extents=(10.0, 10.0, 1.0e-5), friction=0.8),
        body,
    ], frames=220, substeps=8))
    final_z = float(_final_location(result, "PrecisionHull")[2])
    sleeping = int(result.get("final_state", {}).get("sleeping_bodies", 0))
    passed = 0.20 <= final_z <= 0.34 and sleeping == 1
    return {
        "name": "High-detail precision hull",
        "passed": passed,
        "metrics": {"vertices": point_count, "final_z": final_z, "sleeping": sleeping},
    }


def _run_irregular_mass_ratio_pile(backend: JoltBackend) -> Dict[str, Any]:
    bodies = [_body("Ground", "STATIC", "PLANE", (0.0, 0.0, 0.0), half_extents=(12.0, 12.0, 1.0e-5), friction=0.8)]
    masses = (38.5, 8.0, 2.0, 0.5, 0.08, 0.0077)
    index = 0
    for layer in range(4):
        for row in range(4):
            for column in range(4):
                hx = 0.065 + 0.008 * ((index * 3) % 4)
                hy = 0.060 + 0.007 * ((index * 5) % 5)
                hz = 0.060 + 0.009 * ((index * 7) % 4)
                body = _body(
                    f"Irregular_{index:03d}", "DYNAMIC", "CONVEX_HULL",
                    ((column - 1.5) * 0.17, (row - 1.5) * 0.17, 0.08 + layer * 0.18),
                    half_extents=(hx, hy, hz), mass=masses[index % len(masses)], friction=0.65,
                    ccd=(masses[index % len(masses)] < 0.01),
                )
                # Deterministic asymmetric hull: preserves convexity while
                # exercising non-box inertia and contact geometry.
                body["convex_vertices"] = [
                    [x * (1.0 + 0.08 * ((vertex + index) % 3)),
                     y * (1.0 - 0.05 * ((vertex + 2 * index) % 2)),
                     z * (1.0 + 0.06 * ((vertex + index) % 2))]
                    for vertex, (x, y, z) in enumerate(body["convex_vertices"])
                ]
                bodies.append(body)
                index += 1
    result = backend.bake(_payload("irregular-mass-ratio-pile", bodies, frames=220, substeps=8))
    final = result.get("final_state", {})
    totals = result.get("diagnostic_totals", {})
    active = int(final.get("active_bodies", -1))
    sleeping = int(final.get("sleeping_bodies", -1))
    passed = active == 0 and sleeping == 64 and int(totals.get("executed_substeps", 999999)) < 1500
    return {
        "name": "Irregular 5000-to-1 fracture pile",
        "passed": passed,
        "metrics": {
            "active": active, "sleeping": sleeping,
            "early_sleep_frame": totals.get("early_sleep_frame"),
            "executed_substeps": int(totals.get("executed_substeps", 0)),
        },
    }


def _run_contact_buffer(backend: JoltBackend) -> Dict[str, Any]:
    payload = _payload("contact-buffer", [
        _body("Ground", "STATIC", "PLANE", (0.0, 0.0, 0.0), half_extents=(10.0, 10.0, 1.0e-5)),
        _body("ContactBox", "DYNAMIC", "BOX", (0.0, 0.0, 1.0), half_extents=(0.25, 0.25, 0.25)),
    ], frames=70, substeps=8)
    payload["diagnostics"] = {"enabled": False, "contacts": True, "side_stick": False}
    result = backend.bake(payload)
    totals = result.get("diagnostic_totals", {})
    events = int(totals.get("contact_events", 0))
    passed = events > 0 and bool(totals.get("penetration_depth_available"))
    return {
        "name": "Zero-copy contact buffer",
        "passed": passed,
        "metrics": {
            "contact_events": events,
            "penetration_depth_available": bool(totals.get("penetration_depth_available")),
        },
    }


def _run_binary_cache_roundtrip(backend: JoltBackend) -> Dict[str, Any]:
    payload = _payload("binary-cache", [
        _body("Ground", "STATIC", "PLANE", (0.0, 0.0, 0.0), half_extents=(10.0, 10.0, 1.0e-5)),
        _body("CacheBody", "DYNAMIC", "BOX", (0.0, 0.0, 1.5), half_extents=(0.2, 0.3, 0.25)),
    ], frames=80, substeps=6)
    result = backend.bake(payload)
    with tempfile.TemporaryDirectory(prefix="ka_rigid_cache_test_") as directory:
        started = time.perf_counter()
        path = write_cache(directory, result)
        write_seconds = time.perf_counter() - started
        loaded = read_cache(directory)
        comparison = compare_frames(result["frames"], loaded["frames"], tolerance=1.0e-6)
        size_bytes = os.path.getsize(path)
    direct_block = result.get("_binary_frame_block", {})
    direct_values = direct_block.get("values")
    expected_values = len(result.get("frames", {})) * len(direct_block.get("body_names", [])) * 7
    direct_count = len(direct_values) if direct_values is not None else 0
    passed = bool(comparison.get("match")) and size_bytes > 0 and direct_count == expected_values
    return {
        "name": "Binary cache roundtrip",
        "passed": passed,
        "metrics": {
            "write_seconds": round(write_seconds, 6),
            "size_bytes": size_bytes,
            "max_error": comparison.get("max_error"),
            "compared_values": comparison.get("compared_values"),
            "direct_values": direct_count,
            "expected_direct_values": expected_values,
        },
    }


def _run_initial_frame_integrity(backend: JoltBackend) -> Dict[str, Any]:
    expected_location = (1.25, -0.75, 2.5)
    payload = _payload("initial-frame-integrity", [
        _body("Ground", "STATIC", "PLANE", (0.0, 0.0, 0.0), half_extents=(10.0, 10.0, 1.0e-5)),
        _body("InitialBody", "DYNAMIC", "BOX", expected_location, half_extents=(0.2, 0.3, 0.4)),
    ], frames=4, gravity=(0.0, 0.0, 0.0), substeps=2)
    payload["store_python_frames"] = False
    payload["diagnostics"] = {"enabled": False, "contacts": False}
    result = backend.bake(payload)
    decoded = decode_direct_frame_block(result.get("_binary_frame_block", {}))
    first = decoded.get("1", {}).get("InitialBody", {})
    location = tuple(float(value) for value in first.get("location", ()))
    rotation = tuple(float(value) for value in first.get("rotation", ()))
    max_error = max((abs(location[index] - expected_location[index]) for index in range(3)), default=float("inf"))
    passed = max_error <= 1.0e-6 and rotation == (1.0, 0.0, 0.0, 0.0)
    return {
        "name": "Exact initial cache frame",
        "passed": passed,
        "metrics": {
            "expected_location": list(expected_location),
            "cached_location": list(location),
            "cached_rotation": list(rotation),
            "max_error": max_error,
        },
    }


def _run_production_binary_only(backend: JoltBackend) -> Dict[str, Any]:
    payload = _payload("production-binary", [
        _body("Ground", "STATIC", "PLANE", (0.0, 0.0, 0.0), half_extents=(10.0, 10.0, 1.0e-5)),
        *[
            _body(
                f"ProductionBody{index:02d}",
                "DYNAMIC",
                "BOX",
                ((index % 4) * 0.24 - 0.36, (index // 4) * 0.24 - 0.24, 1.0 + (index % 3) * 0.2),
                half_extents=(0.09, 0.09, 0.09),
            )
            for index in range(12)
        ],
    ], frames=90, substeps=6)
    payload["store_python_frames"] = False
    payload["diagnostics"] = {"enabled": False, "contacts": False}
    result = backend.bake(payload)
    direct = result.get("_binary_frame_block", {})
    decoded = decode_direct_frame_block(direct)
    with tempfile.TemporaryDirectory(prefix="ka_rigid_production_cache_test_") as directory:
        path = write_cache(directory, result)
        loaded = read_cache(directory)
        comparison = compare_frames(decoded, loaded.get("frames", {}), tolerance=1.0e-6)
        size_bytes = os.path.getsize(path)
    totals = result.get("diagnostic_totals", {})
    frame_count = int(result.get("frame_count", 0))
    passed = (
        not result.get("frames")
        and len(decoded) == frame_count == 90
        and bool(comparison.get("match"))
        and int(totals.get("python_frame_snapshots", -1)) == 0
        and bool(totals.get("binary_only_cache"))
        and size_bytes > 0
    )
    return {
        "name": "Default binary-only cache",
        "passed": passed,
        "metrics": {
            "frame_count": frame_count,
            "python_frame_snapshots": totals.get("python_frame_snapshots"),
            "binary_frame_values": totals.get("binary_frame_values"),
            "cache_size_bytes": size_bytes,
            "max_roundtrip_error": comparison.get("max_error"),
        },
    }



def _run_independent_diagnostics(backend: JoltBackend) -> Dict[str, Any]:
    bodies = [
        _body("Ground", "STATIC", "PLANE", (0.0, 0.0, 0.0), half_extents=(10.0, 10.0, 1.0e-5)),
        _body("DiagnosticBody", "DYNAMIC", "BOX", (0.0, 0.0, 1.0), half_extents=(0.25, 0.25, 0.25)),
    ]

    contact_payload = _payload("independent-contact", copy.deepcopy(bodies), frames=70, substeps=8)
    contact_payload["store_python_frames"] = False
    contact_payload["diagnostics"] = {
        "enabled": False, "contacts": True, "log_contacts": True,
        "payload": False, "side_stick": False,
    }
    contact_result = backend.bake(contact_payload)
    contact_totals = contact_result.get("diagnostic_totals", {})

    payload_only = _payload("independent-payload", copy.deepcopy(bodies), frames=40, substeps=6)
    payload_only["store_python_frames"] = False
    payload_only["diagnostics"] = {
        "enabled": False, "contacts": False, "log_contacts": False,
        "payload": True, "side_stick": False,
    }
    payload_result = backend.bake(payload_only)
    payload_totals = payload_result.get("diagnostic_totals", {})

    passed = (
        not contact_result.get("frames")
        and bool(contact_result.get("contact_diagnostics_enabled"))
        and not bool(contact_result.get("payload_diagnostics_enabled"))
        and int(contact_totals.get("contact_events", 0)) > 0
        and not contact_result.get("body_speed_peaks")
        and bool(contact_totals.get("binary_only_cache"))
        and not payload_result.get("frames")
        and not bool(payload_result.get("contact_diagnostics_enabled"))
        and bool(payload_result.get("payload_diagnostics_enabled"))
        and int(payload_totals.get("contact_events", 0)) == 0
        and bool(payload_result.get("body_speed_peaks"))
        and bool(payload_totals.get("binary_only_cache"))
    )
    return {
        "name": "Independent binary diagnostics",
        "passed": passed,
        "metrics": {
            "contact_events": int(contact_totals.get("contact_events", 0)),
            "contact_body_peaks": len(contact_result.get("body_speed_peaks", [])),
            "payload_contact_events": int(payload_totals.get("contact_events", 0)),
            "payload_body_peaks": len(payload_result.get("body_speed_peaks", [])),
            "contact_python_frames": len(contact_result.get("frames", {})),
            "payload_python_frames": len(payload_result.get("frames", {})),
        },
    }


def _run_diagnostic_log_filtering(backend: JoltBackend) -> Dict[str, Any]:
    bodies = [
        _body("Ground", "STATIC", "PLANE", (0.0, 0.0, 0.0), half_extents=(10.0, 10.0, 1.0e-5)),
        _body("LogBody", "DYNAMIC", "BOX", (0.0, 0.0, 1.0), half_extents=(0.25, 0.25, 0.25)),
    ]

    def bake_log(name: str, diagnostics: Dict[str, Any]) -> str:
        with tempfile.TemporaryDirectory(prefix=f"ka_rigid_{name}_") as directory:
            path = os.path.join(directory, "diagnostics.log")
            payload = _payload(name, copy.deepcopy(bodies), frames=35, substeps=6)
            payload["store_python_frames"] = False
            payload["diagnostics"] = {"enabled": True, "path": path, **diagnostics}
            backend.bake(payload)
            with open(path, "r", encoding="utf-8") as stream:
                return stream.read()

    base_log = bake_log(
        "log-base",
        {"contacts": False, "log_contacts": False, "payload": False, "side_stick": False},
    )
    contact_log = bake_log(
        "log-contact",
        {"contacts": True, "log_contacts": True, "payload": False, "side_stick": False},
    )
    payload_log = bake_log(
        "log-payload",
        {"contacts": False, "log_contacts": False, "payload": True, "side_stick": False},
    )

    base_clean = (
        "FRAME_COMPLETE" not in base_log
        and "strongest_contact_pairs" not in base_log
        and "body_speed_peaks" not in base_log
    )
    contact_filtered = (
        "FRAME_COMPLETE" in contact_log
        and "strongest_contact_pairs" in contact_log
        and "body_speed_peaks" not in contact_log
    )
    payload_filtered = (
        "FRAME_COMPLETE" not in payload_log
        and "strongest_contact_pairs" not in payload_log
        and "body_speed_peaks" in payload_log
    )
    return {
        "name": "Diagnostic log filtering",
        "passed": bool(base_clean and contact_filtered and payload_filtered),
        "metrics": {
            "base_clean": base_clean,
            "contact_filtered": contact_filtered,
            "payload_filtered": payload_filtered,
            "base_bytes": len(base_log.encode("utf-8")),
            "contact_bytes": len(contact_log.encode("utf-8")),
            "payload_bytes": len(payload_log.encode("utf-8")),
        },
    }


def _run_rotation_aware_substeps(_backend: JoltBackend) -> Dict[str, Any]:
    """Fast angular surface travel must escalate the adaptive step tier."""
    base_motion = {
        "active_bodies": 1,
        "max_linear_speed": 0.0,
        "max_angular_speed": 0.0,
        "max_angular_surface_speed": 0.0,
        "minimum_feature_length": 0.05,
        "active_ccd": True,
    }
    rotating_motion = dict(base_motion)
    rotating_motion["max_angular_surface_speed"] = 10.0
    baseline = JoltBackend._adaptive_substep_count_from_motion(
        base_motion, 1.0 / 60.0, 4, 20, 9.81
    )
    rotating = JoltBackend._adaptive_substep_count_from_motion(
        rotating_motion, 1.0 / 60.0, 4, 20, 9.81
    )
    passed = baseline == 4 and rotating > baseline
    return {
        "name": "Rotation-aware adaptive substeps",
        "passed": bool(passed),
        "metrics": {
            "baseline_substeps": baseline,
            "rotating_substeps": rotating,
            "angular_surface_speed": rotating_motion["max_angular_surface_speed"],
            "minimum_feature_length": rotating_motion["minimum_feature_length"],
        },
    }


def _run_thread_heuristic(_backend: JoltBackend) -> Dict[str, Any]:
    samples = {
        16: recommended_jolt_threads(16, cpu_count=32),
        116: recommended_jolt_threads(116, cpu_count=32),
        512: recommended_jolt_threads(512, cpu_count=32),
        2048: recommended_jolt_threads(2048, cpu_count=32),
        5000: recommended_jolt_threads(5000, cpu_count=32),
    }
    expected = {16: 2, 116: 4, 512: 4, 2048: 6, 5000: 8}
    capped = recommended_jolt_threads(5000, cpu_count=4)
    passed = samples == expected and capped == 3
    return {
        "name": "Conservative Jolt thread heuristic",
        "passed": passed,
        "metrics": {"samples": samples, "cpu4_cap": capped},
    }

def _run_distance_rope_constraint(backend: JoltBackend) -> Dict[str, Any]:
    """A Dynamic sphere must swing without exceeding its authored rope length."""
    anchor_id = "ab164f0a-78e8-5bf5-a47a-155d5e86bb0d"
    ball_id = "470e49fc-ac87-50a3-8362-4d97095a5427"
    constraint_id = "33a0a78f-b58d-5154-aa90-ae950da97a45"
    anchor = _body("RopeAnchor", "STATIC", "SPHERE", (0.0, 0.0, 5.0), radius=0.03)
    ball = _body(
        "WreckingBall",
        "DYNAMIC",
        "SPHERE",
        (4.0, 0.0, 2.0),
        radius=0.5,
        mass=1000.0,
        velocity=(0.0, 1.0, 0.0),
        ccd=True,
    )
    anchor["stable_id"] = anchor_id
    anchor["collision_mask"] = 0
    ball["stable_id"] = ball_id
    payload = _payload("distance-rope", [anchor, ball], frames=121, substeps=12)
    payload["constraints"] = [{
        "stable_id": constraint_id,
        "display_name": "Regression Rope",
        "constraint_type": "DISTANCE",
        "distance_mode": "ROPE",
        "body_a": anchor_id,
        "body_b": ball_id,
        "body_a_name": "RopeAnchor",
        "body_b_name": "WreckingBall",
        "min_distance": 0.0,
        "max_distance": 5.0,
        "enabled": True,
    }]
    result = backend.bake(payload)
    distances = []
    moved = False
    initial = tuple(result["frames"]["1"]["WreckingBall"]["location"])
    for frame in result["frames"].values():
        anchor_location = frame["RopeAnchor"]["location"]
        ball_location = frame["WreckingBall"]["location"]
        distance = math.sqrt(sum((float(b) - float(a)) ** 2 for a, b in zip(anchor_location, ball_location)))
        distances.append(distance)
        moved = moved or math.sqrt(sum((float(value) - float(start)) ** 2 for value, start in zip(ball_location, initial))) > 0.1
    maximum = max(distances)
    tolerance = 2.0e-3
    passed = (
        bool(result.get("distance_constraints_enabled"))
        and int(result.get("distance_constraint_count", 0)) == 1
        and maximum <= 5.0 + tolerance
        and moved
    )
    return {
        "name": "Distance rope constraint",
        "passed": bool(passed),
        "metrics": {
            "constraint_count": int(result.get("distance_constraint_count", 0)),
            "maximum_distance": maximum,
            "authored_length": 5.0,
            "maximum_error": maximum - 5.0,
            "moved": moved,
        },
    }


def _run_distance_rope_bond_rebuild(backend: JoltBackend) -> Dict[str, Any]:
    """An unrelated fracture rebuild must not invalidate a rope endpoint."""
    names = ("RopeAnchor", "WreckingBall", "BondA", "BondB", "Breaker")
    ids = {
        name: str(uuid.uuid5(uuid.NAMESPACE_DNS, f"ka-rope-rebuild-{name}"))
        for name in names
    }
    anchor = _body("RopeAnchor", "STATIC", "SPHERE", (0.0, 0.0, 5.0), radius=0.03)
    ball = _body(
        "WreckingBall", "DYNAMIC", "SPHERE", (4.0, 0.0, 5.0),
        radius=0.45, mass=1000.0, velocity=(0.0, 2.0, 0.0), ccd=True,
    )
    bond_a = _body("BondA", "DYNAMIC", "BOX", (10.0, 0.0, 1.0), mass=10.0)
    bond_b = _body("BondB", "DYNAMIC", "BOX", (11.0, 0.0, 1.0), mass=10.0)
    breaker = _body(
        "Breaker", "DYNAMIC", "SPHERE", (15.0, 0.0, 1.0),
        radius=0.35, mass=100.0, velocity=(-25.0, 0.0, 0.0), ccd=True,
    )
    bodies = [anchor, ball, bond_a, bond_b, breaker]
    for body in bodies:
        body["stable_id"] = ids[body["name"]]
        body["linear_damping"] = 0.0
        body["angular_damping"] = 0.0
    anchor["collision_mask"] = 0

    payload = _payload(
        "distance-rope-bond-rebuild", bodies,
        frames=121, gravity=(0.0, 0.0, 0.0), substeps=10,
    )
    payload["sleep_enabled"] = False
    payload["stability"] = {"bond_stability_mode": "RIGID"}
    payload["constraints"] = [
        {
            "stable_id": str(uuid.uuid5(uuid.NAMESPACE_DNS, "ka-rope-rebuild-bond")),
            "constraint_type": "BREAKABLE_FIXED",
            "body_a": ids["BondA"],
            "body_b": ids["BondB"],
            "body_a_name": "BondA",
            "body_b_name": "BondB",
            "anchor": [10.5, 0.0, 1.0],
            "normal": [1.0, 0.0, 0.0],
            "area": 1.0,
            "break_force": 50.0,
            "break_torque": 50.0,
            "damage_accumulation": 0.0,
            "damage": 0.0,
            "enabled": True,
        },
        {
            "stable_id": str(uuid.uuid5(uuid.NAMESPACE_DNS, "ka-rope-rebuild-distance")),
            "display_name": "Rebuild-safe Rope",
            "constraint_type": "DISTANCE",
            "distance_mode": "ROPE",
            "body_a": ids["RopeAnchor"],
            "body_b": ids["WreckingBall"],
            "body_a_name": "RopeAnchor",
            "body_b_name": "WreckingBall",
            "min_distance": 0.0,
            "max_distance": 4.0,
            "enabled": True,
        },
    ]
    result = backend.bake(payload)
    distances = []
    for snapshot in result["frames"].values():
        distances.append(math.dist(
            snapshot["RopeAnchor"]["location"],
            snapshot["WreckingBall"]["location"],
        ))
    totals = result.get("diagnostic_totals", {})
    maximum = max(distances)
    passed = (
        int(totals.get("bond_break_events", 0)) >= 1
        and int(totals.get("bond_cluster_rebuilds", 0)) >= 2
        and int(totals.get("bond_preserved_external_dynamic_bodies", 0)) >= 2
        and not bool(totals.get("distance_constraint_rebind_required"))
        and int(totals.get("distance_constraint_rebinds", 0)) == 0
        and int(totals.get("distance_constraint_count", 0)) == 1
        and maximum <= 4.002
    )
    return {
        "name": "Distance rope survives bond rebuild",
        "passed": bool(passed),
        "metrics": {
            "maximum_distance": maximum,
            "maximum_error": maximum - 4.0,
            "bond_break_events": totals.get("bond_break_events"),
            "bond_cluster_rebuilds": totals.get("bond_cluster_rebuilds"),
            "preserved_external_dynamic_bodies": totals.get(
                "bond_preserved_external_dynamic_bodies"
            ),
            "distance_constraint_rebind_required": totals.get(
                "distance_constraint_rebind_required"
            ),
            "distance_constraint_rebinds": totals.get("distance_constraint_rebinds"),
            "distance_constraint_count": totals.get("distance_constraint_count"),
        },
    }


def _run_determinism(backend: JoltBackend, tolerance: float) -> Dict[str, Any]:
    payload = _payload("determinism", [
        _body("Ground", "STATIC", "PLANE", (0.0, 0.0, 0.0), half_extents=(10.0, 10.0, 1.0e-5), friction=0.7),
        _body("BodyA", "DYNAMIC", "BOX", (-0.2, 0.0, 1.5), half_extents=(0.2, 0.3, 0.25), velocity=(0.4, 0.0, 0.0)),
        _body("BodyB", "DYNAMIC", "SPHERE", (0.2, 0.0, 2.0), radius=0.2, velocity=(-0.3, 0.0, 0.0)),
    ], frames=120, substeps=6)
    first = backend.bake(copy.deepcopy(payload))
    second = backend.bake(copy.deepcopy(payload))
    comparison = compare_frames(first["frames"], second["frames"], tolerance=tolerance)
    return {"name": "Deterministic repeat", "passed": bool(comparison["match"]), "metrics": comparison}


def run_regression_suite(*, determinism_tolerance: float = 1.0e-6) -> Dict[str, Any]:
    started = time.perf_counter()
    backend = JoltBackend()
    tests = []
    for runner in (
        _run_simulation_scene_roundtrip, _run_simulation_scene_runtime_fallback, _run_simulation_scene_identity,
        _run_coacd_decomposition, _run_drop, _run_restitution, _run_stack, _run_friction, _run_low_friction_antistick_contact, _run_rigid_bond_island, _run_rigid_static_anchor, _run_component_mass_conditioning, _run_mass_aware_dense_anchor_release, _run_rigid_static_anchor_authored_pose, _run_rigid_static_anchor_neighbor_rest, _run_rigid_bond_collision_filter, _run_rigid_bond_island_sleep, _run_rigid_hull_fallback_ground_contact, _run_rigid_authored_ground_rest, _run_rigid_authored_ground_wake, _run_ccd, _run_sharp_hull_ground_contact,
        _run_managed_ground_guard, _run_compound, _run_compound_convex_cluster,
        _run_dense_fracture_pile, _run_confirmed_hybrid_sleep,
        _run_high_detail_convex_hull, _run_irregular_mass_ratio_pile, _run_contact_buffer,
        _run_binary_cache_roundtrip, _run_initial_frame_integrity, _run_production_binary_only, _run_independent_diagnostics,
        _run_diagnostic_log_filtering, _run_rotation_aware_substeps, _run_thread_heuristic,
        _run_distance_rope_constraint, _run_distance_rope_bond_rebuild,
    ):
        test_started = time.perf_counter()
        try:
            item = runner(backend)
        except Exception as exc:
            item = {"name": runner.__name__.removeprefix("_run_").replace("_", " ").title(), "passed": False, "error": str(exc), "metrics": {}}
        item["elapsed_seconds"] = round(time.perf_counter() - test_started, 6)
        tests.append(item)
    test_started = time.perf_counter()
    try:
        item = _run_determinism(backend, determinism_tolerance)
    except Exception as exc:
        item = {"name": "Deterministic repeat", "passed": False, "error": str(exc), "metrics": {}}
    item["elapsed_seconds"] = round(time.perf_counter() - test_started, 6)
    tests.append(item)

    passed = sum(bool(item.get("passed")) for item in tests)
    return {
        "suite_version": 33,
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "passed": passed,
        "failed": len(tests) - passed,
        "total": len(tests),
        "success": passed == len(tests),
        "elapsed_seconds": round(time.perf_counter() - started, 6),
        "tests": tests,
        "digest": frames_digest({str(index): {item["name"]: item.get("metrics", {})} for index, item in enumerate(tests)}),
    }


def write_regression_report(directory: str, report: Dict[str, Any]) -> str:
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, REGRESSION_FILENAME)
    if os.path.isfile(path):
        try:
            with open(path, "r", encoding="utf-8") as stream:
                previous = json.load(stream)
            previous_tests = {item.get("name"): item for item in previous.get("tests", [])}
            changes = []
            for item in report.get("tests", []):
                old = previous_tests.get(item.get("name"))
                if old and bool(old.get("passed")) != bool(item.get("passed")):
                    changes.append({"name": item.get("name"), "previous": bool(old.get("passed")), "current": bool(item.get("passed"))})
            report["comparison_to_previous"] = {
                "previous_created_utc": previous.get("created_utc"),
                "previous_passed": previous.get("passed"),
                "pass_state_changes": changes,
                "elapsed_delta_seconds": float(report.get("elapsed_seconds", 0.0)) - float(previous.get("elapsed_seconds", 0.0)),
            }
        except Exception:
            pass
    temporary = path + ".tmp"
    with open(temporary, "w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, ensure_ascii=False, sort_keys=True)
    os.replace(temporary, path)
    return path
