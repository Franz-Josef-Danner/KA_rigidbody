"""Self-contained Jolt regression tests that do not modify the open Blender scene."""

from __future__ import annotations

import copy
import json
import math
import os
import time
import tempfile
from typing import Any, Dict, List

from ..backends.jolt import JoltBackend, recommended_jolt_threads
from .determinism import compare_frames, frames_digest
from .cache import decode_direct_frame_block, write_cache, read_cache
from .coacd_bridge import coacd_status, decompose as coacd_decompose

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
        return {"name": "Bundled CoACD decomposition", "passed": False, "error": detail, "metrics": {}}
    parts = coacd_decompose(vertices, indices, {
        "threshold": 0.01,
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
    passed = 2 <= part_count <= 4 and all(count >= 4 for count in vertex_counts)
    return {
        "name": "Bundled CoACD decomposition",
        "passed": passed,
        "metrics": {"part_count": part_count, "vertex_counts": vertex_counts, "status": detail},
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


def _run_ccd(backend: JoltBackend) -> Dict[str, Any]:
    bodies = [
        _body("Wall", "STATIC", "BOX", (0.0, 0.0, 0.0), half_extents=(0.05, 1.0, 1.0), friction=0.5),
        _body("Projectile", "DYNAMIC", "SPHERE", (-2.0, 0.0, 0.0), radius=0.05, mass=0.02, velocity=(45.0, 0.0, 0.0), ccd=True),
    ]
    result = backend.bake(_payload("ccd", bodies, frames=30, gravity=(0.0, 0.0, 0.0), substeps=2))
    final_x = float(_final_location(result, "Projectile")[0])
    passed = final_x < 0.25
    return {"name": "CCD thin wall", "passed": passed, "metrics": {"final_x": final_x}}



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
        and int(totals.get("native_body_count", 0)) == 3
        and int(totals.get("compound_constraint_count", 0)) == 1
    )
    return {
        "name": "Compound Convex fixed cluster",
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
        _run_coacd_decomposition, _run_drop, _run_restitution, _run_stack, _run_friction, _run_ccd,
        _run_managed_ground_guard, _run_compound, _run_compound_convex_cluster,
        _run_dense_fracture_pile, _run_confirmed_hybrid_sleep,
        _run_high_detail_convex_hull, _run_irregular_mass_ratio_pile, _run_contact_buffer,
        _run_binary_cache_roundtrip, _run_production_binary_only, _run_independent_diagnostics,
        _run_diagnostic_log_filtering, _run_thread_heuristic,
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
        "suite_version": 13,
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
