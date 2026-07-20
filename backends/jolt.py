"""Native Jolt backend using the bundled Culverin CPython extension."""

from __future__ import annotations

import array
import math
import os
import time
from collections import deque
from dataclasses import dataclass

try:
    import numpy as _np
except Exception:  # Blender builds normally include NumPy; keep a safe fallback.
    _np = None
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from .base import BackendError, BackendStatus, PhysicsBackend, ProgressCallback
from .culverin_loader import BUNDLED_CULVERIN_VERSION, CulverinLoadError, culverin_status, load_culverin
from .native_bridge import NativeBridgeLoadError
from .native_jolt_adapter import load_native_jolt, resolve_bridge_path
from ..core.coordinates import (
    add_vec3,
    blender_quat_to_jolt,
    blender_vec_to_jolt,
    jolt_quat_to_blender,
    jolt_vec_to_blender,
    length_vec3,
    quat_conjugate_wxyz,
    quat_rotate_vector_wxyz,
    scale_vec3,
    subtract_vec3,
)
from ..core.diagnostics import write_diagnostic
from ..core.simulation_scene import solver_payload


def recommended_jolt_threads(dynamic_count: int, cpu_count: Optional[int] = None) -> int:
    """Choose a worker count that avoids oversubscribing small fracture scenes."""
    available = max(1, int(cpu_count if cpu_count is not None else (os.cpu_count() or 4)) - 1)
    count = max(0, int(dynamic_count))
    if count <= 32:
        target = 2
    elif count <= 750:
        target = 4
    elif count <= 3000:
        target = 6
    elif count <= 10000:
        target = 8
    else:
        target = 12
    return max(1, min(available, target))


def _quat_normalize_xyzw(q: Sequence[float]) -> Tuple[float, float, float, float]:
    x, y, z, w = map(float, q[:4])
    length = math.sqrt(x * x + y * y + z * z + w * w)
    if length <= 1.0e-20:
        return (0.0, 0.0, 0.0, 1.0)
    inverse = 1.0 / length
    return (x * inverse, y * inverse, z * inverse, w * inverse)


def _quat_conjugate_xyzw(q: Sequence[float]) -> Tuple[float, float, float, float]:
    x, y, z, w = map(float, q[:4])
    return (-x, -y, -z, w)


def _quat_multiply_xyzw(
    first: Sequence[float], second: Sequence[float]
) -> Tuple[float, float, float, float]:
    ax, ay, az, aw = map(float, first[:4])
    bx, by, bz, bw = map(float, second[:4])
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_rotate_xyzw(q: Sequence[float], value: Sequence[float]) -> Tuple[float, float, float]:
    normalized = _quat_normalize_xyzw(q)
    pure = (float(value[0]), float(value[1]), float(value[2]), 0.0)
    rotated = _quat_multiply_xyzw(
        _quat_multiply_xyzw(normalized, pure),
        _quat_conjugate_xyzw(normalized),
    )
    return (rotated[0], rotated[1], rotated[2])


def _cross_vec3(first: Sequence[float], second: Sequence[float]) -> Tuple[float, float, float]:
    ax, ay, az = map(float, first[:3])
    bx, by, bz = map(float, second[:3])
    return (ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx)


def _normalize_vec3(value: Sequence[float]) -> Tuple[float, float, float]:
    length = length_vec3(value)
    if length <= 1.0e-20:
        return (0.0, 1.0, 0.0)
    return tuple(float(component) / length for component in value[:3])


def _dot_vec3(first: Sequence[float], second: Sequence[float]) -> float:
    return sum(float(first[index]) * float(second[index]) for index in range(3))


def _safe_convex_sphere_cloud(
    points: Sequence[Sequence[float]],
    *,
    surface_fraction: float = 0.98,
    radius_fraction: float = 0.95,
    max_points: int = 32,
) -> List[Tuple[Tuple[float, float, float], float]]:
    """Return primitive spheres that are guaranteed to remain inside a convex hull.

    Culverin primitive compounds cannot contain convex children. Using the outer
    bounding box of a single-hull fallback can therefore penetrate nearby static
    geometry. This routine reconstructs the hull support planes and places a
    deterministic cloud of inward-offset spheres instead.
    """
    unique: List[Tuple[float, float, float]] = []
    seen = set()
    for value in points:
        point = tuple(float(component) for component in value[:3])
        key = tuple(round(component, 9) for component in point)
        if key in seen:
            continue
        seen.add(key)
        unique.append(point)
    if len(unique) < 4:
        return []
    if len(unique) > max_points:
        center_all = tuple(sum(point[axis] for point in unique) / len(unique) for axis in range(3))
        directions = [
            (x, y, z)
            for x in (-1.0, 0.0, 1.0)
            for y in (-1.0, 0.0, 1.0)
            for z in (-1.0, 0.0, 1.0)
            if x or y or z
        ]
        selected: List[Tuple[float, float, float]] = []
        for direction in directions:
            point = max(unique, key=lambda item: (_dot_vec3(item, direction), item))
            if point not in selected:
                selected.append(point)
            if len(selected) >= max_points:
                break
        while len(selected) < max_points:
            remaining = [point for point in unique if point not in selected]
            if not remaining:
                break
            point = max(
                remaining,
                key=lambda item: (
                    min(sum((item[axis] - other[axis]) ** 2 for axis in range(3)) for other in selected)
                    if selected else sum((item[axis] - center_all[axis]) ** 2 for axis in range(3)),
                    item,
                ),
            )
            selected.append(point)
        unique = selected

    scale = max(1.0, max(abs(component) for point in unique for component in point))
    tolerance = scale * 1.0e-7
    planes: Dict[Tuple[float, float, float, float], Tuple[Tuple[float, float, float], float]] = {}
    count = len(unique)
    for first in range(count - 2):
        a = unique[first]
        for second in range(first + 1, count - 1):
            ab = subtract_vec3(unique[second], a)
            for third in range(second + 1, count):
                ac = subtract_vec3(unique[third], a)
                normal = _cross_vec3(ab, ac)
                length = length_vec3(normal)
                if length <= tolerance:
                    continue
                normal = tuple(component / length for component in normal)
                distance = _dot_vec3(normal, a)
                minimum = float("inf")
                maximum = float("-inf")
                for point in unique:
                    signed = _dot_vec3(normal, point) - distance
                    minimum = min(minimum, signed)
                    maximum = max(maximum, signed)
                    if minimum < -tolerance and maximum > tolerance:
                        break
                if maximum <= tolerance:
                    pass
                elif minimum >= -tolerance:
                    normal = tuple(-component for component in normal)
                    distance = -distance
                else:
                    continue
                key = tuple(round(component, 7) for component in (*normal, distance))
                planes[key] = (normal, distance)
    if not planes:
        return []

    center = tuple(sum(point[axis] for point in unique) / len(unique) for axis in range(3))

    def inside_radius(point: Sequence[float]) -> float:
        return min(distance - _dot_vec3(normal, point) for normal, distance in planes.values())

    result: List[Tuple[Tuple[float, float, float], float]] = []
    central_radius = inside_radius(center) * radius_fraction
    if central_radius > 1.0e-5:
        result.append((center, central_radius))
    fraction = max(0.5, min(0.995, float(surface_fraction)))
    for vertex in unique:
        point = tuple(center[axis] + fraction * (vertex[axis] - center[axis]) for axis in range(3))
        radius = inside_radius(point) * radius_fraction
        if radius <= 1.0e-5:
            continue
        result.append((point, radius))
    return result



@dataclass
class _RuntimeCluster:
    stable_id: str
    handle: int
    members: List["_RuntimeBody"]
    local_positions_jolt: Dict[str, Tuple[float, float, float]]
    local_rotations_jolt: Dict[str, Tuple[float, float, float, float]]
    mass: float
    linear_damping: float = 0.0
    angular_damping: float = 0.0
    buffer_index: int = -1


@dataclass
class _RuntimeBody:
    stable_id: str
    name: str
    handle: int
    body_type: str
    scale: Tuple[float, float, float]
    input_location: Tuple[float, float, float]
    input_rotation: Tuple[float, float, float, float]
    shape_center: Tuple[float, float, float]
    com_offset_local: Tuple[float, float, float]
    linear_damping: float
    angular_damping: float
    radius: float
    feature_length: float
    mass: float
    ccd: bool
    collision_category: int = 1
    collision_mask: int = 0xFFFF
    handles: Tuple[int, ...] = ()
    constraint_handles: Tuple[int, ...] = ()
    buffer_index: int = -1
    low_motion_time: float = 0.0
    max_linear_speed: float = 0.0
    max_angular_speed: float = 0.0
    max_linear_speed_frame: int = 0
    max_angular_speed_frame: int = 0
    rest_position_jolt: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    rest_rotation_jolt: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    cluster: Optional[_RuntimeCluster] = None
    source_body: Optional[Dict[str, Any]] = None


@dataclass
class _RuntimeBond:
    stable_id: str
    handle: int
    body_a: _RuntimeBody
    body_b: _RuntimeBody
    anchor: Tuple[float, float, float]
    normal: Tuple[float, float, float]
    area: float
    break_force: float
    break_torque: float
    damage_accumulation: float
    damage: float = 0.0
    broken: bool = False
    broken_frame: int = 0
    broken_substep: int = 0
    peak_force: float = 0.0
    peak_torque: float = 0.0
    solver_bound: bool = False
    anchor_local_a_jolt: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    anchor_local_b_jolt: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    normal_local_a_jolt: Tuple[float, float, float] = (0.0, 1.0, 0.0)
    normal_local_b_jolt: Tuple[float, float, float] = (0.0, 1.0, 0.0)


class JoltBackend(PhysicsBackend):
    identifier = "JOLT"
    name = "Jolt Native"

    @classmethod
    def status(cls, preferences=None) -> BackendStatus:
        configured = str(getattr(preferences, "jolt_bridge_path", "") or "") if preferences else ""
        bridge_path = resolve_bridge_path(configured)
        if bridge_path:
            try:
                module = load_native_jolt(bridge_path)
                return BackendStatus(
                    cls.identifier, cls.name, True, False,
                    f"{module.__version__} loaded. True StaticCompoundShape Compound Convex bodies are active. Adapter status: beta.",
                )
            except NativeBridgeLoadError as exc:
                bridge_error = str(exc)
        else:
            bridge_error = "No compiled ABI-v2 bridge found"
        available, detail = culverin_status()
        if available:
            detail += (
                " Convex hulls, primitive compounds, static triangle meshes, inertia, rotation, CCD and sleeping are active. "
                f"True convex compounds are unavailable ({bridge_error}); dynamic Compound Convex uses the complete single convex hull fallback."
            )
        return BackendStatus(cls.identifier, cls.name, available, False, detail + (" Adapter status: beta." if available else ""))

    def bake(self, scene_payload: Dict, progress: ProgressCallback = None) -> Dict:
        scene_payload = solver_payload(scene_payload)
        all_constraint_payload = [
            dict(item) for item in scene_payload.get("constraints", []) or []
            if bool(item.get("enabled", True))
            and str(item.get("constraint_type", "")) in {"BREAKABLE_FIXED", "FIXED", "DISTANCE"}
        ]
        bond_constraint_payload = [
            item for item in all_constraint_payload
            if str(item.get("constraint_type", "")) in {"BREAKABLE_FIXED", "FIXED"}
        ]
        distance_constraint_payload = [
            item for item in all_constraint_payload
            if str(item.get("constraint_type", "")) == "DISTANCE"
        ]
        bridge_path = resolve_bridge_path(str(scene_payload.get("native_jolt_bridge_path", "") or ""))
        runtime_error = None
        # ABI-v2 currently has no external constraint ABI. Constraint scenes
        # deliberately use bundled Culverin, which exposes Fixed and Distance joints.
        if bridge_path and not all_constraint_payload:
            try:
                culverin = load_native_jolt(bridge_path)
            except NativeBridgeLoadError as exc:
                runtime_error = str(exc)
                culverin = None
        else:
            culverin = None
            if bridge_path and all_constraint_payload:
                runtime_error = "ABI-v2 external constraints unavailable; using Culverin for authored constraints"
        if culverin is None:
            try:
                culverin = load_culverin()
            except CulverinLoadError as exc:
                detail = f" Native bridge failed: {runtime_error}." if runtime_error else ""
                raise BackendError(str(exc) + detail) from exc
        native_compound_convex = bool(getattr(culverin, "NATIVE_COMPOUND_CONVEX", False))
        native_bridge_active = bool(getattr(culverin, "NATIVE_BRIDGE", False))

        diagnostic_settings = scene_payload.get("diagnostics", {})
        stability_settings = scene_payload.get("stability", {}) or {}
        bond_stability_mode = str(stability_settings.get("bond_stability_mode", "RIGID")).upper()
        if bond_stability_mode not in {"RIGID", "FLEXIBLE"}:
            bond_stability_mode = "RIGID"
        # Blender bakes always keep the direct binary frame stream. The hidden
        # override exists only for regression fixtures that need dictionaries.
        store_python_frames = bool(scene_payload.get("store_python_frames", False))
        log_enabled = bool(diagnostic_settings.get("enabled", False))
        log_path = diagnostic_settings.get("path")
        force_contacts = bool(diagnostic_settings.get("force_contacts", False))
        bond_contact_monitoring = bool(bond_constraint_payload)
        contact_diagnostics = bool(diagnostic_settings.get("contacts", False) or force_contacts or bond_contact_monitoring)
        contact_logging = bool(diagnostic_settings.get("log_contacts", diagnostic_settings.get("contacts", False)))
        payload_diagnostics = bool(diagnostic_settings.get("payload", False))
        side_stick_diagnostics = bool(diagnostic_settings.get("side_stick", False)) and contact_diagnostics
        side_stick_logging = bool(diagnostic_settings.get("log_side_stick", False)) and side_stick_diagnostics
        # Per-frame records belong to the explicitly selected contact diagnostics.
        # Log Output alone and the internal compound guard must not enable them.
        frame_logging = bool(log_enabled and contact_logging)
        track_body_peaks = bool(payload_diagnostics)
        side_stick_min_frames = max(1, int(diagnostic_settings.get("side_stick_min_frames", 8)))
        side_stick_normal_z = min(1.0, max(0.0, float(diagnostic_settings.get("side_stick_normal_z", 0.35))))
        side_stick_slide_speed = max(0.0, float(diagnostic_settings.get("side_stick_slide_speed", 0.05)))

        def log(event: str, *, level: str = "INFO", **data: Any) -> None:
            write_diagnostic(log_enabled, log_path, "JOLT_BACKEND", event, level=level, data=data)

        bodies = list(scene_payload.get("bodies", []))
        if not bodies:
            raise BackendError("The Jolt scene contains no bodies.")
        body_shapes = {
            str(body.get("name")): str(body.get("collision_shape", "UNKNOWN"))
            for body in bodies
        }

        started = time.perf_counter()
        dynamic_count = sum(body.get("body_type") == "DYNAMIC" for body in bodies)
        body_count = len(bodies)
        # Both the ABI-v2 true convex compound and the Culverin primitive
        # compound fallback use exactly one native body per logical object.
        native_body_count = body_count
        native_dynamic_count = dynamic_count
        requested_threads = int(scene_payload.get("jolt_threads", 0))
        configured_threads = int(scene_payload.get("jolt_threads_requested", requested_threads))
        reproducibility_mode = str(scene_payload.get("reproducibility_mode", "REPEATABLE")).upper()
        deterministic_mode = reproducibility_mode != "PERFORMANCE"
        automatic_threads = recommended_jolt_threads(native_dynamic_count)
        thread_count = 1 if reproducibility_mode == "STRICT" else (requested_threads if requested_threads > 0 else automatic_threads)

        world_settings = {
            "gravity": blender_vec_to_jolt(scene_payload.get("gravity", (0.0, 0.0, -9.81))),
            "penetration_slop": max(1.0e-5, float(scene_payload.get("penetration_slop", 0.02))),
            "max_bodies": max(1024, native_body_count + 128),
            # Compound colliders remain one native body, so contact capacity
            # scales with logical bodies instead of decomposed child count.
            "max_pairs": max(65536, min(4_000_000, native_body_count * 512)),
            "max_contact_constraints": max(32768, min(2_000_000, native_body_count * 256)),
            "temp_allocator_size": max(32 * 1024 * 1024, min(1024 * 1024 * 1024, native_body_count * 96 * 1024)),
            "num_threads": max(1, min(64, thread_count)),
        }

        try:
            world = culverin.PhysicsWorld(settings=world_settings)
        except Exception as exc:
            raise BackendError(f"Jolt world creation failed: {exc}") from exc

        handle_to_name: Dict[int, Any] = {}
        runtimes: List[_RuntimeBody] = []
        shape_statistics: Dict[str, int] = {}
        creation_warnings: List[str] = []
        compound_constraint_count = 0
        runtime_bonds: List[_RuntimeBond] = []
        bond_constraint_stats: Dict[str, Any] = {
            "graph_bonds": 0,
            "selected_constraints": 0,
            "created_constraints": 0,
            "constraint_limit": 256,
            "backbone_edges": 0,
            "reinforcement_edges": 0,
        }
        distance_constraint_handles: List[int] = []
        distance_constraint_stats: Dict[str, int] = {
            "requested": len(distance_constraint_payload),
            "created": 0,
            "omitted": 0,
            "constraint_limit": 256,
        }
        distance_constraint_rebind_required = False
        bond_cluster_stats: Dict[str, int] = {
            "clusters": 0,
            "clustered_bodies": 0,
            "singletons": dynamic_count,
            "native_dynamic_bodies": dynamic_count,
            "unsupported_static_bonds": 0,
            "static_anchor_bonds": 0,
            "static_anchor_constraints": 0,
            "static_anchor_omitted": 0,
            "static_anchor_pairs": 0,
        }
        bond_collision_filter_stats: Dict[str, Any] = {
            "components": 0,
            "filtered_components": 0,
            "filtered_bodies": 0,
            "overflow_components": 0,
            "available_category_bits": 0,
        }

        log(
            "INITIALIZING",
            scene=scene_payload.get("scene_name"),
            signature=scene_payload.get("signature"),
            culverin_version=str(getattr(culverin, "__version__", BUNDLED_CULVERIN_VERSION)),
            native_bridge=bool(getattr(culverin, "NATIVE_BRIDGE", False)),
            native_compound_convex=native_compound_convex,
            body_count=body_count,
            dynamic_bodies=dynamic_count,
            native_body_count=native_body_count,
            native_dynamic_bodies=native_dynamic_count,
            world_settings=world_settings,
            deterministic_mode=deterministic_mode,
            reproducibility_mode=reproducibility_mode,
            configured_threads=configured_threads,
            effective_threads=thread_count,
            binary_frame_cache=not store_python_frames,
            store_python_frames=store_python_frames,
            contact_diagnostics=contact_diagnostics,
            contact_logging=contact_logging,
            payload_diagnostics=payload_diagnostics,
            breakable_bond_count=len(bond_constraint_payload),
            distance_constraint_count=len(distance_constraint_payload),
            bond_contact_monitoring=bond_contact_monitoring,
            native_bridge_constraint_fallback=runtime_error if all_constraint_payload else None,
            substeps=int(scene_payload.get("substeps", 1)),
            solver_iterations_requested=int(scene_payload.get("solver_iterations", 8)),
            solver_iterations_note="Culverin 0.13.2 does not expose Jolt velocity/position iteration counts; native defaults are used.",
            ccd_policy="JOLT_LINEAR_CAST_ARMED_PER_REQUESTED_DYNAMIC_BODY",
            rigid_cluster_collider="COMPLETE_OUTER_CONVEX_HULL",
        )

        try:
            for index, body in enumerate(bodies):
                runtime = self._create_body(culverin, world, body, index, creation_warnings)
                runtimes.append(runtime)
                for native_handle in self._runtime_handles(runtime):
                    handle_to_name[int(native_handle)] = runtime.name
                shape = (
                    "PLANE"
                    if bool(body.get("managed_ground", False)) or str(body.get("name", "")).startswith("KA_Physics_Ground")
                    else str(body.get("collision_shape", "BOX"))
                )
                shape_statistics[shape] = shape_statistics.get(shape, 0) + 1

            # Flush queued body creation and make state buffers available.
            world.step(0.0)
            for runtime in runtimes:
                try:
                    runtime.buffer_index = int(world.get_index(runtime.handle))
                except (TypeError, ValueError):
                    runtime.buffer_index = -1
            self._calibrate_com_offsets(world, runtimes)
            self._capture_rest_transforms(world, runtimes)
            compound_constraint_count = sum(len(runtime.constraint_handles) for runtime in runtimes)
            runtime_bonds, bond_constraint_stats = self._create_breakable_bonds(
                culverin,
                world,
                bond_constraint_payload,
                runtimes,
                creation_warnings,
                create_native_constraints=bond_stability_mode != "RIGID",
            )
            bonded_dynamic_ids = {
                endpoint.stable_id
                for bond in runtime_bonds
                for endpoint in (bond.body_a, bond.body_b)
                if endpoint.body_type == "DYNAMIC"
            }
            distance_constraint_rebind_required = any(
                str(record.get("body_a", "")) in bonded_dynamic_ids
                or str(record.get("body_b", "")) in bonded_dynamic_ids
                for record in distance_constraint_payload
            )
            if runtime_bonds and bond_stability_mode == "RIGID":
                bond_cluster_stats = self._rebuild_rigid_bond_clusters(
                    culverin, world, runtimes, runtime_bonds, handle_to_name, creation_warnings,
                    allow_initial_sleep=bool(scene_payload.get("sleep_enabled", True)),
                )
                native_dynamic_count = int(bond_cluster_stats.get("native_dynamic_bodies", native_dynamic_count))
                native_body_count = sum(runtime.body_type != "DYNAMIC" for runtime in runtimes) + native_dynamic_count
                bond_constraint_stats["created_constraints"] = int(
                    bond_cluster_stats.get("static_anchor_constraints", 0)
                )
                bond_constraint_stats["selected_constraints"] = int(
                    bond_cluster_stats.get("static_anchor_constraints", 0)
                )
            elif runtime_bonds:
                bond_collision_filter_stats = self._apply_bond_island_collision_filters(
                    world, runtimes, runtime_bonds
                )
                world.step(0.0)
            distance_constraint_handles, distance_constraint_stats = self._create_distance_constraints(
                culverin,
                world,
                distance_constraint_payload,
                runtimes,
                creation_warnings,
                constraint_limit=self._distance_constraint_budget(
                    compound_constraint_count,
                    bond_stability_mode,
                    bond_constraint_stats,
                    bond_cluster_stats,
                ),
            )
        except Exception as exc:
            raise BackendError(f"Jolt body creation failed: {exc}") from exc

        managed_ground_levels = self._managed_ground_levels(runtimes)
        culverin_ground_compensation = bool(managed_ground_levels) and not native_bridge_active

        frame_start = int(scene_payload["frame_start"])
        frame_end = int(scene_payload["frame_end"])
        fps = max(1.0e-6, float(scene_payload["fps"]))
        frame_dt = 1.0 / fps
        substeps = max(1, int(scene_payload.get("substeps", 1)))
        adaptive_substeps = bool(scene_payload.get("adaptive_substeps", False))
        minimum_substeps = max(1, min(substeps, int(scene_payload.get("minimum_substeps", substeps))))
        step_dt = frame_dt / substeps
        early_sleep_termination = bool(scene_payload.get("early_sleep_termination", True))
        early_sleep_frames = max(1, int(scene_payload.get("early_sleep_frames", 3)))
        sleep_confirmation_frames = max(
            early_sleep_frames, side_stick_min_frames if side_stick_diagnostics else 1
        )
        sleep_enabled = bool(scene_payload.get("sleep_enabled", True))
        sleep_mode = str(scene_payload.get("sleep_mode", "NATIVE")).upper()
        if sleep_mode not in {"NATIVE", "HYBRID", "CUSTOM"}:
            sleep_mode = "NATIVE"
        sleep_linear_threshold = max(0.0, float(scene_payload.get("sleep_linear_threshold", 0.05)))
        sleep_angular_threshold = max(0.0, float(scene_payload.get("sleep_angular_threshold", 0.1)))
        sleep_time = max(0.0, float(scene_payload.get("sleep_time", 0.5)))

        runtime_by_name = {runtime.name: runtime for runtime in runtimes}
        initial_snapshot, initial_values = self._input_snapshot_and_values(runtimes)
        frames: Dict[str, Dict] = {str(frame_start): initial_snapshot} if store_python_frames else {}
        binary_frame_numbers: List[str] = [str(frame_start)]
        binary_values = array.array("f")
        binary_values.extend(initial_values)
        binary_body_names = [runtime.name for runtime in runtimes]
        binary_body_scales = {runtime.name: list(runtime.scale) for runtime in runtimes}
        total_frames = max(1, frame_end - frame_start)
        if progress:
            progress(0, total_frames)

        totals: Dict[str, Any] = {
            "executed_substeps": 0,
            "logical_body_count": body_count,
            "native_body_count": native_body_count,
            "native_dynamic_body_count": native_dynamic_count,
            "compound_constraint_count": compound_constraint_count,
            "distance_constraint_requested": int(distance_constraint_stats.get("requested", 0)),
            "distance_constraint_count": int(distance_constraint_stats.get("created", 0)),
            "distance_constraint_omitted": int(distance_constraint_stats.get("omitted", 0)),
            "distance_constraint_limit": int(distance_constraint_stats.get("constraint_limit", 256)),
            "distance_constraint_rebinds": 0,
            "distance_constraint_destroyed_for_rebind": 0,
            "distance_constraint_rebind_required": bool(
                distance_constraint_rebind_required
            ),
            "bond_graph_count": len(runtime_bonds),
            "bond_constraint_count": int(bond_constraint_stats.get("created_constraints", 0)),
            "bond_constraint_limit": int(bond_constraint_stats.get("constraint_limit", 256)),
            "bond_backbone_edges": int(bond_constraint_stats.get("backbone_edges", 0)),
            "bond_reinforcement_edges": int(bond_constraint_stats.get("reinforcement_edges", 0)),
            "bond_rigid_stabilization": bond_stability_mode == "RIGID",
            "bond_stabilization_strategy": (
                "RIGID_COMPOUND_ISLANDS_WITH_STATIC_ANCHORS"
                if bond_stability_mode == "RIGID" else "NATIVE_FIXED_ONLY"
            ),
            "bond_cluster_count": int(bond_cluster_stats.get("clusters", 0)),
            "bond_clustered_bodies": int(bond_cluster_stats.get("clustered_bodies", 0)),
            "bond_cluster_singletons": int(bond_cluster_stats.get("singletons", 0)),
            "bond_preserved_external_dynamic_bodies": int(
                bond_cluster_stats.get("preserved_external_dynamic_bodies", 0)
            ),
            "bond_static_anchor_bonds": int(bond_cluster_stats.get("static_anchor_bonds", 0)),
            "bond_static_anchor_constraints": int(bond_cluster_stats.get("static_anchor_constraints", 0)),
            "bond_static_anchor_omitted": int(bond_cluster_stats.get("static_anchor_omitted", 0)),
            "bond_static_anchor_pairs": int(bond_cluster_stats.get("static_anchor_pairs", 0)),
            "bond_static_anchor_collision_filter_requested_pairs": int(
                bond_cluster_stats.get("static_anchor_collision_filter_requested_pairs", 0)
            ),
            "bond_static_anchor_initial_overlap_pairs": int(
                bond_cluster_stats.get("static_anchor_initial_overlap_pairs", 0)
            ),
            "bond_static_anchor_collision_filter_pairs": int(
                bond_cluster_stats.get("static_anchor_collision_filter_pairs", 0)
            ),
            "bond_static_anchor_collision_filter_dynamic_actors": int(
                bond_cluster_stats.get("static_anchor_collision_filter_dynamic_actors", 0)
            ),
            "bond_static_anchor_collision_filter_overflow": int(
                bond_cluster_stats.get("static_anchor_collision_filter_overflow", 0)
            ),
            "bond_static_anchor_collision_filter_rebuilds": (
                1 if runtime_bonds and bond_stability_mode == "RIGID" else 0
            ),
            "bond_cluster_rebuilds": 1 if runtime_bonds and bond_stability_mode == "RIGID" else 0,
            "bond_supported_cluster_deactivations": int(
                bond_cluster_stats.get("initially_supported_clusters", 0)
            ),
            "bond_internal_collision_filtering": bool(runtime_bonds and bond_stability_mode != "RIGID"),
            "bond_collision_filter_components": int(bond_collision_filter_stats.get("components", 0)),
            "bond_collision_filter_filtered_components": int(bond_collision_filter_stats.get("filtered_components", 0)),
            "bond_collision_filter_bodies": int(bond_collision_filter_stats.get("filtered_bodies", 0)),
            "bond_collision_filter_overflow_components": int(bond_collision_filter_stats.get("overflow_components", 0)),
            "bond_collision_filter_rebuilds": 1 if runtime_bonds and bond_stability_mode != "RIGID" else 0,
            "bond_projection_passes": 0,
            "bond_projection_bodies": 0,
            "bond_projection_max_correction": 0.0,
            "bond_island_sleep_requests": 0,
            "bond_island_sleep_confirmed": 0,
            "bond_break_events": 0,
            "bond_contact_monitoring": bond_contact_monitoring,
            "contact_collection_enabled": contact_diagnostics,
            "contact_collection_reason": (
                "breakable_bond_monitoring"
                if bond_contact_monitoring
                else diagnostic_settings.get("contact_reason", "detailed_contact_diagnostics" if contact_diagnostics else "disabled")
            ),
            "contact_events": 0,
            "contact_added": 0,
            "contact_persisted": 0,
            "contact_removed": 0,
            "max_contact_impulse": 0.0,
            "max_contact_pair": None,
            "penetration_depth_available": False,
            "adaptive_substeps": adaptive_substeps,
            "minimum_substeps": minimum_substeps,
            "maximum_substeps": substeps,
            "minimum_executed_substeps_per_frame": None,
            "maximum_executed_substeps_per_frame": 0,
            "sleep_deactivation_requests": 0,
            "sleep_deactivation_confirmed": 0,
            "sleep_deactivation_rejected": 0,
            "early_sleep_frame": None,
            "visual_ground_compensation_enabled": culverin_ground_compensation,
            "visual_ground_compensation_ground_levels": list(managed_ground_levels),
            "visual_ground_compensation_frames": 0,
            "visual_ground_compensation_bodies": 0,
            "visual_ground_compensation_max_correction": 0.0,
        }
        bond_events: List[Dict[str, Any]] = []
        pair_stats: Dict[Tuple[str, str], Dict[str, Any]] = {}
        body_contact_peaks: Dict[str, Dict[str, Any]] = (
            {
                runtime.name: {"max_impulse": 0.0, "frame": 0, "other": None}
                for runtime in runtimes
            }
            if contact_diagnostics
            else {}
        )

        log(
            "INITIALIZED",
            frame_start=frame_start,
            frame_end=frame_end,
            fps=fps,
            frame_dt=frame_dt,
            substeps=substeps,
            adaptive_substeps=adaptive_substeps,
            minimum_substeps=minimum_substeps,
            step_dt=step_dt,
            early_sleep_termination=early_sleep_termination,
            early_sleep_frames=early_sleep_frames,
            effective_sleep_confirmation_frames=sleep_confirmation_frames,
            body_count=int(getattr(world, "count", body_count)),
            shape_count=int(getattr(world, "shape_count", 0)),
            shapes=shape_statistics,
            creation_warnings=creation_warnings,
            breakable_bonds_created=len(runtime_bonds),
            breakable_constraints_created=int(bond_constraint_stats.get("created_constraints", 0)),
            breakable_constraint_limit=int(bond_constraint_stats.get("constraint_limit", 256)),
            distance_constraints_requested=int(distance_constraint_stats.get("requested", 0)),
            distance_constraints_created=int(distance_constraint_stats.get("created", 0)),
            distance_constraints_omitted=int(distance_constraint_stats.get("omitted", 0)),
            distance_constraint_rebind_required=bool(distance_constraint_rebind_required),
            breakable_backbone_edges=int(bond_constraint_stats.get("backbone_edges", 0)),
            breakable_reinforcement_edges=int(bond_constraint_stats.get("reinforcement_edges", 0)),
            bond_cluster_count=int(bond_cluster_stats.get("clusters", 0)),
            bond_clustered_bodies=int(bond_cluster_stats.get("clustered_bodies", 0)),
            bond_preserved_external_dynamic_bodies=int(
                bond_cluster_stats.get("preserved_external_dynamic_bodies", 0)
            ),
            rigid_static_anchor_bonds=int(bond_cluster_stats.get("static_anchor_bonds", 0)),
            rigid_static_anchor_constraints=int(bond_cluster_stats.get("static_anchor_constraints", 0)),
            rigid_static_anchor_omitted=int(bond_cluster_stats.get("static_anchor_omitted", 0)),
            rigid_static_anchor_pairs=int(bond_cluster_stats.get("static_anchor_pairs", 0)),
            rigid_static_anchor_initial_overlap_pairs=int(
                bond_cluster_stats.get("static_anchor_initial_overlap_pairs", 0)
            ),
            rigid_static_anchor_collision_filter_pairs=int(
                bond_cluster_stats.get("static_anchor_collision_filter_pairs", 0)
            ),
            rigid_static_anchor_collision_filter_dynamic_actors=int(
                bond_cluster_stats.get("static_anchor_collision_filter_dynamic_actors", 0)
            ),
            rigid_static_anchor_collision_filter_overflow=int(
                bond_cluster_stats.get("static_anchor_collision_filter_overflow", 0)
            ),
            bond_supported_cluster_deactivations=int(
                bond_cluster_stats.get("initially_supported_clusters", 0)
            ),
            native_dynamic_body_count=native_dynamic_count,
            bond_stabilization_strategy=(
                "RIGID_COMPOUND_ISLANDS_WITH_STATIC_ANCHORS"
                if bond_stability_mode == "RIGID" else "NATIVE_FIXED_ONLY"
            ),
            bond_stability_mode=bond_stability_mode,
            breakable_bonds_requested=len(bond_constraint_payload),
            contact_diagnostics=contact_diagnostics,
            sleeping_mode=sleep_mode,
            initial_state=self._state_diagnostics(world, runtimes, frame_start) if frame_logging else None,
        )

        sleeping_streak = 0
        bond_island_sleep_timers: Dict[Tuple[str, ...], float] = {}
        gravity_magnitude = length_vec3(scene_payload.get("gravity", (0.0, 0.0, -9.81)))
        minimum_initial_feature = min(
            (runtime.feature_length for runtime in runtimes if runtime.body_type == "DYNAMIC"),
            default=0.05,
        )
        last_state: Dict[str, Any] = {
            "frame": frame_start,
            "dynamic_bodies": dynamic_count,
            "static_bodies": sum(runtime.body_type == "STATIC" for runtime in runtimes),
            "kinematic_bodies": sum(runtime.body_type == "KINEMATIC" for runtime in runtimes),
            "active_bodies": dynamic_count,
            "sleeping_bodies": 0,
            "max_linear_speed": 0.0,
            "max_linear_speed_body": None,
            "max_angular_speed": 0.0,
            "max_angular_speed_body": None,
            "max_angular_surface_speed": 0.0,
            "minimum_feature_length": float(minimum_initial_feature),
            "minimum_feature_radius": float(minimum_initial_feature),
            "active_ccd": any(runtime.ccd for runtime in runtimes if runtime.body_type == "DYNAMIC"),
            "motion_energy_proxy": 0.0,
            "deactivation_requests": 0,
            "deactivation_confirmed": 0,
            "deactivation_rejected": 0,
        }
        energy_tail = deque(maxlen=16)
        bulk_frame_sample_seconds = 0.0
        for offset, frame in enumerate(range(frame_start + 1, frame_end + 1), start=1):
            frame_substeps = (
                self._adaptive_substep_count_from_motion(
                    last_state, frame_dt, minimum_substeps, substeps, gravity_magnitude
                )
                if adaptive_substeps
                else substeps
            )
            frame_step_dt = frame_dt / frame_substeps
            previous_minimum = totals.get("minimum_executed_substeps_per_frame")
            totals["minimum_executed_substeps_per_frame"] = (
                frame_substeps if previous_minimum is None else min(int(previous_minimum), frame_substeps)
            )
            totals["maximum_executed_substeps_per_frame"] = max(
                int(totals.get("maximum_executed_substeps_per_frame", 0)), frame_substeps
            )
            frame_contacts = {
                "contact_events": 0,
                "contact_added": 0,
                "contact_persisted": 0,
                "contact_removed": 0,
                "max_contact_impulse": 0.0,
                "max_contact_pair": None,
            }
            frame_pair_contacts: Dict[Tuple[str, str], Dict[str, Any]] = {}

            for _substep in range(1, frame_substeps + 1):
                if not sleep_enabled:
                    for runtime in runtimes:
                        if runtime.body_type == "DYNAMIC":
                            for native_handle in self._runtime_handles(runtime):
                                world.activate(native_handle)
                pre_step_motion = (
                    self._capture_pre_step_motion(world, runtimes)
                    if runtime_bonds else {}
                )
                try:
                    world.step(frame_step_dt)
                except Exception as exc:
                    raise BackendError(f"Jolt simulation failed at frame {frame}: {exc}") from exc
                totals["executed_substeps"] += 1
                if runtime_bonds:
                    body_contacts = self._collect_contacts(
                        culverin,
                        world,
                        handle_to_name,
                        runtime_by_name,
                        pair_stats,
                        frame_pair_contacts,
                        frame_contacts,
                        totals,
                        body_contact_peaks,
                        frame,
                    )
                    broken = self._evaluate_breakable_bonds(
                        world,
                        runtime_bonds,
                        body_contacts,
                        runtime_by_name,
                        pre_step_motion,
                        frame_step_dt,
                        frame,
                        _substep,
                        log,
                    )
                    if broken:
                        bond_events.extend(broken)
                        totals["bond_break_events"] += len(broken)
                        if bond_stability_mode == "RIGID":
                            destroyed_distance_constraints = 0
                            if distance_constraint_payload and distance_constraint_rebind_required:
                                destroyed_distance_constraints = self._destroy_constraint_handles(
                                    world, distance_constraint_handles, creation_warnings
                                )
                                distance_constraint_handles = []
                            bond_cluster_stats = self._rebuild_rigid_bond_clusters(
                                culverin, world, runtimes, runtime_bonds, handle_to_name, creation_warnings,
                                allow_initial_sleep=bool(scene_payload.get("sleep_enabled", True)),
                            )
                            if distance_constraint_payload and distance_constraint_rebind_required:
                                distance_constraint_handles, distance_constraint_stats = self._create_distance_constraints(
                                    culverin,
                                    world,
                                    distance_constraint_payload,
                                    runtimes,
                                    creation_warnings,
                                    constraint_limit=self._distance_constraint_budget(
                                        compound_constraint_count,
                                        bond_stability_mode,
                                        bond_constraint_stats,
                                        bond_cluster_stats,
                                    ),
                                )
                                totals["distance_constraint_rebinds"] += 1
                                totals["distance_constraint_destroyed_for_rebind"] += int(
                                    destroyed_distance_constraints
                                )
                                totals["distance_constraint_requested"] = int(
                                    distance_constraint_stats.get("requested", 0)
                                )
                                totals["distance_constraint_count"] = int(
                                    distance_constraint_stats.get("created", 0)
                                )
                                totals["distance_constraint_omitted"] = int(
                                    distance_constraint_stats.get("omitted", 0)
                                )
                                totals["distance_constraint_limit"] = int(
                                    distance_constraint_stats.get("constraint_limit", 256)
                                )
                                log(
                                    "DISTANCE_CONSTRAINTS_REBOUND",
                                    frame=frame,
                                    substep=_substep,
                                    destroyed=destroyed_distance_constraints,
                                    recreated=int(distance_constraint_stats.get("created", 0)),
                                    omitted=int(distance_constraint_stats.get("omitted", 0)),
                                    preserved_external_dynamic_bodies=int(
                                        bond_cluster_stats.get("preserved_external_dynamic_bodies", 0)
                                    ),
                                )
                            totals["bond_cluster_count"] = int(bond_cluster_stats.get("clusters", 0))
                            totals["bond_clustered_bodies"] = int(bond_cluster_stats.get("clustered_bodies", 0))
                            totals["bond_cluster_singletons"] = int(bond_cluster_stats.get("singletons", 0))
                            totals["bond_preserved_external_dynamic_bodies"] = int(
                                bond_cluster_stats.get("preserved_external_dynamic_bodies", 0)
                            )
                            totals["bond_static_anchor_bonds"] = int(bond_cluster_stats.get("static_anchor_bonds", 0))
                            totals["bond_static_anchor_constraints"] = int(bond_cluster_stats.get("static_anchor_constraints", 0))
                            totals["bond_static_anchor_omitted"] = int(bond_cluster_stats.get("static_anchor_omitted", 0))
                            totals["bond_static_anchor_pairs"] = int(bond_cluster_stats.get("static_anchor_pairs", 0))
                            totals["bond_static_anchor_collision_filter_requested_pairs"] = int(
                                bond_cluster_stats.get("static_anchor_collision_filter_requested_pairs", 0)
                            )
                            totals["bond_static_anchor_initial_overlap_pairs"] = int(
                                bond_cluster_stats.get("static_anchor_initial_overlap_pairs", 0)
                            )
                            totals["bond_static_anchor_collision_filter_pairs"] = int(
                                bond_cluster_stats.get("static_anchor_collision_filter_pairs", 0)
                            )
                            totals["bond_static_anchor_collision_filter_dynamic_actors"] = int(
                                bond_cluster_stats.get("static_anchor_collision_filter_dynamic_actors", 0)
                            )
                            totals["bond_static_anchor_collision_filter_overflow"] = int(
                                bond_cluster_stats.get("static_anchor_collision_filter_overflow", 0)
                            )
                            totals["bond_static_anchor_collision_filter_rebuilds"] += 1
                            totals["bond_constraint_count"] = int(bond_cluster_stats.get("static_anchor_constraints", 0))
                            totals["native_dynamic_body_count"] = int(
                                bond_cluster_stats.get("native_dynamic_bodies", totals.get("native_dynamic_body_count", 0))
                            )
                            totals["native_body_count"] = sum(
                                runtime.body_type != "DYNAMIC" for runtime in runtimes
                            ) + int(totals["native_dynamic_body_count"])
                            totals["bond_cluster_rebuilds"] += 1
                            totals["bond_supported_cluster_deactivations"] += int(
                                bond_cluster_stats.get("initially_supported_clusters", 0)
                            )
                        else:
                            bond_collision_filter_stats = self._apply_bond_island_collision_filters(
                                world, runtimes, runtime_bonds
                            )
                            totals["bond_collision_filter_components"] = int(
                                bond_collision_filter_stats.get("components", 0)
                            )
                            totals["bond_collision_filter_filtered_components"] = int(
                                bond_collision_filter_stats.get("filtered_components", 0)
                            )
                            totals["bond_collision_filter_bodies"] = int(
                                bond_collision_filter_stats.get("filtered_bodies", 0)
                            )
                            totals["bond_collision_filter_overflow_components"] = int(
                                bond_collision_filter_stats.get("overflow_components", 0)
                            )
                            totals["bond_collision_filter_rebuilds"] += 1


            # Without breakable bonds, contacts can remain batched once per
            # rendered frame. Bond scenes drain them per substep so a connection
            # can be released before the next solver step.
            if contact_diagnostics and not runtime_bonds:
                self._collect_contacts(
                    culverin,
                    world,
                    handle_to_name,
                    runtime_by_name,
                    pair_stats,
                    frame_pair_contacts,
                    frame_contacts,
                    totals,
                    body_contact_peaks,
                    frame,
                )

            if side_stick_diagnostics:
                self._finalize_side_stick_frame(
                    pair_stats,
                    frame_pair_contacts,
                    frame,
                    side_stick_normal_z,
                    side_stick_slide_speed,
                )

            if runtime_bonds and bond_stability_mode == "RIGID" and sleep_enabled and not any(
                runtime.cluster is not None for runtime in runtimes
            ):
                island_sleep = self._apply_bond_island_sleep(
                    world, runtimes, runtime_bonds, frame_pair_contacts,
                    bond_island_sleep_timers, frame_dt,
                    sleep_linear_threshold, sleep_angular_threshold, sleep_time,
                )
                totals["bond_island_sleep_requests"] += int(island_sleep.get("requests", 0))
                totals["bond_island_sleep_confirmed"] += int(island_sleep.get("confirmed", 0))
                totals["bond_island_sleep_linear_speed"] = float(island_sleep.get("linear_speed", 0.0))
                totals["bond_island_sleep_angular_speed"] = float(island_sleep.get("angular_speed", 0.0))
                totals["bond_island_sleep_timer"] = float(island_sleep.get("timer", 0.0))
                totals["bond_island_sleep_dynamic_external"] = bool(island_sleep.get("dynamic_external", False))

            sample_started = time.perf_counter()
            state = self._apply_damping_and_sleep(
                world,
                runtimes,
                frame,
                frame_dt,
                sleep_enabled,
                sleep_mode,
                sleep_linear_threshold,
                sleep_angular_threshold,
                sleep_time,
                build_snapshot=store_python_frames,
                track_body_peaks=track_body_peaks,
            )
            bulk_frame_sample_seconds += time.perf_counter() - sample_started
            snapshot = state.pop("_snapshot")
            frame_values = state.pop("_frame_values")
            ground_compensation = {
                "corrected_groups": 0.0,
                "corrected_bodies": 0.0,
                "max_correction": 0.0,
            }
            if culverin_ground_compensation:
                ground_compensation = self._apply_culverin_ground_contact_compensation(
                    runtimes, snapshot, frame_values, managed_ground_levels
                )
                if ground_compensation["corrected_groups"] > 0.0:
                    totals["visual_ground_compensation_frames"] += 1
                    totals["visual_ground_compensation_bodies"] += int(
                        ground_compensation["corrected_bodies"]
                    )
                    totals["visual_ground_compensation_max_correction"] = max(
                        float(totals["visual_ground_compensation_max_correction"]),
                        float(ground_compensation["max_correction"]),
                    )
            last_state = state
            energy_tail.append(float(state.get("motion_energy_proxy", 0.0)))
            totals["sleep_deactivation_requests"] += int(state.get("deactivation_requests", 0))
            totals["sleep_deactivation_confirmed"] += int(state.get("deactivation_confirmed", 0))
            totals["sleep_deactivation_rejected"] += int(state.get("deactivation_rejected", 0))

            if store_python_frames and snapshot is not None:
                frames[str(frame)] = snapshot
            binary_frame_numbers.append(str(frame))
            binary_values.extend(frame_values)
            if frame == frame_start + 1:
                totals["first_simulated_frame_contacts"] = dict(frame_contacts)
                totals["first_simulated_frame_state"] = dict(state)
            if frame_logging:
                log(
                    "FRAME_COMPLETE",
                    frame=frame,
                    substeps=frame_substeps,
                    adaptive_substeps=adaptive_substeps,
                    dt=frame_step_dt,
                    contacts=frame_contacts if contact_diagnostics else {"collection": "disabled"},
                    state=state,
                    visual_ground_compensation=ground_compensation,
                )
            if progress:
                progress(offset, total_frames)

            if early_sleep_termination and sleep_enabled and state["dynamic_bodies"] > 0 and state["active_bodies"] == 0:
                sleeping_streak += 1
            else:
                sleeping_streak = 0
            if sleeping_streak >= sleep_confirmation_frames and frame < frame_end:
                totals["early_sleep_frame"] = frame
                for remaining_frame in range(frame + 1, frame_end + 1):
                    if store_python_frames and snapshot is not None:
                        frames[str(remaining_frame)] = snapshot
                    binary_frame_numbers.append(str(remaining_frame))
                    binary_values.extend(frame_values)
                if progress:
                    progress(total_frames, total_frames)
                log(
                    "EARLY_SLEEP_TERMINATION",
                    frame=frame,
                    confirmation_frames=sleeping_streak,
                    configured_confirmation_frames=early_sleep_frames,
                    filled_frames=frame_end - frame,
                )
                break

        if totals.get("minimum_executed_substeps_per_frame") is None:
            totals["minimum_executed_substeps_per_frame"] = 0
        totals["bulk_frame_sample_seconds"] = float(bulk_frame_sample_seconds)
        totals["final_motion_energy_proxy"] = float(last_state.get("motion_energy_proxy", 0.0))
        totals["tail_max_motion_energy_proxy"] = float(max(energy_tail, default=0.0))
        totals["tail_motion_energy_samples"] = len(energy_tail)
        totals["binary_frame_values"] = len(binary_values)
        totals["python_frame_snapshots"] = len(frames)
        totals["binary_only_cache"] = bool(not store_python_frames)
        totals["cache_frame_source"] = "backend-direct-float32"
        bond_states = [
            {
                "bond_id": bond.stable_id,
                "body_a": bond.body_a.name,
                "body_b": bond.body_b.name,
                "status": "BROKEN" if bond.broken else "INTACT",
                "damage": float(bond.damage),
                "broken_frame": int(bond.broken_frame),
                "broken_substep": int(bond.broken_substep),
                "peak_force": float(bond.peak_force),
                "peak_torque": float(bond.peak_torque),
                "break_force": float(bond.break_force),
                "break_torque": float(bond.break_torque),
                "solver_constraint": bool(bond.solver_bound),
            }
            for bond in runtime_bonds
        ]
        totals["intact_bonds_final"] = sum(not bond.broken for bond in runtime_bonds)
        totals["broken_bonds_final"] = sum(bond.broken for bond in runtime_bonds)
        elapsed = time.perf_counter() - started
        top_pairs = (
            sorted(
                (
                    {
                        "pair": list(pair),
                        "events": stats["events"],
                        "max_impulse": stats["max_impulse"],
                        "max_penetration": float(stats.get("max_penetration", 0.0)),
                        "frame": stats["frame"],
                        "contact_frames": stats.get("contact_frames", 0),
                        "average_abs_vertical_normal": (
                            stats.get("vertical_normal_sum", 0.0) / max(1, stats.get("normal_samples", 0))
                        ),
                        "minimum_slide_speed": (0.0 if not math.isfinite(float(stats.get("minimum_slide_speed", 0.0))) else float(stats.get("minimum_slide_speed", 0.0))),
                        "maximum_slide_speed": stats.get("maximum_slide_speed", 0.0),
                        "maximum_relative_normal_speed": stats.get("maximum_relative_normal_speed", 0.0),
                        "longest_low_speed_side_streak": int(stats.get("side_stick_best_frames", 0)),
                        "shape_pair": [body_shapes.get(pair[0], "UNKNOWN"), body_shapes.get(pair[1], "UNKNOWN")],
                    }
                    for pair, stats in pair_stats.items()
                ),
                key=lambda item: (item["max_impulse"], item["events"]),
                reverse=True,
            )[:20]
            if contact_diagnostics
            else []
        )
        side_stick_candidates = []
        if side_stick_diagnostics:
            for pair, stats in pair_stats.items():
                streak_frames = int(stats.get("side_stick_best_frames", 0))
                if streak_frames < side_stick_min_frames:
                    continue
                shape_pair = [body_shapes.get(pair[0], "UNKNOWN"), body_shapes.get(pair[1], "UNKNOWN")]
                side_stick_candidates.append({
                    "pair": list(pair),
                    "shape_pair": shape_pair,
                    "compound_pair": bool(
                        shape_pair[0] in {"COMPOUND", "COMPOUND_CONVEX"}
                        and shape_pair[1] in {"COMPOUND", "COMPOUND_CONVEX"}
                    ),
                    "contact_frames": int(stats.get("contact_frames", 0)),
                    "continuous_low_speed_frames": streak_frames,
                    "first_frame": int(stats.get("side_stick_best_start", 0)),
                    "last_frame": int(stats.get("side_stick_best_end", 0)),
                    "average_abs_vertical_normal": float(stats.get("side_stick_best_average_vertical", 0.0)),
                    "minimum_slide_speed": float(stats.get("side_stick_best_min_slide", 0.0)),
                    "maximum_slide_speed": float(stats.get("side_stick_best_max_slide", 0.0)),
                    "max_impulse": float(stats.get("side_stick_best_max_impulse", 0.0)),
                    "last_position": stats.get("side_stick_best_last_position"),
                    "last_normal": stats.get("side_stick_best_last_normal"),
                })
            side_stick_candidates.sort(
                key=lambda item: (
                    item["continuous_low_speed_frames"],
                    -item["average_abs_vertical_normal"],
                    item["max_impulse"],
                ),
                reverse=True,
            )
            compound_side_stick_candidates = [
                candidate for candidate in side_stick_candidates if candidate.get("compound_pair")
            ]
            side_stick_candidates = side_stick_candidates[:20]
        else:
            compound_side_stick_candidates = []
        body_peaks = (
            sorted(
                (
                    {
                        "name": runtime.name,
                        "max_linear_speed": runtime.max_linear_speed,
                        "max_linear_speed_frame": runtime.max_linear_speed_frame,
                        "max_angular_speed": runtime.max_angular_speed,
                        "max_angular_speed_frame": runtime.max_angular_speed_frame,
                    }
                    for runtime in runtimes
                    if runtime.body_type == "DYNAMIC"
                ),
                key=lambda item: max(item["max_linear_speed"], item["max_angular_speed"]),
                reverse=True,
            )
            if track_body_peaks
            else []
        )
        final_state = dict(last_state)
        final_state["frame"] = frame_end
        contact_peaks = (
            sorted(
                (
                    {"name": name, **stats}
                    for name, stats in body_contact_peaks.items()
                    if stats["max_impulse"] > 0.0
                ),
                key=lambda item: item["max_impulse"],
                reverse=True,
            )
            if contact_diagnostics
            else []
        )

        log_totals = dict(totals)
        if not contact_logging:
            for key in (
                "contact_events", "contact_added", "contact_persisted", "contact_removed",
                "max_contact_impulse", "max_contact_pair", "penetration_depth_available",
            ):
                log_totals.pop(key, None)
        complete_log = {
            "elapsed_seconds": round(elapsed, 6),
            "frame_count": frame_end - frame_start + 1,
            "totals": log_totals,
            "final_state": final_state,
            "bond_events": bond_events,
            "bond_states": bond_states,
            "limitations": [
                *(
                    []
                    if not contact_logging or totals.get("penetration_depth_available")
                    else ["Contact penetration depth is unavailable through the active compatibility path."]
                ),
                *(
                    []
                    if bool(getattr(culverin, "NATIVE_BRIDGE", False))
                    else ["Per-body damping and Hybrid settle assistance are evaluated once per rendered frame until Culverin exposes native damping settings."]
                ),
                "Contact event collection is opt-in or automatically enabled by the compound runtime guard or breakable bonds.",
                *(
                    ["Breakable bond force and torque are estimated from external contact impulses because Culverin does not expose Jolt constraint reaction lambdas."]
                    if runtime_bonds
                    else []
                ),
                *(
                    ["Flexible cohesion is limited to 256 native Fixed constraints; the add-on therefore uses a deterministic spanning backbone and prioritizes the strongest remaining interfaces."]
                    if runtime_bonds and bond_stability_mode != "RIGID" and int(bond_constraint_stats.get("graph_bonds", 0)) > int(bond_constraint_stats.get("created_constraints", 0))
                    else []
                ),
                *(
                    []
                    if bool(getattr(culverin, "NATIVE_BRIDGE", False))
                    else ["Culverin 0.13.2 still uses Jolt's native velocity/position iteration defaults."]
                ),
                *(
                    ["Authored Rope/Rod constraints currently bind native body centers; use the generated non-colliding Static anchor for an exact suspension point."]
                    if distance_constraint_handles
                    else []
                ),
            ],
        }
        if contact_logging:
            complete_log.update(
                strongest_contact_pairs=top_pairs,
                body_contact_peaks=contact_peaks,
            )
        if side_stick_logging:
            complete_log.update(
                side_stick_candidates=side_stick_candidates,
                compound_side_stick_candidates=compound_side_stick_candidates,
                side_stick_settings={
                    "enabled": True,
                    "minimum_continuous_frames": side_stick_min_frames,
                    "maximum_abs_vertical_normal": side_stick_normal_z,
                    "maximum_slide_speed": side_stick_slide_speed,
                },
            )
        if payload_diagnostics:
            complete_log["body_speed_peaks"] = body_peaks
        log("BAKE_COMPLETE", **complete_log)

        return {
            "backend": self.identifier,
            "backend_detail": self.status().detail,
            "scene_signature": scene_payload["signature"],
            "scene_name": scene_payload["scene_name"],
            "frame_start": frame_start,
            "frame_end": frame_end,
            "frame_count": frame_end - frame_start + 1,
            "fps": fps,
            "diagnostic_totals": totals,
            "contact_diagnostics_enabled": contact_diagnostics,
            "payload_diagnostics_enabled": payload_diagnostics,
            "strongest_contact_pairs": top_pairs,
            "side_stick_candidates": side_stick_candidates,
            "compound_side_stick_candidates": compound_side_stick_candidates,
            "body_speed_peaks": body_peaks,
            "body_contact_peaks": contact_peaks,
            "bond_events": bond_events,
            "bond_states": bond_states,
            "breakable_bonds_enabled": bool(runtime_bonds),
            "distance_constraints_enabled": bool(distance_constraint_handles),
            "distance_constraint_count": len(distance_constraint_handles),
            "bond_force_model": "MASS_AWARE_CONTACT_MOMENTUM_V3" if runtime_bonds else None,
            "bond_stability_mode": bond_stability_mode if runtime_bonds else None,
            "final_state": final_state,
            "frames": frames,
            "_first_snapshot": initial_snapshot,
            "_binary_frame_block": {
                "frame_numbers": binary_frame_numbers,
                "body_names": binary_body_names,
                "body_scales": binary_body_scales,
                "values": binary_values,
            },
        }

    @staticmethod
    def _create_body(culverin, world, body: Mapping[str, Any], index: int, warnings: List[str]) -> _RuntimeBody:
        name = str(body.get("name", f"Body_{index:04d}"))
        requested_body_type = str(body.get("body_type", "DYNAMIC"))
        requested_collision_shape = str(body.get("collision_shape", "BOX"))
        managed_ground = bool(body.get("managed_ground", False)) or name.startswith("KA_Physics_Ground")
        body_type = "STATIC" if managed_ground else requested_body_type
        collision_shape = "PLANE" if managed_ground else requested_collision_shape
        if managed_ground and (requested_body_type != "STATIC" or requested_collision_shape != "PLANE"):
            warnings.append(
                f"{name}: managed ground was forced to a Static Plane "
                f"(requested {requested_body_type}/{requested_collision_shape})."
            )
        location = tuple(map(float, body.get("location", (0.0, 0.0, 0.0))))
        rotation = tuple(map(float, body.get("rotation", (1.0, 0.0, 0.0, 0.0))))
        scale = tuple(map(float, body.get("scale", (1.0, 1.0, 1.0))))
        shape_center = tuple(map(float, body.get("shape_center", (0.0, 0.0, 0.0))))
        center_world = quat_rotate_vector_wxyz(rotation, shape_center)
        shape_world_location = add_vec3(location, center_world)
        pos = blender_vec_to_jolt(shape_world_location)
        rot = blender_quat_to_jolt(rotation)
        user_data = index + 1
        category = max(1, int(body.get("collision_layer", 1))) & 0xFFFF
        mask = int(body.get("collision_mask", 0xFFFF)) & 0xFFFF
        friction = max(0.0, float(body.get("friction", 0.5)))
        restitution = max(0.0, min(1.0, float(body.get("restitution", 0.0))))
        ccd = False if managed_ground else bool(body.get("ccd", True))
        requested_mass = max(1.0e-6, float(body.get("mass", 1.0)))

        adjustments = list(body.get("stability_adjustments") or [])
        if adjustments:
            warnings.append(f"{name}: stability adjustments applied: {', '.join(map(str, adjustments))}.")

        motion_map = {
            "STATIC": culverin.MOTION_STATIC,
            "KINEMATIC": culverin.MOTION_KINEMATIC,
            "DYNAMIC": culverin.MOTION_DYNAMIC,
        }
        motion = motion_map.get(body_type, culverin.MOTION_DYNAMIC)
        mass = requested_mass if body_type == "DYNAMIC" else -1.0

        native_bridge = bool(getattr(culverin, "NATIVE_BRIDGE", False))
        native_compound_convex = bool(getattr(culverin, "NATIVE_COMPOUND_CONVEX", False))

        def common_at(world_pos, part_mass=mass):
            values = {
                "pos": blender_vec_to_jolt(world_pos),
                "rot": rot,
                "motion": motion,
                "mass": part_mass,
                "user_data": user_data,
                "category": category,
                "mask": mask,
                "friction": friction,
                "restitution": restitution,
                "ccd": ccd,
            }
            if native_bridge:
                values["linear_damping"] = max(0.0, float(body.get("linear_damping", 0.0)))
                values["angular_damping"] = max(0.0, float(body.get("angular_damping", 0.0)))
            return values

        common = common_at(shape_world_location)

        def create_single_hull(source_points, center):
            center = tuple(map(float, center))
            world_center = add_vec3(location, quat_rotate_vector_wxyz(rotation, center))
            hull_common = common_at(world_center)
            if len(source_points) < 4:
                warnings.append(f"{name}: convex hull has fewer than four points; a box fallback was used.")
                half = tuple(map(float, body.get("half_extents", (0.5, 0.5, 0.5))))
                size = (max(1.0e-5, half[0]), max(1.0e-5, half[2]), max(1.0e-5, half[1]))
                return int(world.create_body(shape=culverin.SHAPE_BOX, size=size, **hull_common)), center
            values = array.array("f")
            for point in source_points:
                values.extend(blender_vec_to_jolt(subtract_vec3(point, center)))
            return int(world.create_convex_hull(points=values.tobytes(), **hull_common)), center

        handles: List[int] = []
        constraint_handles: List[int] = []
        runtime_center = shape_center

        if collision_shape == "SPHERE":
            handle = int(world.create_body(
                shape=culverin.SHAPE_SPHERE,
                size=max(1.0e-5, float(body.get("radius", 0.5))),
                **common,
            ))
            handles = [handle]
        elif collision_shape == "BOX":
            half = tuple(map(float, body.get("half_extents", (0.5, 0.5, 0.5))))
            size = (max(1.0e-5, half[0]), max(1.0e-5, half[2]), max(1.0e-5, half[1]))
            handle = int(world.create_body(shape=culverin.SHAPE_BOX, size=size, **common))
            handles = [handle]
        elif collision_shape == "PLANE":
            if body_type == "DYNAMIC":
                raise BackendError(f"{name}: a plane cannot be dynamic.")
            handle = int(world.create_body(shape=culverin.SHAPE_PLANE, size=(0.0, 1.0, 0.0, 0.0), **common))
            handles = [handle]
        elif collision_shape == "CONVEX_HULL":
            handle, runtime_center = create_single_hull(body.get("convex_vertices") or [], shape_center)
            handles = [handle]
        elif collision_shape == "COMPOUND_CONVEX":
            source_parts = list(body.get("compound_parts") or [])
            if not source_parts:
                warnings.append(f"{name}: Compound Convex contains no valid parts; a single convex hull fallback was used.")
                handle, runtime_center = create_single_hull(body.get("convex_vertices") or [], shape_center)
                handles = [handle]
            else:
                if native_compound_convex:
                    try:
                        native_parts = []
                        for part_index, part in enumerate(source_parts):
                            part_points = list(part.get("vertices") or [])
                            if len(part_points) < 4:
                                continue
                            part_center = tuple(map(float, part.get("center", (0.0, 0.0, 0.0))))
                            values = array.array("f")
                            for point in part_points:
                                values.extend(blender_vec_to_jolt(subtract_vec3(point, part_center)))
                            native_parts.append({
                                "pos": blender_vec_to_jolt(subtract_vec3(part_center, shape_center)),
                                "rot": (0.0, 0.0, 0.0, 1.0),
                                "points": values.tobytes(),
                                "user_data": part_index,
                            })
                        if not native_parts:
                            raise RuntimeError("CoACD produced no usable convex child hulls")
                        handle = int(world.create_compound_convex(parts=native_parts, **common))
                        handles = [handle]
                        runtime_center = shape_center
                        warnings.append(f"{name}: Compound Convex uses one native Jolt StaticCompoundShape with {len(native_parts)} child hulls.")
                    except Exception as exc:
                        warnings.append(f"{name}: native Compound Convex creation failed ({exc}); a single convex hull fallback was used.")
                        handle, runtime_center = create_single_hull(body.get("convex_vertices") or [], shape_center)
                        handles = [handle]
                else:
                    # Culverin 0.13.2 cannot attach convex hull children to a
                    # compound. Interior primitive proxies visibly underfill the
                    # render mesh and can rest on the floor while the mesh clips
                    # through it. Prefer the complete outer convex hull: it may
                    # fill concavities, but it preserves the visible contact shell
                    # and cannot produce the severe under-coverage failure.
                    warnings.append(
                        f"{name}: true convex compound children are unavailable; "
                        "the complete single convex hull fallback was used instead of interior boxes."
                    )
                    handle, runtime_center = create_single_hull(
                        body.get("convex_vertices") or [], shape_center
                    )
                    handles = [handle]
        elif collision_shape == "COMPOUND":
            # Legacy 0.4.x primitive-box compound retained for regression and old
            # payload compatibility. New scenes use COMPOUND_CONVEX.
            source_parts = body.get("compound_parts") or []
            if not source_parts:
                warnings.append(f"{name}: compound proxy contains no parts; a convex hull fallback was used.")
                handle, runtime_center = create_single_hull(body.get("convex_vertices") or [], shape_center)
                handles = [handle]
            else:
                parts = []
                for part in source_parts:
                    center = tuple(map(float, part.get("center", (0.0, 0.0, 0.0))))
                    local_center = subtract_vec3(center, shape_center)
                    half = tuple(map(float, part.get("half_extents", (0.5, 0.5, 0.5))))
                    part_pos = blender_vec_to_jolt(local_center)
                    part_rot = (0.0, 0.0, 0.0, 1.0)
                    part_size = (max(1.0e-5, half[0]), max(1.0e-5, half[2]), max(1.0e-5, half[1]))
                    parts.append((part_pos, part_rot, culverin.SHAPE_BOX, part_size))
                try:
                    handle = int(world.create_compound_body(parts=parts, **common))
                    handles = [handle]
                except Exception as exc:
                    warnings.append(f"{name}: compound creation failed ({exc}); a convex hull fallback was used.")
                    handle, runtime_center = create_single_hull(body.get("convex_vertices") or [], shape_center)
                    handles = [handle]
        elif collision_shape == "MESH":
            if body_type != "STATIC":
                raise BackendError(f"{name}: triangle meshes are static-only; use Compound Convex for moving bodies.")
            vertices = body.get("mesh_vertices") or []
            indices = body.get("mesh_indices") or []
            if len(vertices) < 3 or len(indices) < 3:
                raise BackendError(f"{name}: static mesh contains no triangles.")
            vertex_bytes = array.array("f")
            for point in vertices:
                vertex_bytes.extend(blender_vec_to_jolt(subtract_vec3(point, shape_center)))
            index_bytes = array.array("I", (int(value) for value in indices))
            mesh_arguments = {
                "pos": pos,
                "rot": rot,
                "vertices": vertex_bytes.tobytes(),
                "indices": index_bytes.tobytes(),
                "user_data": user_data,
                "category": category,
                "mask": mask,
            }
            if native_bridge:
                # The ABI-v2 bridge uses the same descriptor for all shapes.
                # Passing the complete static-body contract avoids the adapter's
                # dynamic defaults and enables native material properties.
                mesh_arguments.update(common)
                mesh_arguments["pos"] = pos
                mesh_arguments["rot"] = rot
                mesh_arguments.pop("user_data", None)
                mesh_arguments["user_data"] = user_data
                mesh_arguments.pop("category", None)
                mesh_arguments["category"] = category
                mesh_arguments.pop("mask", None)
                mesh_arguments["mask"] = mask
            handle = int(world.create_mesh_body(**mesh_arguments))
            handles = [handle]
            if not native_bridge and (abs(friction - 0.2) > 1.0e-6 or restitution > 1.0e-6):
                warnings.append(
                    f"{name}: Culverin 0.13.2 does not expose friction/restitution parameters for static mesh creation."
                )
        else:
            raise BackendError(f"{name}: unsupported collision shape {collision_shape}.")

        handle = int(handles[0])
        if body_type == "DYNAMIC":
            base_linear = tuple(map(float, body.get("linear_velocity", (0.0, 0.0, 0.0))))
            base_angular = tuple(map(float, body.get("angular_velocity", (0.0, 0.0, 0.0))))
            for child_index, child in enumerate(handles):
                child_linear = base_linear
                if collision_shape == "COMPOUND_CONVEX" and len(handles) > 1 and child_index < len(source_parts):
                    part_center = tuple(map(float, source_parts[child_index].get("center", (0.0, 0.0, 0.0))))
                    offset = quat_rotate_vector_wxyz(rotation, part_center)
                    tangential = (
                        base_angular[1] * offset[2] - base_angular[2] * offset[1],
                        base_angular[2] * offset[0] - base_angular[0] * offset[2],
                        base_angular[0] * offset[1] - base_angular[1] * offset[0],
                    )
                    child_linear = add_vec3(base_linear, tangential)
                linear = blender_vec_to_jolt(child_linear)
                angular = blender_vec_to_jolt(base_angular)
                world.set_linear_velocity(child, *linear)
                world.set_angular_velocity(child, *angular)
                world.activate(child)

        authored_feature_length = float(body.get("minimum_feature_length", 0.0) or 0.0)
        if authored_feature_length > 0.0:
            feature_length = authored_feature_length
        elif collision_shape == "SPHERE":
            feature_length = 2.0 * max(1.0e-5, float(body.get("radius", 0.5)))
        else:
            feature_length = 2.0 * min(
                (
                    abs(float(value))
                    for value in body.get("half_extents", (0.5, 0.5, 0.5))[:3]
                ),
                default=0.5,
            )

        return _RuntimeBody(
            stable_id=str(body.get("stable_id", name)),
            name=name,
            handle=handle,
            body_type=body_type,
            scale=scale,
            input_location=location,
            input_rotation=rotation,
            shape_center=runtime_center,
            com_offset_local=runtime_center,
            linear_damping=0.0 if native_bridge else max(0.0, float(body.get("linear_damping", 0.0))),
            angular_damping=0.0 if native_bridge else max(0.0, float(body.get("angular_damping", 0.0))),
            radius=max(1.0e-5, float(body.get("radius", 0.5))),
            feature_length=max(1.0e-5, feature_length),
            mass=requested_mass if body_type == "DYNAMIC" else -1.0,
            ccd=ccd,
            collision_category=category,
            collision_mask=mask,
            handles=tuple(handles),
            constraint_handles=tuple(constraint_handles),
            source_body=dict(body),
        )

    @staticmethod
    def _select_constraint_backbone(
        bonds: Sequence[_RuntimeBond], limit: int
    ) -> Tuple[List[_RuntimeBond], Dict[str, int]]:
        """Select a deterministic, connected constraint subset.

        Culverin 0.13.2 hard-limits a PhysicsWorld to 256 constraints. Selecting
        records by UUID left arbitrary regions of dense fracture graphs entirely
        unbound. A maximum-area spanning forest first guarantees one mechanical
        path through every authored bond island, after which the largest remaining
        interfaces reinforce the network until the native budget is exhausted.
        """
        maximum = max(0, int(limit))
        parent: Dict[str, str] = {}
        rank: Dict[str, int] = {}

        def find(value: str) -> str:
            parent.setdefault(value, value)
            rank.setdefault(value, 0)
            root = value
            while parent[root] != root:
                root = parent[root]
            while parent[value] != value:
                next_value = parent[value]
                parent[value] = root
                value = next_value
            return root

        def union(first: str, second: str) -> bool:
            root_a = find(first)
            root_b = find(second)
            if root_a == root_b:
                return False
            rank_a = rank[root_a]
            rank_b = rank[root_b]
            if rank_a < rank_b:
                root_a, root_b = root_b, root_a
            parent[root_b] = root_a
            if rank_a == rank_b:
                rank[root_a] += 1
            return True

        ordered = sorted(
            bonds,
            key=lambda bond: (-float(bond.area), bond.stable_id),
        )
        selected: List[_RuntimeBond] = []
        selected_ids: set[str] = set()
        backbone_edges = 0
        required_backbone_edges = 0
        for bond in ordered:
            if union(bond.body_a.stable_id, bond.body_b.stable_id):
                required_backbone_edges += 1
                if len(selected) < maximum:
                    selected.append(bond)
                    selected_ids.add(bond.stable_id)
                    backbone_edges += 1
        if len(selected) < maximum:
            for bond in ordered:
                if bond.stable_id in selected_ids:
                    continue
                selected.append(bond)
                selected_ids.add(bond.stable_id)
                if len(selected) >= maximum:
                    break
        return selected, {
            "selected_constraints": len(selected),
            "backbone_edges": backbone_edges,
            "required_backbone_edges": required_backbone_edges,
            "reinforcement_edges": max(0, len(selected) - backbone_edges),
        }

    @classmethod
    def _apply_bond_island_collision_filters(
        cls,
        world,
        runtimes: Sequence[_RuntimeBody],
        bonds: Sequence[_RuntimeBond],
    ) -> Dict[str, Any]:
        """Disable self-collision inside each intact multi-body bond island.

        Jolt's contact solver must not separate fragments that the intact bond
        graph immediately projects back together. That solver/projection loop
        injects motion into the complete cluster, especially while it rests on
        the ground. Filters are rebuilt whenever a bond break changes topology,
        so newly disconnected islands collide with each other again.
        """
        runtime_list = list(runtimes)
        by_id = {body.stable_id: body for body in runtime_list}
        parent = {body.stable_id: body.stable_id for body in runtime_list}

        def find(value: str) -> str:
            root = value
            while parent[root] != root:
                root = parent[root]
            while parent[value] != value:
                next_value = parent[value]
                parent[value] = root
                value = next_value
            return root

        def union(first: str, second: str) -> None:
            root_a = find(first)
            root_b = find(second)
            if root_a == root_b:
                return
            if root_a < root_b:
                parent[root_b] = root_a
            else:
                parent[root_a] = root_b

        for bond in bonds:
            if bond.broken:
                continue
            if bond.body_a.stable_id in parent and bond.body_b.stable_id in parent:
                union(bond.body_a.stable_id, bond.body_b.stable_id)

        grouped: Dict[str, List[_RuntimeBody]] = {}
        for stable_id, body in by_id.items():
            grouped.setdefault(find(stable_id), []).append(body)
        components = [
            sorted(component, key=lambda body: body.stable_id)
            for component in grouped.values()
            if len(component) > 1
        ]
        components.sort(key=lambda component: component[0].stable_id)

        used_category_bits = 0
        for body in runtime_list:
            used_category_bits |= int(body.collision_category) & 0xFFFF
        available_bits = [
            1 << index for index in range(16)
            if not (used_category_bits & (1 << index))
        ]

        active_category = {
            body.stable_id: max(1, int(body.collision_category)) & 0xFFFF
            for body in runtime_list
        }
        filtered_component: Dict[str, int] = {}
        filtered_body_ids: set[str] = set()
        overflow_components = 0
        for component_index, component in enumerate(components):
            if component_index >= len(available_bits):
                overflow_components += 1
                continue
            category = int(available_bits[component_index])
            component_key = component[0].stable_id
            filtered_component[component_key] = category
            for body in component:
                active_category[body.stable_id] = category
                filtered_body_ids.add(body.stable_id)

        component_key_by_body: Dict[str, str] = {}
        for component in components:
            key = component[0].stable_id
            if key not in filtered_component:
                continue
            for body in component:
                component_key_by_body[body.stable_id] = key

        active_masks: Dict[str, int] = {}
        for body in runtime_list:
            mask = 0
            own_component = component_key_by_body.get(body.stable_id)
            for other in runtime_list:
                if other is body:
                    continue
                if own_component is not None and component_key_by_body.get(other.stable_id) == own_component:
                    continue
                original_pair_enabled = (
                    bool(int(body.collision_mask) & int(other.collision_category))
                    and bool(int(other.collision_mask) & int(body.collision_category))
                )
                if original_pair_enabled:
                    mask |= int(active_category[other.stable_id])
            active_masks[body.stable_id] = mask & 0xFFFF

        for body in runtime_list:
            category = int(active_category[body.stable_id]) & 0xFFFF
            mask = int(active_masks[body.stable_id]) & 0xFFFF
            for handle in cls._runtime_handles(body):
                world.set_collision_filter(int(handle), category, mask)

        return {
            "components": len(components),
            "filtered_components": len(filtered_component),
            "filtered_bodies": len(filtered_body_ids),
            "overflow_components": overflow_components,
            "available_category_bits": len(available_bits),
        }

    @staticmethod
    def _bond_local_frame(
        body: _RuntimeBody,
        anchor_jolt: Sequence[float],
        normal_jolt: Sequence[float],
    ) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        inverse_rotation = _quat_conjugate_xyzw(body.rest_rotation_jolt)
        local_anchor = _quat_rotate_xyzw(
            inverse_rotation,
            subtract_vec3(anchor_jolt, body.rest_position_jolt),
        )
        local_normal = _normalize_vec3(_quat_rotate_xyzw(inverse_rotation, normal_jolt))
        return local_anchor, local_normal

    @classmethod
    def _create_breakable_bonds(
        cls,
        culverin,
        world,
        constraints: Iterable[Mapping[str, Any]],
        runtimes: Sequence[_RuntimeBody],
        warnings: List[str],
        *,
        create_native_constraints: bool = True,
    ) -> Tuple[List[_RuntimeBond], Dict[str, Any]]:
        by_id = {runtime.stable_id: runtime for runtime in runtimes}
        by_name = {runtime.name: runtime for runtime in runtimes}
        result: List[_RuntimeBond] = []
        for record in sorted(constraints, key=lambda item: str(item.get("stable_id", ""))):
            body_a = by_id.get(str(record.get("body_a", ""))) or by_name.get(str(record.get("body_a_name", "")))
            body_b = by_id.get(str(record.get("body_b", ""))) or by_name.get(str(record.get("body_b_name", "")))
            bond_id = str(record.get("stable_id", ""))
            if body_a is None or body_b is None or body_a is body_b:
                warnings.append(f"Bond {bond_id}: referenced body is unavailable; bond skipped.")
                continue
            if body_a.body_type == "STATIC" and body_b.body_type == "STATIC":
                warnings.append(f"Bond {bond_id}: static/static bond skipped.")
                continue
            anchor = tuple(map(float, record.get("anchor", (0.0, 0.0, 0.0))))[:3]
            normal = _normalize_vec3(tuple(map(float, record.get("normal", (0.0, 0.0, 1.0))))[:3])
            anchor_jolt = blender_vec_to_jolt(anchor)
            normal_jolt = _normalize_vec3(blender_vec_to_jolt(normal))
            anchor_local_a, normal_local_a = cls._bond_local_frame(body_a, anchor_jolt, normal_jolt)
            anchor_local_b, normal_local_b = cls._bond_local_frame(body_b, anchor_jolt, normal_jolt)
            result.append(_RuntimeBond(
                stable_id=bond_id,
                handle=0,
                body_a=body_a,
                body_b=body_b,
                anchor=anchor,
                normal=normal,
                area=max(0.0, float(record.get("area", 0.0))),
                break_force=max(0.0, float(record.get("break_force", 0.0))),
                break_torque=max(0.0, float(record.get("break_torque", 0.0))),
                damage_accumulation=max(0.0, float(record.get("damage_accumulation", 0.0))),
                damage=max(0.0, float(record.get("damage", 0.0))),
                anchor_local_a_jolt=anchor_local_a,
                anchor_local_b_jolt=anchor_local_b,
                normal_local_a_jolt=normal_local_a,
                normal_local_b_jolt=normal_local_b,
            ))

        constraint_limit = 256
        if create_native_constraints:
            selected, selection_stats = cls._select_constraint_backbone(result, constraint_limit)
        else:
            selected = []
            selection_stats = {
                "selected_constraints": 0,
                "backbone_edges": 0,
                "required_backbone_edges": max(0, len({b.body_a.stable_id for b in result} | {b.body_b.stable_id for b in result}) - 1),
                "reinforcement_edges": 0,
            }
        created = 0
        for bond in selected:
            try:
                bond.handle = int(world.create_constraint(
                    int(culverin.CONSTRAINT_FIXED),
                    int(bond.body_a.handle),
                    int(bond.body_b.handle),
                    None,
                ))
                bond.solver_bound = True
                created += 1
            except Exception as exc:
                warnings.append(f"Bond {bond.stable_id}: Fixed constraint creation failed: {exc}")

        omitted = max(0, len(result) - created)
        if omitted and create_native_constraints:
            warnings.append(
                f"Bond graph contains {len(result)} bonds; Culverin created {created} Fixed constraints. "
                "The remaining authored edges still participate in fracture topology but are not independently solver-bound."
            )
        if create_native_constraints and int(selection_stats.get("required_backbone_edges", 0)) > created:
            warnings.append(
                "The native constraint budget is smaller than the bond graph's spanning forest. "
                "The native constraint budget cannot cover the complete spanning forest; mechanical cohesion cannot be guaranteed for every graph region."
            )
        stats: Dict[str, Any] = {
            "graph_bonds": len(result),
            "created_constraints": created,
            "constraint_limit": constraint_limit,
            **selection_stats,
        }
        return result, stats


    @staticmethod
    def _distance_constraint_budget(
        compound_constraint_count: int,
        bond_stability_mode: str,
        bond_constraint_stats: Mapping[str, Any],
        bond_cluster_stats: Mapping[str, Any],
    ) -> int:
        """Return the remaining Culverin constraint budget without double counting.

        In rigid bond mode ``created_constraints`` mirrors the native static
        anchor count. Older code subtracted that value and the cluster anchor
        count a second time, reducing the external-constraint budget twice.
        """
        if str(bond_stability_mode).upper() == "RIGID":
            bond_native_count = int(bond_cluster_stats.get("static_anchor_constraints", 0))
        else:
            bond_native_count = int(bond_constraint_stats.get("created_constraints", 0))
        return max(0, 256 - int(compound_constraint_count) - max(0, bond_native_count))


    @staticmethod
    def _destroy_constraint_handles(world, handles: Iterable[int], warnings: List[str]) -> int:
        """Destroy native constraints before any endpoint body is replaced."""
        destroyed = 0
        for handle in tuple(handles):
            try:
                world.destroy_constraint(int(handle))
                destroyed += 1
            except Exception as exc:
                warnings.append(
                    f"Distance constraint {int(handle)} could not be destroyed before body rebuild: {exc}"
                )
        return destroyed


    @staticmethod
    def _create_distance_constraints(
        culverin,
        world,
        constraints: Sequence[Mapping[str, Any]],
        runtimes: Sequence[_RuntimeBody],
        warnings: List[str],
        *,
        constraint_limit: int = 256,
    ) -> Tuple[List[int], Dict[str, int]]:
        """Create stable center-to-center Rope/Rod constraints.

        Culverin 0.13.2 accepts the native Distance settings in the extended
        ``(point_a, point_b, min, max)`` form. Zero points intentionally bind
        the native body centers. A dedicated non-colliding Static body can be
        placed anywhere in Blender to act as an exact suspension anchor.
        """
        by_id = {runtime.stable_id: runtime for runtime in runtimes}
        by_name = {runtime.name: runtime for runtime in runtimes}
        ordered = sorted(constraints, key=lambda item: str(item.get("stable_id", "")))
        limit = max(0, int(constraint_limit))
        handles: List[int] = []
        skipped = 0

        for record in ordered[:limit]:
            identifier = str(record.get("stable_id", ""))
            body_a_id = str(record.get("body_a", ""))
            body_b_id = str(record.get("body_b", ""))
            # Stable IDs are authoritative. Name fallback is retained only for
            # legacy payloads that genuinely have no persistent ID. Falling
            # back after a non-empty ID fails can silently attach a rope to a
            # different body after duplication or topology rebuilds.
            body_a = by_id.get(body_a_id) if body_a_id else by_name.get(
                str(record.get("body_a_name", ""))
            )
            body_b = by_id.get(body_b_id) if body_b_id else by_name.get(
                str(record.get("body_b_name", ""))
            )
            if body_a is None or body_b is None or body_a is body_b:
                warnings.append(f"Distance constraint {identifier}: referenced body is unavailable; skipped.")
                skipped += 1
                continue
            if body_a.body_type == "STATIC" and body_b.body_type == "STATIC":
                warnings.append(f"Distance constraint {identifier}: Static/Static pair skipped.")
                skipped += 1
                continue
            if int(body_a.handle) == int(body_b.handle):
                warnings.append(
                    f"Distance constraint {identifier}: both endpoints resolve to the same rigid bond island; skipped."
                )
                skipped += 1
                continue

            minimum = max(0.0, float(record.get("min_distance", 0.0)))
            maximum = max(1.0e-5, float(record.get("max_distance", 0.0)))
            minimum = min(minimum, maximum)
            try:
                try:
                    handle = int(world.create_constraint(
                        int(culverin.CONSTRAINT_DISTANCE),
                        int(body_a.handle),
                        int(body_b.handle),
                        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), minimum, maximum),
                    ))
                except TypeError:
                    # Retain compatibility with a future Culverin build that
                    # follows its documented compact ``(min, max)`` signature.
                    handle = int(world.create_constraint(
                        int(culverin.CONSTRAINT_DISTANCE),
                        int(body_a.handle),
                        int(body_b.handle),
                        (minimum, maximum),
                    ))
                handles.append(handle)
            except Exception as exc:
                warnings.append(f"Distance constraint {identifier}: creation failed: {exc}")
                skipped += 1

        omitted_by_limit = max(0, len(ordered) - limit)
        if omitted_by_limit:
            warnings.append(
                f"{len(ordered)} authored Distance constraints exceed the remaining native budget; "
                f"{omitted_by_limit} were omitted."
            )
        if handles:
            world.step(0.0)
        return handles, {
            "requested": len(ordered),
            "created": len(handles),
            "omitted": skipped + omitted_by_limit,
            "constraint_limit": limit,
        }


    @classmethod
    def _runtime_pose_jolt(
        cls,
        world,
        body: _RuntimeBody,
    ) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
        """Return a logical fragment COM pose, including compound bond islands."""
        cluster = body.cluster
        if cluster is None:
            position = world.get_position(body.handle) or body.rest_position_jolt
            rotation = world.get_rotation(body.handle) or body.rest_rotation_jolt
            return tuple(map(float, position[:3])), _quat_normalize_xyzw(rotation)
        cluster_position = world.get_position(cluster.handle)
        cluster_rotation = world.get_rotation(cluster.handle)
        if cluster_position is None or cluster_rotation is None:
            return body.rest_position_jolt, body.rest_rotation_jolt
        cluster_position = tuple(map(float, cluster_position[:3]))
        cluster_rotation = _quat_normalize_xyzw(cluster_rotation)
        local_position = cluster.local_positions_jolt[body.stable_id]
        local_rotation = cluster.local_rotations_jolt[body.stable_id]
        position = add_vec3(cluster_position, _quat_rotate_xyzw(cluster_rotation, local_position))
        rotation = _quat_normalize_xyzw(_quat_multiply_xyzw(cluster_rotation, local_rotation))
        return position, rotation

    @classmethod
    def _runtime_velocity_jolt(
        cls,
        world,
        body: _RuntimeBody,
    ) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        cluster = body.cluster
        handle = cluster.handle if cluster is not None else body.handle
        linear = tuple(map(float, (world.get_velocity(handle) or (0.0, 0.0, 0.0))[:3]))
        angular = tuple(map(float, (world.get_angular_velocity(handle) or (0.0, 0.0, 0.0))[:3]))
        if cluster is None:
            return linear, angular
        cluster_position = world.get_position(cluster.handle) or (0.0, 0.0, 0.0)
        body_position, _rotation = cls._runtime_pose_jolt(world, body)
        offset = subtract_vec3(body_position, cluster_position)
        return add_vec3(linear, _cross_vec3(angular, offset)), angular

    @classmethod
    def _capture_pre_step_motion(
        cls,
        world,
        runtimes: Sequence[_RuntimeBody],
    ) -> Dict[str, Dict[str, Tuple[float, ...]]]:
        """Capture actor motion before contact resolution for mass-aware loads."""
        by_handle: Dict[int, Dict[str, Tuple[float, ...]]] = {}
        result: Dict[str, Dict[str, Tuple[float, ...]]] = {}
        for runtime in runtimes:
            handle = int(runtime.cluster.handle if runtime.cluster is not None else runtime.handle)
            motion = by_handle.get(handle)
            if motion is None:
                position_jolt = tuple(map(float, (world.get_position(handle) or (0.0, 0.0, 0.0))[:3]))
                linear_jolt = tuple(map(float, (world.get_velocity(handle) or (0.0, 0.0, 0.0))[:3]))
                angular_jolt = tuple(map(float, (world.get_angular_velocity(handle) or (0.0, 0.0, 0.0))[:3]))
                motion = {
                    "position": tuple(map(float, jolt_vec_to_blender(position_jolt))),
                    "linear": tuple(map(float, jolt_vec_to_blender(linear_jolt))),
                    "angular": tuple(map(float, jolt_vec_to_blender(angular_jolt))),
                }
                by_handle[handle] = motion
            result[runtime.name] = motion
        return result

    @staticmethod
    def _contact_point_velocity(
        motion: Optional[Mapping[str, Sequence[float]]],
        point: Sequence[float],
    ) -> Tuple[float, float, float]:
        if not motion:
            return (0.0, 0.0, 0.0)
        position = tuple(map(float, motion.get("position", (0.0, 0.0, 0.0))))
        linear = tuple(map(float, motion.get("linear", (0.0, 0.0, 0.0))))
        angular = tuple(map(float, motion.get("angular", (0.0, 0.0, 0.0))))
        return add_vec3(linear, _cross_vec3(angular, subtract_vec3(point, position)))

    @staticmethod
    def _refresh_handle_map(
        runtimes: Sequence[_RuntimeBody],
        handle_to_name: Dict[int, Any],
    ) -> None:
        grouped: Dict[int, List[str]] = {}
        for runtime in runtimes:
            for handle in JoltBackend._runtime_handles(runtime):
                names = grouped.setdefault(int(handle), [])
                if runtime.name not in names:
                    names.append(runtime.name)
        handle_to_name.clear()
        for handle, names in grouped.items():
            handle_to_name[handle] = names[0] if len(names) == 1 else tuple(sorted(names))

    @classmethod
    def _cluster_parts_for_runtime(
        cls,
        culverin,
        runtime: _RuntimeBody,
        member_position_jolt: Sequence[float],
        member_rotation_jolt: Sequence[float],
        cluster_position_jolt: Sequence[float],
    ) -> List[Tuple[Any, Any, int, Any]]:
        """Convert one logical fragment collider into primitive compound children."""
        source = dict(runtime.source_body or {})
        member_rotation_jolt = _quat_normalize_xyzw(member_rotation_jolt)
        member_rotation_blender = jolt_quat_to_blender(member_rotation_jolt)
        member_com_blender = jolt_vec_to_blender(member_position_jolt)
        member_origin_blender = subtract_vec3(
            member_com_blender,
            quat_rotate_vector_wxyz(member_rotation_blender, runtime.com_offset_local),
        )

        def world_center(local_center: Sequence[float]) -> Tuple[float, float, float]:
            return add_vec3(
                member_origin_blender,
                quat_rotate_vector_wxyz(member_rotation_blender, local_center),
            )

        def box_part(local_center, half_extents, local_rotation=(1.0, 0.0, 0.0, 0.0)):
            center_jolt = blender_vec_to_jolt(world_center(local_center))
            local_center_jolt = subtract_vec3(center_jolt, cluster_position_jolt)
            child_rotation = _quat_normalize_xyzw(
                _quat_multiply_xyzw(member_rotation_jolt, blender_quat_to_jolt(local_rotation))
            )
            half = tuple(max(1.0e-5, float(value)) for value in half_extents[:3])
            size_jolt = (half[0], half[2], half[1])
            return (local_center_jolt, child_rotation, culverin.SHAPE_BOX, size_jolt)

        shape = str(source.get("collision_shape", "BOX"))
        parts: List[Tuple[Any, Any, int, Any]] = []
        if shape == "COMPOUND_CONVEX":
            for part in source.get("compound_parts", []) or []:
                half = tuple(map(float, part.get("box_half_extents", (0.0, 0.0, 0.0))))
                if min(half, default=0.0) <= 0.0:
                    points = list(part.get("vertices", []))
                    if not points:
                        continue
                    minimum = tuple(min(float(point[axis]) for point in points) for axis in range(3))
                    maximum = tuple(max(float(point[axis]) for point in points) for axis in range(3))
                    center = tuple((minimum[axis] + maximum[axis]) * 0.5 for axis in range(3))
                    half = tuple(max(1.0e-5, (maximum[axis] - minimum[axis]) * 0.5) for axis in range(3))
                else:
                    center = tuple(map(float, part.get("box_center", part.get("center", runtime.com_offset_local))))
                rotation = tuple(map(float, part.get("box_rotation", (1.0, 0.0, 0.0, 0.0))))
                parts.append(box_part(center, half, rotation))
        elif shape == "COMPOUND":
            for part in source.get("compound_parts", []) or []:
                parts.append(box_part(
                    tuple(map(float, part.get("center", runtime.com_offset_local))),
                    tuple(map(float, part.get("half_extents", source.get("half_extents", (0.5, 0.5, 0.5))))),
                ))
        elif shape == "SPHERE":
            center_jolt = blender_vec_to_jolt(world_center(source.get("shape_center", runtime.com_offset_local)))
            local_center_jolt = subtract_vec3(center_jolt, cluster_position_jolt)
            parts.append((
                local_center_jolt,
                member_rotation_jolt,
                culverin.SHAPE_SPHERE,
                max(1.0e-5, float(source.get("radius", runtime.radius))),
            ))
        elif shape == "CONVEX_HULL":
            sphere_cloud = _safe_convex_sphere_cloud(source.get("convex_vertices", []) or [])
            for sphere_center, sphere_radius in sphere_cloud:
                center_jolt = blender_vec_to_jolt(world_center(sphere_center))
                local_center_jolt = subtract_vec3(center_jolt, cluster_position_jolt)
                parts.append((
                    local_center_jolt,
                    member_rotation_jolt,
                    culverin.SHAPE_SPHERE,
                    max(1.0e-5, float(sphere_radius)),
                ))
            if not sphere_cloud:
                # Degenerate legacy payloads have no hull points. Keep a small
                # centered primitive rather than the outer AABB, which may cross
                # the authored surface and cause a start-frame depenetration jump.
                half = tuple(map(float, source.get("half_extents", (runtime.radius, runtime.radius, runtime.radius))))
                safe_radius = max(1.0e-5, min(half) * 0.25)
                center_jolt = blender_vec_to_jolt(world_center(source.get("shape_center", runtime.com_offset_local)))
                parts.append((
                    subtract_vec3(center_jolt, cluster_position_jolt),
                    member_rotation_jolt,
                    culverin.SHAPE_SPHERE,
                    safe_radius,
                ))
        else:
            parts.append(box_part(
                tuple(map(float, source.get("shape_center", runtime.com_offset_local))),
                tuple(map(float, source.get("half_extents", (runtime.radius, runtime.radius, runtime.radius)))),
            ))
        return parts

    @classmethod
    def _cluster_outer_hull_points(
        cls,
        runtime: _RuntimeBody,
        member_position_jolt: Sequence[float],
        member_rotation_jolt: Sequence[float],
    ) -> List[Tuple[float, float, float]]:
        """Return world-space Jolt points that fully cover one logical member.

        Rigid bond islands need one native actor. When Culverin cannot place
        convex children in a compound, an outer hull of all member colliders is
        the only available one-body representation that does not underfill the
        visible surface. Concavities may be filled, but authored exterior points
        remain inside or on the collision shape.
        """
        source = dict(runtime.source_body or {})
        member_rotation_jolt = _quat_normalize_xyzw(member_rotation_jolt)
        member_rotation_blender = jolt_quat_to_blender(member_rotation_jolt)
        member_com_blender = jolt_vec_to_blender(member_position_jolt)
        member_origin_blender = subtract_vec3(
            member_com_blender,
            quat_rotate_vector_wxyz(member_rotation_blender, runtime.com_offset_local),
        )

        local_points: List[Tuple[float, float, float]] = []
        shape = str(source.get("collision_shape", "BOX"))
        if shape in {"CONVEX_HULL", "COMPOUND_CONVEX"}:
            local_points.extend(
                tuple(map(float, point[:3]))
                for point in source.get("convex_vertices", []) or []
            )
            if len(local_points) < 4:
                for part in source.get("compound_parts", []) or []:
                    local_points.extend(
                        tuple(map(float, point[:3]))
                        for point in part.get("vertices", []) or []
                    )
        elif shape == "SPHERE":
            center = tuple(map(float, source.get("shape_center", runtime.com_offset_local)))
            radius = max(1.0e-5, float(source.get("radius", runtime.radius)))
            # A cube circumscribes the sphere and therefore cannot underfill it.
            local_points.extend(
                (center[0] + x * radius, center[1] + y * radius, center[2] + z * radius)
                for x in (-1.0, 1.0)
                for y in (-1.0, 1.0)
                for z in (-1.0, 1.0)
            )
        else:
            center = tuple(map(float, source.get("shape_center", runtime.com_offset_local)))
            half = tuple(
                max(1.0e-5, abs(float(value)))
                for value in source.get("half_extents", (runtime.radius, runtime.radius, runtime.radius))[:3]
            )
            local_points.extend(
                (center[0] + x * half[0], center[1] + y * half[1], center[2] + z * half[2])
                for x in (-1.0, 1.0)
                for y in (-1.0, 1.0)
                for z in (-1.0, 1.0)
            )

        result: List[Tuple[float, float, float]] = []
        seen = set()
        for point in local_points:
            world_point = add_vec3(
                member_origin_blender,
                quat_rotate_vector_wxyz(member_rotation_blender, point),
            )
            converted = tuple(map(float, blender_vec_to_jolt(world_point)))
            key = tuple(round(value, 8) for value in converted)
            if key in seen:
                continue
            seen.add(key)
            result.append(converted)
        return result


    @classmethod
    def _rigid_component_starts_supported(
        cls,
        component: Sequence[_RuntimeBody],
        poses: Mapping[str, Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]],
        velocities: Mapping[str, Tuple[Tuple[float, float, float], Tuple[float, float, float]]],
        static_runtimes: Sequence[_RuntimeBody],
    ) -> bool:
        """Return True when a zero-velocity rigid island is already on managed ground.

        Creating such an island active lets gravity settle the conservative
        primitive proxy before native sleeping engages.  The visible render mesh
        can then move several millimetres even though the authored pose was
        already a valid resting pose.  A sleeping Jolt dynamic body still wakes
        automatically when another active body impacts it.
        """
        for runtime in component:
            linear, angular = velocities.get(runtime.stable_id, ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))
            if length_vec3(linear) > 1.0e-5 or length_vec3(angular) > 1.0e-5:
                return False

        ground_levels: List[float] = []
        for runtime in static_runtimes:
            source = dict(runtime.source_body or {})
            if str(source.get("collision_shape", "")) != "PLANE":
                continue
            if not bool(source.get("managed_ground", False)):
                continue
            center = source.get("shape_center", (0.0, 0.0, 0.0))
            ground_levels.append(float(center[2]))
        if not ground_levels:
            return False

        minimum_z = float("inf")
        found_points = False
        allowed_gap = 0.00175
        for runtime in component:
            source = dict(runtime.source_body or {})
            quality = dict(source.get("collider_quality", {}) or {})
            inset = max(
                0.0,
                float(quality.get("separation_inset_applied", 0.0) or 0.0),
                float(quality.get("separation_inset_requested", 0.0) or 0.0),
            )
            if inset > 0.0:
                allowed_gap = max(allowed_gap, min(0.003, inset * 1.5 + 0.00025))
            points = list(source.get("convex_vertices", []) or [])
            if not points:
                for part in source.get("compound_parts", []) or []:
                    points.extend(part.get("vertices", []) or [])
            if not points:
                continue
            position_jolt, rotation_jolt = poses[runtime.stable_id]
            rotation_blender = jolt_quat_to_blender(rotation_jolt)
            com_blender = jolt_vec_to_blender(position_jolt)
            origin_blender = subtract_vec3(
                com_blender,
                quat_rotate_vector_wxyz(rotation_blender, runtime.com_offset_local),
            )
            for point in points:
                world_point = add_vec3(
                    origin_blender,
                    quat_rotate_vector_wxyz(rotation_blender, point),
                )
                minimum_z = min(minimum_z, float(world_point[2]))
                found_points = True
        if not found_points:
            return False

        ground_z = min(ground_levels, key=lambda value: abs(value - minimum_z))
        gap = minimum_z - ground_z
        # The fitted fracture hull is intentionally inset by about one
        # millimetre.  Accept that authored clearance, but never freeze a body
        # that begins visibly penetrating or actually falling above the plane.
        return -0.0015 <= gap <= allowed_gap

    @classmethod
    def _create_rigid_static_anchor_constraints(
        cls,
        culverin,
        world,
        bonds: Sequence[_RuntimeBond],
        warnings: List[str],
        *,
        constraint_limit: int = 256,
    ) -> Dict[str, int]:
        """Bind intact rigid dynamic islands to authored static bodies.

        Dynamic-Dynamic bonds are represented by one compound actor in RIGID
        mode. A Dynamic-Static bond cannot be merged into that actor, so it must
        remain an actual native Fixed constraint. Constraints are recreated after
        every topology rebuild because the dynamic actor handles change.
        """
        candidates: List[Tuple[_RuntimeBond, _RuntimeBody, _RuntimeBody, Tuple[str, str]]] = []
        for bond in bonds:
            if bond.broken:
                continue
            a_dynamic = bond.body_a.body_type == "DYNAMIC"
            b_dynamic = bond.body_b.body_type == "DYNAMIC"
            if a_dynamic == b_dynamic:
                continue
            dynamic = bond.body_a if a_dynamic else bond.body_b
            static = bond.body_b if a_dynamic else bond.body_a
            dynamic_key = dynamic.cluster.stable_id if dynamic.cluster is not None else dynamic.stable_id
            pair_key = (str(dynamic_key), str(static.stable_id))
            candidates.append((bond, dynamic, static, pair_key))

        def priority(item):
            bond = item[0]
            return (-float(bond.break_force), -float(bond.break_torque), -float(bond.area), str(bond.stable_id))

        grouped: Dict[Tuple[str, str], List[Tuple[_RuntimeBond, _RuntimeBody, _RuntimeBody, Tuple[str, str]]]] = {}
        for item in candidates:
            grouped.setdefault(item[3], []).append(item)
        selected: List[Tuple[_RuntimeBond, _RuntimeBody, _RuntimeBody, Tuple[str, str]]] = []
        selected_ids = set()
        # Guarantee one mechanical anchor per dynamic-island/static-body pair.
        for pair_key in sorted(grouped):
            strongest = sorted(grouped[pair_key], key=priority)[0]
            selected.append(strongest)
            selected_ids.add(strongest[0].stable_id)
        remaining = sorted(
            (item for item in candidates if item[0].stable_id not in selected_ids),
            key=priority,
        )
        selected.extend(remaining)
        selected = selected[:max(0, int(constraint_limit))]

        created = 0
        anchored_pairs = set()
        for bond, dynamic, static, pair_key in selected:
            try:
                bond.handle = int(world.create_constraint(
                    int(culverin.CONSTRAINT_FIXED),
                    int(dynamic.handle),
                    int(static.handle),
                    None,
                ))
                bond.solver_bound = True
                created += 1
                anchored_pairs.add(pair_key)
            except Exception as exc:
                bond.handle = 0
                bond.solver_bound = False
                warnings.append(
                    f"Bond {bond.stable_id}: rigid Dynamic-Static anchor creation failed: {exc}"
                )

        omitted = max(0, len(candidates) - created)
        if omitted:
            warnings.append(
                f"Rigid mode requested {len(candidates)} Dynamic-Static anchor bonds; "
                f"{created} native Fixed constraints were created within the {constraint_limit} constraint limit."
            )
        return {
            "requested": len(candidates),
            "created": created,
            "omitted": omitted,
            "anchored_pairs": len(anchored_pairs),
        }

    @classmethod
    def _rigid_anchor_initial_static_overlap_pairs(
        cls,
        world,
        runtimes: Sequence[_RuntimeBody],
        bonds: Sequence[_RuntimeBond],
        dynamic_poses: Mapping[str, Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]],
        *,
        tolerance: float = 1.0e-5,
    ) -> set[Tuple[int, int]]:
        """Return anchored dynamic/static actor pairs overlapping in the authored pose.

        Culverin represents every rigid bond island by one complete outer convex
        hull. That hull can overlap a neighbouring static fragment even when no
        authored Dynamic-Static bond exists between those two logical bodies.
        The fixed anchor keeps the island in the authored pose, so resolving such
        initial support overlaps only injects an unwanted frame-2 correction.
        The returned pairs are filtered only while the dynamic actor still has at
        least one intact static anchor; topology rebuilds restore normal contact
        automatically after the last anchor breaks.
        """
        runtime_list = list(runtimes)
        nodes: Dict[int, List[_RuntimeBody]] = {}
        for runtime in runtime_list:
            nodes.setdefault(int(runtime.handle), []).append(runtime)

        anchored_dynamic_handles: set[int] = set()
        authored_pairs: set[Tuple[int, int]] = set()
        for bond in bonds:
            if bond.broken:
                continue
            a_dynamic = bond.body_a.body_type == "DYNAMIC"
            b_dynamic = bond.body_b.body_type == "DYNAMIC"
            if a_dynamic == b_dynamic:
                continue
            dynamic = bond.body_a if a_dynamic else bond.body_b
            static = bond.body_b if a_dynamic else bond.body_a
            dynamic_handle = int(dynamic.handle)
            static_handle = int(static.handle)
            if dynamic_handle == static_handle:
                continue
            anchored_dynamic_handles.add(dynamic_handle)
            authored_pairs.add((dynamic_handle, static_handle))
        if not anchored_dynamic_handles:
            return set()

        def actor_bounds(handle: int, members: Sequence[_RuntimeBody]):
            points: List[Tuple[float, float, float]] = []
            for runtime in members:
                pose = dynamic_poses.get(runtime.stable_id)
                if pose is None:
                    pose = cls._runtime_pose_jolt(world, runtime)
                points.extend(cls._cluster_outer_hull_points(runtime, pose[0], pose[1]))
            if not points:
                return None
            minimum = tuple(min(float(point[axis]) for point in points) for axis in range(3))
            maximum = tuple(max(float(point[axis]) for point in points) for axis in range(3))
            return minimum, maximum

        bounds_by_handle: Dict[int, Any] = {}
        for handle, members in nodes.items():
            bounds_by_handle[handle] = actor_bounds(handle, members)

        static_handles = [
            handle for handle, members in nodes.items()
            if members and all(member.body_type != "DYNAMIC" for member in members)
        ]

        def overlaps(first, second) -> bool:
            if first is None or second is None:
                return False
            first_min, first_max = first
            second_min, second_max = second
            margin = max(0.0, float(tolerance))
            return all(
                float(first_min[axis]) <= float(second_max[axis]) + margin
                and float(second_min[axis]) <= float(first_max[axis]) + margin
                for axis in range(3)
            )

        result: set[Tuple[int, int]] = set()
        for dynamic_handle in sorted(anchored_dynamic_handles):
            dynamic_bounds = bounds_by_handle.get(dynamic_handle)
            for static_handle in static_handles:
                if dynamic_handle == static_handle:
                    continue
                pair = (dynamic_handle, static_handle)
                if pair in authored_pairs:
                    continue
                if overlaps(dynamic_bounds, bounds_by_handle.get(static_handle)):
                    result.add(pair)
        return result


    @classmethod
    def _apply_rigid_static_anchor_collision_filters(
        cls,
        world,
        runtimes: Sequence[_RuntimeBody],
        bonds: Sequence[_RuntimeBond],
        *,
        initial_overlap_pairs: Optional[set[Tuple[int, int]]] = None,
    ) -> Dict[str, int]:
        """Disable contact solving between intact rigid anchors and their static bodies.

        A RIGID dynamic island is represented by one outer convex hull. That hull
        intentionally fills concavities and can overlap a static support that is
        already attached by a Fixed anchor. Letting both the contact solver and
        the Fixed constraint act on the same pair creates an immediate authored-
        pose correction on frame 2. A dedicated category bit per anchored dynamic
        actor lets the static endpoint reject only that actor while preserving its
        collisions with every other body. Filters are rebuilt after every bond
        topology change, so collision is restored as soon as the last anchor pair
        between two actors breaks.
        """
        runtime_list = list(runtimes)
        nodes: Dict[int, Dict[str, Any]] = {}
        for runtime in runtime_list:
            handle = int(runtime.handle)
            node = nodes.setdefault(handle, {"members": []})
            node["members"].append(runtime)

        for handle, node in nodes.items():
            members = list(node["members"])
            category = 0
            mask = 0xFFFF
            for runtime in members:
                category |= int(runtime.collision_category) & 0xFFFF
                mask &= int(runtime.collision_mask) & 0xFFFF
            node["category"] = max(1, category) & 0xFFFF
            node["mask"] = mask & 0xFFFF

        authored_pairs: set[Tuple[int, int]] = set()
        anchored_dynamic_handles: set[int] = set()
        for bond in bonds:
            if bond.broken:
                continue
            a_dynamic = bond.body_a.body_type == "DYNAMIC"
            b_dynamic = bond.body_b.body_type == "DYNAMIC"
            if a_dynamic == b_dynamic:
                continue
            dynamic = bond.body_a if a_dynamic else bond.body_b
            static = bond.body_b if a_dynamic else bond.body_a
            dynamic_handle = int(dynamic.handle)
            static_handle = int(static.handle)
            if dynamic_handle == static_handle or dynamic_handle not in nodes or static_handle not in nodes:
                continue
            authored_pairs.add((dynamic_handle, static_handle))
            anchored_dynamic_handles.add(dynamic_handle)

        overlap_pairs = {
            (int(dynamic_handle), int(static_handle))
            for dynamic_handle, static_handle in (initial_overlap_pairs or set())
            if int(dynamic_handle) in anchored_dynamic_handles
            and int(dynamic_handle) in nodes
            and int(static_handle) in nodes
            and int(dynamic_handle) != int(static_handle)
        }
        excluded_pairs = authored_pairs | overlap_pairs

        used_category_bits = 0
        for node in nodes.values():
            used_category_bits |= int(node["category"]) & 0xFFFF
        available_bits = [
            1 << index for index in range(16)
            if not (used_category_bits & (1 << index))
        ]

        selected_handles = sorted(anchored_dynamic_handles)[:len(available_bits)]
        category_override = {
            handle: int(available_bits[index])
            for index, handle in enumerate(selected_handles)
        }
        active_category = {
            handle: int(category_override.get(handle, node["category"])) & 0xFFFF
            for handle, node in nodes.items()
        }

        effective_exclusions = {
            pair for pair in excluded_pairs if pair[0] in category_override
        }
        active_masks: Dict[int, int] = {}
        for handle, node in nodes.items():
            mask = 0
            for other_handle, other_node in nodes.items():
                if other_handle == handle:
                    continue
                original_pair_enabled = (
                    bool(int(node["mask"]) & int(other_node["category"]))
                    and bool(int(other_node["mask"]) & int(node["category"]))
                )
                if not original_pair_enabled:
                    continue
                if (handle, other_handle) in effective_exclusions or (other_handle, handle) in effective_exclusions:
                    continue
                mask |= int(active_category[other_handle])
            active_masks[handle] = mask & 0xFFFF

        for handle in sorted(nodes):
            world.set_collision_filter(
                int(handle),
                int(active_category[handle]) & 0xFFFF,
                int(active_masks[handle]) & 0xFFFF,
            )

        return {
            "requested_pairs": len(excluded_pairs),
            "authored_pairs": len(authored_pairs),
            "initial_overlap_pairs": len(overlap_pairs - authored_pairs),
            "filtered_pairs": len(effective_exclusions),
            "filtered_dynamic_actors": len(category_override),
            "overflow_dynamic_actors": max(0, len(anchored_dynamic_handles) - len(category_override)),
            "available_category_bits": len(available_bits),
        }


    @classmethod
    def _rebuild_rigid_bond_clusters(
        cls,
        culverin,
        world,
        runtimes: Sequence[_RuntimeBody],
        bonds: Sequence[_RuntimeBond],
        handle_to_name: Dict[int, Any],
        warnings: List[str],
        *,
        allow_initial_sleep: bool = True,
    ) -> Dict[str, int]:
        """Represent every intact dynamic bond island as one native rigid body."""
        all_dynamic = [runtime for runtime in runtimes if runtime.body_type == "DYNAMIC"]
        bonded_dynamic_ids = {
            endpoint.stable_id
            for bond in bonds
            for endpoint in (bond.body_a, bond.body_b)
            if endpoint.body_type == "DYNAMIC"
        }
        # External projectiles, wrecking balls and other authored dynamic bodies
        # that do not participate in the fracture bond graph must keep their
        # native handles. Recreating every dynamic actor invalidated authored
        # Distance constraints whenever any fracture bond broke.
        dynamic = [
            runtime for runtime in all_dynamic
            if runtime.stable_id in bonded_dynamic_ids
        ]
        preserved_external_dynamic = [
            runtime for runtime in all_dynamic
            if runtime.stable_id not in bonded_dynamic_ids
        ]
        parent = {runtime.stable_id: runtime.stable_id for runtime in dynamic}

        def find(value: str) -> str:
            root = value
            while parent[root] != root:
                root = parent[root]
            while parent[value] != value:
                next_value = parent[value]
                parent[value] = root
                value = next_value
            return root

        def union(first: str, second: str) -> None:
            root_a = find(first)
            root_b = find(second)
            if root_a == root_b:
                return
            if root_a < root_b:
                parent[root_b] = root_a
            else:
                parent[root_a] = root_b

        for bond in bonds:
            if bond.broken:
                continue
            a_dynamic = bond.body_a.stable_id in parent
            b_dynamic = bond.body_b.stable_id in parent
            if a_dynamic and b_dynamic:
                union(bond.body_a.stable_id, bond.body_b.stable_id)

        components: Dict[str, List[_RuntimeBody]] = {}
        for runtime in dynamic:
            components.setdefault(find(runtime.stable_id), []).append(runtime)
        ordered_components = [sorted(component, key=lambda item: item.stable_id) for component in components.values()]
        ordered_components.sort(key=lambda component: component[0].stable_id)

        poses = {runtime.stable_id: cls._runtime_pose_jolt(world, runtime) for runtime in dynamic}
        velocities = {runtime.stable_id: cls._runtime_velocity_jolt(world, runtime) for runtime in dynamic}

        # Existing rigid static anchors reference the soon-to-be-replaced dynamic
        # actor handles. Remove them before rebuilding the island topology.
        for bond in bonds:
            if not bond.solver_bound or not bond.handle:
                continue
            try:
                world.destroy_constraint(int(bond.handle))
            except Exception:
                pass
            bond.handle = 0
            bond.solver_bound = False

        old_handles = sorted({int(runtime.handle) for runtime in dynamic})
        for handle in old_handles:
            try:
                world.destroy_body(handle)
            except Exception as exc:
                warnings.append(f"Rigid bond island: failed to destroy native body {handle}: {exc}")

        clusters: List[_RuntimeCluster] = []
        recreated_singletons = 0
        clustered_bodies = 0
        initially_supported_clusters = 0
        for component_index, component in enumerate(ordered_components):
            if len(component) == 1:
                runtime = component[0]
                position_jolt, rotation_jolt = poses[runtime.stable_id]
                linear_jolt, angular_jolt = velocities[runtime.stable_id]
                rotation_blender = jolt_quat_to_blender(rotation_jolt)
                com_blender = jolt_vec_to_blender(position_jolt)
                origin_blender = subtract_vec3(
                    com_blender,
                    quat_rotate_vector_wxyz(rotation_blender, runtime.com_offset_local),
                )
                source = dict(runtime.source_body or {})
                source.update({
                    "location": list(origin_blender),
                    "rotation": list(rotation_blender),
                    "linear_velocity": list(jolt_vec_to_blender(linear_jolt)),
                    "angular_velocity": list(jolt_vec_to_blender(angular_jolt)),
                })
                replacement = cls._create_body(culverin, world, source, 100000 + component_index, warnings)
                runtime.handle = replacement.handle
                runtime.handles = replacement.handles
                runtime.constraint_handles = ()
                runtime.buffer_index = -1
                runtime.cluster = None
                runtime.input_location = tuple(origin_blender)
                runtime.input_rotation = tuple(rotation_blender)
                runtime.linear_damping = replacement.linear_damping
                runtime.angular_damping = replacement.angular_damping
                runtime.rest_position_jolt = replacement.rest_position_jolt
                runtime.rest_rotation_jolt = replacement.rest_rotation_jolt
                runtime.source_body = replacement.source_body
                recreated_singletons += 1
                continue

            total_mass = sum(max(1.0e-8, float(runtime.mass)) for runtime in component)
            cluster_position = tuple(
                sum(float(poses[runtime.stable_id][0][axis]) * max(1.0e-8, float(runtime.mass)) for runtime in component)
                / total_mass
                for axis in range(3)
            )
            local_positions: Dict[str, Tuple[float, float, float]] = {}
            local_rotations: Dict[str, Tuple[float, float, float, float]] = {}
            outer_points: List[Tuple[float, float, float]] = []
            linear_sum = [0.0, 0.0, 0.0]
            angular_sum = [0.0, 0.0, 0.0]
            friction_sum = 0.0
            linear_damping_sum = 0.0
            angular_damping_sum = 0.0
            restitution = 0.0
            ccd = False
            category = int(component[0].collision_category)
            mask = int(component[0].collision_mask)
            for runtime in component:
                position_jolt, rotation_jolt = poses[runtime.stable_id]
                linear_jolt, angular_jolt = velocities[runtime.stable_id]
                mass = max(1.0e-8, float(runtime.mass))
                outer_points.extend(cls._cluster_outer_hull_points(
                    runtime, position_jolt, rotation_jolt
                ))
                for axis in range(3):
                    linear_sum[axis] += float(linear_jolt[axis]) * mass
                    angular_sum[axis] += float(angular_jolt[axis]) * mass
                source = runtime.source_body or {}
                friction_sum += max(0.0, float(source.get("friction", 0.2))) * mass
                linear_damping_sum += max(0.0, float(source.get("linear_damping", runtime.linear_damping))) * mass
                angular_damping_sum += max(0.0, float(source.get("angular_damping", runtime.angular_damping))) * mass
                restitution = max(restitution, max(0.0, min(1.0, float(source.get("restitution", 0.0)))))
                ccd = ccd or bool(source.get("ccd", runtime.ccd))
                category |= int(runtime.collision_category)
                mask &= int(runtime.collision_mask)
            if len(outer_points) < 4:
                raise BackendError("Rigid bond island contains no usable outer hull points.")

            minimum = tuple(min(point[axis] for point in outer_points) for axis in range(3))
            maximum = tuple(max(point[axis] for point in outer_points) for axis in range(3))
            hull_origin = tuple((minimum[axis] + maximum[axis]) * 0.5 for axis in range(3))
            point_values = array.array("f")
            for point in outer_points:
                point_values.extend(subtract_vec3(point, hull_origin))
            handle = int(world.create_convex_hull(
                pos=hull_origin,
                rot=(0.0, 0.0, 0.0, 1.0),
                points=point_values.tobytes(),
                motion=culverin.MOTION_DYNAMIC,
                mass=total_mass,
                user_data=200000 + component_index,
                category=max(1, category) & 0xFFFF,
                mask=mask & 0xFFFF,
                friction=friction_sum / total_mass,
                restitution=restitution,
                ccd=ccd,
            ))
            # Flush creation so the exact solver COM is available before logical
            # member frames are attached to the actor.
            world.step(0.0)
            actor_position = tuple(map(float, (world.get_position(handle) or hull_origin)[:3]))
            actor_rotation = _quat_normalize_xyzw(
                world.get_rotation(handle) or (0.0, 0.0, 0.0, 1.0)
            )
            inverse_actor_rotation = _quat_conjugate_xyzw(actor_rotation)
            for runtime in component:
                member_position, member_rotation = poses[runtime.stable_id]
                local_positions[runtime.stable_id] = _quat_rotate_xyzw(
                    inverse_actor_rotation, subtract_vec3(member_position, actor_position)
                )
                local_rotations[runtime.stable_id] = _quat_normalize_xyzw(
                    _quat_multiply_xyzw(inverse_actor_rotation, member_rotation)
                )
            geometric_com_offset = subtract_vec3(actor_position, cluster_position)
            warnings.append(
                f"Rigid bond island {component_index}: using one complete outer convex hull "
                f"from {len(outer_points)} authored collider points; interior primitive proxies were disabled."
            )

            cluster = _RuntimeCluster(
                stable_id=f"bond-island:{component[0].stable_id}",
                handle=handle,
                members=component,
                local_positions_jolt=local_positions,
                local_rotations_jolt=local_rotations,
                mass=total_mass,
                linear_damping=linear_damping_sum / total_mass,
                angular_damping=angular_damping_sum / total_mass,
            )
            clusters.append(cluster)
            average_linear = tuple(value / total_mass for value in linear_sum)
            average_angular = tuple(value / total_mass for value in angular_sum)
            body_linear = add_vec3(average_linear, _cross_vec3(average_angular, geometric_com_offset))
            world.set_linear_velocity(handle, *body_linear)
            world.set_angular_velocity(handle, *average_angular)
            starts_supported = bool(allow_initial_sleep) and cls._rigid_component_starts_supported(
                component, poses, velocities,
                [runtime for runtime in runtimes if runtime.body_type != "DYNAMIC"],
            )
            if starts_supported:
                world.set_linear_velocity(handle, 0.0, 0.0, 0.0)
                world.set_angular_velocity(handle, 0.0, 0.0, 0.0)
                world.deactivate(handle)
                initially_supported_clusters += 1
            else:
                world.activate(handle)
            for runtime in component:
                runtime.handle = handle
                runtime.handles = (handle,)
                runtime.constraint_handles = ()
                runtime.buffer_index = -1
                runtime.cluster = cluster
                clustered_bodies += 1

        world.step(0.0)
        anchor_stats = cls._create_rigid_static_anchor_constraints(
            culverin, world, bonds, warnings, constraint_limit=256
        )
        initial_static_overlap_pairs = cls._rigid_anchor_initial_static_overlap_pairs(
            world, runtimes, bonds, poses
        )
        anchor_filter_stats = cls._apply_rigid_static_anchor_collision_filters(
            world, runtimes, bonds,
            initial_overlap_pairs=initial_static_overlap_pairs,
        )
        if int(anchor_filter_stats.get("overflow_dynamic_actors", 0)) > 0:
            warnings.append(
                "Rigid Dynamic-Static anchor collision filtering exceeded the available "
                "Jolt category bits; some anchored actors still collide with their static endpoints."
            )
        world.step(0.0)
        for runtime in runtimes:
            try:
                runtime.buffer_index = int(world.get_index(runtime.handle))
            except (TypeError, ValueError):
                runtime.buffer_index = -1
            if runtime.cluster is not None:
                runtime.cluster.buffer_index = runtime.buffer_index
        cls._refresh_handle_map(runtimes, handle_to_name)
        return {
            "clusters": len(clusters),
            "clustered_bodies": clustered_bodies,
            "singletons": recreated_singletons,
            "preserved_external_dynamic_bodies": len(preserved_external_dynamic),
            "native_dynamic_bodies": (
                len(preserved_external_dynamic) + len(clusters) + recreated_singletons
            ),
            "static_anchor_bonds": int(anchor_stats.get("requested", 0)),
            "static_anchor_constraints": int(anchor_stats.get("created", 0)),
            "static_anchor_omitted": int(anchor_stats.get("omitted", 0)),
            "static_anchor_pairs": int(anchor_stats.get("anchored_pairs", 0)),
            "static_anchor_collision_filter_requested_pairs": int(
                anchor_filter_stats.get("requested_pairs", 0)
            ),
            "static_anchor_initial_overlap_pairs": int(
                anchor_filter_stats.get("initial_overlap_pairs", 0)
            ),
            "static_anchor_collision_filter_pairs": int(
                anchor_filter_stats.get("filtered_pairs", 0)
            ),
            "static_anchor_collision_filter_dynamic_actors": int(
                anchor_filter_stats.get("filtered_dynamic_actors", 0)
            ),
            "static_anchor_collision_filter_overflow": int(
                anchor_filter_stats.get("overflow_dynamic_actors", 0)
            ),
            "unsupported_static_bonds": int(anchor_stats.get("omitted", 0)),
            "initially_supported_clusters": initially_supported_clusters,
        }

    @classmethod
    def _body_pose_jolt(
        cls,
        world,
        body: _RuntimeBody,
        cache: Dict[str, Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]],
    ) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
        cached = cache.get(body.stable_id)
        if cached is not None:
            return cached
        result = cls._runtime_pose_jolt(world, body)
        cache[body.stable_id] = result
        return result

    @classmethod
    def _current_bond_frame(
        cls,
        world,
        bond: _RuntimeBond,
        pose_cache: Dict[str, Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]],
    ) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        position_a, rotation_a = cls._body_pose_jolt(world, bond.body_a, pose_cache)
        position_b, rotation_b = cls._body_pose_jolt(world, bond.body_b, pose_cache)
        anchor_a = add_vec3(position_a, _quat_rotate_xyzw(rotation_a, bond.anchor_local_a_jolt))
        anchor_b = add_vec3(position_b, _quat_rotate_xyzw(rotation_b, bond.anchor_local_b_jolt))
        normal_a = _quat_rotate_xyzw(rotation_a, bond.normal_local_a_jolt)
        normal_b = _quat_rotate_xyzw(rotation_b, bond.normal_local_b_jolt)
        anchor_jolt = scale_vec3(add_vec3(anchor_a, anchor_b), 0.5)
        normal_jolt = _normalize_vec3(add_vec3(normal_a, normal_b))
        return jolt_vec_to_blender(anchor_jolt), _normalize_vec3(jolt_vec_to_blender(normal_jolt))

    @classmethod
    def _evaluate_breakable_bonds(
        cls,
        world,
        bonds: Sequence[_RuntimeBond],
        body_contacts: Mapping[str, Sequence[Mapping[str, Any]]],
        runtime_by_name: Mapping[str, _RuntimeBody],
        pre_step_motion: Mapping[str, Mapping[str, Sequence[float]]],
        step_dt: float,
        frame: int,
        substep: int,
        log,
    ) -> List[Dict[str, Any]]:
        intact = [bond for bond in bonds if not bond.broken]
        if not intact or not body_contacts:
            return []
        degree: Dict[str, int] = {}
        component_parent: Dict[str, str] = {}

        def component_find(value: str) -> str:
            component_parent.setdefault(value, value)
            root = value
            while component_parent[root] != root:
                root = component_parent[root]
            while component_parent[value] != value:
                next_value = component_parent[value]
                component_parent[value] = root
                value = next_value
            return root

        def component_union(first: str, second: str) -> None:
            root_a = component_find(first)
            root_b = component_find(second)
            if root_a != root_b:
                if root_a < root_b:
                    component_parent[root_b] = root_a
                else:
                    component_parent[root_a] = root_b

        static_anchor_degree: Dict[str, int] = {}
        for bond in intact:
            degree[bond.body_a.name] = degree.get(bond.body_a.name, 0) + 1
            degree[bond.body_b.name] = degree.get(bond.body_b.name, 0) + 1
            component_union(bond.body_a.name, bond.body_b.name)
            a_dynamic = bond.body_a.body_type == "DYNAMIC"
            b_dynamic = bond.body_b.body_type == "DYNAMIC"
            if a_dynamic != b_dynamic:
                dynamic = bond.body_a if a_dynamic else bond.body_b
                key = dynamic.cluster.stable_id if dynamic.cluster is not None else dynamic.stable_id
                static_anchor_degree[str(key)] = static_anchor_degree.get(str(key), 0) + 1
        dt = max(1.0e-8, float(step_dt))
        broken_events: List[Dict[str, Any]] = []
        pose_cache: Dict[str, Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]] = {}
        contact_mass_cache: Dict[str, float] = {}

        def contact_mass(name: str) -> float:
            cached = contact_mass_cache.get(name)
            if cached is not None:
                return cached
            runtime = runtime_by_name.get(name)
            if runtime is None or runtime.body_type != "DYNAMIC":
                value = float("inf")
            elif runtime.cluster is not None:
                value = max(1.0e-8, float(runtime.cluster.mass))
            else:
                value = max(1.0e-8, float(runtime.mass))
            contact_mass_cache[name] = value
            return value

        def reduced_contact_mass(first_name: str, second_name: str) -> float:
            first_mass = contact_mass(first_name)
            second_mass = contact_mass(second_name)
            if math.isinf(first_mass) and math.isinf(second_mass):
                return 0.0
            if math.isinf(first_mass):
                return second_mass
            if math.isinf(second_mass):
                return first_mass
            return (first_mass * second_mass) / max(1.0e-8, first_mass + second_mass)
        for bond in intact:
            bond_anchor, bond_normal = cls._current_bond_frame(world, bond, pose_cache)
            best: Optional[Dict[str, Any]] = None
            for endpoint, other in ((bond.body_a, bond.body_b), (bond.body_b, bond.body_a)):
                mixed_static_anchor = (endpoint.body_type == "DYNAMIC") != (other.body_type == "DYNAMIC")
                # Static support contacts do not load the authored anchor. For a
                # rigid dynamic island, however, an impact on any member is
                # transmitted through the one compound actor to every static
                # anchor attached to that island.
                if mixed_static_anchor and endpoint.body_type != "DYNAMIC":
                    continue
                if mixed_static_anchor and endpoint.cluster is not None:
                    contact_names = [member.name for member in endpoint.cluster.members]
                    anchor_key = str(endpoint.cluster.stable_id)
                    shared_load = max(1, int(static_anchor_degree.get(anchor_key, 1)))
                else:
                    contact_names = [endpoint.name]
                    shared_load = max(1, int(degree.get(endpoint.name, 1)))
                for contact_name in contact_names:
                    for contact in body_contacts.get(contact_name, ()):
                        contact_other = str(contact.get("other", ""))
                        if contact_other == other.name:
                            continue
                        if (
                            contact_other in component_parent
                            and component_find(contact_other) == component_find(endpoint.name)
                        ):
                            continue
                        impulse = abs(float(contact.get("impulse", 0.0)))
                        point = tuple(map(float, contact.get("position", bond_anchor)))
                        normal = tuple(map(float, contact.get("normal", (0.0, 0.0, 0.0))))
                        normal_length = math.sqrt(sum(value * value for value in normal))
                        relative_velocity = tuple(map(float, contact.get("relative_velocity", (0.0, 0.0, 0.0))))
                        first_velocity = cls._contact_point_velocity(pre_step_motion.get(contact_name), point)
                        second_velocity = cls._contact_point_velocity(pre_step_motion.get(contact_other), point)
                        pre_step_relative_velocity = subtract_vec3(first_velocity, second_velocity)
                        relative_normal_speed = 0.0
                        raw_relative_normal_speed = 0.0
                        pre_step_relative_normal_speed = 0.0
                        if normal_length > 1.0e-12:
                            raw_relative_normal_speed = abs(
                                sum(relative_velocity[i] * normal[i] for i in range(3)) / normal_length
                            )
                            pre_step_relative_normal_speed = abs(
                                sum(pre_step_relative_velocity[i] * normal[i] for i in range(3)) / normal_length
                            )
                            relative_normal_speed = max(raw_relative_normal_speed, pre_step_relative_normal_speed)
                        reduced_mass = reduced_contact_mass(contact_name, contact_other)
                        momentum_impulse = reduced_mass * relative_normal_speed
                        effective_impulse = max(impulse, momentum_impulse)
                        if effective_impulse <= 0.0:
                            continue
                        bond_length = math.sqrt(sum(value * value for value in bond_normal))
                        alignment = 0.0
                        if normal_length > 1.0e-12 and bond_length > 1.0e-12:
                            alignment = abs(sum(normal[i] * bond_normal[i] for i in range(3)) / (normal_length * bond_length))
                        direction_factor = 0.35 + 0.65 * min(1.0, alignment)
                        estimated_force = (effective_impulse / dt) * direction_factor / shared_load
                        lever = math.sqrt(sum((point[i] - bond_anchor[i]) ** 2 for i in range(3)))
                        estimated_torque = estimated_force * lever
                        candidate_force_ratio = estimated_force / bond.break_force if bond.break_force > 0.0 else 0.0
                        candidate_torque_ratio = estimated_torque / bond.break_torque if bond.break_torque > 0.0 else 0.0
                        candidate = {
                            "endpoint": contact_name,
                            "anchor_endpoint": endpoint.name,
                            "other": contact_other,
                            "impulse": impulse,
                            "effective_impulse": effective_impulse,
                            "momentum_impulse": momentum_impulse,
                            "reduced_mass": reduced_mass,
                            "relative_normal_speed": relative_normal_speed,
                            "raw_relative_normal_speed": raw_relative_normal_speed,
                            "pre_step_relative_normal_speed": pre_step_relative_normal_speed,
                            "position": list(point),
                            "normal": list(normal),
                            "bond_anchor": list(bond_anchor),
                            "bond_normal": list(bond_normal),
                            "estimated_force": estimated_force,
                            "estimated_torque": estimated_torque,
                            "load_ratio": max(candidate_force_ratio, candidate_torque_ratio),
                        }
                        if best is None or float(candidate["load_ratio"]) > float(best["load_ratio"]):
                            best = candidate
            if best is None:
                continue
            estimated_force = float(best["estimated_force"])
            estimated_torque = float(best["estimated_torque"])
            bond.peak_force = max(bond.peak_force, estimated_force)
            bond.peak_torque = max(bond.peak_torque, estimated_torque)
            force_ratio = estimated_force / bond.break_force if bond.break_force > 0.0 else 0.0
            torque_ratio = estimated_torque / bond.break_torque if bond.break_torque > 0.0 else 0.0
            load_ratio = max(force_ratio, torque_ratio)
            if bond.damage_accumulation > 0.0 and load_ratio > 0.25:
                bond.damage += (load_ratio * load_ratio) * dt * bond.damage_accumulation
            should_break = load_ratio >= 1.0 or bond.damage >= 1.0
            if not should_break:
                continue
            was_solver_bound = bool(bond.solver_bound and bond.handle)
            if was_solver_bound:
                try:
                    world.destroy_constraint(int(bond.handle))
                    bond.handle = 0
                    bond.solver_bound = False
                except Exception as exc:
                    log(
                        "BOND_BREAK_FAILED",
                        level="WARNING",
                        bond_id=bond.stable_id,
                        frame=int(frame),
                        substep=int(substep),
                        error=str(exc),
                    )
                    continue
            bond.broken = True
            bond.broken_frame = int(frame)
            bond.broken_substep = int(substep)
            for runtime in (bond.body_a, bond.body_b):
                if runtime.body_type == "DYNAMIC":
                    try:
                        world.activate(int(runtime.handle))
                    except Exception:
                        pass
            event = {
                "event_type": "BOND_BREAK",
                "bond_id": bond.stable_id,
                "body_a": bond.body_a.name,
                "body_b": bond.body_b.name,
                "frame": int(frame),
                "substep": int(substep),
                "estimated_force": estimated_force,
                "estimated_torque": estimated_torque,
                "break_force": bond.break_force,
                "break_torque": bond.break_torque,
                "solver_constraint": was_solver_bound,
                "damage": bond.damage,
                **best,
            }
            broken_events.append(event)
            log("BOND_BREAK", **event)
        return broken_events

    @staticmethod
    def _runtime_handles(runtime: _RuntimeBody) -> Tuple[int, ...]:
        if runtime.cluster is not None:
            return (int(runtime.cluster.handle),)
        return runtime.handles or (int(runtime.handle),)

    @staticmethod
    def _buffer_views(world):
        try:
            return (
                memoryview(world.positions),
                memoryview(world.rotations),
                memoryview(world.velocities),
                memoryview(world.angular_velocities),
            )
        except Exception:
            return None

    @staticmethod
    def _buffer_vec3(view, index: int):
        if view is None or index < 0:
            return None
        base = index * 4
        try:
            return (float(view[base]), float(view[base + 1]), float(view[base + 2]))
        except (IndexError, TypeError, ValueError):
            return None

    @staticmethod
    def _buffer_quat(view, index: int):
        if view is None or index < 0:
            return None
        base = index * 4
        try:
            return (float(view[base]), float(view[base + 1]), float(view[base + 2]), float(view[base + 3]))
        except (IndexError, TypeError, ValueError):
            return None

    @classmethod
    def _calibrate_com_offsets(cls, world, runtimes: Iterable[_RuntimeBody]) -> None:
        buffers = cls._buffer_views(world)
        positions = buffers[0] if buffers else None
        rotations = buffers[1] if buffers else None
        for runtime in runtimes:
            position = cls._buffer_vec3(positions, runtime.buffer_index) or world.get_position(runtime.handle)
            rotation = cls._buffer_quat(rotations, runtime.buffer_index) or world.get_rotation(runtime.handle)
            if position is None or rotation is None:
                continue
            com_world_blender = jolt_vec_to_blender(position)
            initial_rotation = jolt_quat_to_blender(rotation)
            world_delta = subtract_vec3(com_world_blender, runtime.input_location)
            local_delta = quat_rotate_vector_wxyz(quat_conjugate_wxyz(initial_rotation), world_delta)
            runtime.com_offset_local = local_delta

    @classmethod
    def _capture_rest_transforms(cls, world, runtimes: Iterable[_RuntimeBody]) -> None:
        """Capture native COM transforms before the first simulated step.

        Rigid bond stabilization works in solver coordinates and must preserve
        the relative COM transform rather than Blender object origins. Fracture
        pieces commonly share one object origin while their collider centers
        are distributed through the source object.
        """
        buffers = cls._buffer_views(world)
        positions = buffers[0] if buffers else None
        rotations = buffers[1] if buffers else None
        for runtime in runtimes:
            position = cls._buffer_vec3(positions, runtime.buffer_index) or world.get_position(runtime.handle)
            rotation = cls._buffer_quat(rotations, runtime.buffer_index) or world.get_rotation(runtime.handle)
            if position is None:
                position = blender_vec_to_jolt(
                    add_vec3(
                        runtime.input_location,
                        quat_rotate_vector_wxyz(runtime.input_rotation, runtime.com_offset_local),
                    )
                )
            if rotation is None:
                rotation = blender_quat_to_jolt(runtime.input_rotation)
            runtime.rest_position_jolt = tuple(map(float, position[:3]))
            runtime.rest_rotation_jolt = _quat_normalize_xyzw(rotation)

    @staticmethod
    def _input_snapshot_and_values(
        runtimes: Iterable[_RuntimeBody],
    ) -> Tuple[Dict[str, Dict[str, List[float]]], array.array]:
        """Store frame one exactly as supplied by Blender.

        Culverin's zero-copy transform buffers are populated only after the
        first native step. Reading them immediately after body creation returned
        zero transforms for every body and collapsed cache frame one to the
        origin. The initial cache sample must therefore come from the immutable
        input transforms carried by each runtime body.
        """
        result: Dict[str, Dict[str, List[float]]] = {}
        values = array.array("f")
        for runtime in runtimes:
            location_values = [float(value) for value in runtime.input_location]
            rotation_values = [float(value) for value in runtime.input_rotation]
            result[runtime.name] = {
                "location": location_values,
                "rotation": rotation_values,
                "scale": list(runtime.scale),
            }
            values.extend((*location_values[:3], *rotation_values[:4]))
        return result, values

    @classmethod
    def _snapshot_and_values(
        cls, world, runtimes: Iterable[_RuntimeBody]
    ) -> Tuple[Dict[str, Dict[str, List[float]]], array.array]:
        """Read transforms once and produce both playback dictionaries and cache floats."""
        result: Dict[str, Dict[str, List[float]]] = {}
        values = array.array("f")
        buffers = cls._buffer_views(world)
        positions = buffers[0] if buffers else None
        rotations = buffers[1] if buffers else None
        for runtime in runtimes:
            if runtime.cluster is not None:
                position, rotation = cls._runtime_pose_jolt(world, runtime)
            else:
                position = cls._buffer_vec3(positions, runtime.buffer_index) or world.get_position(runtime.handle)
                rotation = cls._buffer_quat(rotations, runtime.buffer_index) or world.get_rotation(runtime.handle)
            if position is None or rotation is None:
                origin_world = runtime.input_location
                rotation_blender = runtime.input_rotation
            else:
                rotation_blender = jolt_quat_to_blender(rotation)
                com_world_blender = jolt_vec_to_blender(position)
                offset_world = quat_rotate_vector_wxyz(rotation_blender, runtime.com_offset_local)
                origin_world = subtract_vec3(com_world_blender, offset_world)
            location_values = [float(value) for value in origin_world]
            rotation_values = [float(value) for value in rotation_blender]
            result[runtime.name] = {
                "location": location_values,
                "rotation": rotation_values,
                "scale": list(runtime.scale),
            }
            values.extend((*location_values[:3], *rotation_values[:4]))
        return result, values

    @classmethod
    def _snapshot(cls, world, runtimes: Iterable[_RuntimeBody]) -> Dict[str, Dict[str, List[float]]]:
        return cls._snapshot_and_values(world, runtimes)[0]

    @staticmethod
    def _managed_ground_levels(runtimes: Iterable[_RuntimeBody]) -> List[float]:
        """Return horizontal managed-ground heights in Blender coordinates.

        KA's generated ground is an identity-oriented infinite plane. Keeping the
        extraction explicit prevents this fallback from affecting arbitrary static
        meshes or user-authored tilted planes.
        """
        levels: List[float] = []
        for runtime in runtimes:
            if runtime.body_type != "STATIC":
                continue
            source = dict(runtime.source_body or {})
            if not bool(source.get("managed_ground", False)):
                continue
            if str(source.get("collision_shape", "")) != "PLANE":
                continue
            rotation = tuple(map(float, source.get("rotation", (1.0, 0.0, 0.0, 0.0))))
            # Managed ground must remain horizontal. Ignore an accidentally
            # rotated legacy object rather than applying a wrong vertical guard.
            normal = quat_rotate_vector_wxyz(rotation, (0.0, 0.0, 1.0))
            if abs(float(normal[2])) < 0.9999:
                continue
            location = tuple(map(float, source.get("location", (0.0, 0.0, 0.0))))
            center = tuple(map(float, source.get("shape_center", (0.0, 0.0, 0.0))))
            world_center = add_vec3(location, quat_rotate_vector_wxyz(rotation, center))
            levels.append(float(world_center[2]))
        return sorted(set(round(value, 9) for value in levels))

    @classmethod
    def _apply_culverin_ground_contact_compensation(
        cls,
        runtimes: Sequence[_RuntimeBody],
        snapshot: Optional[Dict[str, Dict[str, List[float]]]],
        frame_values: array.array,
        ground_levels: Sequence[float],
    ) -> Dict[str, float]:
        """Keep sharp rendered hulls above managed ground in cached output.

        Culverin 0.13.2 does not expose Jolt's ConvexHullShapeSettings
        ``mMaxConvexRadius``. Jolt therefore rounds convex-hull corners inward.
        The physical rounded shape can correctly rest on the plane while a sharp
        source-hull vertex is visibly below it. This method compensates only the
        cached Blender transforms and never teleports the native simulation.

        Members of one rigid bond cluster receive one common vertical offset so
        intact fragments cannot separate visually. The optional native ABI bridge
        sets the convex radius to zero and therefore does not use this fallback.
        """
        if not ground_levels or len(frame_values) < len(runtimes) * 7:
            return {"corrected_groups": 0.0, "corrected_bodies": 0.0, "max_correction": 0.0}

        ground_z = max(float(value) for value in ground_levels)
        threshold = 1.0e-4
        group_corrections: Dict[Tuple[str, int], float] = {}
        group_members: Dict[Tuple[str, int], List[int]] = {}

        for index, runtime in enumerate(runtimes):
            if runtime.body_type != "DYNAMIC":
                continue
            source = dict(runtime.source_body or {})
            if str(source.get("collision_shape", "")) not in {"CONVEX_HULL", "COMPOUND_CONVEX"}:
                continue
            points = list(source.get("convex_vertices", []) or [])
            if len(points) < 4:
                continue
            base = index * 7
            location = tuple(float(frame_values[base + axis]) for axis in range(3))
            rotation = tuple(float(frame_values[base + 3 + axis]) for axis in range(4))
            minimum_z = min(
                location[2] + quat_rotate_vector_wxyz(rotation, point)[2]
                for point in points
            )
            if minimum_z >= ground_z - threshold:
                continue

            quality = dict(source.get("collider_quality", {}) or {})
            approximation_margin = max(
                0.0,
                float(quality.get("separation_inset_applied", 0.0) or 0.0)
                + float(quality.get("max_error", 0.0) or 0.0),
            )
            # Avoid visible hover from unusually loose quality settings while
            # still covering the millimetre-scale fracture inset and fitting error.
            approximation_margin = min(0.005, approximation_margin)
            correction = ground_z + approximation_margin - minimum_z
            if correction <= 0.0:
                continue

            cluster = runtime.cluster
            key = ("cluster", int(cluster.handle)) if cluster is not None else ("body", int(runtime.handle))
            group_corrections[key] = max(group_corrections.get(key, 0.0), float(correction))
            group_members.setdefault(key, []).append(index)

        corrected_indices = set()
        for key, correction in group_corrections.items():
            if correction <= 0.0:
                continue
            # A cluster key may initially contain only the penetrating member.
            # Apply the same shift to every logical member attached to that actor.
            if key[0] == "cluster":
                indices = [
                    index for index, runtime in enumerate(runtimes)
                    if runtime.cluster is not None and int(runtime.cluster.handle) == key[1]
                ]
            else:
                indices = group_members.get(key, [])
            for index in indices:
                base = index * 7
                frame_values[base + 2] = float(frame_values[base + 2]) + correction
                runtime = runtimes[index]
                if snapshot is not None and runtime.name in snapshot:
                    snapshot[runtime.name]["location"][2] = (
                        float(snapshot[runtime.name]["location"][2]) + correction
                    )
                corrected_indices.add(index)

        return {
            "corrected_groups": float(len(group_corrections)),
            "corrected_bodies": float(len(corrected_indices)),
            "max_correction": float(max(group_corrections.values(), default=0.0)),
        }

    @staticmethod
    def _active_buffer_indices(world) -> Optional[set[int]]:
        try:
            raw = world.get_active_indices()
            values = array.array("I")
            values.frombytes(raw)
            return set(int(value) for value in values)
        except Exception:
            return None

    @classmethod
    def _adaptive_substep_count(
        cls, world, runtimes: Iterable[_RuntimeBody], frame_dt: float, minimum: int, maximum: int, gravity: float
    ) -> int:
        """Estimate the required solver frequency from motion, not body count.

        The previous dense-scene guard forced every frame to the maximum whenever
        more than roughly one third of the bodies were active. A single jittering
        contact island therefore prevented adaptation for the complete bake.
        """
        if maximum <= minimum:
            return maximum
        buffers = cls._buffer_views(world)
        velocities = buffers[2] if buffers else None
        angular_velocities = buffers[3] if buffers else None
        active_indices = cls._active_buffer_indices(world)
        max_linear = 0.0
        max_angular = 0.0
        minimum_feature = float("inf")
        active_count = 0
        active_ccd = False
        for runtime in runtimes:
            if runtime.body_type != "DYNAMIC":
                continue
            is_active = (
                runtime.buffer_index in active_indices
                if active_indices is not None and runtime.buffer_index >= 0
                else bool(world.is_active(runtime.handle))
            )
            if not is_active:
                continue
            active_count += 1
            linear = cls._buffer_vec3(velocities, runtime.buffer_index) or world.get_velocity(runtime.handle) or (0.0, 0.0, 0.0)
            angular = cls._buffer_vec3(angular_velocities, runtime.buffer_index) or world.get_angular_velocity(runtime.handle) or (0.0, 0.0, 0.0)
            max_linear = max(max_linear, length_vec3(linear))
            max_angular = max(max_angular, length_vec3(angular))
            active_ccd = active_ccd or bool(runtime.ccd)
            if not runtime.ccd:
                minimum_feature = min(minimum_feature, runtime.radius)
        if active_count == 0:
            return minimum
        if not math.isfinite(minimum_feature):
            minimum_feature = min((r.radius for r in runtimes if r.body_type == "DYNAMIC"), default=0.05)

        predicted_travel = (max_linear + gravity * frame_dt) * frame_dt
        linear_required = int(math.ceil(predicted_travel / max(1.0e-4, minimum_feature * 0.70)))
        angular_required = int(math.ceil((max_angular * frame_dt) / 0.35))
        required = max(minimum, linear_required, angular_required, 1)
        # Active CCD bodies are safe against tunnelling, but impacts still need a
        # moderate contact frequency. Escalate only when actual motion warrants it.
        if active_ccd and (max_linear * frame_dt) > minimum_feature:
            required = max(required, min(maximum, minimum + 2))

        # Use three stable tiers rather than changing the step count every frame.
        middle = max(minimum, min(maximum, int(math.ceil((minimum + maximum) * 0.5))))
        if required <= minimum:
            return minimum
        if required <= middle:
            return middle
        return maximum


    @staticmethod
    def _adaptive_substep_count_from_motion(
        motion: Mapping[str, Any], frame_dt: float, minimum: int, maximum: int, gravity: float
    ) -> int:
        """Choose the next frame's tier from the previous bulk state sample."""
        if maximum <= minimum:
            return maximum
        active_count = int(motion.get("active_bodies", 0))
        if active_count <= 0:
            return minimum
        max_linear = max(0.0, float(motion.get("max_linear_speed", 0.0)))
        max_angular = max(0.0, float(motion.get("max_angular_speed", 0.0)))
        minimum_feature = max(
            1.0e-4,
            float(motion.get("minimum_feature_length", motion.get("minimum_feature_radius", 0.05))),
        )
        active_ccd = bool(motion.get("active_ccd", False))

        max_angular_surface = max(
            0.0, float(motion.get("max_angular_surface_speed", 0.0))
        )
        predicted_linear_travel = (max_linear + gravity * frame_dt) * frame_dt
        predicted_rotational_travel = max_angular_surface * frame_dt
        swept_motion = predicted_linear_travel + predicted_rotational_travel
        # Keep the combined translational and rotational sweep below roughly one
        # third of the thinnest active collider feature. Jolt LinearCast handles
        # translation continuously; the rotational term is still required because
        # a long fast-spinning fragment can rotate through a plane between casts.
        target_travel = max(1.0e-4, minimum_feature * 0.35)
        required = max(minimum, int(math.ceil(swept_motion / target_travel)), 1)
        if active_ccd and predicted_linear_travel > minimum_feature * 0.5:
            required = max(required, min(maximum, minimum + 2))
        middle = max(minimum, min(maximum, int(math.ceil((minimum + maximum) * 0.5))))
        if required <= minimum:
            return minimum
        if required <= middle:
            return middle
        return maximum


    @classmethod
    def _collect_contacts(
        cls,
        culverin,
        world,
        handle_to_name,
        runtime_by_name,
        pair_stats,
        frame_pair_contacts,
        frame_stats,
        totals,
        body_contact_peaks,
        frame: int,
    ) -> Dict[str, List[Dict[str, Any]]]:
        """Consume accumulated contacts and return per-body impact samples.

        Prefer Culverin's zero-copy 128-byte records. The dictionary API remains
        as a compatibility fallback for future or non-NumPy Blender builds.
        """
        body_frame_contacts: Dict[str, List[Dict[str, Any]]] = {}
        raw_records = None
        if _np is not None:
            try:
                double_precision = bool(getattr(culverin, "USE_DOUBLE_PRECISION", True))
                pos_type = _np.float64 if double_precision else _np.float32
                fields = [
                    ("body1", _np.uint64), ("body2", _np.uint64),
                    ("px", pos_type), ("py", pos_type), ("pz", pos_type),
                    ("nx", _np.float32), ("ny", _np.float32), ("nz", _np.float32),
                    ("impulse", _np.float32), ("sliding_speed", _np.float32),
                    ("flags", _np.uint32),
                ]
                if not double_precision:
                    fields.append(("pad_slim", _np.uint32, (3,)))
                fields.extend([
                    ("udata1", _np.uint64), ("udata2", _np.uint64),
                    ("rvx", _np.float32), ("rvy", _np.float32), ("rvz", _np.float32),
                    ("toi", _np.float32), ("penetration", _np.float32),
                    ("mat1", _np.uint32), ("mat2", _np.uint32),
                    ("sub1", _np.uint32), ("sub2", _np.uint32),
                    ("pad_fat", _np.uint32, (3,)),
                ])
                dtype = _np.dtype(fields)
                raw_view = world.get_contact_events_raw()
                if raw_view is not None and raw_view.nbytes >= dtype.itemsize:
                    raw_records = _np.frombuffer(raw_view, dtype=dtype)
                    totals["penetration_depth_available"] = True
            except Exception:
                raw_records = None

        if raw_records is not None:
            events = (
                (
                    int(record["body1"]), int(record["body2"]),
                    (float(record["px"]), float(record["py"]), float(record["pz"])),
                    (float(record["nx"]), float(record["ny"]), float(record["nz"])),
                    float(record["impulse"]), float(record["sliding_speed"]),
                    int(record["flags"]), float(record["penetration"]),
                    (float(record["rvx"]), float(record["rvy"]), float(record["rvz"])),
                )
                for record in raw_records
            )
        else:
            try:
                high_level = world.get_contact_events_ex()
            except Exception:
                return body_frame_contacts
            events = (
                (
                    int(event.get("bodies", (0, 0))[0]),
                    int(event.get("bodies", (0, 0))[1]),
                    tuple(event.get("position", (0.0, 0.0, 0.0))),
                    tuple(event.get("normal", (0.0, 0.0, 0.0))),
                    abs(float(event.get("impulse", 0.0))),
                    float(event.get("slide_speed", math.sqrt(max(0.0, float(event.get("slide_sq", 0.0)))))),
                    int(event.get("type", -1)),
                    float(event.get("penetration", 0.0)),
                    tuple(event.get("relative_velocity", (0.0, 0.0, 0.0))),
                )
                for event in high_level
            )

        for body1, body2, position_jolt, normal_jolt, impulse, slide_speed, event_type, penetration, relative_velocity_jolt in events:
            def resolve_name(handle: int) -> str:
                value = handle_to_name.get(handle, f"handle:{handle}")
                if not isinstance(value, (tuple, list)):
                    return str(value)
                candidates = [str(name) for name in value if str(name) in runtime_by_name]
                if not candidates:
                    return f"handle:{handle}"
                if event_type == int(culverin.EVENT_REMOVED):
                    return candidates[0]
                return min(
                    candidates,
                    key=lambda name: length_vec3(subtract_vec3(
                        cls._runtime_pose_jolt(world, runtime_by_name[name])[0],
                        position_jolt,
                    )),
                )

            first = resolve_name(body1)
            second = resolve_name(body2)
            if first == second:
                continue
            pair = tuple(sorted((first, second)))
            impulse = abs(float(impulse))

            frame_stats["contact_events"] += 1
            totals["contact_events"] += 1
            if event_type == int(culverin.EVENT_ADDED):
                frame_stats["contact_added"] += 1
                totals["contact_added"] += 1
            elif event_type == int(culverin.EVENT_PERSISTED):
                frame_stats["contact_persisted"] += 1
                totals["contact_persisted"] += 1
            elif event_type == int(culverin.EVENT_REMOVED):
                frame_stats["contact_removed"] += 1
                totals["contact_removed"] += 1

            if impulse > frame_stats["max_contact_impulse"]:
                frame_stats["max_contact_impulse"] = impulse
                frame_stats["max_contact_pair"] = list(pair)
            if impulse > totals["max_contact_impulse"]:
                totals["max_contact_impulse"] = impulse
                totals["max_contact_pair"] = list(pair)

            try:
                normal_blender = jolt_vec_to_blender(normal_jolt)
            except Exception:
                normal_blender = (0.0, 0.0, 0.0)
            try:
                position_blender = list(jolt_vec_to_blender(position_jolt))
            except Exception:
                position_blender = [0.0, 0.0, 0.0]
            try:
                relative_velocity_blender = list(jolt_vec_to_blender(relative_velocity_jolt))
            except Exception:
                relative_velocity_blender = [0.0, 0.0, 0.0]

            stats = pair_stats.setdefault(pair, {
                "events": 0,
                "max_impulse": 0.0,
                "max_penetration": 0.0,
                "frame": 0,
                "first_frame": int(frame),
                "last_frame": int(frame) - 1,
                "contact_frames": 0,
                "vertical_normal_sum": 0.0,
                "normal_samples": 0,
                "minimum_slide_speed": float("inf"),
                "maximum_slide_speed": 0.0,
                "maximum_relative_normal_speed": 0.0,
                "last_position": None,
                "last_normal": None,
            })
            stats["events"] += 1
            stats["max_penetration"] = max(float(stats.get("max_penetration", 0.0)), max(0.0, float(penetration)))
            if int(stats.get("last_frame", -1)) != int(frame) and event_type != int(culverin.EVENT_REMOVED):
                stats["contact_frames"] = int(stats.get("contact_frames", 0)) + 1
                stats["last_frame"] = int(frame)
            if impulse > stats["max_impulse"]:
                stats["max_impulse"] = impulse
                stats["frame"] = int(frame)
            if event_type != int(culverin.EVENT_REMOVED):
                stats["vertical_normal_sum"] = float(stats.get("vertical_normal_sum", 0.0)) + abs(float(normal_blender[2]))
                stats["normal_samples"] = int(stats.get("normal_samples", 0)) + 1
                stats["minimum_slide_speed"] = min(float(stats.get("minimum_slide_speed", float("inf"))), slide_speed)
                stats["maximum_slide_speed"] = max(float(stats.get("maximum_slide_speed", 0.0)), slide_speed)
                relative_normal_speed = 0.0
                normal_length = length_vec3(normal_blender)
                if normal_length > 1.0e-12:
                    relative_normal_speed = abs(_dot_vec3(relative_velocity_blender, normal_blender) / normal_length)
                stats["maximum_relative_normal_speed"] = max(
                    float(stats.get("maximum_relative_normal_speed", 0.0)), relative_normal_speed
                )
                stats["last_position"] = position_blender
                stats["last_normal"] = list(normal_blender)
                frame_pair = frame_pair_contacts.setdefault(pair, {
                    "vertical_normal_sum": 0.0,
                    "normal_samples": 0,
                    "minimum_slide_speed": float("inf"),
                    "maximum_slide_speed": 0.0,
                    "maximum_impulse": 0.0,
                    "last_position": None,
                    "last_normal": None,
                })
                frame_pair["vertical_normal_sum"] = float(frame_pair.get("vertical_normal_sum", 0.0)) + abs(float(normal_blender[2]))
                frame_pair["normal_samples"] = int(frame_pair.get("normal_samples", 0)) + 1
                frame_pair["minimum_slide_speed"] = min(float(frame_pair.get("minimum_slide_speed", float("inf"))), slide_speed)
                frame_pair["maximum_slide_speed"] = max(float(frame_pair.get("maximum_slide_speed", 0.0)), slide_speed)
                frame_pair["maximum_impulse"] = max(float(frame_pair.get("maximum_impulse", 0.0)), impulse)
                frame_pair["last_position"] = position_blender
                frame_pair["last_normal"] = list(normal_blender)

            for name, other in ((first, second), (second, first)):
                peak = body_contact_peaks.setdefault(name, {"max_impulse": 0.0, "frame": 0, "other": None})
                if impulse > peak["max_impulse"]:
                    peak["max_impulse"] = impulse
                    peak["frame"] = int(frame)
                    peak["other"] = other
                body_frame_contacts.setdefault(name, []).append({
                    "other": other,
                    "impulse": impulse,
                    "position": position_blender,
                    "normal": list(normal_blender),
                    "relative_velocity": list(relative_velocity_blender),
                    "event_type": event_type,
                })
        return body_frame_contacts


    @staticmethod
    def _finalize_side_stick_frame(
        pair_stats: Dict[Tuple[str, str], Dict[str, Any]],
        frame_pair_contacts: Dict[Tuple[str, str], Dict[str, Any]],
        frame: int,
        maximum_abs_vertical_normal: float,
        maximum_slide_speed: float,
    ) -> None:
        """Track uninterrupted low-speed side-contact streaks per rendered frame."""
        for pair, stats in pair_stats.items():
            frame_data = frame_pair_contacts.get(pair)
            qualifies = False
            average_vertical = 0.0
            frame_min_slide = 0.0
            frame_max_slide = 0.0
            if frame_data is not None:
                samples = max(1, int(frame_data.get("normal_samples", 0)))
                average_vertical = float(frame_data.get("vertical_normal_sum", 0.0)) / samples
                frame_min_slide = float(frame_data.get("minimum_slide_speed", 0.0))
                if not math.isfinite(frame_min_slide):
                    frame_min_slide = 0.0
                frame_max_slide = float(frame_data.get("maximum_slide_speed", 0.0))
                qualifies = (
                    average_vertical <= maximum_abs_vertical_normal
                    and frame_max_slide <= maximum_slide_speed
                )

            if not qualifies:
                stats["side_stick_current_frames"] = 0
                stats["side_stick_current_vertical_sum"] = 0.0
                stats["side_stick_current_min_slide"] = float("inf")
                stats["side_stick_current_max_slide"] = 0.0
                stats["side_stick_current_max_impulse"] = 0.0
                continue

            previous_end = int(stats.get("side_stick_current_end", frame - 1))
            if int(stats.get("side_stick_current_frames", 0)) <= 0 or previous_end != frame - 1:
                stats["side_stick_current_frames"] = 0
                stats["side_stick_current_start"] = int(frame)
                stats["side_stick_current_vertical_sum"] = 0.0
                stats["side_stick_current_min_slide"] = float("inf")
                stats["side_stick_current_max_slide"] = 0.0
                stats["side_stick_current_max_impulse"] = 0.0

            current_frames = int(stats.get("side_stick_current_frames", 0)) + 1
            stats["side_stick_current_frames"] = current_frames
            stats["side_stick_current_end"] = int(frame)
            stats["side_stick_current_vertical_sum"] = float(stats.get("side_stick_current_vertical_sum", 0.0)) + average_vertical
            stats["side_stick_current_min_slide"] = min(float(stats.get("side_stick_current_min_slide", float("inf"))), frame_min_slide)
            stats["side_stick_current_max_slide"] = max(float(stats.get("side_stick_current_max_slide", 0.0)), frame_max_slide)
            stats["side_stick_current_max_impulse"] = max(
                float(stats.get("side_stick_current_max_impulse", 0.0)),
                float(frame_data.get("maximum_impulse", 0.0)) if frame_data else 0.0,
            )

            if current_frames >= int(stats.get("side_stick_best_frames", 0)):
                stats["side_stick_best_frames"] = current_frames
                stats["side_stick_best_start"] = int(stats.get("side_stick_current_start", frame))
                stats["side_stick_best_end"] = int(frame)
                stats["side_stick_best_average_vertical"] = float(stats.get("side_stick_current_vertical_sum", 0.0)) / max(1, current_frames)
                stats["side_stick_best_min_slide"] = float(stats.get("side_stick_current_min_slide", 0.0))
                stats["side_stick_best_max_slide"] = float(stats.get("side_stick_current_max_slide", 0.0))
                stats["side_stick_best_max_impulse"] = float(stats.get("side_stick_current_max_impulse", 0.0))
                stats["side_stick_best_last_position"] = frame_data.get("last_position") if frame_data else None
                stats["side_stick_best_last_normal"] = frame_data.get("last_normal") if frame_data else None

    @classmethod
    def _apply_bond_island_sleep(
        cls,
        world,
        runtimes: Sequence[_RuntimeBody],
        bonds: Sequence[_RuntimeBond],
        frame_pair_contacts: Mapping[Tuple[str, str], Mapping[str, Any]],
        timers: Dict[Tuple[str, ...], float],
        frame_dt: float,
        linear_threshold: float,
        angular_threshold: float,
        sleep_time: float,
    ) -> Dict[str, int]:
        """Deactivate a supported intact bond island as one unit.

        Jolt's per-body sleep can keep a dense constrained fracture object awake
        indefinitely. Sleeping members one by one is unsafe because the remaining
        constraints pull them apart. This gate evaluates and deactivates the whole
        connected component together, and only while it is supported exclusively
        by static/kinematic contacts.
        """
        runtime_by_id = {body.stable_id: body for body in runtimes}
        runtime_by_name = {body.name: body for body in runtimes}
        adjacency: Dict[str, List[str]] = {}
        for bond in bonds:
            if bond.broken:
                continue
            a = bond.body_a.stable_id
            b = bond.body_b.stable_id
            adjacency.setdefault(a, []).append(b)
            adjacency.setdefault(b, []).append(a)
        visited: set[str] = set()
        requests = 0
        confirmed = 0
        last_linear_speed = 0.0
        last_angular_speed = 0.0
        last_timer = 0.0
        last_dynamic_external = False
        flush = False
        active_keys: set[Tuple[str, ...]] = set()
        for start in sorted(adjacency):
            if start in visited:
                continue
            stack = [start]
            visited.add(start)
            component_ids: List[str] = []
            while stack:
                current = stack.pop()
                component_ids.append(current)
                for neighbour in adjacency.get(current, ()):
                    if neighbour not in visited:
                        visited.add(neighbour)
                        stack.append(neighbour)
            component = [runtime_by_id[value] for value in component_ids if value in runtime_by_id]
            dynamic = [body for body in component if body.body_type == "DYNAMIC"]
            if len(dynamic) <= 1:
                continue
            key = tuple(sorted(body.stable_id for body in dynamic))
            active_keys.add(key)
            names = {body.name for body in dynamic}
            supported = False
            dynamic_external_contact = False
            for pair in frame_pair_contacts:
                first, second = pair
                first_inside = first in names
                second_inside = second in names
                if first_inside == second_inside:
                    continue
                other_name = second if first_inside else first
                other = runtime_by_name.get(other_name)
                if other is None:
                    continue
                if other.body_type in {"STATIC", "KINEMATIC"}:
                    supported = True
                else:
                    dynamic_external_contact = True
            total_mass = 0.0
            linear_sum = [0.0, 0.0, 0.0]
            angular_sum = [0.0, 0.0, 0.0]
            any_active = False
            for body in dynamic:
                linear = world.get_velocity(body.handle) or (0.0, 0.0, 0.0)
                angular = world.get_angular_velocity(body.handle) or (0.0, 0.0, 0.0)
                mass = max(1.0e-8, float(body.mass))
                total_mass += mass
                for axis in range(3):
                    linear_sum[axis] += float(linear[axis]) * mass
                    angular_sum[axis] += float(angular[axis]) * mass
                any_active = any_active or bool(world.is_active(body.handle))
            component_linear = tuple(value / max(1.0e-12, total_mass) for value in linear_sum)
            component_angular = tuple(value / max(1.0e-12, total_mass) for value in angular_sum)
            island_linear_limit = max(0.15, float(linear_threshold) * 3.0)
            island_angular_limit = max(0.5, float(angular_threshold) * 3.0)
            last_linear_speed = length_vec3(component_linear)
            last_angular_speed = length_vec3(component_angular)
            last_dynamic_external = bool(dynamic_external_contact)
            low_motion = (
                last_linear_speed <= island_linear_limit
                and last_angular_speed <= island_angular_limit
            )
            if supported and not dynamic_external_contact and low_motion:
                timers[key] = float(timers.get(key, 0.0)) + float(frame_dt)
            else:
                timers[key] = 0.0
            last_timer = float(timers[key])
            if any_active and timers[key] >= min(0.25, max(0.0, float(sleep_time))):
                requests += len(dynamic)
                for body in dynamic:
                    for handle in cls._runtime_handles(body):
                        world.set_linear_velocity(handle, 0.0, 0.0, 0.0)
                        world.set_angular_velocity(handle, 0.0, 0.0, 0.0)
                        world.deactivate(handle)
                flush = True
        for key in list(timers):
            if key not in active_keys:
                timers.pop(key, None)
        if flush:
            world.step(0.0)
            for key in active_keys:
                component = [runtime_by_id[value] for value in key if value in runtime_by_id]
                confirmed += sum(not bool(world.is_active(body.handle)) for body in component)
        return {
            "requests": requests, "confirmed": confirmed,
            "linear_speed": last_linear_speed, "angular_speed": last_angular_speed,
            "timer": last_timer, "dynamic_external": last_dynamic_external,
        }

    @classmethod
    def _apply_damping_and_sleep(
        cls,
        world,
        runtimes: Iterable[_RuntimeBody],
        frame: int,
        frame_dt: float,
        sleep_enabled: bool,
        sleep_mode: str,
        linear_threshold: float,
        angular_threshold: float,
        sleep_time: float,
        *,
        build_snapshot: bool = True,
        track_body_peaks: bool = True,
    ) -> Dict[str, Any]:
        """Read all native buffers once for transforms, motion, damping and sleep.

        Native production frames now share one bulk pass for snapshot creation,
        speed/energy diagnostics and the next adaptive-substep decision. Hybrid
        and Custom only perform a second lightweight active-state pass when a
        queued activation/deactivation batch must be confirmed by Jolt.
        """
        runtime_list = list(runtimes)
        buffers = cls._buffer_views(world)
        positions = buffers[0] if buffers else None
        rotations = buffers[1] if buffers else None
        velocities = buffers[2] if buffers else None
        angular_velocities = buffers[3] if buffers else None
        active_indices = cls._active_buffer_indices(world)

        snapshot: Optional[Dict[str, Dict[str, List[float]]]] = {} if build_snapshot else None
        frame_values = array.array("f")
        samples: Dict[str, Tuple[Tuple[float, float, float], Tuple[float, float, float], float, float]] = {}
        deactivation_candidates: List[_RuntimeBody] = []
        activation_candidates: List[_RuntimeBody] = []
        minimum_feature = float("inf")
        active_ccd = False
        max_angular_surface_speed = 0.0
        motion_energy = 0.0

        for runtime in runtime_list:
            if runtime.cluster is not None:
                position, rotation = cls._runtime_pose_jolt(world, runtime)
            else:
                position = cls._buffer_vec3(positions, runtime.buffer_index) or world.get_position(runtime.handle)
                rotation = cls._buffer_quat(rotations, runtime.buffer_index) or world.get_rotation(runtime.handle)
            if position is None or rotation is None:
                origin_world = runtime.input_location
                rotation_blender = runtime.input_rotation
            else:
                rotation_blender = jolt_quat_to_blender(rotation)
                com_world_blender = jolt_vec_to_blender(position)
                offset_world = quat_rotate_vector_wxyz(rotation_blender, runtime.com_offset_local)
                origin_world = subtract_vec3(com_world_blender, offset_world)
            location_values = tuple(float(value) for value in origin_world)
            rotation_values = tuple(float(value) for value in rotation_blender)
            if snapshot is not None:
                snapshot[runtime.name] = {
                    "location": list(location_values),
                    "rotation": list(rotation_values),
                    "scale": list(runtime.scale),
                }
            frame_values.extend((*location_values[:3], *rotation_values[:4]))

            if runtime.body_type != "DYNAMIC":
                continue
            if runtime.cluster is not None:
                linear, angular = cls._runtime_velocity_jolt(world, runtime)
            else:
                linear = cls._buffer_vec3(velocities, runtime.buffer_index) or world.get_velocity(runtime.handle) or (0.0, 0.0, 0.0)
                angular = cls._buffer_vec3(angular_velocities, runtime.buffer_index) or world.get_angular_velocity(runtime.handle) or (0.0, 0.0, 0.0)
            is_active = (
                runtime.buffer_index in active_indices
                if active_indices is not None and runtime.buffer_index >= 0
                else bool(world.is_active(runtime.handle))
            )

            if is_active:
                cluster = runtime.cluster
                damping_owner = cluster is None or runtime is cluster.members[0]
                linear_damping = cluster.linear_damping if cluster is not None else runtime.linear_damping
                angular_damping = cluster.angular_damping if cluster is not None else runtime.angular_damping
                native_handle = cluster.handle if cluster is not None else runtime.handle
                if damping_owner and linear_damping > 0.0:
                    damping_factor = math.exp(-linear_damping * frame_dt)
                    native_linear = world.get_velocity(native_handle) or (0.0, 0.0, 0.0)
                    world.set_linear_velocity(native_handle, *scale_vec3(native_linear, damping_factor))
                if damping_owner and angular_damping > 0.0:
                    damping_factor = math.exp(-angular_damping * frame_dt)
                    native_angular = world.get_angular_velocity(native_handle) or (0.0, 0.0, 0.0)
                    world.set_angular_velocity(native_handle, *scale_vec3(native_angular, damping_factor))
                if damping_owner and (linear_damping > 0.0 or angular_damping > 0.0):
                    linear, angular = cls._runtime_velocity_jolt(world, runtime)

            linear_speed = length_vec3(linear)
            angular_speed = length_vec3(angular)
            if track_body_peaks and linear_speed > runtime.max_linear_speed:
                runtime.max_linear_speed = linear_speed
                runtime.max_linear_speed_frame = frame
            if track_body_peaks and angular_speed > runtime.max_angular_speed:
                runtime.max_angular_speed = angular_speed
                runtime.max_angular_speed_frame = frame
            samples[runtime.stable_id] = (linear, angular, linear_speed, angular_speed)

            if is_active:
                minimum_feature = min(
                    minimum_feature, max(1.0e-4, runtime.feature_length)
                )
                active_ccd = active_ccd or bool(runtime.ccd)
                surface_angular = runtime.radius * angular_speed
                max_angular_surface_speed = max(
                    max_angular_surface_speed, surface_angular
                )
                motion_energy += 0.5 * runtime.mass * (
                    linear_speed * linear_speed + surface_angular * surface_angular
                )

            if not sleep_enabled:
                runtime.low_motion_time = 0.0
                if not is_active:
                    for native_handle in cls._runtime_handles(runtime):
                        world.activate(native_handle)
                    activation_candidates.append(runtime)
                continue

            if sleep_mode in {"HYBRID", "CUSTOM"} and is_active:
                angular_surface_speed = angular_speed * max(runtime.radius, 1.0e-4)
                angular_surface_limit = linear_threshold * (1.0 if sleep_mode == "HYBRID" else 0.75)
                low_motion = linear_speed <= linear_threshold and (
                    angular_speed <= angular_threshold or angular_surface_speed <= angular_surface_limit
                )
                if low_motion:
                    runtime.low_motion_time += frame_dt
                    if runtime.low_motion_time >= sleep_time:
                        deactivation_candidates.append(runtime)
                else:
                    runtime.low_motion_time = 0.0
            elif sleep_mode == "NATIVE":
                runtime.low_motion_time = 0.0

        requested_indices = {runtime.buffer_index for runtime in deactivation_candidates if runtime.buffer_index >= 0}
        if deactivation_candidates:
            for runtime in deactivation_candidates:
                for native_handle in cls._runtime_handles(runtime):
                    world.set_linear_velocity(native_handle, 0.0, 0.0, 0.0)
                    world.set_angular_velocity(native_handle, 0.0, 0.0, 0.0)
                    world.deactivate(native_handle)
            world.step(0.0)
        elif activation_candidates:
            world.step(0.0)

        confirmed_active = cls._active_buffer_indices(world)
        active = sleeping = dynamic = static = kinematic = 0
        max_linear = max_angular = 0.0
        max_linear_name: Optional[str] = None
        max_angular_name: Optional[str] = None
        confirmed_requests = 0
        confirmation_needed = bool(deactivation_candidates or activation_candidates)
        if confirmation_needed:
            motion_energy = 0.0
            minimum_feature = float("inf")
            active_ccd = False
            max_angular_surface_speed = 0.0

        for runtime in runtime_list:
            if runtime.body_type == "STATIC":
                static += 1
                continue
            if runtime.body_type == "KINEMATIC":
                kinematic += 1
                continue
            dynamic += 1
            is_active = (
                runtime.buffer_index in confirmed_active
                if confirmed_active is not None and runtime.buffer_index >= 0
                else bool(world.is_active(runtime.handle))
            )
            if runtime.buffer_index in requested_indices and not is_active:
                confirmed_requests += 1
            if not is_active:
                sleeping += 1
                continue

            active += 1
            if confirmation_needed:
                if runtime.cluster is not None:
                    linear, angular = cls._runtime_velocity_jolt(world, runtime)
                else:
                    linear = world.get_velocity(runtime.handle) or samples.get(runtime.stable_id, ((0.0, 0.0, 0.0),) * 2 + (0.0, 0.0))[0]
                    angular = world.get_angular_velocity(runtime.handle) or samples.get(runtime.stable_id, ((0.0, 0.0, 0.0),) * 2 + (0.0, 0.0))[1]
                linear_speed = length_vec3(linear)
                angular_speed = length_vec3(angular)
                minimum_feature = min(
                    minimum_feature, max(1.0e-4, runtime.feature_length)
                )
                active_ccd = active_ccd or bool(runtime.ccd)
                surface_angular = runtime.radius * angular_speed
                max_angular_surface_speed = max(
                    max_angular_surface_speed, surface_angular
                )
                motion_energy += 0.5 * runtime.mass * (
                    linear_speed * linear_speed + surface_angular * surface_angular
                )
            else:
                _linear, _angular, linear_speed, angular_speed = samples.get(
                    runtime.stable_id, ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0, 0.0)
                )
            if linear_speed > max_linear:
                max_linear = linear_speed
                max_linear_name = runtime.name
            if angular_speed > max_angular:
                max_angular = angular_speed
                max_angular_name = runtime.name

        if not math.isfinite(minimum_feature):
            minimum_feature = min(
                (max(1.0e-4, runtime.feature_length) for runtime in runtime_list if runtime.body_type == "DYNAMIC"),
                default=0.05,
            )

        return {
            "frame": frame,
            "dynamic_bodies": dynamic,
            "static_bodies": static,
            "kinematic_bodies": kinematic,
            "active_bodies": active,
            "sleeping_bodies": sleeping,
            "max_linear_speed": max_linear,
            "max_linear_speed_body": max_linear_name,
            "max_angular_speed": max_angular,
            "max_angular_speed_body": max_angular_name,
            "max_angular_surface_speed": float(max_angular_surface_speed),
            "minimum_feature_length": float(minimum_feature),
            "minimum_feature_radius": float(minimum_feature),
            "active_ccd": bool(active_ccd),
            "motion_energy_proxy": float(motion_energy),
            "deactivation_requests": len(deactivation_candidates),
            "deactivation_confirmed": confirmed_requests,
            "deactivation_rejected": len(deactivation_candidates) - confirmed_requests,
            "_snapshot": snapshot,
            "_frame_values": frame_values,
        }


    @classmethod
    def _state_diagnostics(cls, world, runtimes: Iterable[_RuntimeBody], frame: int) -> Dict[str, Any]:
        active = 0
        sleeping = 0
        max_linear = 0.0
        max_linear_name: Optional[str] = None
        max_angular = 0.0
        max_angular_name: Optional[str] = None
        dynamic = 0
        static = 0
        kinematic = 0
        buffers = cls._buffer_views(world)
        velocities = buffers[2] if buffers else None
        angular_velocities = buffers[3] if buffers else None
        active_indices = cls._active_buffer_indices(world)

        for runtime in runtimes:
            if runtime.body_type == "DYNAMIC":
                dynamic += 1
                is_active = (
                    runtime.buffer_index in active_indices
                    if active_indices is not None and runtime.buffer_index >= 0
                    else bool(world.is_active(runtime.handle))
                )
                active += int(is_active)
                sleeping += int(not is_active)
                if is_active:
                    if runtime.cluster is not None:
                        linear, angular = cls._runtime_velocity_jolt(world, runtime)
                    else:
                        linear = cls._buffer_vec3(velocities, runtime.buffer_index) or world.get_velocity(runtime.handle) or (0.0, 0.0, 0.0)
                        angular = cls._buffer_vec3(angular_velocities, runtime.buffer_index) or world.get_angular_velocity(runtime.handle) or (0.0, 0.0, 0.0)
                    linear_speed = length_vec3(linear)
                    angular_speed = length_vec3(angular)
                    if linear_speed > max_linear:
                        max_linear = linear_speed
                        max_linear_name = runtime.name
                    if angular_speed > max_angular:
                        max_angular = angular_speed
                        max_angular_name = runtime.name
            elif runtime.body_type == "KINEMATIC":
                kinematic += 1
            else:
                static += 1

        return {
            "frame": frame,
            "dynamic_bodies": dynamic,
            "static_bodies": static,
            "kinematic_bodies": kinematic,
            "active_bodies": active,
            "sleeping_bodies": sleeping,
            "max_linear_speed": max_linear,
            "max_linear_speed_body": max_linear_name,
            "max_angular_speed": max_angular,
            "max_angular_speed_body": max_angular_name,
        }
