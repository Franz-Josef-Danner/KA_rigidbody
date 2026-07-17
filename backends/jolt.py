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
from typing import Any, Dict, Iterable, List, Mapping, Optional, Tuple

from .base import BackendError, BackendStatus, PhysicsBackend, ProgressCallback
from .culverin_loader import BUNDLED_CULVERIN_VERSION, CulverinLoadError, culverin_status, load_culverin
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



@dataclass
class _RuntimeBody:
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
    mass: float
    ccd: bool
    handles: Tuple[int, ...] = ()
    constraint_handles: Tuple[int, ...] = ()
    buffer_index: int = -1
    low_motion_time: float = 0.0
    max_linear_speed: float = 0.0
    max_angular_speed: float = 0.0
    max_linear_speed_frame: int = 0
    max_angular_speed_frame: int = 0


class JoltBackend(PhysicsBackend):
    identifier = "JOLT"
    name = "Jolt Native"

    @classmethod
    def status(cls, preferences=None) -> BackendStatus:
        available, detail = culverin_status()
        if available:
            detail += " Convex hulls, CoACD Compound Convex clusters, primitive compounds, static triangle meshes, inertia, rotation, CCD and sleeping are active."
        return BackendStatus(cls.identifier, cls.name, available, False, detail + (" Adapter status: beta." if available else ""))

    def bake(self, scene_payload: Dict, progress: ProgressCallback = None) -> Dict:
        try:
            culverin = load_culverin()
        except CulverinLoadError as exc:
            raise BackendError(str(exc)) from exc

        diagnostic_settings = scene_payload.get("diagnostics", {})
        # Blender bakes always keep the direct binary frame stream. The hidden
        # override exists only for regression fixtures that need dictionaries.
        store_python_frames = bool(scene_payload.get("store_python_frames", False))
        log_enabled = bool(diagnostic_settings.get("enabled", False))
        log_path = diagnostic_settings.get("path")
        force_contacts = bool(diagnostic_settings.get("force_contacts", False))
        contact_diagnostics = bool(diagnostic_settings.get("contacts", False) or force_contacts)
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
        native_body_count = sum(
            max(1, len(body.get("compound_parts") or []))
            if str(body.get("collision_shape", "")) == "COMPOUND_CONVEX"
            else 1
            for body in bodies
        )
        native_dynamic_count = sum(
            (max(1, len(body.get("compound_parts") or [])) if str(body.get("collision_shape", "")) == "COMPOUND_CONVEX" else 1)
            for body in bodies if body.get("body_type") == "DYNAMIC"
        )
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
            # Contact capacity scales with actual native child bodies. Compound
            # Convex currently expands one logical object into several fixed hulls.
            "max_pairs": max(65536, min(4_000_000, native_body_count * 512)),
            "max_contact_constraints": max(32768, min(2_000_000, native_body_count * 256)),
            "temp_allocator_size": max(32 * 1024 * 1024, min(1024 * 1024 * 1024, native_body_count * 96 * 1024)),
            "num_threads": max(1, min(64, thread_count)),
        }

        try:
            world = culverin.PhysicsWorld(settings=world_settings)
        except Exception as exc:
            raise BackendError(f"Jolt world creation failed: {exc}") from exc

        handle_to_name: Dict[int, str] = {}
        runtimes: List[_RuntimeBody] = []
        shape_statistics: Dict[str, int] = {}
        creation_warnings: List[str] = []
        compound_constraint_count = 0

        log(
            "INITIALIZING",
            scene=scene_payload.get("scene_name"),
            signature=scene_payload.get("signature"),
            culverin_version=str(getattr(culverin, "__version__", BUNDLED_CULVERIN_VERSION)),
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
            substeps=int(scene_payload.get("substeps", 1)),
            solver_iterations_requested=int(scene_payload.get("solver_iterations", 8)),
            solver_iterations_note="Culverin 0.13.2 does not expose Jolt velocity/position iteration counts; native defaults are used.",
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
            compound_constraint_count = sum(len(runtime.constraint_handles) for runtime in runtimes)
        except Exception as exc:
            raise BackendError(f"Jolt body creation failed: {exc}") from exc

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

        initial_snapshot, initial_values = self._snapshot_and_values(world, runtimes)
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
            "contact_collection_enabled": contact_diagnostics,
            "contact_collection_reason": diagnostic_settings.get("contact_reason", "detailed_contact_diagnostics" if contact_diagnostics else "disabled"),
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
        }
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
            contact_diagnostics=contact_diagnostics,
            sleeping_mode=sleep_mode,
            initial_state=self._state_diagnostics(world, runtimes, frame_start) if frame_logging else None,
        )

        sleeping_streak = 0
        gravity_magnitude = length_vec3(scene_payload.get("gravity", (0.0, 0.0, -9.81)))
        minimum_initial_feature = min(
            (runtime.radius for runtime in runtimes if runtime.body_type == "DYNAMIC" and not runtime.ccd),
            default=min((runtime.radius for runtime in runtimes if runtime.body_type == "DYNAMIC"), default=0.05),
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

            for _substep in range(frame_substeps):
                if not sleep_enabled:
                    for runtime in runtimes:
                        if runtime.body_type == "DYNAMIC":
                            for native_handle in self._runtime_handles(runtime):
                                world.activate(native_handle)
                try:
                    world.step(frame_step_dt)
                except Exception as exc:
                    raise BackendError(f"Jolt simulation failed at frame {frame}: {exc}") from exc
                totals["executed_substeps"] += 1

            # Culverin accumulates events until they are consumed. Reading the
            # zero-copy buffer once per rendered frame avoids eight Python/C++
            # crossings in a typical fracture bake without losing events.
            if contact_diagnostics:
                self._collect_contacts(
                    culverin,
                    world,
                    handle_to_name,
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
            last_state = state
            energy_tail.append(float(state.get("motion_energy_proxy", 0.0)))
            totals["sleep_deactivation_requests"] += int(state.get("deactivation_requests", 0))
            totals["sleep_deactivation_confirmed"] += int(state.get("deactivation_confirmed", 0))
            totals["sleep_deactivation_rejected"] += int(state.get("deactivation_rejected", 0))

            if store_python_frames and snapshot is not None:
                frames[str(frame)] = snapshot
            binary_frame_numbers.append(str(frame))
            binary_values.extend(frame_values)
            if frame_logging:
                log(
                    "FRAME_COMPLETE",
                    frame=frame,
                    substeps=frame_substeps,
                    adaptive_substeps=adaptive_substeps,
                    dt=frame_step_dt,
                    contacts=frame_contacts if contact_diagnostics else {"collection": "disabled"},
                    state=state,
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
            "limitations": [
                *(
                    []
                    if not contact_logging or totals.get("penetration_depth_available")
                    else ["Contact penetration depth is unavailable through the active compatibility path."]
                ),
                "Per-body damping and Hybrid settle assistance are evaluated once per rendered frame until Culverin exposes native damping settings.",
                "Contact event collection is opt-in or automatically enabled by the compound runtime guard.",
                "Culverin 0.13.2 still uses Jolt's native velocity/position iteration defaults.",
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

        def common_at(world_pos, part_mass=mass):
            return {
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
                total_volume = sum(max(0.0, float(part.get("volume", 0.0))) for part in source_parts)
                if total_volume <= 1.0e-18:
                    total_volume = float(len(source_parts))
                try:
                    for part_index, part in enumerate(source_parts):
                        part_points = list(part.get("vertices") or [])
                        if len(part_points) < 4:
                            continue
                        part_center = tuple(map(float, part.get("center", (0.0, 0.0, 0.0))))
                        part_world = add_vec3(location, quat_rotate_vector_wxyz(rotation, part_center))
                        if body_type == "DYNAMIC":
                            volume = max(0.0, float(part.get("volume", 0.0)))
                            if volume <= 1.0e-18:
                                volume = total_volume / max(1, len(source_parts))
                            part_mass = max(1.0e-6, requested_mass * volume / total_volume)
                        else:
                            part_mass = -1.0
                        values = array.array("f")
                        for point in part_points:
                            values.extend(blender_vec_to_jolt(subtract_vec3(point, part_center)))
                        part_handle = int(world.create_convex_hull(
                            points=values.tobytes(),
                            **common_at(part_world, part_mass),
                        ))
                        handles.append(part_handle)
                    if not handles:
                        raise RuntimeError("CoACD produced no usable convex child hulls")

                    handle = handles[0]
                    runtime_center = tuple(map(float, source_parts[0].get("center", shape_center)))
                    if body_type != "STATIC" and len(handles) > 1:
                        for child in handles[1:]:
                            constraint_handles.append(int(world.create_constraint(
                                culverin.CONSTRAINT_FIXED, handle, child, None
                            )))
                    warnings.append(
                        f"{name}: Compound Convex uses {len(handles)} fixed convex child bodies "
                        "because Culverin 0.13.2 does not expose Jolt convex-child compound creation."
                    )
                except Exception as exc:
                    try:
                        if handles:
                            world.destroy_bodies_batch(handles)
                    except Exception:
                        for created in handles:
                            try:
                                world.destroy_body(created)
                            except Exception:
                                pass
                    handles = []
                    constraint_handles = []
                    warnings.append(f"{name}: Compound Convex creation failed ({exc}); a single convex hull fallback was used.")
                    handle, runtime_center = create_single_hull(body.get("convex_vertices") or [], shape_center)
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
            handle = int(world.create_mesh_body(
                pos=pos,
                rot=rot,
                vertices=vertex_bytes.tobytes(),
                indices=index_bytes.tobytes(),
                user_data=user_data,
                category=category,
                mask=mask,
            ))
            handles = [handle]
            if abs(friction - 0.2) > 1.0e-6 or restitution > 1.0e-6:
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
                if collision_shape == "COMPOUND_CONVEX" and child_index < len(source_parts):
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

        return _RuntimeBody(
            name=name,
            handle=handle,
            body_type=body_type,
            scale=scale,
            input_location=location,
            input_rotation=rotation,
            shape_center=runtime_center,
            com_offset_local=runtime_center,
            linear_damping=max(0.0, float(body.get("linear_damping", 0.0))),
            angular_damping=max(0.0, float(body.get("angular_damping", 0.0))),
            radius=max(1.0e-5, float(body.get("radius", 0.5))),
            mass=requested_mass if body_type == "DYNAMIC" else -1.0,
            ccd=ccd,
            handles=tuple(handles),
            constraint_handles=tuple(constraint_handles),
        )

    @staticmethod
    def _runtime_handles(runtime: _RuntimeBody) -> Tuple[int, ...]:
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
        minimum_feature = max(1.0e-4, float(motion.get("minimum_feature_radius", 0.05)))
        active_ccd = bool(motion.get("active_ccd", False))

        predicted_travel = (max_linear + gravity * frame_dt) * frame_dt
        linear_required = int(math.ceil(predicted_travel / max(1.0e-4, minimum_feature * 0.70)))
        angular_required = int(math.ceil((max_angular * frame_dt) / 0.35))
        required = max(minimum, linear_required, angular_required, 1)
        if active_ccd and (max_linear * frame_dt) > minimum_feature:
            required = max(required, min(maximum, minimum + 2))
        middle = max(minimum, min(maximum, int(math.ceil((minimum + maximum) * 0.5))))
        if required <= minimum:
            return minimum
        if required <= middle:
            return middle
        return maximum


    @staticmethod
    def _collect_contacts(
        culverin,
        world,
        handle_to_name,
        pair_stats,
        frame_pair_contacts,
        frame_stats,
        totals,
        body_contact_peaks,
        frame: int,
    ) -> None:
        """Consume accumulated contacts once per rendered frame.

        Prefer Culverin's zero-copy 128-byte records. The dictionary API remains
        as a compatibility fallback for future or non-NumPy Blender builds.
        """
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
                )
                for record in raw_records
            )
        else:
            try:
                high_level = world.get_contact_events_ex()
            except Exception:
                return
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
                )
                for event in high_level
            )

        for body1, body2, position_jolt, normal_jolt, impulse, slide_speed, event_type, penetration in events:
            first = handle_to_name.get(body1, f"handle:{body1}")
            second = handle_to_name.get(body2, f"handle:{body2}")
            # Fixed child hulls belonging to the same logical Compound Convex
            # body are implementation details, not user-visible contact pairs.
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
        samples: Dict[int, Tuple[Tuple[float, float, float], Tuple[float, float, float], float, float]] = {}
        deactivation_candidates: List[_RuntimeBody] = []
        activation_candidates: List[_RuntimeBody] = []
        minimum_feature = float("inf")
        active_ccd = False
        motion_energy = 0.0

        for runtime in runtime_list:
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
            linear = cls._buffer_vec3(velocities, runtime.buffer_index) or world.get_velocity(runtime.handle) or (0.0, 0.0, 0.0)
            angular = cls._buffer_vec3(angular_velocities, runtime.buffer_index) or world.get_angular_velocity(runtime.handle) or (0.0, 0.0, 0.0)
            is_active = (
                runtime.buffer_index in active_indices
                if active_indices is not None and runtime.buffer_index >= 0
                else bool(world.is_active(runtime.handle))
            )

            if is_active:
                if runtime.linear_damping > 0.0:
                    damping_factor = math.exp(-runtime.linear_damping * frame_dt)
                    for native_handle in cls._runtime_handles(runtime):
                        native_linear = world.get_velocity(native_handle) or (0.0, 0.0, 0.0)
                        world.set_linear_velocity(native_handle, *scale_vec3(native_linear, damping_factor))
                    linear = scale_vec3(linear, damping_factor)
                if runtime.angular_damping > 0.0:
                    damping_factor = math.exp(-runtime.angular_damping * frame_dt)
                    for native_handle in cls._runtime_handles(runtime):
                        native_angular = world.get_angular_velocity(native_handle) or (0.0, 0.0, 0.0)
                        world.set_angular_velocity(native_handle, *scale_vec3(native_angular, damping_factor))
                    angular = scale_vec3(angular, damping_factor)

            linear_speed = length_vec3(linear)
            angular_speed = length_vec3(angular)
            if track_body_peaks and linear_speed > runtime.max_linear_speed:
                runtime.max_linear_speed = linear_speed
                runtime.max_linear_speed_frame = frame
            if track_body_peaks and angular_speed > runtime.max_angular_speed:
                runtime.max_angular_speed = angular_speed
                runtime.max_angular_speed_frame = frame
            samples[runtime.buffer_index] = (linear, angular, linear_speed, angular_speed)

            if is_active:
                if not runtime.ccd:
                    minimum_feature = min(minimum_feature, max(1.0e-4, runtime.radius))
                active_ccd = active_ccd or bool(runtime.ccd)
                surface_angular = runtime.radius * angular_speed
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
                linear = world.get_velocity(runtime.handle) or samples.get(runtime.buffer_index, ((0.0, 0.0, 0.0),) * 2 + (0.0, 0.0))[0]
                angular = world.get_angular_velocity(runtime.handle) or samples.get(runtime.buffer_index, ((0.0, 0.0, 0.0),) * 2 + (0.0, 0.0))[1]
                linear_speed = length_vec3(linear)
                angular_speed = length_vec3(angular)
                if not runtime.ccd:
                    minimum_feature = min(minimum_feature, max(1.0e-4, runtime.radius))
                active_ccd = active_ccd or bool(runtime.ccd)
                surface_angular = runtime.radius * angular_speed
                motion_energy += 0.5 * runtime.mass * (
                    linear_speed * linear_speed + surface_angular * surface_angular
                )
            else:
                _linear, _angular, linear_speed, angular_speed = samples.get(
                    runtime.buffer_index, ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0, 0.0)
                )
            if linear_speed > max_linear:
                max_linear = linear_speed
                max_linear_name = runtime.name
            if angular_speed > max_angular:
                max_angular = angular_speed
                max_angular_name = runtime.name

        if not math.isfinite(minimum_feature):
            minimum_feature = min(
                (max(1.0e-4, runtime.radius) for runtime in runtime_list if runtime.body_type == "DYNAMIC"),
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
