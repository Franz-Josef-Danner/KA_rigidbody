"""Built-in reference backend."""

from __future__ import annotations

import time
from typing import Dict

from .base import BackendStatus, PhysicsBackend, ProgressCallback
from ..core.diagnostics import write_diagnostic
from ..core.solver_reference import ReferenceSolver


class ReferenceBackend(PhysicsBackend):
    identifier = "REFERENCE"
    name = "Reference Position/Impulse"

    @classmethod
    def status(cls, preferences=None) -> BackendStatus:
        return BackendStatus(
            identifier=cls.identifier,
            name=cls.name,
            available=True,
            production_ready=False,
            detail="Built-in deterministic pipeline-validation backend; dynamic bodies use bounding spheres.",
        )

    def bake(self, scene_payload: Dict, progress: ProgressCallback = None) -> Dict:
        diagnostic_settings = scene_payload.get("diagnostics", {})
        log_enabled = bool(diagnostic_settings.get("enabled", False))
        log_path = diagnostic_settings.get("path")
        frame_logging = bool(
            log_enabled
            and (diagnostic_settings.get("contacts", False) or diagnostic_settings.get("payload", False))
        )

        def log(event: str, *, level: str = "INFO", **data) -> None:
            write_diagnostic(log_enabled, log_path, "REFERENCE_BACKEND", event, level=level, data=data)

        started = time.perf_counter()
        solver = ReferenceSolver(
            scene_payload["bodies"],
            gravity=scene_payload["gravity"],
            solver_iterations=scene_payload["solver_iterations"],
            sleep_enabled=scene_payload["sleep_enabled"],
            sleep_linear_threshold=scene_payload["sleep_linear_threshold"],
            sleep_angular_threshold=scene_payload["sleep_angular_threshold"],
            sleep_time=scene_payload["sleep_time"],
        )
        frame_start = int(scene_payload["frame_start"])
        frame_end = int(scene_payload["frame_end"])
        fps = max(1.0e-6, float(scene_payload["fps"]))
        frame_dt = 1.0 / fps
        configured_substeps = max(1, int(scene_payload["substeps"]))

        log(
            "INITIALIZED",
            scene=scene_payload.get("scene_name"),
            signature=scene_payload.get("signature"),
            frame_start=frame_start,
            frame_end=frame_end,
            fps=fps,
            frame_dt=frame_dt,
            configured_substeps=configured_substeps,
            solver_iterations=scene_payload["solver_iterations"],
            gravity=scene_payload["gravity"],
            state=solver.diagnostic_state(),
        )

        frames: Dict[str, Dict] = {str(frame_start): solver.snapshot()}
        total = max(1, frame_end - frame_start)
        if progress:
            progress(0, total)

        totals = {
            "broadphase_pairs": 0,
            "dynamic_static_contact_solves": 0,
            "dynamic_dynamic_contact_solves": 0,
            "velocity_impulses": 0,
            "max_penetration": 0.0,
            "executed_substeps": 0,
        }
        if frame_logging:
            log("FRAME_COMPLETE", frame=frame_start, substeps=0, state=solver.diagnostic_state())

        for offset, frame in enumerate(range(frame_start + 1, frame_end + 1), start=1):
            substeps = solver.suggested_substeps(frame_dt, configured_substeps)
            dt = frame_dt / substeps
            frame_stats = {
                "broadphase_pairs": 0,
                "dynamic_static_contact_solves": 0,
                "dynamic_dynamic_contact_solves": 0,
                "velocity_impulses": 0,
                "max_penetration": 0.0,
            }
            last_step = {}
            for _ in range(substeps):
                last_step = solver.step(dt)
                for key in (
                    "broadphase_pairs",
                    "dynamic_static_contact_solves",
                    "dynamic_dynamic_contact_solves",
                    "velocity_impulses",
                ):
                    frame_stats[key] += int(last_step.get(key, 0))
                frame_stats["max_penetration"] = max(
                    frame_stats["max_penetration"],
                    float(last_step.get("max_penetration", 0.0)),
                )

            for key in (
                "broadphase_pairs",
                "dynamic_static_contact_solves",
                "dynamic_dynamic_contact_solves",
                "velocity_impulses",
            ):
                totals[key] += frame_stats[key]
            totals["max_penetration"] = max(totals["max_penetration"], frame_stats["max_penetration"])
            totals["executed_substeps"] += substeps

            frames[str(frame)] = solver.snapshot()
            state = solver.diagnostic_state()
            if frame_logging:
                log(
                    "FRAME_COMPLETE",
                    frame=frame,
                    substeps=substeps,
                    dt=dt,
                    adaptive_substeps=substeps != configured_substeps,
                    stats=frame_stats,
                    state=state,
                )
            if progress:
                progress(offset, total)

        elapsed = time.perf_counter() - started
        log(
            "BAKE_COMPLETE",
            elapsed_seconds=round(elapsed, 6),
            frame_count=len(frames),
            totals=totals,
            final_state=solver.diagnostic_state(),
        )

        return {
            "backend": self.identifier,
            "backend_detail": self.status().detail,
            "scene_signature": scene_payload["signature"],
            "scene_name": scene_payload["scene_name"],
            "frame_start": frame_start,
            "frame_end": frame_end,
            "fps": fps,
            "diagnostic_totals": totals,
            "frames": frames,
        }
