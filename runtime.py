"""Runtime cache playback and Blender handlers."""

from __future__ import annotations

import os
from typing import Dict, Optional, Tuple

import bpy
from bpy.app.handlers import persistent

from .core.cache import cache_file_path, read_cache
from .backends.culverin_loader import BUNDLED_CULVERIN_VERSION
from .core.scene_io import apply_snapshot, ground_objects, repair_managed_ground, resolve_cache_directory
from .diagnostics import log_event, log_exception

_CACHE_MEMORY: Dict[int, Tuple[str, float, Dict]] = {}
_LAST_APPLIED_FRAME: Dict[int, int] = {}
_BAKE_RUNNING = False
_SETTINGS_VERSION = 501
_PENDING_HANDLER_STATUS: Dict[str, int] = {}


def set_bake_running(value: bool) -> None:
    global _BAKE_RUNNING
    _BAKE_RUNNING = bool(value)


def migrate_scene_settings(scene: bpy.types.Scene) -> None:
    """Migrate persisted settings from earlier builds."""
    if not hasattr(scene, "ka_rigid_world"):
        return
    settings = scene.ka_rigid_world
    current = int(getattr(settings, "settings_version", 0))
    if current >= _SETTINGS_VERSION:
        return

    previous_backend = settings.backend
    previous_playback = bool(getattr(settings, "cache_playback", True))
    migration_detail = "No backend change required."
    if previous_backend == "REFERENCE":
        try:
            from .backends.jolt import JoltBackend

            status = JoltBackend.status()
            if status.available:
                settings.backend = "JOLT"
                settings.cache_status = "Updated to Jolt; rebake required"
                migration_detail = "Persisted Reference backend was changed to bundled Jolt."
            else:
                migration_detail = f"Jolt unavailable; Reference retained: {status.detail}"
        except Exception as exc:
            migration_detail = f"Jolt migration check failed; Reference retained: {exc}"

    if current < 304:
        # Version 0.3.4 switches Jolt to native sleeping by default and keeps
        # expensive contact event aggregation opt-in.
        settings.sleep_mode = "NATIVE"
        settings.auto_fix_invalid_colliders = True
        settings.detailed_contact_diagnostics = False
        settings.adaptive_ccd = True
        settings.small_body_policy = "STABILIZE"
        settings.convex_hull_max_vertices = 64
        settings.cache_status = "Updated to 0.3.4; rebake required"
        migration_detail += " Applied 0.3.4 stability defaults."

    if current < 305:
        settings.detailed_payload_diagnostics = False
        settings.cache_status = "Updated to 0.3.5; collider cache will rebuild on first bake"
        migration_detail += " Applied 0.3.5 cache and diagnostics defaults."

    if current < 306:
        settings.adaptive_hull_accuracy = True
        settings.hull_quality_preset = "BALANCED"
        settings.hull_error_tolerance = 0.002
        settings.hull_min_vertices = 24
        settings.deterministic_mode = True
        settings.determinism_tolerance = 1.0e-6
        settings.regression_status = "Not run"
        settings.cache_status = "Updated to 0.3.6; rebake and run quality tests"
        migration_detail += " Applied 0.3.6 determinism and adaptive collider defaults."


    if current < 307:
        # 0.3.7 stability hotfix: 0.3.6 could reduce many fracture colliders
        # to 24/48 points and sorted dynamic bodies ahead of static geometry.
        # Balanced now uses the proven 64-point baseline; body ordering is
        # static -> kinematic -> dynamic.
        if settings.hull_quality_preset == "BALANCED":
            settings.hull_min_vertices = 64
            settings.convex_hull_max_vertices = 64
        settings.cache_status = "Updated to 0.3.7 stability defaults; rebake required"
        migration_detail += " Applied 0.3.7 stable collider and body-order defaults."

    if current < 308:
        settings.duplicate_static_policy = "EXCLUDE"
        for obj in scene.objects:
            if obj.name.startswith("KA_Physics_Ground"):
                obj["ka_rigid_ground"] = True
        settings.cache_status = "Updated to 0.3.8 ground and duplicate-collider defaults; rebake required"
        migration_detail += " Applied 0.3.8 ground singleton and duplicate static-collider defaults."

    if current < 400:
        settings.compound_mode = "AUTO"
        settings.compound_max_parts = 8
        settings.compound_resolution = 7
        settings.compound_trigger_error = 0.004
        settings.compound_inset = 0.0005
        settings.compound_min_coverage = 0.72
        settings.side_stick_diagnostics = True
        settings.side_stick_min_frames = 8
        settings.side_stick_normal_z = 0.35
        settings.side_stick_slide_speed = 0.05
        settings.cache_status = "Updated to 0.4.0 compound collider defaults; clear proxy cache and rebake"
        migration_detail += " Applied 0.4.0 automatic box-compound and side-stick diagnostic defaults."

    if current < 401:
        # Box compounds remain experimental. Production scenes return to the
        # proven single-hull default; users may opt in explicitly.
        settings.compound_mode = "SINGLE"
        settings.compound_min_coverage = 0.92
        settings.compound_max_outside_volume = 0.08
        settings.compound_max_surface_deviation = 0.002
        settings.compound_min_improvement = 0.20
        settings.compound_runtime_guard = True
        settings.cache_status = "Updated to 0.4.1 safe collider defaults; clear proxy cache and rebake"
        migration_detail += " Applied 0.4.1 strict compound validation, Single Hull default and runtime side-stick guard."


    if current < 402:
        settings.reproducibility_mode = "REPEATABLE" if bool(getattr(settings, "deterministic_mode", True)) else "PERFORMANCE"
        settings.adaptive_substeps = True
        settings.minimum_substeps = min(4, max(1, int(settings.substeps)))
        settings.early_sleep_termination = True
        settings.early_sleep_frames = 3
        if settings.hull_quality_preset == "BALANCED":
            settings.hull_min_vertices = 32
            settings.convex_hull_max_vertices = 96
        settings.cache_status = "Updated to 0.4.2 performance pipeline; clear proxy cache and rebake"
        migration_detail += " Applied 0.4.2 adaptive hull, substep and reproducibility defaults."

    if current < 403:
        # 0.4.3 restores stable resting contacts while preserving the persistent
        # proxy cache and native buffer optimizations introduced in 0.4.2.
        if settings.hull_quality_preset == "BALANCED":
            settings.hull_min_vertices = 32
            settings.convex_hull_max_vertices = 64
        settings.sleep_mode = "HYBRID"
        settings.sleep_linear_threshold = 0.05
        settings.sleep_angular_threshold = 0.25
        settings.sleep_time = 0.5
        settings.detailed_contact_diagnostics = False
        settings.side_stick_diagnostics = False
        settings.enforce_mass_ratio_limit = True
        settings.max_mass_ratio = min(float(getattr(settings, "max_mass_ratio", 5000.0)), 5000.0)
        settings.cache_status = "Updated to 0.4.3 stability defaults; clear proxy cache and rebake"
        migration_detail += " Applied 0.4.3 stable hull, hybrid sleep, mass conditioning and diagnostics defaults."

    if current < 404:
        # 0.4.4 only reports sleeping after Jolt confirms the inactive state.
        # Native island sleeping is the production default; Hybrid remains an
        # explicit experimental option with batch confirmation.
        settings.sleep_mode = "NATIVE"
        settings.detailed_contact_diagnostics = False
        settings.side_stick_diagnostics = False
        settings.cache_status = "Updated to 0.4.4 confirmed native sleeping and binary cache; rebake required"
        migration_detail += " Applied 0.4.4 confirmed native sleeping, corrected substep metrics and binary cache defaults."


    if current < 405:
        # 0.4.5 shares one native buffer pass across frame sampling, adaptive
        # scheduling and direct binary cache construction. Hull proxies migrate
        # to the KAHC3 Float64 container on the next payload build.
        settings.sleep_mode = "NATIVE"
        settings.detailed_contact_diagnostics = False
        settings.side_stick_diagnostics = False
        settings.cache_status = "Updated to 0.4.5 bulk frame pipeline; rebake required"
        migration_detail += " Applied 0.4.5 bulk frame sampling, direct cache frames and binary hull cache."

    if current < 406:
        # 0.4.6 protects the managed KA ground from bulk Dynamic/Static
        # assignment. Enabled managed grounds are restored to Static Plane so
        # thin one-sided mesh triangles cannot let fragments pass through.
        repaired_grounds = []
        for ground in ground_objects(scene, enabled_only=True):
            changed = repair_managed_ground(ground)
            if changed:
                repaired_grounds.append({"name": ground.name_full, "changed": changed})
        settings.cache_status = "Updated to 0.4.6 ground protection; old cache ignored, rebake required"
        migration_detail += (
            " Applied 0.4.6 managed-ground protection and plane-collider repair"
            + (f" for {len(repaired_grounds)} ground object(s)." if repaired_grounds else ".")
        )

    if current < 407:
        # 0.4.7 no longer accepts an inward simplified convex hull when the
        # configured geometric tolerance is missed. Those bodies are promoted
        # to their complete convex hull so visible fracture meshes cannot become
        # embedded in a smaller collision proxy.
        settings.cache_status = "Updated to 0.4.7 precision-rescue hulls; old cache ignored, rebake required"
        migration_detail += " Applied 0.4.7 full-hull precision rescue for failed adaptive proxies."


    if current < 408:
        # 0.4.8 separates production and diagnostic execution, adds scale-aware
        # support-error hull selection and explicit low-poly collision proxies.
        settings.hull_error_tolerance = 0.00075
        settings.hull_relative_error_tolerance = 0.005
        settings.hull_rescue_max_vertices = 256
        settings.detailed_contact_diagnostics = False
        settings.detailed_payload_diagnostics = False
        settings.side_stick_diagnostics = False
        settings.cache_status = "Updated to 0.4.8 support-error proxies; clear proxy cache and rebake"
        migration_detail += " Applied 0.4.8 production cache path, scale-aware hull rescue and collision-proxy support."


    if current < 409:
        # 0.4.9 removes the Bake Profile split. Every normal Jolt bake uses the
        # binary frame cache; contact and body diagnostics remain independent.
        settings.cache_status = "Updated to 0.4.9 independent diagnostics; rebake required"
        migration_detail += " Applied 0.4.9 binary-only normal bakes and independent diagnostic switches."


    if current < 500:
        # 0.5.0 replaces the automatic box-compound experiment with an explicit
        # per-body Compound Convex collider backed by bundled CoACD.
        settings.compound_mode = "SINGLE"
        settings.compound_quality_preset = "BALANCED"
        settings.compound_max_parts = 8
        settings.compound_error_tolerance = 0.003
        settings.compound_relative_error_tolerance = 0.005
        settings.compound_max_hull_vertices = 96
        settings.compound_preprocess_resolution = 50
        settings.compound_resolution = 2000
        settings.compound_mcts_iterations = 150
        settings.compound_inset = 0.0005
        settings.compound_runtime_guard = False
        settings.cache_status = "Updated to 0.5.0 Compound Convex colliders; clear proxy cache and rebake"
        migration_detail += " Applied 0.5.0 explicit CoACD Compound Convex collider settings."

    # Playback must not remain silently disabled after an add-on upgrade.
    settings.cache_playback = True
    settings.settings_version = _SETTINGS_VERSION
    log_event(
        scene,
        "MIGRATION",
        "SCENE_SETTINGS_UPDATED",
        from_version=current,
        to_version=_SETTINGS_VERSION,
        previous_backend=previous_backend,
        effective_backend=settings.backend,
        previous_cache_playback=previous_playback,
        effective_cache_playback=bool(settings.cache_playback),
        detail=migration_detail,
    )


def clear_runtime_cache(scene: Optional[bpy.types.Scene] = None) -> None:
    if scene is None:
        _CACHE_MEMORY.clear()
        _LAST_APPLIED_FRAME.clear()
    else:
        key = scene.as_pointer()
        _CACHE_MEMORY.pop(key, None)
        _LAST_APPLIED_FRAME.pop(key, None)


def load_scene_cache(scene: bpy.types.Scene) -> Optional[Dict]:
    settings = scene.ka_rigid_world
    directory = resolve_cache_directory(scene)
    path = cache_file_path(directory)
    if not os.path.isfile(path):
        return None
    modified = os.path.getmtime(path)
    key = scene.as_pointer()
    cached = _CACHE_MEMORY.get(key)
    if cached and cached[0] == path and cached[1] == modified:
        return cached[2]
    payload = read_cache(directory)
    runtime = payload.get("runtime", {})
    expected = {"addon_version": "0.5.1", "culverin_version": BUNDLED_CULVERIN_VERSION}
    mismatches = {
        name: {"cache": runtime.get(name), "current": value}
        for name, value in expected.items()
        if runtime.get(name) != value
    }
    if mismatches:
        log_event(scene, "CACHE", "RUNTIME_MISMATCH", level="WARNING", path=path, mismatches=mismatches)
        settings.cache_status = "Older cache ignored; rebake required"
        return None
    _CACHE_MEMORY[key] = (path, modified, payload)
    log_event(
        scene,
        "CACHE",
        "LOADED_FROM_DISK",
        path=path,
        modified_time=modified,
        frame_count=len(payload.get("frames", {})),
        backend=payload.get("backend"),
    )
    return payload


def _tag_viewports_for_redraw() -> int:
    """Request redraws without relying on one active 3D View context."""
    redraws = 0
    try:
        window_manager = bpy.context.window_manager
        if window_manager is None:
            return 0
        for window in window_manager.windows:
            screen = window.screen
            if screen is None:
                continue
            for area in screen.areas:
                if area.type in {"VIEW_3D", "DOPESHEET_EDITOR", "TIMELINE"}:
                    area.tag_redraw()
                    redraws += 1
    except Exception:
        return redraws
    return redraws


def apply_cached_frame_to_scene(
    scene: bpy.types.Scene,
    frame: Optional[int] = None,
    *,
    source: str = "FRAME_CHANGE_POST",
    force: bool = False,
) -> int:
    """Load and apply one cache frame, independent of handler dispatch."""
    if _BAKE_RUNNING or not hasattr(scene, "ka_rigid_world"):
        return 0
    settings = scene.ka_rigid_world
    if not settings.enabled:
        return 0
    if not force and not settings.cache_playback:
        return 0

    frame_number = int(scene.frame_current if frame is None else frame)
    key = scene.as_pointer()
    try:
        payload = load_scene_cache(scene)
        if payload is None:
            return 0
        snapshot = payload.get("frames", {}).get(str(frame_number))
        if not snapshot:
            return 0
        count = apply_snapshot(snapshot)
        redraw_count = _tag_viewports_for_redraw()
        _LAST_APPLIED_FRAME[key] = frame_number
        settings.cache_status = f"Playback frame {frame_number}: {count} objects"
        log_event(
            scene,
            "CACHE",
            "PLAYBACK_FRAME_APPLIED",
            source=source,
            frame=frame_number,
            transform_count=count,
            redraw_areas=redraw_count,
            backend=payload.get("backend"),
            cache_playback=bool(settings.cache_playback),
        )
        return count
    except Exception as exc:
        settings.cache_status = f"Playback error: {exc}"
        log_exception(scene, "PLAYBACK", "FRAME_CHANGE_FAILED", exc, frame=frame_number, source=source)
        return 0


@persistent
def ka_rigid_frame_change_post(scene, depsgraph=None):
    apply_cached_frame_to_scene(scene, source="FRAME_CHANGE_POST")


def _initialize_available_scenes(*, source: str) -> bool:
    """Migrate/log scenes only after Blender releases its restricted data API."""
    try:
        scenes = tuple(bpy.data.scenes)
    except (AttributeError, RuntimeError):
        return False

    for scene in scenes:
        migrate_scene_settings(scene)
        if _PENDING_HANDLER_STATUS:
            log_event(scene, "PLAYBACK", "HANDLERS_REGISTERED", source=source, **_PENDING_HANDLER_STATUS)
    return True


def _deferred_scene_initialization():
    """Timer callback used because add-on register() runs with bpy.data restricted."""
    if not _initialize_available_scenes(source="DEFERRED_REGISTER"):
        return 0.1
    return None


@persistent
def ka_rigid_load_post(_dummy):
    clear_runtime_cache()
    _initialize_available_scenes(source="LOAD_POST")


def _remove_stale_handlers(handler_list, function_names) -> int:
    """Remove handler objects left behind by an earlier module version."""
    removed = 0
    for handler in tuple(handler_list):
        name = getattr(handler, "__name__", "")
        module = getattr(handler, "__module__", "")
        if name in function_names and ("ka_rigid" in module.lower() or module == __name__):
            try:
                handler_list.remove(handler)
                removed += 1
            except ValueError:
                pass
    return removed


def ensure_handlers_registered() -> Dict[str, int]:
    """Replace stale frame/load handlers with the current module functions."""
    removed_frame = _remove_stale_handlers(
        bpy.app.handlers.frame_change_post,
        {"ka_rigid_frame_change_post"},
    )
    removed_load = _remove_stale_handlers(
        bpy.app.handlers.load_post,
        {"ka_rigid_load_post"},
    )
    bpy.app.handlers.frame_change_post.append(ka_rigid_frame_change_post)
    bpy.app.handlers.load_post.append(ka_rigid_load_post)
    return {
        "removed_frame_handlers": removed_frame,
        "removed_load_handlers": removed_load,
        "frame_handlers": sum(
            getattr(item, "__name__", "") == "ka_rigid_frame_change_post"
            for item in bpy.app.handlers.frame_change_post
        ),
        "load_handlers": sum(
            getattr(item, "__name__", "") == "ka_rigid_load_post"
            for item in bpy.app.handlers.load_post
        ),
    }


def register_handlers() -> None:
    global _PENDING_HANDLER_STATUS
    # Do not touch bpy.data here. Blender exposes _RestrictData while an add-on
    # is being installed/enabled, and bpy.data.scenes is unavailable then.
    _PENDING_HANDLER_STATUS = ensure_handlers_registered()
    if not bpy.app.timers.is_registered(_deferred_scene_initialization):
        bpy.app.timers.register(_deferred_scene_initialization, first_interval=0.0)


def unregister_handlers() -> None:
    global _PENDING_HANDLER_STATUS
    if bpy.app.timers.is_registered(_deferred_scene_initialization):
        bpy.app.timers.unregister(_deferred_scene_initialization)
    _remove_stale_handlers(bpy.app.handlers.frame_change_post, {"ka_rigid_frame_change_post"})
    _remove_stale_handlers(bpy.app.handlers.load_post, {"ka_rigid_load_post"})
    _PENDING_HANDLER_STATUS = {}
    clear_runtime_cache()
