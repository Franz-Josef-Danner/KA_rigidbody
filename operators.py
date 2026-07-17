"""Blender operators for KA Rigid Dynamics."""

import json
import os
import time
from typing import Dict

import bpy
from bpy.props import BoolProperty
from bpy.types import Operator

from .backends import BackendError, get_backend
from .backends.jolt import recommended_jolt_threads
from .core.cache import (
    cache_file_path,
    decode_direct_frame_block,
    direct_frame_block_digest,
    read_cache,
    remove_cache,
    write_cache,
)
from .core.determinism import compare_frames, frames_digest
from .core.regression import run_regression_suite, write_regression_report
from .core.scene_io import (
    apply_snapshot,
    build_scene_payload,
    clear_geometry_cache,
    enabled_body_objects,
    geometry_cache_stats,
    GROUND_OBJECT_NAME,
    GROUND_OBJECT_TAG,
    ground_objects,
    FRACTURE_TAGS,
    fracture_candidates,
    resolve_cache_directory,
    restore_rest_transform,
    repair_managed_ground,
    store_rest_transform,
    preflight_scene,
    validate_scene,
)
from .diagnostics import log_event, log_exception, log_file_path
from .runtime import (
    apply_cached_frame_to_scene,
    clear_runtime_cache,
    ensure_handlers_registered,
    load_scene_cache,
    set_bake_running,
)


def addon_preferences(context):
    addon = context.preferences.addons.get(__package__)
    return addon.preferences if addon else None


def _requires_jolt(objects) -> bool:
    return any(
        obj.ka_rigid_body.enabled
        and obj.ka_rigid_body.collision_shape in {"CONVEX_HULL", "COMPOUND_CONVEX", "MESH"}
        for obj in objects
    )


def _select_jolt_for_complex_scene(context, objects, *, source: str, cancel_if_unavailable: bool = True) -> bool:
    """Ensure complex collision geometry never silently falls back to the Reference solver."""
    scene = context.scene
    world = scene.ka_rigid_world
    if world.backend != "REFERENCE" or not _requires_jolt(objects):
        return True

    status = get_backend("JOLT").status(addon_preferences(context))
    if status.available:
        previous = world.backend
        world.backend = "JOLT"
        world.cache_status = "Jolt selected automatically; rebake required"
        log_event(
            scene,
            "BACKEND",
            "AUTO_SELECTED",
            source=source,
            previous_backend=previous,
            effective_backend="JOLT",
            reason="Complex Convex Hull/Compound Convex/Mesh bodies cannot be simulated correctly by Reference.",
            detail=status.detail,
        )
        return True

    message = (
        "This scene contains Convex Hull, Compound Convex or Mesh bodies. The Reference backend uses simplified "
        f"sphere/box proxies and is blocked for this scene. Jolt is unavailable: {status.detail}"
    )
    log_event(scene, "BACKEND", "COMPLEX_SCENE_BLOCKED", level="ERROR", source=source, reason=message)
    if cancel_if_unavailable:
        raise BackendError(message)
    return False


def _body_diagnostics(payload):
    return [
        {
            "name": body.get("name"),
            "body_type": body.get("body_type"),
            "collision_shape": body.get("collision_shape"),
            "mass": body.get("mass"),
            "raw_mass": body.get("raw_mass"),
            "mass_mode": body.get("mass_mode"),
            "density": body.get("density"),
            "shape_center": body.get("shape_center"),
            "radius": body.get("radius"),
            "half_extents": body.get("half_extents"),
            "source_vertex_count": body.get("source_vertex_count"),
            "render_source_vertex_count": body.get("render_source_vertex_count"),
            "collision_proxy": body.get("collision_proxy"),
            "convex_vertex_count": body.get("convex_vertex_count"),
            "convex_vertex_count_raw": body.get("convex_vertex_count_raw"),
            "collider_quality": body.get("collider_quality"),
            "compound_part_count": body.get("compound_part_count"),
            "compound_quality": body.get("compound_quality"),
            "triangle_count": body.get("triangle_count"),
            "friction": body.get("friction"),
            "restitution": body.get("restitution"),
            "ccd": body.get("ccd"),
            "ccd_requested": body.get("ccd_requested"),
            "ccd_reason": body.get("ccd_reason"),
            "stability_adjustments": body.get("stability_adjustments"),
        }
        for body in payload.get("bodies", [])
    ]


def _payload_summary(payload):
    bodies = payload.get("bodies", [])
    shape_counts = {}
    ccd_bodies = 0
    hull_vertices = 0
    raw_hull_vertices = 0
    adjusted_bodies = 0
    hull_error_max = 0.0
    hull_targets_missed = 0
    hull_precision_rescues = 0
    hull_full_rescues = 0
    hull_budget_rescues = 0
    collision_proxy_bodies = 0
    compound_bodies = 0
    compound_parts = 0
    for body in bodies:
        shape = str(body.get("collision_shape", "UNKNOWN"))
        shape_counts[shape] = shape_counts.get(shape, 0) + 1
        ccd_bodies += int(bool(body.get("ccd")))
        hull_vertices += int(body.get("convex_vertex_count") or 0)
        raw_hull_vertices += int(body.get("convex_vertex_count_raw") or 0)
        adjusted_bodies += int(bool(body.get("stability_adjustments")))
        quality = body.get("collider_quality") or {}
        hull_error_max = max(hull_error_max, float(quality.get("max_error", 0.0)))
        hull_targets_missed += int(quality.get("target_met") is False)
        hull_precision_rescues += int(bool(quality.get("precision_rescue")))
        hull_full_rescues += int(quality.get("rescue_mode") == "complete_hull")
        hull_budget_rescues += int(quality.get("rescue_mode") == "budget_escalation")
        collision_proxy_bodies += int(bool(body.get("collision_proxy")))
        compound_bodies += int(shape == "COMPOUND")
        compound_parts += int(body.get("compound_part_count") or 0)
    return {
        "shape_counts": shape_counts,
        "ccd_bodies": ccd_bodies,
        "hull_vertices": hull_vertices,
        "raw_hull_vertices": raw_hull_vertices,
        "adjusted_bodies": adjusted_bodies,
        "hull_error_max": hull_error_max,
        "hull_targets_missed": hull_targets_missed,
        "hull_precision_rescues": hull_precision_rescues,
        "hull_full_rescues": hull_full_rescues,
        "hull_budget_rescues": hull_budget_rescues,
        "collision_proxy_bodies": collision_proxy_bodies,
        "compound_bodies": compound_bodies,
        "compound_parts": compound_parts,
        "skipped_body_count": len(payload.get("skipped_bodies", [])),
    }


def _compound_runtime_guard_fallbacks(payload: Dict, result: Dict):
    """Return compound bodies implicated in verified compound/compound side-sticks."""
    compound_names = {
        str(body.get("name"))
        for body in payload.get("bodies", [])
        if str(body.get("collision_shape")) == "COMPOUND"
    }
    candidates = []
    fallback_names = set()
    source_candidates = result.get("compound_side_stick_candidates")
    if source_candidates is None:
        source_candidates = result.get("side_stick_candidates", [])
    for candidate in source_candidates or []:
        pair = [str(name) for name in candidate.get("pair", [])]
        if len(pair) != 2 or not all(name in compound_names for name in pair):
            continue
        candidates.append(candidate)
        fallback_names.update(pair)
    return sorted(fallback_names), candidates


def _force_single_hull_fallback(payload: Dict, body_names) -> int:
    names = set(map(str, body_names))
    changed = 0
    for body in payload.get("bodies", []):
        if str(body.get("name")) not in names or str(body.get("collision_shape")) != "COMPOUND":
            continue
        body["collision_shape"] = "CONVEX_HULL"
        body["compound_parts"] = []
        body["compound_part_count"] = 0
        quality = dict(body.get("compound_quality") or {})
        quality["accepted"] = False
        quality["fallback_reason"] = "runtime_side_stick_guard"
        reasons = list(quality.get("fallback_reasons") or [])
        if "runtime_side_stick_guard" not in reasons:
            reasons.append("runtime_side_stick_guard")
        quality["fallback_reasons"] = reasons
        body["compound_quality"] = quality
        adjustments = list(body.get("stability_adjustments") or [])
        adjustments.append("compound_runtime_single_hull")
        body["stability_adjustments"] = adjustments
        changed += 1
    profile = payload.get("build_profile")
    if isinstance(profile, dict):
        profile["compound_runtime_fallbacks"] = changed
    return changed


class KA_RIGID_OT_assign_selected(Operator):
    bl_idname = "ka_rigid.assign_selected"
    bl_label = "Add Selected Bodies"
    bl_options = {"REGISTER", "UNDO"}

    body_type: bpy.props.EnumProperty(
        items=(("DYNAMIC", "Dynamic", ""), ("STATIC", "Static", "")),
        default="DYNAMIC",
    )

    def execute(self, context):
        scene = context.scene
        objects = [obj for obj in context.selected_objects if obj.type in {"MESH", "CURVE", "SURFACE", "FONT"}]
        if not objects:
            log_event(scene, "OPERATOR", "ASSIGN_SELECTED_CANCELLED", level="WARNING", reason="No supported object selected")
            self.report({"WARNING"}, "No supported object selected")
            return {"CANCELLED"}
        fracture_objects = []
        assigned_objects = []
        protected_grounds = []
        density = scene.ka_rigid_world.fracture_density
        for obj in objects:
            if bool(obj.get(GROUND_OBJECT_TAG, False)) or obj.name.startswith(GROUND_OBJECT_NAME):
                changed = repair_managed_ground(obj, store_rest=True)
                protected_grounds.append({"name": obj.name_full, "changed": changed})
                # A managed ground is never part of a Dynamic bulk assignment.
                # Static assignment simply reaffirms the protected Plane setup.
                if self.body_type == "STATIC":
                    assigned_objects.append(obj.name_full)
                continue

            settings = obj.ka_rigid_body
            settings.enabled = True
            settings.body_type = self.body_type
            is_fracture_piece = (
                obj.name.startswith("KA_Fracture_Piece_")
                or any(bool(obj.get(tag, False)) for tag in FRACTURE_TAGS)
            )
            if self.body_type == "DYNAMIC" and is_fracture_piece:
                settings.collision_shape = "CONVEX_HULL"
                settings.mass_mode = "DENSITY"
                settings.density = density
                settings.use_ccd = True
                fracture_objects.append(obj.name_full)
            elif self.body_type == "STATIC" and settings.collision_shape == "CONVEX_HULL":
                settings.collision_shape = "MESH"
            store_rest_transform(obj, force=True)
            assigned_objects.append(obj.name_full)
        if fracture_objects:
            _select_jolt_for_complex_scene(context, objects, source="ASSIGN_SELECTED", cancel_if_unavailable=False)
        scene.ka_rigid_world.cache_status = "Scene changed; rebake required"
        log_event(
            scene,
            "OPERATOR",
            "BODIES_ASSIGNED",
            body_type=self.body_type,
            count=len(assigned_objects),
            objects=assigned_objects,
            protected_grounds=protected_grounds,
            fracture_defaults_applied=fracture_objects,
            fracture_density=density if fracture_objects else None,
        )
        if protected_grounds and self.body_type == "DYNAMIC":
            self.report({"INFO"}, f"Assigned {len(assigned_objects)} bodies; protected {len(protected_grounds)} ground plane")
        else:
            self.report({"INFO"}, f"Assigned {len(assigned_objects)} bodies")
        return {"FINISHED"}


class KA_RIGID_OT_set_selected_collider(Operator):
    bl_idname = "ka_rigid.set_selected_collider"
    bl_label = "Set Selected Collider"
    bl_options = {"REGISTER", "UNDO"}

    collision_shape: bpy.props.EnumProperty(
        items=(
            ("CONVEX_HULL", "Convex Hull", "Fast single convex hull"),
            ("COMPOUND_CONVEX", "Compound Convex", "Precise CoACD convex decomposition"),
        ),
        default="CONVEX_HULL",
    )

    def execute(self, context):
        scene = context.scene
        changed = []
        for obj in context.selected_objects:
            if not hasattr(obj, "ka_rigid_body") or not obj.ka_rigid_body.enabled:
                continue
            if bool(obj.get(GROUND_OBJECT_TAG, False)) or obj.name.startswith(GROUND_OBJECT_NAME):
                continue
            obj.ka_rigid_body.collision_shape = self.collision_shape
            changed.append(obj.name_full)
        if not changed:
            self.report({"WARNING"}, "No enabled KA rigid bodies selected")
            return {"CANCELLED"}
        _select_jolt_for_complex_scene(context, context.selected_objects, source="SET_SELECTED_COLLIDER", cancel_if_unavailable=False)
        scene.ka_rigid_world.cache_status = "Collider changed; clear proxy cache and rebake"
        log_event(
            scene,
            "OPERATOR",
            "SELECTED_COLLIDER_CHANGED",
            collision_shape=self.collision_shape,
            count=len(changed),
            objects=changed,
        )
        self.report({"INFO"}, f"Set {len(changed)} bodies to {self.collision_shape.replace('_', ' ').title()}")
        return {"FINISHED"}


class KA_RIGID_OT_remove_selected(Operator):
    bl_idname = "ka_rigid.remove_selected"
    bl_label = "Remove Selected Bodies"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        scene = context.scene
        removed = []
        for obj in context.selected_objects:
            if hasattr(obj, "ka_rigid_body") and obj.ka_rigid_body.enabled:
                obj.ka_rigid_body.enabled = False
                removed.append(obj.name_full)
        scene.ka_rigid_world.cache_status = "Scene changed; rebake required"
        log_event(scene, "OPERATOR", "BODIES_REMOVED", count=len(removed), objects=removed)
        self.report({"INFO"}, f"Removed {len(removed)} bodies")
        return {"FINISHED"}


class KA_RIGID_OT_set_rest_transform(Operator):
    bl_idname = "ka_rigid.set_rest_transform"
    bl_label = "Set Rest Transform"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        objects = [obj for obj in context.selected_objects if hasattr(obj, "ka_rigid_body") and obj.ka_rigid_body.enabled]
        for obj in objects:
            store_rest_transform(obj, force=True)
        log_event(context.scene, "OPERATOR", "REST_TRANSFORMS_STORED", count=len(objects), objects=[obj.name_full for obj in objects])
        self.report({"INFO"}, f"Stored {len(objects)} rest transforms")
        return {"FINISHED"}


class KA_RIGID_OT_restore_rest_transform(Operator):
    bl_idname = "ka_rigid.restore_rest_transform"
    bl_label = "Restore Rest Transform"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        objects = context.selected_objects or enabled_body_objects(context.scene)
        restored = [obj.name_full for obj in objects if hasattr(obj, "ka_rigid_body") and restore_rest_transform(obj)]
        log_event(context.scene, "OPERATOR", "REST_TRANSFORMS_RESTORED", count=len(restored), objects=restored)
        self.report({"INFO"}, f"Restored {len(restored)} objects")
        return {"FINISHED"}


class KA_RIGID_OT_sync_frame_range(Operator):
    bl_idname = "ka_rigid.sync_frame_range"
    bl_label = "Use Scene Frame Range"

    def execute(self, context):
        settings = context.scene.ka_rigid_world
        settings.frame_start = context.scene.frame_start
        settings.frame_end = context.scene.frame_end
        log_event(context.scene, "OPERATOR", "FRAME_RANGE_SYNCED", frame_start=settings.frame_start, frame_end=settings.frame_end)
        return {"FINISHED"}


class KA_RIGID_OT_validate(Operator):
    bl_idname = "ka_rigid.validate"
    bl_label = "Validate Physics Scene"

    def execute(self, context):
        messages = validate_scene(context.scene)
        for message in messages:
            print(f"KA Rigid Dynamics: {message}")
        log_event(context.scene, "VALIDATION", "SCENE_VALIDATED", message_count=len(messages), messages=messages)
        summary = messages[0] if len(messages) == 1 else f"Validation produced {len(messages)} messages; see System Console"
        self.report({"INFO"}, summary)
        return {"FINISHED"}


class KA_RIGID_OT_fix_invalid_colliders(Operator):
    bl_idname = "ka_rigid.fix_invalid_colliders"
    bl_label = "Fix Invalid Colliders"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        scene = context.scene
        report = preflight_scene(scene, auto_fix=True)
        fixed = list(report.get("fixed", []))
        errors = list(report.get("errors", []))
        log_event(
            scene,
            "PREFLIGHT",
            "COLLIDERS_FIXED",
            fixed=fixed,
            errors=errors,
            warnings=report.get("warnings", []),
            duplicate_static_groups=report.get("duplicate_static_groups", []),
            excluded_static_duplicates=report.get("excluded_static_duplicates", []),
            deleted_static_duplicates=report.get("deleted_static_duplicates", []),
        )
        if fixed:
            scene.ka_rigid_world.cache_status = "Collider settings corrected; rebake required"
            self.report({"INFO"}, f"Corrected {len(fixed)} collider settings")
            return {"FINISHED"}
        if errors:
            self.report({"ERROR"}, errors[0])
            return {"CANCELLED"}
        self.report({"INFO"}, "No invalid collider settings found")
        return {"FINISHED"}


class KA_RIGID_OT_import_fracture(Operator):
    bl_idname = "ka_rigid.import_fracture"
    bl_label = "Import KA Fracture Pieces"
    bl_options = {"REGISTER", "UNDO"}

    selected_only: BoolProperty(name="Selected Only", default=False)

    def execute(self, context):
        scene = context.scene
        objects = [obj for obj in context.selected_objects if obj.type == "MESH"] if self.selected_only else fracture_candidates(context)
        if not objects:
            log_event(scene, "FRACTURE", "IMPORT_CANCELLED", level="WARNING", selected_only=self.selected_only, reason="No fracture pieces found")
            self.report({"WARNING"}, "No KA Fracture pieces or mesh objects found")
            return {"CANCELLED"}
        density = scene.ka_rigid_world.fracture_density
        for obj in objects:
            settings = obj.ka_rigid_body
            settings.enabled = True
            settings.body_type = "DYNAMIC"
            settings.collision_shape = "CONVEX_HULL"
            settings.mass_mode = "DENSITY"
            settings.density = density
            settings.use_ccd = True
            store_rest_transform(obj, force=True)
        _select_jolt_for_complex_scene(context, objects, source="IMPORT_FRACTURE", cancel_if_unavailable=False)
        scene.ka_rigid_world.cache_status = "Fracture pieces imported; bake required"
        log_event(scene, "FRACTURE", "PIECES_IMPORTED", selected_only=self.selected_only, density=density, count=len(objects), objects=[obj.name_full for obj in objects])
        self.report({"INFO"}, f"Imported {len(objects)} fracture pieces")
        return {"FINISHED"}


class KA_RIGID_OT_create_ground(Operator):
    bl_idname = "ka_rigid.create_ground"
    bl_label = "Create or Select Ground Plane"
    bl_options = {"REGISTER", "UNDO"}

    @staticmethod
    def _select_only(context, obj) -> None:
        for selected in list(context.selected_objects):
            selected.select_set(False)
        obj.select_set(True)
        context.view_layer.objects.active = obj

    @staticmethod
    def _resolve_other_grounds(scene, keeper):
        policy = str(getattr(scene.ka_rigid_world, "duplicate_static_policy", "EXCLUDE"))
        excluded = []
        deleted = []
        retained = []
        for obj in list(ground_objects(scene)):
            if obj == keeper:
                continue
            name = obj.name_full
            if policy == "DELETE":
                bpy.data.objects.remove(obj, do_unlink=True)
                deleted.append(name)
            elif policy == "EXCLUDE":
                obj.ka_rigid_body.enabled = False
                excluded.append(name)
            else:
                retained.append(name)
        return policy, excluded, deleted, retained

    def execute(self, context):
        scene = context.scene
        existing = ground_objects(scene)
        if existing:
            obj = existing[0]
            if obj.name != GROUND_OBJECT_NAME and bpy.data.objects.get(GROUND_OBJECT_NAME) is None:
                obj.name = GROUND_OBJECT_NAME
            obj[GROUND_OBJECT_TAG] = True
            settings = obj.ka_rigid_body
            settings.enabled = True
            settings.body_type = "STATIC"
            settings.collision_shape = "PLANE"
            if settings.friction <= 0.0:
                settings.friction = 0.7
            store_rest_transform(obj, force=True)
            policy, excluded, deleted, retained = self._resolve_other_grounds(scene, obj)
            self._select_only(context, obj)
            scene.ka_rigid_world.cache_status = "Ground reused; rebake required"
            log_event(
                scene,
                "OPERATOR",
                "GROUND_REUSED",
                object=obj.name_full,
                location=list(obj.location),
                friction=settings.friction,
                duplicate_policy=policy,
                excluded=excluded,
                deleted=deleted,
                retained=retained,
            )
            if retained:
                self.report({"WARNING"}, f"Ground reused; {len(retained)} additional KA grounds remain enabled")
            elif excluded or deleted:
                self.report({"INFO"}, f"Ground reused; resolved {len(excluded) + len(deleted)} duplicate grounds")
            else:
                self.report({"INFO"}, "Existing ground selected")
            return {"FINISHED"}

        bpy.ops.mesh.primitive_plane_add(size=20.0, enter_editmode=False, align="WORLD", location=scene.cursor.location)
        obj = context.active_object
        obj.name = GROUND_OBJECT_NAME
        obj[GROUND_OBJECT_TAG] = True
        settings = obj.ka_rigid_body
        settings.enabled = True
        settings.body_type = "STATIC"
        settings.collision_shape = "PLANE"
        settings.friction = 0.7
        store_rest_transform(obj, force=True)
        scene.ka_rigid_world.cache_status = "Ground created; bake required"
        log_event(scene, "OPERATOR", "GROUND_CREATED", object=obj.name_full, location=list(obj.location), friction=settings.friction)
        self.report({"INFO"}, "Ground plane created")
        return {"FINISHED"}


class KA_RIGID_OT_bake(Operator):
    bl_idname = "ka_rigid.bake"
    bl_label = "Bake KA Physics"

    def execute(self, context):
        scene = context.scene
        world = scene.ka_rigid_world
        if world.frame_end <= world.frame_start:
            log_event(scene, "BAKE", "CANCELLED", level="ERROR", reason="End frame must be greater than start frame")
            self.report({"ERROR"}, "End frame must be greater than start frame")
            return {"CANCELLED"}

        request_started = time.perf_counter()
        objects = enabled_body_objects(scene)
        preflight_started = time.perf_counter()
        preflight = preflight_scene(scene, auto_fix=bool(world.auto_fix_invalid_colliders))
        preflight_seconds = time.perf_counter() - preflight_started
        log_event(
            scene,
            "PREFLIGHT",
            "BAKE_CHECK",
            auto_fix=bool(world.auto_fix_invalid_colliders),
            errors=preflight.get("errors", []),
            warnings=preflight.get("warnings", []),
            fixed=preflight.get("fixed", []),
            small_bodies=preflight.get("small_bodies", []),
            body_count_before=preflight.get("body_count_before"),
            body_count=preflight.get("body_count"),
            dynamic_count=preflight.get("dynamic_count"),
            static_count=preflight.get("static_count"),
            ground_objects=preflight.get("ground_objects", []),
            duplicate_static_groups=preflight.get("duplicate_static_groups", []),
            excluded_static_duplicates=preflight.get("excluded_static_duplicates", []),
            deleted_static_duplicates=preflight.get("deleted_static_duplicates", []),
            duplicate_static_policy=world.duplicate_static_policy,
            mass_ratio=preflight.get("mass_ratio"),
            mass_ratio_before=preflight.get("mass_ratio_before"),
            mass_conditioning_floor=preflight.get("mass_conditioning_floor"),
            preflight_seconds=round(preflight_seconds, 6),
            collider_cache=geometry_cache_stats(),
        )
        if preflight.get("errors"):
            for message in preflight["errors"]:
                print(f"KA Rigid Dynamics Preflight ERROR: {message}")
            first_error = str(preflight["errors"][0])
            world.cache_status = f"Preflight failed: {first_error}"
            self.report({"ERROR"}, first_error)
            return {"CANCELLED"}
        for message in preflight.get("warnings", []):
            print(f"KA Rigid Dynamics Preflight WARNING: {message}")
        if preflight.get("fixed"):
            world.cache_status = f"Preflight corrected {len(preflight['fixed'])} collider settings"

        handler_status = ensure_handlers_registered()
        world.cache_playback = True
        requested_backend = world.backend
        try:
            _select_jolt_for_complex_scene(context, objects, source="BAKE", cancel_if_unavailable=True)
        except BackendError as exc:
            world.cache_status = f"Backend error: {exc}"
            log_exception(scene, "BAKE", "BACKEND_SELECTION_FAILED", exc, requested_backend=requested_backend)
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}

        log_event(
            scene,
            "BAKE",
            "REQUEST",
            requested_backend=requested_backend,
            backend=world.backend,
            frame_start=world.frame_start,
            frame_end=world.frame_end,
            substeps=world.substeps,
            adaptive_substeps=bool(world.adaptive_substeps),
            minimum_substeps=world.minimum_substeps,
            solver_iterations=world.solver_iterations,
            jolt_threads=world.jolt_threads,
            penetration_slop=world.penetration_slop,
            sleep_mode=world.sleep_mode,
            adaptive_ccd=bool(world.adaptive_ccd),
            small_body_policy=world.small_body_policy,
            enforce_mass_ratio_limit=bool(world.enforce_mass_ratio_limit),
            max_mass_ratio=world.max_mass_ratio,
            convex_hull_max_vertices=world.convex_hull_max_vertices,
            adaptive_hull_accuracy=bool(world.adaptive_hull_accuracy),
            hull_quality_preset=world.hull_quality_preset,
            hull_error_tolerance=world.hull_error_tolerance,
            hull_relative_error_tolerance=world.hull_relative_error_tolerance,
            hull_rescue_max_vertices=world.hull_rescue_max_vertices,
            binary_cache=True,
            compound_quality_preset=world.compound_quality_preset,
            compound_max_parts=world.compound_max_parts,
            compound_error_tolerance=world.compound_error_tolerance,
            compound_relative_error_tolerance=world.compound_relative_error_tolerance,
            compound_max_hull_vertices=world.compound_max_hull_vertices,
            compound_preprocess_resolution=world.compound_preprocess_resolution,
            compound_resolution=world.compound_resolution,
            compound_mcts_iterations=world.compound_mcts_iterations,
            compound_inset=world.compound_inset,
            reproducibility_mode=world.reproducibility_mode,
            deterministic_mode=world.reproducibility_mode != "PERFORMANCE",
            jolt_threads_effective=(
                1
                if world.reproducibility_mode == "STRICT"
                else int(world.jolt_threads)
                if int(world.jolt_threads) > 0
                else recommended_jolt_threads(int(preflight.get("dynamic_count", 0)))
            ),
            early_sleep_termination=bool(world.early_sleep_termination),
            detailed_contact_diagnostics=bool(world.detailed_contact_diagnostics),
            detailed_payload_diagnostics=bool(world.detailed_payload_diagnostics),
            collider_cache=geometry_cache_stats(),
            cache_playback=bool(world.cache_playback),
            handler_status=handler_status,
            log_path=log_file_path(scene),
        )
        if not any(obj.ka_rigid_body.body_type == "DYNAMIC" for obj in objects):
            log_event(scene, "BAKE", "CANCELLED", level="ERROR", reason="No dynamic bodies enabled", body_count=len(objects))
            self.report({"ERROR"}, "No dynamic bodies enabled")
            return {"CANCELLED"}

        preferences = addon_preferences(context)
        backend = get_backend(world.backend)
        status = backend.status(preferences)
        log_event(scene, "BACKEND", "STATUS", identifier=world.backend, available=status.available, production_ready=status.production_ready, detail=status.detail)
        if world.backend != "REFERENCE" and not status.available:
            log_event(scene, "BAKE", "CANCELLED", level="ERROR", reason=status.detail)
            self.report({"ERROR"}, status.detail)
            return {"CANCELLED"}

        original_frame = scene.frame_current
        for obj in objects:
            restore_rest_transform(obj)

        set_bake_running(True)
        wm = context.window_manager
        total = max(1, world.frame_end - world.frame_start)
        wm.progress_begin(0, total)
        started = time.perf_counter()
        bake_succeeded = False
        playback_target_frame = original_frame
        try:
            scene.frame_set(world.frame_start)
            payload_started = time.perf_counter()
            payload = build_scene_payload(scene)
            payload_seconds = time.perf_counter() - payload_started
            compound_guard_active = bool(
                world.compound_runtime_guard
                and any(str(body.get("collision_shape")) == "COMPOUND" for body in payload.get("bodies", []))
            )
            detailed_contacts = bool(world.detailed_contact_diagnostics)
            detailed_payload = bool(world.detailed_payload_diagnostics)
            collect_contacts = bool(detailed_contacts or compound_guard_active)
            payload["diagnostics"] = {
                "enabled": bool(world.log_output),
                "path": log_file_path(scene),
                "contacts": collect_contacts,
                "log_contacts": detailed_contacts,
                "force_contacts": bool(compound_guard_active),
                "contact_reason": (
                    "detailed_contact_diagnostics_and_compound_guard"
                    if detailed_contacts and compound_guard_active
                    else "compound_runtime_guard"
                    if compound_guard_active
                    else "detailed_contact_diagnostics"
                    if detailed_contacts
                    else "disabled"
                ),
                "side_stick": bool((detailed_contacts and world.side_stick_diagnostics) or compound_guard_active),
                "log_side_stick": bool(detailed_contacts and world.side_stick_diagnostics),
                "payload": detailed_payload,
                "side_stick_min_frames": int(world.side_stick_min_frames),
                "side_stick_normal_z": float(world.side_stick_normal_z),
                "side_stick_slide_speed": float(world.side_stick_slide_speed),
            }
            if not any(body.get("body_type") == "DYNAMIC" for body in payload.get("bodies", [])):
                raise BackendError("No dynamic bodies remain after the small-body policy was applied.")
            payload_log_started = time.perf_counter()
            payload_log_data = {
                "signature": payload["signature"],
                "body_count": len(payload["bodies"]),
                "gravity": payload["gravity"],
                "fps": payload["fps"],
                "skipped_bodies": payload.get("skipped_bodies", []),
                "stability": payload.get("stability", {}),
                "summary": _payload_summary(payload),
                "build_profile": payload.get("build_profile", {}),
                "payload_seconds": round(payload_seconds, 6),
            }
            if world.log_output and detailed_payload:
                payload_log_data["bodies"] = _body_diagnostics(payload)
            log_event(scene, "SCENE_IO", "PAYLOAD_BUILT", **payload_log_data)
            payload_log_seconds = time.perf_counter() - payload_log_started

            def progress(current: int, maximum: int) -> None:
                wm.progress_update(current)
                world.cache_status = f"Baking {current}/{maximum}"

            backend_seconds = 0.0
            backend_started = time.perf_counter()
            result = backend.bake(payload, progress=progress)
            backend_seconds += time.perf_counter() - backend_started

            runtime_guard_report = {
                "enabled": bool(compound_guard_active),
                "triggered": False,
                "resolved": True,
                "fallback_bodies": [],
                "first_pass_candidates": [],
                "remaining_candidates": [],
            }
            if compound_guard_active:
                fallback_names, guard_candidates = _compound_runtime_guard_fallbacks(payload, result)
                if fallback_names:
                    changed = _force_single_hull_fallback(payload, fallback_names)
                    runtime_guard_report.update({
                        "triggered": bool(changed),
                        "fallback_bodies": fallback_names,
                        "first_pass_candidates": guard_candidates,
                    })
                    log_event(
                        scene,
                        "QUALITY",
                        "COMPOUND_RUNTIME_FALLBACK",
                        fallback_bodies=fallback_names,
                        candidate_count=len(guard_candidates),
                        candidates=guard_candidates,
                    )
                    world.cache_status = f"Compound guard rebaking {changed} bodies as Single Hull"
                    backend_started = time.perf_counter()
                    result = backend.bake(payload, progress=progress)
                    backend_seconds += time.perf_counter() - backend_started
                    _remaining_names, remaining_candidates = _compound_runtime_guard_fallbacks(payload, result)
                    runtime_guard_report["remaining_candidates"] = remaining_candidates
                    runtime_guard_report["resolved"] = not bool(remaining_candidates)
                    if remaining_candidates:
                        log_event(
                            scene,
                            "QUALITY",
                            "COMPOUND_RUNTIME_GUARD_REMAINING",
                            level="WARNING",
                            candidates=remaining_candidates,
                        )
            result["compound_runtime_guard"] = runtime_guard_report
            result["created_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
            result["runtime"] = dict(payload.get("runtime", {}))
            direct_block = result.get("_binary_frame_block")
            result["result_digest"] = (
                direct_frame_block_digest(direct_block)
                if isinstance(direct_block, dict)
                else frames_digest(result.get("frames", {}))
            )
            frame_count = int(result.get("frame_count") or len(result.get("frames", {})))
            first_snapshot = result.get("_first_snapshot") or result.get("frames", {}).get(str(world.frame_start))
            directory = resolve_cache_directory(scene)
            determinism_check = None
            previous_cache_path = cache_file_path(directory)
            if world.reproducibility_mode != "PERFORMANCE" and os.path.isfile(previous_cache_path):
                try:
                    previous = read_cache(directory)
                    if previous.get("scene_signature") == payload.get("signature"):
                        candidate_frames = result.get("frames", {})
                        if not candidate_frames and isinstance(direct_block, dict):
                            candidate_frames = decode_direct_frame_block(direct_block)
                        determinism_check = compare_frames(
                            previous.get("frames", {}),
                            candidate_frames,
                            tolerance=float(world.determinism_tolerance),
                        )
                        result["determinism_check"] = determinism_check
                        log_event(
                            scene,
                            "QUALITY",
                            "DETERMINISM_CHECK",
                            **determinism_check,
                        )
                except Exception as comparison_error:
                    log_exception(scene, "QUALITY", "DETERMINISM_CHECK_FAILED", comparison_error)
            cache_write_started = time.perf_counter()
            path = write_cache(directory, result)
            cache_write_seconds = time.perf_counter() - cache_write_started
            world.cache_signature = payload["signature"]
            elapsed = time.perf_counter() - started
            guard_count = len(runtime_guard_report.get("fallback_bodies", []))
            guard_note = f"; compound guard fallback {guard_count}" if guard_count else ""
            if determinism_check is not None:
                verdict = "match" if determinism_check.get("match") else "MISMATCH"
                world.cache_status = f"Baked {frame_count} frames in {elapsed:.2f}s; deterministic {verdict}{guard_note}"
            else:
                world.cache_status = f"Baked {frame_count} frames in {elapsed:.2f}s{guard_note}"
            clear_runtime_cache(scene)
            applied = apply_snapshot(first_snapshot) if first_snapshot else 0
            log_event(
                scene,
                "BAKE",
                "COMPLETE",
                backend=result.get("backend"),
                frame_count=frame_count,
                elapsed_seconds=round(elapsed, 6),
                preflight_seconds=round(preflight_seconds, 6),
                payload_seconds=round(payload_seconds, 6),
                payload_log_seconds=round(payload_log_seconds, 6),
                backend_seconds=round(backend_seconds, 6),
                cache_write_seconds=round(cache_write_seconds, 6),
                build_profile=payload.get("build_profile", {}),
                cache_path=path,
                cache_size_bytes=os.path.getsize(path) if os.path.isfile(path) else None,
                first_frame_transforms_applied=applied,
                total_request_seconds=round(time.perf_counter() - request_started, 6),
                result_digest=result.get("result_digest"),
                determinism_check=determinism_check,
                compound_runtime_guard=result.get("compound_runtime_guard", {}),
                runtime=result.get("runtime", {}),
            )
            bake_succeeded = True
            if not (world.frame_start <= original_frame <= world.frame_end):
                playback_target_frame = world.frame_start
            self.report({"INFO"}, f"Bake complete: {path}")
        except BackendError as exc:
            world.cache_status = f"Backend error: {exc}"
            log_exception(scene, "BAKE", "BACKEND_ERROR", exc, backend=world.backend)
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        except Exception as exc:
            world.cache_status = f"Bake failed: {exc}"
            log_exception(scene, "BAKE", "FAILED", exc, backend=world.backend)
            self.report({"ERROR"}, f"Bake failed: {exc}")
            return {"CANCELLED"}
        finally:
            wm.progress_end()
            set_bake_running(False)
            if bake_succeeded:
                scene.frame_set(playback_target_frame)
                playback_applied = apply_cached_frame_to_scene(
                    scene,
                    playback_target_frame,
                    source="BAKE_FINALIZE",
                    force=True,
                )
            else:
                scene.frame_set(original_frame)
                playback_applied = 0
            log_event(
                scene,
                "BAKE",
                "FINALIZED",
                restored_frame=int(scene.frame_current),
                playback_applied=playback_applied,
                cache_playback=bool(world.cache_playback),
                handler_status=ensure_handlers_registered(),
            )
        return {"FINISHED"}


class KA_RIGID_OT_run_regression(Operator):
    bl_idname = "ka_rigid.run_regression"
    bl_label = "Run Quality Tests"
    bl_description = "Run isolated Jolt drop, stack, friction, CCD and determinism tests without changing the open scene"

    def execute(self, context):
        scene = context.scene
        world = scene.ka_rigid_world
        status = get_backend("JOLT").status(addon_preferences(context))
        if not status.available:
            world.regression_status = "Jolt unavailable"
            self.report({"ERROR"}, status.detail)
            return {"CANCELLED"}
        try:
            world.regression_status = "Running tests..."
            report = run_regression_suite(determinism_tolerance=float(world.determinism_tolerance))
            path = write_regression_report(resolve_cache_directory(scene), report)
            world.regression_report_path = path
            world.regression_status = f"Passed {report['passed']}/{report['total']} in {report['elapsed_seconds']:.2f}s"
            log_event(scene, "QUALITY", "REGRESSION_COMPLETE", path=path, **report)
            if report.get("success"):
                self.report({"INFO"}, world.regression_status)
                return {"FINISHED"}
            failed = [item.get("name") for item in report.get("tests", []) if not item.get("passed")]
            self.report({"WARNING"}, f"Quality tests failed: {', '.join(map(str, failed))}")
            return {"FINISHED"}
        except Exception as exc:
            world.regression_status = f"Test error: {exc}"
            log_exception(scene, "QUALITY", "REGRESSION_FAILED", exc)
            self.report({"ERROR"}, world.regression_status)
            return {"CANCELLED"}


class KA_RIGID_OT_clear_cache(Operator):
    bl_idname = "ka_rigid.clear_cache"
    bl_label = "Clear Physics Cache"

    def execute(self, context):
        scene = context.scene
        directory = resolve_cache_directory(scene)
        removed = remove_cache(directory)
        clear_runtime_cache(scene)
        scene.ka_rigid_world.cache_status = "Cache cleared" if removed else "No cache found"
        restored = 0
        for obj in enabled_body_objects(scene):
            restored += int(restore_rest_transform(obj))
        log_event(scene, "CACHE", "CLEARED", removed=removed, directory=directory, restored_objects=restored)
        self.report({"INFO"}, scene.ka_rigid_world.cache_status)
        return {"FINISHED"}


class KA_RIGID_OT_clear_collider_cache(Operator):
    bl_idname = "ka_rigid.clear_collider_cache"
    bl_label = "Clear Collider Cache"
    bl_description = "Discard cached evaluated geometry and convex hull proxies"

    def execute(self, context):
        removed = clear_geometry_cache(clear_persistent=True, persistent_directory=resolve_cache_directory(context.scene))
        context.scene.ka_rigid_world.cache_status = f"Collider cache cleared ({removed} entries)"
        log_event(
            context.scene,
            "SCENE_IO",
            "COLLIDER_CACHE_CLEARED",
            removed_entries=removed,
            cache=geometry_cache_stats(),
        )
        self.report({"INFO"}, f"Cleared {removed} collider cache entries")
        return {"FINISHED"}


class KA_RIGID_OT_apply_cached_frame(Operator):
    bl_idname = "ka_rigid.apply_cached_frame"
    bl_label = "Apply Cached Frame"

    def execute(self, context):
        scene = context.scene
        payload = load_scene_cache(scene)
        if payload is None:
            log_event(scene, "CACHE", "APPLY_CANCELLED", level="WARNING", reason="No cache found", frame=scene.frame_current)
            self.report({"WARNING"}, "No cache found")
            return {"CANCELLED"}
        snapshot = payload.get("frames", {}).get(str(scene.frame_current))
        if not snapshot:
            log_event(scene, "CACHE", "APPLY_CANCELLED", level="WARNING", reason="Current frame is not cached", frame=scene.frame_current)
            self.report({"WARNING"}, "Current frame is not cached")
            return {"CANCELLED"}
        count = apply_snapshot(snapshot)
        log_event(scene, "CACHE", "FRAME_APPLIED", frame=scene.frame_current, transform_count=count)
        self.report({"INFO"}, f"Applied {count} cached transforms")
        return {"FINISHED"}


class KA_RIGID_OT_export_scene(Operator):
    bl_idname = "ka_rigid.export_scene"
    bl_label = "Export Solver Scene"

    def execute(self, context):
        scene = context.scene
        try:
            world = scene.ka_rigid_world
            preflight = preflight_scene(scene, auto_fix=bool(world.auto_fix_invalid_colliders))
            if preflight.get("errors"):
                raise BackendError(str(preflight["errors"][0]))
            for obj in enabled_body_objects(scene):
                restore_rest_transform(obj)
            payload = build_scene_payload(scene)
            compound_guard_active = bool(
                world.compound_runtime_guard
                and any(str(body.get("collision_shape")) == "COMPOUND" for body in payload.get("bodies", []))
            )
            detailed_contacts = bool(world.detailed_contact_diagnostics)
            detailed_payload = bool(world.detailed_payload_diagnostics)
            payload["diagnostics"] = {
                "enabled": bool(world.log_output),
                "path": log_file_path(scene),
                "contacts": bool(detailed_contacts or compound_guard_active),
                "log_contacts": detailed_contacts,
                "force_contacts": bool(compound_guard_active),
                "contact_reason": (
                    "detailed_contact_diagnostics_and_compound_guard"
                    if detailed_contacts and compound_guard_active
                    else "compound_runtime_guard"
                    if compound_guard_active
                    else "detailed_contact_diagnostics"
                    if detailed_contacts
                    else "disabled"
                ),
                "side_stick": bool((detailed_contacts and world.side_stick_diagnostics) or compound_guard_active),
                "log_side_stick": bool(detailed_contacts and world.side_stick_diagnostics),
                "payload": detailed_payload,
                "side_stick_min_frames": int(world.side_stick_min_frames),
                "side_stick_normal_z": float(world.side_stick_normal_z),
                "side_stick_slide_speed": float(world.side_stick_slide_speed),
            }
            directory = resolve_cache_directory(scene)
            os.makedirs(directory, exist_ok=True)
            path = os.path.join(directory, "ka_rigid_scene.json")
            with open(path, "w", encoding="utf-8") as stream:
                json.dump(payload, stream, indent=2, ensure_ascii=False)
            log_event(scene, "SCENE_IO", "SCENE_EXPORTED", path=path, size_bytes=os.path.getsize(path), body_count=len(payload["bodies"]), signature=payload["signature"])
            self.report({"INFO"}, f"Scene exported: {path}")
            return {"FINISHED"}
        except Exception as exc:
            log_exception(scene, "SCENE_IO", "EXPORT_FAILED", exc)
            self.report({"ERROR"}, f"Scene export failed: {exc}")
            return {"CANCELLED"}


CLASSES = (
    KA_RIGID_OT_assign_selected,
    KA_RIGID_OT_set_selected_collider,
    KA_RIGID_OT_remove_selected,
    KA_RIGID_OT_set_rest_transform,
    KA_RIGID_OT_restore_rest_transform,
    KA_RIGID_OT_sync_frame_range,
    KA_RIGID_OT_validate,
    KA_RIGID_OT_fix_invalid_colliders,
    KA_RIGID_OT_import_fracture,
    KA_RIGID_OT_create_ground,
    KA_RIGID_OT_bake,
    KA_RIGID_OT_run_regression,
    KA_RIGID_OT_clear_cache,
    KA_RIGID_OT_clear_collider_cache,
    KA_RIGID_OT_apply_cached_frame,
    KA_RIGID_OT_export_scene,
)


def register_operators() -> None:
    for cls in CLASSES:
        bpy.utils.register_class(cls)


def unregister_operators() -> None:
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)
