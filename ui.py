"""User interface panels."""

from __future__ import annotations

import bpy
from bpy.types import Panel

from .backends import BACKEND_CLASSES
from .operators import addon_preferences
from .core.coacd_bridge import coacd_status
from .core.constraints import constraint_count


class KA_RIGID_PT_world(Panel):
    bl_label = "KA Rigid Dynamics"
    bl_idname = "KA_RIGID_PT_world"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "KA Physics"

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        world = scene.ka_rigid_world

        layout.prop(world, "enabled")
        column = layout.column()
        column.enabled = world.enabled
        column.prop(world, "backend")

        backend_class = BACKEND_CLASSES.get(world.backend)
        status = backend_class.status(addon_preferences(context)) if backend_class else None
        if status:
            box = column.box()
            box.alert = not status.available and world.backend != "REFERENCE"
            box.label(text=status.name, icon="CHECKMARK" if status.available else "ERROR")
            for line in status.detail.splitlines() or [status.detail]:
                box.label(text=line[:110])

        if world.backend == "REFERENCE" and any(
            getattr(obj, "ka_rigid_body", None)
            and obj.ka_rigid_body.enabled
            and obj.ka_rigid_body.collision_shape in {"CONVEX_HULL", "COMPOUND_CONVEX", "MESH"}
            for obj in scene.objects
        ):
            warning = column.box()
            warning.alert = True
            warning.label(text="Reference ist für diese Szene gesperrt.", icon="ERROR")
            warning.label(text="Convex Hull/Compound Convex/Mesh benötigt Jolt.")
            warning.label(text="Beim Bake wird Jolt automatisch gewählt.")

        frame_box = column.box()
        frame_box.label(text="Simulation", icon="TIME")
        row = frame_box.row(align=True)
        row.prop(world, "frame_start")
        row.prop(world, "frame_end")
        frame_box.operator("ka_rigid.sync_frame_range", icon="FILE_REFRESH")
        frame_box.prop(world, "use_scene_gravity")
        if not world.use_scene_gravity:
            frame_box.prop(world, "gravity")
        row = frame_box.row(align=True)
        row.prop(world, "substeps")
        row.prop(world, "adaptive_substeps")
        if world.adaptive_substeps:
            frame_box.prop(world, "minimum_substeps")
        if world.backend == "REFERENCE":
            frame_box.prop(world, "solver_iterations")
        elif world.backend == "JOLT":
            thread_row = frame_box.row(align=True)
            thread_row.enabled = world.reproducibility_mode != "STRICT"
            thread_row.prop(world, "jolt_threads")
            frame_box.prop(world, "penetration_slop")
            if world.reproducibility_mode == "STRICT":
                frame_box.label(text="Strict mode fixes Jolt to one worker thread.", icon="INFO")
            else:
                frame_box.label(text="Jolt uses native multi-threading and Culverin's internal solver settings.", icon="INFO")
        else:
            frame_box.prop(world, "solver_iterations")

        sleep_box = column.box()
        sleep_box.label(text="Sleeping", icon="PAUSE")
        sleep_box.prop(world, "sleep_enabled")
        if world.sleep_enabled:
            if world.backend == "JOLT":
                sleep_box.prop(world, "sleep_mode")
            if world.backend != "JOLT" or world.sleep_mode in {"HYBRID", "CUSTOM"}:
                row = sleep_box.row(align=True)
                row.prop(world, "sleep_linear_threshold")
                row.prop(world, "sleep_angular_threshold")
                sleep_box.prop(world, "sleep_time")
                if world.backend == "JOLT" and world.sleep_mode == "HYBRID":
                    sleep_box.label(text="Native sleeping plus conservative low-motion settling.", icon="INFO")
            elif world.backend == "JOLT":
                sleep_box.label(text="Only native Jolt island sleeping is used.", icon="INFO")
            sleep_box.prop(world, "early_sleep_termination")
            if world.early_sleep_termination:
                sleep_box.prop(world, "early_sleep_frames")

        cache_box = column.box()
        cache_box.label(text="Cache", icon="DISK_DRIVE")
        cache_box.prop(world, "cache_directory")
        cache_box.prop(world, "cache_playback")
        cache_box.label(text=world.cache_status)
        row = cache_box.row(align=True)
        row.scale_y = 1.25
        row.operator("ka_rigid.bake", icon="REC")
        row.operator("ka_rigid.clear_cache", icon="TRASH")
        row = cache_box.row(align=True)
        row.operator("ka_rigid.apply_cached_frame", icon="IMPORT")
        row.operator("ka_rigid.export_scene", icon="EXPORT")

        utilities = column.box()
        utilities.label(text="Setup", icon="TOOL_SETTINGS")
        row = utilities.row(align=True)
        dynamic = row.operator("ka_rigid.assign_selected", text="Dynamic")
        dynamic.body_type = "DYNAMIC"
        static = row.operator("ka_rigid.assign_selected", text="Static")
        static.body_type = "STATIC"
        row = utilities.row(align=True)
        row.operator("ka_rigid.remove_selected", icon="X")
        row.operator("ka_rigid.create_ground", text="Ground", icon="MESH_PLANE")
        utilities.operator("ka_rigid.validate", icon="CHECKMARK")

        diagnostics = column.box()
        diagnostics.label(text="Diagnostics", icon="CONSOLE")
        diagnostics.prop(world, "log_output")
        diagnostics.prop(world, "detailed_contact_diagnostics")
        if world.detailed_contact_diagnostics:
            diagnostics.prop(world, "side_stick_diagnostics")
            if world.side_stick_diagnostics:
                row = diagnostics.row(align=True)
                row.prop(world, "side_stick_min_frames")
                row.prop(world, "side_stick_slide_speed")
                diagnostics.prop(world, "side_stick_normal_z")
            diagnostics.alert = True
            diagnostics.label(text="Contact diagnostics are expensive on dense fracture scenes.", icon="ERROR")
        diagnostics.prop(world, "detailed_payload_diagnostics")
        if world.detailed_payload_diagnostics:
            diagnostics.label(text="Per-body payload logging creates large log files.", icon="INFO")


class KA_RIGID_PT_stability(Panel):
    bl_label = "Stability & Collision Proxies"
    bl_idname = "KA_RIGID_PT_stability"
    bl_parent_id = "KA_RIGID_PT_world"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "KA Physics"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        layout = self.layout
        world = context.scene.ka_rigid_world

        collider = layout.box()
        collider.label(text="Collision Proxies", icon="MOD_PHYSICS")
        row = collider.row(align=True)
        row.operator("ka_rigid.fix_invalid_colliders", icon="MODIFIER")
        row.operator("ka_rigid.clear_collider_cache", text="Clear Proxy Cache", icon="TRASH")
        row = collider.row(align=True)
        fast = row.operator("ka_rigid.set_selected_collider", text="Selected: Convex Hull", icon="MESH_ICOSPHERE")
        fast.collision_shape = "CONVEX_HULL"
        precise = row.operator("ka_rigid.set_selected_collider", text="Selected: Compound", icon="MOD_EXPLODE")
        precise.collision_shape = "COMPOUND_CONVEX"

        hull = collider.box()
        hull.label(text="Convex Hull", icon="MESH_ICOSPHERE")
        hull.prop(world, "hull_quality_preset")
        if world.hull_quality_preset == "CUSTOM":
            row = hull.row(align=True)
            row.prop(world, "hull_error_tolerance")
            row.prop(world, "hull_relative_error_tolerance")
            row = hull.row(align=True)
            row.prop(world, "hull_min_vertices")
            row.prop(world, "convex_hull_max_vertices")
            hull.prop(world, "hull_rescue_max_vertices")
        elif world.hull_quality_preset == "FAST":
            hull.label(text="Fast: 24–40 support points; lowest setup cost.", icon="INFO")
        elif world.hull_quality_preset == "ACCURATE":
            hull.label(text="Accurate: 64–128 support points; still one convex shell.", icon="INFO")
        else:
            hull.label(text="Balanced: 32–64 support points with precision rescue.", icon="INFO")

        compound = collider.box()
        compound.label(text="Compound Convex", icon="MOD_EXPLODE")
        available, detail = coacd_status()
        status_row = compound.row()
        status_row.alert = not available
        status_row.label(text=("CoACD available" if available else "CoACD unavailable"), icon="CHECKMARK" if available else "ERROR")
        compound.prop(world, "compound_quality_preset")
        if world.compound_quality_preset == "CUSTOM":
            row = compound.row(align=True)
            row.prop(world, "compound_error_tolerance")
            row.prop(world, "compound_relative_error_tolerance")
            row = compound.row(align=True)
            row.prop(world, "compound_max_parts")
            row.prop(world, "compound_max_hull_vertices")
            row = compound.row(align=True)
            row.prop(world, "compound_preprocess_resolution")
            row.prop(world, "compound_resolution")
            compound.prop(world, "compound_mcts_iterations")
            compound.prop(world, "compound_inset")
        elif world.compound_quality_preset == "FAST":
            compound.label(text="Fast: up to 4 convex parts, approx. 10 mm target.", icon="INFO")
        elif world.compound_quality_preset == "ACCURATE":
            compound.label(text="Accurate: up to 16 convex parts, approx. 1 mm target.", icon="INFO")
        else:
            compound.label(text="Balanced: up to 8 convex parts, approx. 3 mm target.", icon="INFO")
        compound.label(text="Select Compound Convex directly on bodies that need concave contact.", icon="INFO")
        compound.label(text="CoACD runs in an isolated worker; a bad mesh cannot terminate Blender.", icon="SHIELD")
        compound.label(text="Without true convex children, dynamic Compound Convex falls back to one complete convex hull.", icon="INFO")
        compound.label(text="Decomposition is cached; the first bake is slower.", icon="TIME")

        small = layout.box()
        small.label(text="Small Bodies", icon="PARTICLES")
        small.prop(world, "small_body_policy")
        row = small.row(align=True)
        row.prop(world, "minimum_dynamic_mass")
        row.prop(world, "minimum_body_radius")
        small.prop(world, "enforce_mass_ratio_limit")
        ratio_row = small.row()
        ratio_row.enabled = bool(world.enforce_mass_ratio_limit and world.small_body_policy == "STABILIZE")
        ratio_row.prop(world, "max_mass_ratio")

        ccd = layout.box()
        ccd.label(text="Continuous Collision", icon="FORCE_FORCE")
        ccd.prop(world, "adaptive_ccd")
        if world.adaptive_ccd:
            ccd.label(text="Jolt LinearCast stays armed after sleeping bodies are hit.", icon="INFO")
            ccd.label(text="Per-body CCD remains the master enable switch.", icon="INFO")


class KA_RIGID_PT_quality(Panel):
    bl_label = "Quality & Regression"
    bl_idname = "KA_RIGID_PT_quality"
    bl_parent_id = "KA_RIGID_PT_world"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "KA Physics"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        layout = self.layout
        world = context.scene.ka_rigid_world

        deterministic = layout.box()
        deterministic.label(text="Reproducibility", icon="FILE_REFRESH")
        deterministic.prop(world, "reproducibility_mode")
        if world.reproducibility_mode != "PERFORMANCE":
            deterministic.prop(world, "determinism_tolerance")
        if world.reproducibility_mode == "STRICT":
            deterministic.label(text="One worker thread; highest repeatability, lowest throughput.", icon="INFO")
        elif world.reproducibility_mode == "REPEATABLE":
            deterministic.label(text="Stable ordering with multi-threading and result comparison.", icon="INFO")
        else:
            deterministic.label(text="Maximum throughput; repeated results are not compared.", icon="INFO")

        tests = layout.box()
        tests.label(text="Automated Test Suite", icon="CHECKMARK")
        tests.operator("ka_rigid.run_regression", icon="PLAY")
        tests.label(text=world.regression_status)
        if world.regression_report_path:
            tests.label(text=f"Report: {world.regression_report_path[-80:]}", icon="TEXT")


class KA_RIGID_PT_bonds(Panel):
    bl_label = "Breakable Cohesion"
    bl_idname = "KA_RIGID_PT_bonds"
    bl_parent_id = "KA_RIGID_PT_world"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "KA Physics"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        layout = self.layout
        world = context.scene.ka_rigid_world
        layout.prop(world, "bond_enabled")
        settings = layout.column()
        settings.enabled = world.bond_enabled
        settings.prop(world, "bond_stability_mode")
        settings.prop(world, "bond_connection_distance")
        row = settings.row(align=True)
        row.prop(world, "bond_break_force")
        row.prop(world, "bond_break_torque")
        settings.prop(world, "bond_damage_accumulation")
        settings.label(text=f"Stored Bonds: {world.bond_count}", icon="LINKED")
        row = settings.row(align=True)
        row.operator("ka_rigid.generate_bonds", text="Generate Bonds", icon="LINKED")
        selected_bonds = row.operator("ka_rigid.generate_bonds", text="Selected Only")
        selected_bonds.selected_only = True
        settings.operator("ka_rigid.clear_bonds", icon="TRASH")
        settings.label(text="Rigid mode keeps intact bond islands solid.", icon="INFO")
        settings.label(text="Bonds can connect Dynamic-Dynamic and Dynamic-Static bodies.", icon="INFO")


class KA_RIGID_PT_constraints(Panel):
    bl_label = "Rope Constraints"
    bl_idname = "KA_RIGID_PT_constraints"
    bl_parent_id = "KA_RIGID_PT_world"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "KA Physics"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        layout.label(text=f"Enabled Constraints: {constraint_count(scene)}", icon="CONSTRAINT")
        row = layout.row(align=True)
        row.operator("ka_rigid.create_rope_constraint", text="From Selected", icon="LINKED")
        row.operator("ka_rigid.create_rope_anchor", text="Anchor at Cursor", icon="PIVOT_CURSOR")
        layout.label(text="Rope limits only the maximum distance.", icon="INFO")
        layout.label(text="The cursor anchor is Static and does not collide.", icon="INFO")


class KA_RIGID_PT_constraint(Panel):
    bl_label = "Selected Constraint"
    bl_idname = "KA_RIGID_PT_constraint"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "KA Physics"

    @classmethod
    def poll(cls, context):
        obj = context.active_object
        if obj is None or not hasattr(obj, "ka_rigid_constraint"):
            return False
        settings = obj.ka_rigid_constraint
        return bool(settings.enabled or settings.body_a is not None or settings.body_b is not None)

    def draw(self, context):
        layout = self.layout
        obj = context.active_object
        settings = obj.ka_rigid_constraint
        layout.label(text=obj.name, icon="CONSTRAINT")
        layout.prop(settings, "enabled")
        column = layout.column()
        column.enabled = settings.enabled
        column.prop(settings, "constraint_mode")
        column.prop(settings, "body_a")
        column.prop(settings, "body_b")
        column.prop(settings, "use_current_distance")
        if settings.use_current_distance:
            if settings.body_a is not None and settings.body_b is not None:
                distance = float(
                    (settings.body_b.matrix_world.translation - settings.body_a.matrix_world.translation).length
                )
                column.label(text=f"Current center distance: {distance:.4f} m", icon="DRIVER_DISTANCE")
        else:
            column.prop(settings, "distance")
        column.operator("ka_rigid.store_constraint_length", icon="EYEDROPPER")
        if settings.constraint_mode == "ROPE":
            column.label(text="Minimum distance: 0 m", icon="INFO")
        else:
            column.label(text="Rod mode also transmits compression.", icon="INFO")
        layout.separator()
        layout.operator("ka_rigid.delete_constraint", icon="TRASH")


class KA_RIGID_PT_body(Panel):
    bl_label = "Selected Body"
    bl_idname = "KA_RIGID_PT_body"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "KA Physics"

    @classmethod
    def poll(cls, context):
        return context.active_object is not None

    def draw(self, context):
        layout = self.layout
        obj = context.active_object
        settings = obj.ka_rigid_body
        layout.label(text=obj.name, icon="OBJECT_DATA")
        layout.prop(settings, "enabled")
        column = layout.column()
        column.enabled = settings.enabled
        column.prop(settings, "body_type")
        column.prop(settings, "collision_shape")
        if settings.collision_shape in {"CONVEX_HULL", "COMPOUND_CONVEX", "MESH"}:
            column.prop(settings, "collision_proxy")
            if settings.collision_proxy is not None:
                column.label(text="Proxy supplies collision geometry only.", icon="MOD_SIMPLIFY")
        if settings.collision_shape == "COMPOUND_CONVEX":
            column.label(text="Concave approximation using one stable compound body.", icon="INFO")
            column.label(text="Use only where a Single Hull creates visible gaps.", icon="INFO")
        elif settings.collision_shape == "MESH":
            column.label(text="Exact triangle contact; available only for Static bodies.", icon="INFO")
        size_mass = column.box()
        size_mass.label(text="Dimensions & Mass", icon="ARROW_LEFTRIGHT")
        size_mass.prop(obj, "dimensions")
        size_mass.prop(obj, "scale")
        if settings.body_type == "DYNAMIC":
            size_mass.prop(settings, "mass_mode")
            size_mass.prop(settings, "mass" if settings.mass_mode == "MASS" else "density")
            if settings.mass_mode == "DENSITY":
                size_mass.label(text="Mass is calculated from evaluated mesh volume at bake time.", icon="INFO")
        else:
            size_mass.label(text="Static bodies have no simulated mass.", icon="INFO")
        material = column.box()
        material.label(text="Material")
        row = material.row(align=True)
        row.prop(settings, "friction")
        row.prop(settings, "restitution")
        row = material.row(align=True)
        row.prop(settings, "linear_damping")
        row.prop(settings, "angular_damping")
        velocity = column.box()
        velocity.label(text="Initial Motion")
        velocity.prop(settings, "initial_linear_velocity")
        velocity.prop(settings, "initial_angular_velocity")
        velocity.prop(settings, "use_ccd")
        collision = column.box()
        row = collision.row(align=True)
        row.prop(settings, "collision_layer")
        row.prop(settings, "collision_mask")
        row = column.row(align=True)
        row.operator("ka_rigid.set_rest_transform", icon="KEYFRAME_HLT")
        row.operator("ka_rigid.restore_rest_transform", icon="LOOP_BACK")


CLASSES = (
    KA_RIGID_PT_world,
    KA_RIGID_PT_stability,
    KA_RIGID_PT_quality,
    KA_RIGID_PT_bonds,
    KA_RIGID_PT_constraints,
    KA_RIGID_PT_constraint,
    KA_RIGID_PT_body,
)


def register_ui() -> None:
    for cls in CLASSES:
        bpy.utils.register_class(cls)


def unregister_ui() -> None:
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)
