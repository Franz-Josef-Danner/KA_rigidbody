"""Blender properties for KA Rigid Dynamics."""

import bpy
from bpy.props import (
    BoolProperty,
    EnumProperty,
    FloatProperty,
    FloatVectorProperty,
    IntProperty,
    PointerProperty,
    StringProperty,
)
from bpy.types import AddonPreferences, PropertyGroup

from .core.stability_defaults import PENETRATION_SLOP_DEFAULT


_COLLISION_SHAPE_ITEMS = (
    # Keep the numeric identifiers stable with versions <= 0.4.9. Blender stores
    # enum values numerically in .blend files, so changing these numbers would
    # silently reinterpret existing collider settings.
    ("SPHERE", "Sphere", "Fast sphere collision proxy", 0),
    ("BOX", "Box", "Fast oriented box proxy", 1),
    ("PLANE", "Plane (Static Only)", "Infinite local XY plane; static only", 2),
    ("CONVEX_HULL", "Convex Hull", "One native Jolt convex hull; fastest irregular dynamic collider", 3),
    ("MESH", "Triangle Mesh (Static Only)", "Exact static triangle mesh", 4),
    ("COMPOUND_CONVEX", "Compound Convex", "CoACD decomposition into several rigid convex hulls for concave moving objects", 5),
)


def _body_type_updated(settings, _context) -> None:
    """Keep collider choices valid when the body type changes."""
    shape = str(getattr(settings, "collision_shape", "CONVEX_HULL"))
    if settings.body_type != "STATIC" and shape in {"MESH", "PLANE"}:
        settings.collision_shape = "CONVEX_HULL"


def _collision_shape_updated(settings, _context) -> None:
    """Correct only combinations that Jolt cannot represent safely."""
    shape = str(getattr(settings, "collision_shape", "CONVEX_HULL"))
    if shape == "MESH" and settings.body_type != "STATIC":
        settings.collision_shape = "CONVEX_HULL"
    elif shape == "PLANE" and settings.body_type != "STATIC":
        settings.collision_shape = "CONVEX_HULL"


def _constraint_body_poll(_settings, obj) -> bool:
    return bool(
        obj is not None
        and hasattr(obj, "ka_rigid_body")
        and obj.ka_rigid_body.enabled
    )


class KA_RIGID_AddonPreferences(AddonPreferences):
    bl_idname = __package__

    jolt_bridge_path: StringProperty(
        name="Jolt bridge",
        subtype="FILE_PATH",
        description="Optional compiled KA Jolt 5.6 ABI-v2 bridge; when empty the bundled bridge is discovered automatically",
    )

    physx_bridge_path: StringProperty(
        name="PhysX bridge",
        subtype="FILE_PATH",
        description="Path to the compiled ka_physx_bridge DLL/shared library",
    )

    def draw(self, context):
        layout = self.layout
        layout.label(text="Native backends", icon="PLUGIN")
        layout.prop(self, "jolt_bridge_path")
        layout.prop(self, "physx_bridge_path")
        box = layout.box()
        box.label(text="Jolt ABI-v2 supports true Compound Convex bodies.", icon="INFO")
        box.label(text="Without it, bundled Culverin remains the automatic fallback.")
        box.label(text="Supported target platforms: Windows x64 and Linux x64.")


class KA_RIGID_WorldSettings(PropertyGroup):
    settings_version: IntProperty(default=0, options={"HIDDEN"})
    enabled: BoolProperty(name="Enable KA Physics", default=True)
    backend: EnumProperty(
        name="Backend",
        items=(
            ("REFERENCE", "Reference", "Functional pipeline-validation solver"),
            ("JOLT", "Jolt", "Native multi-core CPU backend"),
            ("PHYSX", "PhysX", "Native CPU/GPU backend"),
        ),
        default="JOLT",
    )
    frame_start: IntProperty(name="Start", default=1, min=-100000, max=100000)
    frame_end: IntProperty(name="End", default=250, min=-100000, max=100000)
    use_scene_gravity: BoolProperty(name="Use Scene Gravity", default=True)
    gravity: FloatVectorProperty(name="Gravity", default=(0.0, 0.0, -9.81), size=3, subtype="ACCELERATION")
    substeps: IntProperty(
        name="Maximum Substeps", default=8, min=1, max=128,
        description="Maximum substeps per rendered frame; Adaptive Substeps can use fewer during calm phases",
    )
    adaptive_substeps: BoolProperty(
        name="Adaptive Substeps", default=True,
        description="Reduce substeps during low-speed phases while retaining the configured maximum for demanding contacts",
    )
    minimum_substeps: IntProperty(
        name="Minimum Substeps", default=4, min=1, max=64,
        description="Lower substep bound used by the adaptive scheduler",
    )
    solver_iterations: IntProperty(
        name="Solver Iterations",
        default=8,
        min=1,
        max=64,
        description="Reference backend iterations; Jolt uses its native internal solver settings",
    )
    jolt_threads: IntProperty(name="Jolt Threads", default=0, min=0, max=64, description="0 selects an automatic worker count")
    penetration_slop: FloatProperty(
        name="Penetration Slop",
        default=PENETRATION_SLOP_DEFAULT,
        min=0.00001,
        soft_max=0.05,
        unit="LENGTH",
        description="Allowed contact penetration tolerance passed to Jolt",
    )

    sleep_enabled: BoolProperty(name="Sleeping", default=True)
    sleep_mode: EnumProperty(
        name="Sleeping Mode",
        items=(
            ("NATIVE", "Native Jolt", "Use only Jolt's native island sleeping"),
            ("HYBRID", "Hybrid Experimental", "Batch low-motion deactivation and count a body as sleeping only after Jolt confirms it"),
            ("CUSTOM", "Custom Thresholds", "Use the add-on's explicit linear/angular thresholds after each rendered frame"),
        ),
        default="NATIVE",
    )
    sleep_linear_threshold: FloatProperty(name="Linear Threshold", default=0.05, min=0.0, soft_max=2.0, unit="VELOCITY")
    sleep_angular_threshold: FloatProperty(name="Angular Threshold", default=0.25, min=0.0, soft_max=5.0)
    sleep_time: FloatProperty(name="Sleep Time", default=0.5, min=0.0, soft_max=5.0, unit="TIME")
    early_sleep_termination: BoolProperty(
        name="Stop When All Bodies Sleep", default=True,
        description="Stop stepping the native solver after all dynamic bodies remain asleep for several frames",
    )
    early_sleep_frames: IntProperty(
        name="Sleep Confirmation Frames", default=3, min=1, max=30,
        description="Consecutive fully sleeping frames required before the remaining cache frames are filled",
    )

    auto_fix_invalid_colliders: BoolProperty(
        name="Auto-Fix Invalid Colliders",
        default=True,
        description="Convert Mesh colliders on dynamic/kinematic bodies to Convex Hull before baking",
    )
    duplicate_static_policy: EnumProperty(
        name="Duplicate Static Colliders",
        items=(
            ("EXCLUDE", "Exclude from Bake", "Keep one collider enabled and disable overlapping static duplicates"),
            ("DELETE", "Delete Duplicates", "Keep one collider and permanently delete overlapping static duplicates"),
            ("WARN", "Warn Only", "Report duplicate static colliders but leave them enabled"),
        ),
        default="EXCLUDE",
        description="How Preflight handles multiple KA grounds and fully overlapping static colliders",
    )
    convex_hull_max_vertices: IntProperty(
        name="Hull Vertex Limit",
        default=64,
        min=0,
        max=1024,
        description="Maximum support points used for generated convex hulls; 0 keeps the complete hull",
    )

    adaptive_hull_accuracy: BoolProperty(
        name="Adaptive Hull Accuracy",
        default=True,
        description="Increase hull support points until the directional shape error is below the selected tolerance",
    )
    hull_quality_preset: EnumProperty(
        name="Collider Quality",
        items=(
            ("FAST", "Fast", "24 to 40 support points; use only when speed is more important than contact stability"),
            ("BALANCED", "Balanced / Stable", "Adaptive 32 to 64 point convex proxies, optimized for stable resting contacts"),
            ("ACCURATE", "Accurate", "64 to 128 support points with approx. 0.5 mm support error"),
            ("CUSTOM", "Custom", "Use the custom error, minimum and maximum point settings"),
        ),
        default="BALANCED",
    )
    hull_error_tolerance: FloatProperty(
        name="Absolute Hull Error",
        default=0.00075,
        min=0.00001,
        soft_max=0.02,
        unit="LENGTH",
        description="Minimum absolute directional support error allowed for adaptive convex proxies",
    )
    hull_relative_error_tolerance: FloatProperty(
        name="Relative Hull Error",
        default=0.005,
        min=0.0,
        soft_max=0.05,
        subtype="FACTOR",
        description="Scale-aware support error as a fraction of the collider bounding-box diagonal",
    )
    hull_rescue_max_vertices: IntProperty(
        name="Rescue Vertex Limit",
        default=256,
        min=4,
        max=4096,
        description="Maximum adaptive budget used before correctness falls back to the complete convex hull",
    )
    hull_min_vertices: IntProperty(
        name="Hull Minimum",
        default=32,
        min=4,
        max=1024,
        description="Initial support-point budget used by adaptive hull generation",
    )

    # Legacy scene fields are kept hidden so older .blend files migrate cleanly.
    compound_mode: EnumProperty(
        name="Legacy Dynamic Collider",
        items=(("SINGLE", "Single", "Legacy"), ("AUTO", "Auto", "Legacy"), ("ALWAYS", "Always", "Legacy")),
        default="SINGLE",
        options={"HIDDEN"},
    )
    compound_quality_preset: EnumProperty(
        name="Compound Quality",
        items=(
            ("FAST", "Fast", "Up to 4 convex parts with a coarse 10 mm target"),
            ("BALANCED", "Balanced", "Up to 8 convex parts with a 3 mm target"),
            ("ACCURATE", "Accurate", "Up to 16 convex parts with a 1 mm target"),
            ("CUSTOM", "Custom", "Use the advanced CoACD settings"),
        ),
        default="BALANCED",
        description="Quality used only by bodies set to Compound Convex",
    )
    compound_max_parts: IntProperty(
        name="Maximum Parts", default=8, min=2, max=32,
        description="Maximum number of convex pieces generated for one Compound Convex body",
    )
    compound_error_tolerance: FloatProperty(
        name="Absolute Error", default=0.003, min=0.00001, soft_max=0.05, unit="LENGTH",
        description="CoACD concavity target in scene units",
    )
    compound_relative_error_tolerance: FloatProperty(
        name="Relative Error", default=0.005, min=0.0, soft_max=0.05, subtype="FACTOR",
        description="Additional scale-aware error target as a fraction of the collider diagonal",
    )
    compound_max_hull_vertices: IntProperty(
        name="Vertices per Part", default=96, min=8, max=256,
        description="Maximum convex-hull vertices retained for each decomposed part",
    )
    compound_preprocess_resolution: IntProperty(
        name="Preprocess Resolution", default=50, min=10, max=200,
        description="CoACD preprocessing resolution",
    )
    compound_resolution: IntProperty(
        name="Search Resolution", default=2000, min=100, max=10000,
        description="CoACD decomposition search resolution",
    )
    compound_mcts_iterations: IntProperty(
        name="Search Iterations", default=150, min=10, max=1000,
        description="CoACD Monte-Carlo tree-search iterations",
    )
    compound_inset: FloatProperty(
        name="Part Inset", default=0.0005, min=0.0, soft_max=0.01, unit="LENGTH",
        description="Shrink each decomposed convex part slightly before creating a native or primitive compound",
    )
    # Deprecated box-compound quality fields. They remain readable for migration only.
    compound_trigger_error: FloatProperty(name="Legacy Trigger", default=0.004, options={"HIDDEN"})
    compound_min_coverage: FloatProperty(name="Legacy Coverage", default=0.92, options={"HIDDEN"})
    compound_max_outside_volume: FloatProperty(name="Legacy Outside", default=0.08, options={"HIDDEN"})
    compound_max_surface_deviation: FloatProperty(name="Legacy Deviation", default=0.002, options={"HIDDEN"})
    compound_min_improvement: FloatProperty(name="Legacy Improvement", default=0.20, options={"HIDDEN"})
    compound_runtime_guard: BoolProperty(
        name="Compound Contact Guard", default=False, options={"HIDDEN"},
        description="Legacy box-compound fallback guard; disabled for CoACD convex clusters",
    )

    reproducibility_mode: EnumProperty(
        name="Execution Mode",
        items=(
            ("PERFORMANCE", "Performance", "Use automatic multi-threading without comparing repeated results"),
            ("REPEATABLE", "Repeatable", "Use stable ordering, multi-threading and compare matching scene bakes"),
            ("STRICT", "Strict", "Force one Jolt worker thread for maximum repeatability"),
        ),
        default="REPEATABLE",
    )
    deterministic_mode: BoolProperty(
        name="Legacy Deterministic Mode", default=True, options={"HIDDEN"},
        description="Legacy compatibility flag; Execution Mode now controls reproducibility",
    )
    determinism_tolerance: FloatProperty(
        name="Comparison Tolerance",
        default=0.000001,
        min=0.0,
        soft_max=0.001,
        precision=8,
        description="Maximum transform difference accepted when comparing repeated bakes",
    )
    regression_status: StringProperty(name="Regression Status", default="Not run")
    regression_report_path: StringProperty(options={"HIDDEN"})

    small_body_policy: EnumProperty(
        name="Small Body Handling",
        items=(
            ("SIMULATE", "Simulate Unchanged", "Keep very small bodies unchanged and only report them"),
            ("STABILIZE", "Stabilize", "Clamp very small masses and use a Box proxy when the body radius is below the minimum"),
            ("SKIP", "Skip Simulation", "Exclude bodies below the minimum mass or radius from the solver payload"),
        ),
        default="STABILIZE",
    )
    minimum_dynamic_mass: FloatProperty(
        name="Minimum Mass",
        default=0.001,
        min=0.000001,
        soft_max=1.0,
        unit="MASS",
        description="Stability floor for dynamic body mass",
    )
    minimum_body_radius: FloatProperty(
        name="Minimum Radius",
        default=0.005,
        min=0.00001,
        soft_max=0.1,
        unit="LENGTH",
        description="Bodies below this approximate radius are stabilized or skipped according to Small Body Handling",
    )
    enforce_mass_ratio_limit: BoolProperty(
        name="Condition Extreme Mass Ratios",
        default=True,
        description="In Stabilize mode, condition very small solver masses inside each authored Dynamic-Dynamic bond component; independent projectiles keep their authored mass",
    )
    max_mass_ratio: FloatProperty(
        name="Mass Ratio Limit",
        default=5000.0,
        min=10.0,
        soft_max=100000.0,
        description="Maximum solver-mass ratio inside each bonded dynamic component; independent bodies are not rescaled and displayed/source masses remain unchanged",
    )

    adaptive_ccd: BoolProperty(
        name="Adaptive CCD",
        default=True,
        description="Arm Jolt LinearCast for every dynamic body that requests CCD; Jolt performs the expensive cast only when the per-step motion requires it",
    )
    ccd_max_radius: FloatProperty(
        name="CCD Max Radius",
        default=0.05,
        min=0.00001,
        soft_max=1.0,
        unit="LENGTH",
        description="Legacy compatibility value; Jolt now evaluates LinearCast demand per simulation step",
    )
    ccd_speed_threshold: FloatProperty(
        name="CCD Speed Threshold",
        default=5.0,
        min=0.0,
        soft_max=100.0,
        unit="VELOCITY",
        description="Legacy compatibility value; authored start velocity no longer disables later impact CCD",
    )

    detailed_contact_diagnostics: BoolProperty(
        name="Detailed Contact Diagnostics",
        default=False,
        description="Read and aggregate native contact events only. Log Output controls whether the collected details are written",
    )

    side_stick_diagnostics: BoolProperty(
        name="Side-Stick Diagnostics",
        default=False,
        description="When detailed contacts are enabled, report long low-speed contacts with mostly horizontal normals",
    )
    side_stick_min_frames: IntProperty(
        name="Minimum Contact Frames", default=8, min=1, max=250,
        description="Minimum uninterrupted rendered-frame streak before a contact pair is reported as a possible side-stick",
    )
    side_stick_normal_z: FloatProperty(
        name="Maximum Vertical Normal", default=0.35, min=0.0, max=1.0, subtype="FACTOR",
        description="Normals below this absolute vertical component are treated as side contacts",
    )
    side_stick_slide_speed: FloatProperty(
        name="Maximum Slide Speed", default=0.05, min=0.0, soft_max=1.0, unit="VELOCITY",
        description="Only low-sliding persistent contacts are reported as possible side-sticks",
    )

    detailed_payload_diagnostics: BoolProperty(
        name="Detailed Payload Diagnostics",
        default=False,
        description="Collect detailed per-body payload and speed-peak data. Log Output writes it; large fracture scenes create very large logs",
    )

    cache_directory: StringProperty(
        name="Cache Directory",
        subtype="DIR_PATH",
        description="Empty uses //ka_rigid_cache for saved .blend files or the system temp directory",
    )
    cache_playback: BoolProperty(name="Cache Playback", default=True)
    log_output: BoolProperty(
        name="Log Ausgaben",
        description="Write general events and only the explicitly enabled contact/payload details to ka_rigid_dynamics.log; does not activate analyses",
        default=False,
    )
    cache_status: StringProperty(name="Cache Status", default="Not baked")
    cache_signature: StringProperty(options={"HIDDEN"})
    bond_enabled: BoolProperty(
        name="Enable Bonds",
        description="Create breakable Fixed constraints from the persisted bond graph during Jolt bakes",
        default=True,
    )
    bond_stability_mode: EnumProperty(
        name="Cohesion Mode",
        description="Rigid simulates each intact bond island as one solid compound actor and splits it only after fracture; Flexible uses a native Fixed-constraint network",
        items=(
            ("RIGID", "Rigid", "Recommended for stone, concrete and statues; native constraint backbone with coordinated settling of the intact island"),
            ("FLEXIBLE", "Flexible", "Use the native Fixed-constraint network and allow solver compliance between fragments"),
        ),
        default="RIGID",
    )
    bond_connection_distance: FloatProperty(
        name="Connection Distance",
        description="Maximum world-space mesh vertex distance used when generating neighboring bonds",
        default=0.002,
        min=0.0,
        soft_max=0.05,
        precision=4,
        unit="LENGTH",
    )
    bond_break_force: FloatProperty(
        name="Break Force",
        description="Estimated external force in Newton that breaks a generated bond; zero disables force breaking",
        default=10000.0,
        min=0.0,
        soft_max=1000000.0,
    )
    bond_break_torque: FloatProperty(
        name="Break Torque",
        description="Estimated external torque in Newton metre that breaks a generated bond; zero disables torque breaking",
        default=1000.0,
        min=0.0,
        soft_max=100000.0,
    )
    bond_damage_accumulation: FloatProperty(
        name="Damage Accumulation",
        description="Accumulated sub-threshold bond damage per second; zero uses only immediate force and torque thresholds",
        default=0.0,
        min=0.0,
        soft_max=10.0,
    )
    bond_count: IntProperty(name="Bond Count", default=0, min=0, options={"HIDDEN"})
    bond_data: StringProperty(name="Bond Data", default="", options={"HIDDEN"})


class KA_RIGID_BodySettings(PropertyGroup):
    enabled: BoolProperty(name="KA Rigid Body", default=False)
    body_type: EnumProperty(
        name="Type",
        items=(
            ("DYNAMIC", "Dynamic", "Moved by the solver"),
            ("STATIC", "Static", "Fixed collision object"),
            ("KINEMATIC", "Kinematic", "Reserved for animated driver input"),
        ),
        default="DYNAMIC",
        update=_body_type_updated,
    )
    collision_shape: EnumProperty(
        name="Collision Shape",
        items=_COLLISION_SHAPE_ITEMS,
        default="CONVEX_HULL",
        update=_collision_shape_updated,
    )
    collision_proxy: PointerProperty(
        name="Collision Proxy",
        type=bpy.types.Object,
        description="Optional low-poly mesh used only for collision geometry; mass and rendering remain based on this body",
    )
    mass_mode: EnumProperty(
        name="Mass Source",
        items=(
            ("MASS", "Mass", "Use a fixed body mass"),
            ("DENSITY", "Density", "Calculate mass from evaluated mesh volume"),
        ),
        default="MASS",
    )
    mass: FloatProperty(name="Mass", default=1.0, min=0.000001, soft_max=10000.0, unit="MASS")
    density: FloatProperty(name="Density", default=1000.0, min=0.000001, soft_max=10000.0, unit="MASS")
    friction: FloatProperty(name="Friction", default=0.5, min=0.0, soft_max=2.0)
    restitution: FloatProperty(name="Restitution", default=0.0, min=0.0, soft_max=1.0)
    linear_damping: FloatProperty(name="Linear Damping", default=0.04, min=0.0, soft_max=5.0)
    angular_damping: FloatProperty(name="Angular Damping", default=0.1, min=0.0, soft_max=5.0)
    initial_linear_velocity: FloatVectorProperty(name="Initial Velocity", default=(0.0, 0.0, 0.0), size=3, subtype="VELOCITY")
    initial_angular_velocity: FloatVectorProperty(name="Initial Angular Velocity", default=(0.0, 0.0, 0.0), size=3)
    use_ccd: BoolProperty(name="Continuous Collision", default=True)
    collision_layer: IntProperty(name="Layer", default=0, min=0, max=15)
    collision_mask: IntProperty(name="Mask", default=0xFFFF, min=0, max=0xFFFF)
    rest_transform_stored: BoolProperty(default=False, options={"HIDDEN"})
    rest_location: FloatVectorProperty(size=3, subtype="TRANSLATION", options={"HIDDEN"})
    rest_rotation: FloatVectorProperty(size=4, subtype="QUATERNION", default=(1.0, 0.0, 0.0, 0.0), options={"HIDDEN"})
    rest_scale: FloatVectorProperty(size=3, default=(1.0, 1.0, 1.0), options={"HIDDEN"})


class KA_RIGID_ConstraintSettings(PropertyGroup):
    enabled: BoolProperty(
        name="KA Constraint",
        default=False,
        description="Include this authored mechanical constraint in the next Jolt bake",
    )
    constraint_mode: EnumProperty(
        name="Mode",
        items=(
            ("ROPE", "Rope", "Maximum distance only; transmits tension but no compression"),
            ("ROD", "Rod", "Fixed distance; behaves like a massless rigid bar"),
        ),
        default="ROPE",
    )
    body_a: PointerProperty(
        name="Anchor Body",
        type=bpy.types.Object,
        poll=_constraint_body_poll,
        description="First KA body; normally the Static suspension anchor",
    )
    body_b: PointerProperty(
        name="Swinging Body",
        type=bpy.types.Object,
        poll=_constraint_body_poll,
        description="Second KA body; normally the Dynamic wrecking ball",
    )
    use_current_distance: BoolProperty(
        name="Use Current Distance",
        default=True,
        description="Measure the native body-center distance again at bake time",
    )
    distance: FloatProperty(
        name="Length",
        default=5.0,
        min=0.00001,
        soft_max=100.0,
        unit="LENGTH",
        description="Maximum rope length or fixed rod length when Current Distance is disabled",
    )


CLASSES = (
    KA_RIGID_AddonPreferences,
    KA_RIGID_WorldSettings,
    KA_RIGID_BodySettings,
    KA_RIGID_ConstraintSettings,
)


def register_properties() -> None:
    for cls in CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.Scene.ka_rigid_world = PointerProperty(type=KA_RIGID_WorldSettings)
    bpy.types.Object.ka_rigid_body = PointerProperty(type=KA_RIGID_BodySettings)
    bpy.types.Object.ka_rigid_constraint = PointerProperty(type=KA_RIGID_ConstraintSettings)


def unregister_properties() -> None:
    if hasattr(bpy.types.Object, "ka_rigid_constraint"):
        del bpy.types.Object.ka_rigid_constraint
    if hasattr(bpy.types.Object, "ka_rigid_body"):
        del bpy.types.Object.ka_rigid_body
    if hasattr(bpy.types.Scene, "ka_rigid_world"):
        del bpy.types.Scene.ka_rigid_world
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)
