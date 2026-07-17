"""KA Rigid Dynamics Blender add-on."""

bl_info = {
    "name": "KA Rigid Dynamics",
    "author": "KA",
    "version": (0, 5, 1),
    "blender": (4, 2, 0),
    "location": "View3D > Sidebar > KA Physics",
    "description": "Native Jolt rigid-body simulation with cache playback and KA Fracture integration.",
    "category": "Physics",
}


def register():
    from .operators import register_operators
    from .properties import register_properties
    from .runtime import register_handlers
    from .ui import register_ui

    register_properties()
    register_operators()
    register_ui()
    register_handlers()


def unregister():
    from .core.scene_io import clear_geometry_cache
    from .operators import unregister_operators
    from .properties import unregister_properties
    from .runtime import unregister_handlers
    from .ui import unregister_ui

    unregister_handlers()
    clear_geometry_cache()
    unregister_ui()
    unregister_operators()
    unregister_properties()
