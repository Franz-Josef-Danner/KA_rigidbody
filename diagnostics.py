"""Blender-facing diagnostic helpers."""

from __future__ import annotations

import os
import traceback
from typing import Any

import bpy

from .core.diagnostics import write_diagnostic
from .core.scene_io import resolve_cache_directory

LOG_FILENAME = "ka_rigid_dynamics.log"


def log_file_path(scene: bpy.types.Scene) -> str:
    return os.path.join(resolve_cache_directory(scene), LOG_FILENAME)


def logging_enabled(scene: bpy.types.Scene) -> bool:
    return bool(
        hasattr(scene, "ka_rigid_world")
        and getattr(scene.ka_rigid_world, "log_output", False)
    )


def log_event(
    scene: bpy.types.Scene,
    component: str,
    event: str,
    *,
    level: str = "INFO",
    **data: Any,
) -> None:
    write_diagnostic(
        logging_enabled(scene),
        log_file_path(scene),
        component,
        event,
        level=level,
        data=data,
    )


def log_exception(
    scene: bpy.types.Scene,
    component: str,
    event: str,
    exc: BaseException,
    **data: Any,
) -> None:
    data.update(
        error_type=type(exc).__name__,
        error=str(exc),
        traceback=traceback.format_exc(),
    )
    log_event(scene, component, event, level="ERROR", **data)
