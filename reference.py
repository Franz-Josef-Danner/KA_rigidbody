"""PhysX backend discovery."""

from __future__ import annotations

import importlib.util
from typing import Dict

from .base import BackendError, BackendStatus, PhysicsBackend, ProgressCallback
from .native_bridge import bridge_status


class PhysXBackend(PhysicsBackend):
    identifier = "PHYSX"
    name = "PhysX Native / ovphysx"

    @classmethod
    def status(cls, preferences=None) -> BackendStatus:
        path = getattr(preferences, "physx_bridge_path", "") if preferences else ""
        native_available, native_detail = bridge_status(path)
        ovphysx_available = importlib.util.find_spec("ovphysx") is not None
        if native_available:
            detail = native_detail + " Simulation calls are reserved for the next bridge milestone."
            return BackendStatus(cls.identifier, cls.name, True, False, detail)
        if ovphysx_available:
            return BackendStatus(
                cls.identifier,
                cls.name,
                True,
                False,
                "ovphysx is importable, but its pre-release USD scene adapter is not enabled in version 0.1.0.",
            )
        return BackendStatus(
            cls.identifier,
            cls.name,
            False,
            False,
            native_detail + " ovphysx is not installed in Blender's Python environment.",
        )

    def bake(self, scene_payload: Dict, progress: ProgressCallback = None) -> Dict:
        raise BackendError(
            "The PhysX adapter is discovered but not enabled in version 0.1.0. Use Reference for pipeline tests."
        )
