"""Backend protocol for KA Rigid Dynamics."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, Optional

ProgressCallback = Optional[Callable[[int, int], None]]


@dataclass(frozen=True)
class BackendStatus:
    identifier: str
    name: str
    available: bool
    production_ready: bool
    detail: str


class BackendError(RuntimeError):
    pass


class PhysicsBackend:
    identifier = "BASE"
    name = "Base"

    @classmethod
    def status(cls, preferences=None) -> BackendStatus:
        raise NotImplementedError

    def bake(self, scene_payload: Dict, progress: ProgressCallback = None) -> Dict:
        raise NotImplementedError
