"""Backend registry."""

from __future__ import annotations

from .base import BackendError, BackendStatus, PhysicsBackend
from .jolt import JoltBackend
from .physx import PhysXBackend
from .reference import ReferenceBackend

BACKEND_CLASSES = {
    ReferenceBackend.identifier: ReferenceBackend,
    JoltBackend.identifier: JoltBackend,
    PhysXBackend.identifier: PhysXBackend,
}


def get_backend(identifier: str) -> PhysicsBackend:
    backend_class = BACKEND_CLASSES.get(identifier)
    if backend_class is None:
        raise BackendError(f"Unknown backend: {identifier}")
    return backend_class()
