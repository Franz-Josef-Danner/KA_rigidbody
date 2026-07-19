"""ctypes adapter that presents the ABI-v2 Jolt bridge like Culverin.

Keeping this surface close to Culverin lets the mature bake/cache loop run on
both implementations while the native bridge adds true convex compounds.
"""

from __future__ import annotations

import array
import ctypes
import os
import platform
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Iterable, Optional, Sequence

from .native_bridge import NativeBridgeLoadError, load_bridge

MOTION_STATIC = 0
MOTION_KINEMATIC = 1
MOTION_DYNAMIC = 2
SHAPE_BOX = 0
SHAPE_SPHERE = 1
SHAPE_PLANE = 4
CONSTRAINT_FIXED = 0
EVENT_ADDED = 0
EVENT_PERSISTED = 1
EVENT_REMOVED = 2
USE_DOUBLE_PRECISION = False
NATIVE_COMPOUND_CONVEX = True
NATIVE_BRIDGE = True
__version__ = "Jolt 5.6.0 ABI-2"

_INVALID_BODY = 0xFFFFFFFF


class Vec3(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float), ("y", ctypes.c_float), ("z", ctypes.c_float)]


class Quat(ctypes.Structure):
    _fields_ = [("w", ctypes.c_float), ("x", ctypes.c_float), ("y", ctypes.c_float), ("z", ctypes.c_float)]


class Transform(ctypes.Structure):
    _fields_ = [("position", Vec3), ("rotation", Quat)]


class WorldDesc(ctypes.Structure):
    _fields_ = [
        ("gravity", Vec3),
        ("max_bodies", ctypes.c_uint32),
        ("max_body_pairs", ctypes.c_uint32),
        ("max_contact_constraints", ctypes.c_uint32),
        ("temp_allocator_bytes", ctypes.c_uint32),
        ("worker_threads", ctypes.c_uint32),
        ("penetration_slop", ctypes.c_float),
    ]


class BodyDesc(ctypes.Structure):
    _fields_ = [
        ("transform", Transform),
        ("motion_type", ctypes.c_uint32),
        ("mass", ctypes.c_float),
        ("user_data", ctypes.c_uint64),
        ("category", ctypes.c_uint32),
        ("mask", ctypes.c_uint32),
        ("friction", ctypes.c_float),
        ("restitution", ctypes.c_float),
        ("linear_damping", ctypes.c_float),
        ("angular_damping", ctypes.c_float),
        ("continuous_collision", ctypes.c_uint32),
    ]


class CompoundChild(ctypes.Structure):
    _fields_ = [
        ("local_transform", Transform),
        ("vertices", ctypes.POINTER(Vec3)),
        ("vertex_count", ctypes.c_uint32),
        ("user_data", ctypes.c_uint32),
    ]


class ContactEvent(ctypes.Structure):
    _fields_ = [
        ("body1", ctypes.c_uint32),
        ("body2", ctypes.c_uint32),
        ("event_type", ctypes.c_uint32),
        ("point", Vec3),
        ("normal", Vec3),
        ("impulse", ctypes.c_float),
        ("penetration", ctypes.c_float),
    ]


def _vec3(value: Sequence[float]) -> Vec3:
    return Vec3(float(value[0]), float(value[1]), float(value[2]))


def _quat_xyzw(value: Sequence[float]) -> Quat:
    return Quat(float(value[3]), float(value[0]), float(value[1]), float(value[2]))


def _xyzw(value: Quat) -> tuple[float, float, float, float]:
    return (float(value.x), float(value.y), float(value.z), float(value.w))


def _default_bridge_path() -> str:
    root = Path(__file__).resolve().parent.parent
    machine = platform.machine().lower()
    if sys.platform == "win32" and machine in {"amd64", "x86_64"}:
        candidate = root / "vendor" / "jolt_bridge" / "win_amd64" / "ka_jolt_bridge.dll"
    elif sys.platform.startswith("linux") and machine in {"amd64", "x86_64"}:
        candidate = root / "vendor" / "jolt_bridge" / "linux_x86_64" / "libka_jolt_bridge.so"
    else:
        return ""
    return str(candidate) if candidate.is_file() else ""


def resolve_bridge_path(configured: str = "") -> str:
    configured = os.path.abspath(os.path.expanduser(configured)) if configured else ""
    return configured if configured and os.path.isfile(configured) else _default_bridge_path()


def _configure(lib: ctypes.CDLL) -> None:
    lib.ka_physics_backend_name.restype = ctypes.c_char_p
    lib.ka_physics_backend_version.restype = ctypes.c_char_p
    lib.ka_physics_capabilities.restype = ctypes.c_uint64
    lib.ka_physics_last_error.restype = ctypes.c_char_p
    lib.ka_world_create.argtypes = [ctypes.POINTER(WorldDesc)]
    lib.ka_world_create.restype = ctypes.c_void_p
    lib.ka_world_destroy.argtypes = [ctypes.c_void_p]
    lib.ka_world_step.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_uint32]
    lib.ka_world_step.restype = ctypes.c_int
    lib.ka_world_body_count.argtypes = [ctypes.c_void_p]
    lib.ka_world_body_count.restype = ctypes.c_uint32
    lib.ka_body_create_primitive.argtypes = [ctypes.c_void_p, ctypes.POINTER(BodyDesc), ctypes.c_uint32, ctypes.POINTER(ctypes.c_float), ctypes.c_uint32]
    lib.ka_body_create_primitive.restype = ctypes.c_uint32
    lib.ka_body_create_convex.argtypes = [ctypes.c_void_p, ctypes.POINTER(BodyDesc), ctypes.POINTER(Vec3), ctypes.c_uint32]
    lib.ka_body_create_convex.restype = ctypes.c_uint32
    lib.ka_body_create_mesh.argtypes = [ctypes.c_void_p, ctypes.POINTER(BodyDesc), ctypes.POINTER(Vec3), ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint32]
    lib.ka_body_create_mesh.restype = ctypes.c_uint32
    lib.ka_body_create_compound_convex.argtypes = [ctypes.c_void_p, ctypes.POINTER(BodyDesc), ctypes.POINTER(CompoundChild), ctypes.c_uint32]
    lib.ka_body_create_compound_convex.restype = ctypes.c_uint32
    lib.ka_body_destroy.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    lib.ka_body_destroy.restype = ctypes.c_int
    lib.ka_body_get_transform.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.POINTER(Transform)]
    lib.ka_body_get_transform.restype = ctypes.c_int
    lib.ka_body_get_linear_velocity.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.POINTER(Vec3)]
    lib.ka_body_get_linear_velocity.restype = ctypes.c_int
    lib.ka_body_get_angular_velocity.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.POINTER(Vec3)]
    lib.ka_body_get_angular_velocity.restype = ctypes.c_int
    lib.ka_body_set_linear_velocity.argtypes = [ctypes.c_void_p, ctypes.c_uint32, Vec3]
    lib.ka_body_set_linear_velocity.restype = ctypes.c_int
    lib.ka_body_set_angular_velocity.argtypes = [ctypes.c_void_p, ctypes.c_uint32, Vec3]
    lib.ka_body_set_angular_velocity.restype = ctypes.c_int
    lib.ka_body_activate.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    lib.ka_body_activate.restype = ctypes.c_int
    lib.ka_body_deactivate.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    lib.ka_body_deactivate.restype = ctypes.c_int
    lib.ka_body_is_active.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    lib.ka_body_is_active.restype = ctypes.c_int
    lib.ka_world_drain_contact_events.argtypes = [ctypes.c_void_p, ctypes.POINTER(ContactEvent), ctypes.c_uint32]
    lib.ka_world_drain_contact_events.restype = ctypes.c_uint32


def _error(lib: ctypes.CDLL, fallback: str) -> RuntimeError:
    raw = lib.ka_physics_last_error()
    detail = raw.decode("utf-8", "replace") if raw else fallback
    return RuntimeError(detail or fallback)


class PhysicsWorld:
    def __init__(self, settings: Optional[dict[str, Any]] = None, *, bridge_path: str = "") -> None:
        settings = dict(settings or {})
        path = resolve_bridge_path(bridge_path or str(settings.pop("bridge_path", "")))
        if not path:
            raise NativeBridgeLoadError("No compiled Jolt ABI-v2 bridge was found.")
        self._lib = load_bridge(path)
        _configure(self._lib)
        desc = WorldDesc(
            _vec3(settings.get("gravity", (0.0, -9.81, 0.0))),
            max(128, int(settings.get("max_bodies", 65536))),
            max(1024, int(settings.get("max_pairs", 65536))),
            max(1024, int(settings.get("max_contact_constraints", 32768))),
            max(16 * 1024 * 1024, int(settings.get("temp_allocator_size", 64 * 1024 * 1024))),
            max(1, int(settings.get("num_threads", 1))),
            max(1.0e-6, float(settings.get("penetration_slop", 0.005))),
        )
        self._world = self._lib.ka_world_create(ctypes.byref(desc))
        if not self._world:
            raise _error(self._lib, "Native Jolt world creation failed")
        self._handles: list[int] = []
        self._index: dict[int, int] = {}

    def __del__(self) -> None:
        world = getattr(self, "_world", None)
        if world:
            try:
                self._lib.ka_world_destroy(world)
            finally:
                self._world = None

    def _desc(self, *, pos=(0.0, 0.0, 0.0), rot=(0.0, 0.0, 0.0, 1.0), motion=MOTION_DYNAMIC,
              mass=-1.0, user_data=0, category=0xFFFF, mask=0xFFFF, friction=0.2,
              restitution=0.0, ccd=False, linear_damping=0.0, angular_damping=0.0, **_ignored) -> BodyDesc:
        return BodyDesc(
            Transform(_vec3(pos), _quat_xyzw(rot)),
            int(motion), float(mass), int(user_data), int(category), int(mask),
            float(friction), float(restitution), float(linear_damping), float(angular_damping), int(bool(ccd)),
        )

    def _register(self, handle: int) -> int:
        if int(handle) == _INVALID_BODY:
            raise _error(self._lib, "Native Jolt body creation failed")
        handle = int(handle)
        self._index[handle] = len(self._handles)
        self._handles.append(handle)
        return handle

    def create_body(self, *, shape=SHAPE_BOX, size=0.5, **kwargs) -> int:
        desc = self._desc(**kwargs)
        if isinstance(size, (int, float)):
            values = [float(size)]
        else:
            values = [float(v) for v in size]
        data = (ctypes.c_float * len(values))(*values)
        native_shape = 0 if shape == SHAPE_BOX else 1 if shape == SHAPE_SPHERE else 2 if shape == SHAPE_PLANE else -1
        if native_shape < 0:
            raise ValueError(f"Unsupported native primitive shape: {shape}")
        return self._register(self._lib.ka_body_create_primitive(self._world, ctypes.byref(desc), native_shape, data, len(values)))

    @staticmethod
    def _float_points(buffer: Any) -> tuple[Any, int]:
        values = array.array("f")
        values.frombytes(bytes(buffer))
        if len(values) % 3:
            raise ValueError("Point buffer length must be divisible by three")
        count = len(values) // 3
        data = (Vec3 * count)(*(Vec3(values[i], values[i + 1], values[i + 2]) for i in range(0, len(values), 3)))
        return data, count

    def create_convex_hull(self, *, points, **kwargs) -> int:
        desc = self._desc(**kwargs)
        data, count = self._float_points(points)
        return self._register(self._lib.ka_body_create_convex(self._world, ctypes.byref(desc), data, count))

    def create_mesh_body(self, *, vertices, indices, **kwargs) -> int:
        desc = self._desc(**kwargs)
        vertex_data, vertex_count = self._float_points(vertices)
        index_values = array.array("I")
        index_values.frombytes(bytes(indices))
        index_data = (ctypes.c_uint32 * len(index_values))(*index_values)
        return self._register(self._lib.ka_body_create_mesh(
            self._world, ctypes.byref(desc), vertex_data, vertex_count, index_data, len(index_values)
        ))

    def create_compound_convex(self, *, parts: Sequence[dict[str, Any]], **kwargs) -> int:
        desc = self._desc(**kwargs)
        keepalive = []
        children = (CompoundChild * len(parts))()
        for index, part in enumerate(parts):
            points, count = self._float_points(part["points"])
            keepalive.append(points)
            children[index] = CompoundChild(
                Transform(_vec3(part.get("pos", (0.0, 0.0, 0.0))), _quat_xyzw(part.get("rot", (0.0, 0.0, 0.0, 1.0)))),
                points, count, int(part.get("user_data", index)),
            )
        return self._register(self._lib.ka_body_create_compound_convex(
            self._world, ctypes.byref(desc), children, len(parts)
        ))

    def create_compound_body(self, *, parts, **kwargs) -> int:
        convex_parts = []
        for index, (pos, rot, shape, size) in enumerate(parts):
            if shape != SHAPE_BOX:
                raise ValueError("ABI-v2 primitive compounds currently support box children only")
            hx, hy, hz = map(float, size)
            values = array.array("f")
            for x in (-hx, hx):
                for y in (-hy, hy):
                    for z in (-hz, hz):
                        values.extend((x, y, z))
            convex_parts.append({"pos": pos, "rot": rot, "points": values.tobytes(), "user_data": index})
        return self.create_compound_convex(parts=convex_parts, **kwargs)

    def create_constraint(self, *_args, **_kwargs) -> int:
        raise RuntimeError("The native bridge uses one true compound body; internal fixed constraints are not created")

    def destroy_body(self, handle: int) -> None:
        if not self._lib.ka_body_destroy(self._world, int(handle)):
            raise _error(self._lib, "Native Jolt body destruction failed")

    def destroy_bodies_batch(self, handles: Iterable[int]) -> None:
        for handle in handles:
            self.destroy_body(int(handle))

    def step(self, dt: float = 0.0) -> None:
        # Culverin accepts step(0) as a buffer flush. Native Jolt bodies are
        # immediately queryable, so avoid sending a zero time step to Update.
        if float(dt) <= 0.0:
            return
        if not self._lib.ka_world_step(self._world, float(dt), 1):
            raise _error(self._lib, "Native Jolt step failed")

    @property
    def count(self) -> int:
        return int(self._lib.ka_world_body_count(self._world))

    @property
    def shape_count(self) -> int:
        return self.count

    def get_index(self, handle: int) -> int:
        return self._index.get(int(handle), -1)

    def get_position(self, handle: int):
        value = Transform()
        return (value.position.x, value.position.y, value.position.z) if self._lib.ka_body_get_transform(self._world, int(handle), ctypes.byref(value)) else None

    def get_rotation(self, handle: int):
        value = Transform()
        return _xyzw(value.rotation) if self._lib.ka_body_get_transform(self._world, int(handle), ctypes.byref(value)) else None

    def _velocity(self, handle: int, angular: bool):
        value = Vec3()
        function = self._lib.ka_body_get_angular_velocity if angular else self._lib.ka_body_get_linear_velocity
        return (value.x, value.y, value.z) if function(self._world, int(handle), ctypes.byref(value)) else None

    def get_velocity(self, handle: int): return self._velocity(handle, False)
    def get_angular_velocity(self, handle: int): return self._velocity(handle, True)

    def set_linear_velocity(self, handle: int, x: float, y: float, z: float) -> None:
        if not self._lib.ka_body_set_linear_velocity(self._world, int(handle), Vec3(x, y, z)):
            raise _error(self._lib, "Setting linear velocity failed")

    def set_angular_velocity(self, handle: int, x: float, y: float, z: float) -> None:
        if not self._lib.ka_body_set_angular_velocity(self._world, int(handle), Vec3(x, y, z)):
            raise _error(self._lib, "Setting angular velocity failed")

    def activate(self, handle: int) -> None: self._lib.ka_body_activate(self._world, int(handle))
    def deactivate(self, handle: int) -> None: self._lib.ka_body_deactivate(self._world, int(handle))
    def is_active(self, handle: int) -> bool: return bool(self._lib.ka_body_is_active(self._world, int(handle)))

    def get_active_indices(self) -> bytes:
        values = array.array("I", (self._index[h] for h in self._handles if self.is_active(h)))
        return values.tobytes()

    def get_contact_events_raw(self):
        return None

    def get_contact_events_ex(self):
        count = int(self._lib.ka_world_drain_contact_events(self._world, None, 0))
        if count <= 0:
            return []
        records = (ContactEvent * count)()
        count = int(self._lib.ka_world_drain_contact_events(self._world, records, count))
        return [
            {
                "bodies": (int(record.body1), int(record.body2)),
                "position": (float(record.point.x), float(record.point.y), float(record.point.z)),
                "normal": (float(record.normal.x), float(record.normal.y), float(record.normal.z)),
                "impulse": float(record.impulse),
                "slide_speed": 0.0,
                "type": int(record.event_type),
                "penetration": float(record.penetration),
            }
            for record in records[:count]
        ]


def load_native_jolt(path: str = ""):
    resolved = resolve_bridge_path(path)
    if not resolved:
        raise NativeBridgeLoadError("No compiled Jolt ABI-v2 bridge was found.")
    library = load_bridge(resolved)
    _configure(library)
    name = (library.ka_physics_backend_name() or b"Jolt bridge").decode("utf-8", "replace")
    version = (library.ka_physics_backend_version() or b"unknown").decode("utf-8", "replace")

    class BoundPhysicsWorld(PhysicsWorld):
        def __init__(self, settings=None, bodies=None):
            super().__init__(settings, bridge_path=resolved)
            if bodies:
                raise NotImplementedError("Bulk constructor bodies are not used by KA Rigid Dynamics")

    return SimpleNamespace(
        PhysicsWorld=BoundPhysicsWorld,
        MOTION_STATIC=MOTION_STATIC,
        MOTION_KINEMATIC=MOTION_KINEMATIC,
        MOTION_DYNAMIC=MOTION_DYNAMIC,
        SHAPE_BOX=SHAPE_BOX,
        SHAPE_SPHERE=SHAPE_SPHERE,
        SHAPE_PLANE=SHAPE_PLANE,
        CONSTRAINT_FIXED=CONSTRAINT_FIXED,
        EVENT_ADDED=EVENT_ADDED,
        EVENT_PERSISTED=EVENT_PERSISTED,
        EVENT_REMOVED=EVENT_REMOVED,
        USE_DOUBLE_PRECISION=USE_DOUBLE_PRECISION,
        NATIVE_COMPOUND_CONVEX=True,
        NATIVE_BRIDGE=True,
        __version__=f"{name} {version}",
    )
