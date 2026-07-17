"""Small deterministic reference rigid-body solver.

This module deliberately has no Blender dependency. It provides an end-to-end
backend for validating scene export, caching and playback while the native Jolt
and PhysX backends are developed. Collision geometry is intentionally limited:
all dynamic bodies use bounding spheres; static colliders are planes, boxes or
bounding spheres.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Dict, Iterable, List, Sequence, Tuple

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]  # w, x, y, z

_EPS = 1.0e-9


def _v_add(a: Sequence[float], b: Sequence[float]) -> List[float]:
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def _v_sub(a: Sequence[float], b: Sequence[float]) -> List[float]:
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def _v_mul(a: Sequence[float], s: float) -> List[float]:
    return [a[0] * s, a[1] * s, a[2] * s]


def _v_dot(a: Sequence[float], b: Sequence[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _v_cross(a: Sequence[float], b: Sequence[float]) -> List[float]:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _v_len_sq(a: Sequence[float]) -> float:
    return _v_dot(a, a)


def _v_len(a: Sequence[float]) -> float:
    return math.sqrt(_v_len_sq(a))


def _v_normalized(a: Sequence[float], fallback: Sequence[float] = (0.0, 0.0, 1.0)) -> List[float]:
    length = _v_len(a)
    if length <= _EPS:
        return [float(fallback[0]), float(fallback[1]), float(fallback[2])]
    return [a[0] / length, a[1] / length, a[2] / length]


def _clamp(value: float, low: float, high: float) -> float:
    return low if value < low else high if value > high else value


def _quat_normalized(q: Sequence[float]) -> List[float]:
    length = math.sqrt(sum(component * component for component in q))
    if length <= _EPS:
        return [1.0, 0.0, 0.0, 0.0]
    return [component / length for component in q]


def _quat_mul(a: Sequence[float], b: Sequence[float]) -> List[float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]


def _quat_rotate(q: Sequence[float], vector: Sequence[float]) -> List[float]:
    qn = _quat_normalized(q)
    pure = [0.0, vector[0], vector[1], vector[2]]
    conjugate = [qn[0], -qn[1], -qn[2], -qn[3]]
    rotated = _quat_mul(_quat_mul(qn, pure), conjugate)
    return rotated[1:4]


def _integrate_quaternion(q: Sequence[float], angular_velocity: Sequence[float], dt: float) -> List[float]:
    speed = _v_len(angular_velocity)
    if speed <= _EPS:
        return list(q)
    half_angle = 0.5 * speed * dt
    axis = _v_mul(angular_velocity, 1.0 / speed)
    delta = [
        math.cos(half_angle),
        axis[0] * math.sin(half_angle),
        axis[1] * math.sin(half_angle),
        axis[2] * math.sin(half_angle),
    ]
    return _quat_normalized(_quat_mul(delta, q))


@dataclass
class BodyState:
    name: str
    body_type: str
    shape: str
    location: List[float]
    rotation: List[float]
    scale: List[float]
    half_extents: List[float]
    radius: float
    mass: float
    friction: float
    restitution: float
    linear_damping: float
    angular_damping: float
    linear_velocity: List[float]
    angular_velocity: List[float]
    ccd: bool = False
    sleeping: bool = False
    sleep_timer: float = 0.0
    collision_layer: int = 1
    collision_mask: int = 0xFFFFFFFF
    inv_mass: float = field(init=False)

    def __post_init__(self) -> None:
        self.inv_mass = 0.0 if self.body_type != "DYNAMIC" or self.mass <= _EPS else 1.0 / self.mass
        self.radius = max(float(self.radius), 1.0e-5)
        self.rotation = _quat_normalized(self.rotation)

    def can_collide(self, other: "BodyState") -> bool:
        return bool((self.collision_mask & other.collision_layer) and (other.collision_mask & self.collision_layer))


class ReferenceSolver:
    """A compact position/impulse solver intended for pipeline validation."""

    def __init__(
        self,
        bodies: Iterable[Dict],
        gravity: Sequence[float] = (0.0, 0.0, -9.81),
        solver_iterations: int = 8,
        sleep_enabled: bool = True,
        sleep_linear_threshold: float = 0.05,
        sleep_angular_threshold: float = 0.1,
        sleep_time: float = 0.5,
    ) -> None:
        self.gravity = [float(v) for v in gravity]
        self.solver_iterations = max(1, int(solver_iterations))
        self.sleep_enabled = bool(sleep_enabled)
        self.sleep_linear_threshold = max(0.0, float(sleep_linear_threshold))
        self.sleep_angular_threshold = max(0.0, float(sleep_angular_threshold))
        self.sleep_time = max(0.0, float(sleep_time))
        self.bodies: List[BodyState] = [self._body_from_dict(data) for data in bodies]
        self.dynamic_bodies = [body for body in self.bodies if body.body_type == "DYNAMIC"]
        self.static_bodies = [body for body in self.bodies if body.body_type != "DYNAMIC"]
        self.last_step_stats: Dict[str, float] = {}

    @staticmethod
    def _body_from_dict(data: Dict) -> BodyState:
        return BodyState(
            name=str(data["name"]),
            body_type=str(data.get("body_type", "DYNAMIC")),
            shape=str(data.get("collision_shape", "SPHERE")),
            location=[float(v) for v in data.get("location", (0.0, 0.0, 0.0))],
            rotation=[float(v) for v in data.get("rotation", (1.0, 0.0, 0.0, 0.0))],
            scale=[float(v) for v in data.get("scale", (1.0, 1.0, 1.0))],
            half_extents=[max(1.0e-5, float(v)) for v in data.get("half_extents", (0.5, 0.5, 0.5))],
            radius=max(1.0e-5, float(data.get("radius", 0.5))),
            mass=max(0.0, float(data.get("mass", 1.0))),
            friction=max(0.0, float(data.get("friction", 0.5))),
            restitution=max(0.0, float(data.get("restitution", 0.0))),
            linear_damping=max(0.0, float(data.get("linear_damping", 0.04))),
            angular_damping=max(0.0, float(data.get("angular_damping", 0.1))),
            linear_velocity=[float(v) for v in data.get("linear_velocity", (0.0, 0.0, 0.0))],
            angular_velocity=[float(v) for v in data.get("angular_velocity", (0.0, 0.0, 0.0))],
            ccd=bool(data.get("ccd", False)),
            collision_layer=int(data.get("collision_layer", 1)),
            collision_mask=int(data.get("collision_mask", 0xFFFFFFFF)),
        )

    def snapshot(self) -> Dict[str, Dict[str, List[float]]]:
        return {
            body.name: {
                "location": list(body.location),
                "rotation": list(body.rotation),
                "scale": list(body.scale),
                "linear_velocity": list(body.linear_velocity),
                "angular_velocity": list(body.angular_velocity),
                "sleeping": body.sleeping,
            }
            for body in self.bodies
        }

    def suggested_substeps(self, frame_dt: float, configured_substeps: int, maximum: int = 64) -> int:
        configured = max(1, int(configured_substeps))
        required = configured
        for body in self.dynamic_bodies:
            if not body.ccd or body.sleeping:
                continue
            travel = _v_len(body.linear_velocity) * frame_dt
            allowed = max(body.radius * 0.4, 1.0e-4)
            required = max(required, int(math.ceil(travel / allowed)))
        return min(maximum, required)

    def diagnostic_state(self) -> Dict[str, float]:
        active = [body for body in self.dynamic_bodies if not body.sleeping]
        return {
            "dynamic_bodies": len(self.dynamic_bodies),
            "static_bodies": len(self.static_bodies),
            "active_bodies": len(active),
            "sleeping_bodies": len(self.dynamic_bodies) - len(active),
            "max_linear_speed": max((_v_len(body.linear_velocity) for body in self.dynamic_bodies), default=0.0),
            "max_angular_speed": max((_v_len(body.angular_velocity) for body in self.dynamic_bodies), default=0.0),
        }

    def step(self, dt: float) -> Dict[str, float]:
        self.last_step_stats = {
            "broadphase_pairs": 0,
            "dynamic_static_contact_solves": 0,
            "dynamic_dynamic_contact_solves": 0,
            "velocity_impulses": 0,
            "max_penetration": 0.0,
        }
        if dt <= 0.0:
            self.last_step_stats.update(self.diagnostic_state())
            return dict(self.last_step_stats)

        for body in self.dynamic_bodies:
            if body.sleeping:
                continue
            body.linear_velocity = _v_add(body.linear_velocity, _v_mul(self.gravity, dt))
            body.linear_velocity = _v_mul(body.linear_velocity, math.exp(-body.linear_damping * dt))
            body.angular_velocity = _v_mul(body.angular_velocity, math.exp(-body.angular_damping * dt))
            body.location = _v_add(body.location, _v_mul(body.linear_velocity, dt))
            body.rotation = _integrate_quaternion(body.rotation, body.angular_velocity, dt)

        for iteration in range(self.solver_iterations):
            apply_velocity = iteration == 0
            for body in self.dynamic_bodies:
                if body.sleeping:
                    continue
                for static in self.static_bodies:
                    if body.can_collide(static):
                        self._solve_dynamic_static(body, static, apply_velocity)

            pairs = self._dynamic_pairs()
            self.last_step_stats["broadphase_pairs"] += len(pairs)
            for first, second in pairs:
                if first.can_collide(second):
                    self._solve_dynamic_dynamic(first, second, apply_velocity)

        self._update_sleeping(dt)
        self.last_step_stats.update(self.diagnostic_state())
        return dict(self.last_step_stats)

    def _record_contact(self, kind: str, penetration: float, apply_velocity: bool) -> None:
        key = "dynamic_static_contact_solves" if kind == "STATIC" else "dynamic_dynamic_contact_solves"
        self.last_step_stats[key] = self.last_step_stats.get(key, 0) + 1
        self.last_step_stats["max_penetration"] = max(
            float(self.last_step_stats.get("max_penetration", 0.0)),
            float(penetration),
        )

    def _dynamic_pairs(self):
        active = [body for body in self.dynamic_bodies if not body.sleeping]
        if len(active) < 2:
            return []

        cell_size = max(1.0e-4, 2.0 * max(body.radius for body in active))
        grid = {}
        for index, body in enumerate(active):
            cell = tuple(math.floor(component / cell_size) for component in body.location)
            grid.setdefault(cell, []).append(index)

        pairs = []
        seen = set()
        neighbor_offsets = (
            (dx, dy, dz)
            for dx in (-1, 0, 1)
            for dy in (-1, 0, 1)
            for dz in (-1, 0, 1)
        )
        offsets = tuple(neighbor_offsets)
        for cell, indices in grid.items():
            for dx, dy, dz in offsets:
                neighbor = (cell[0] + dx, cell[1] + dy, cell[2] + dz)
                for first_index in indices:
                    for second_index in grid.get(neighbor, ()):
                        if first_index >= second_index:
                            continue
                        key = (first_index, second_index)
                        if key in seen:
                            continue
                        seen.add(key)
                        pairs.append((active[first_index], active[second_index]))
        return pairs

    def _solve_dynamic_static(self, body: BodyState, static: BodyState, apply_velocity: bool) -> None:
        if static.shape == "PLANE":
            normal = _quat_rotate(static.rotation, (0.0, 0.0, 1.0))
            normal = _v_normalized(normal)
            signed_distance = _v_dot(_v_sub(body.location, static.location), normal)
            penetration = body.radius - signed_distance
            if penetration > 0.0:
                self._record_contact("STATIC", penetration, apply_velocity)
                body.location = _v_add(body.location, _v_mul(normal, penetration))
                if apply_velocity:
                    self._resolve_velocity_against_static(body, static, normal)
            return

        if static.shape == "BOX" or static.shape in {"MESH", "CONVEX_HULL"}:
            normal, penetration = self._sphere_aabb_contact(body.location, body.radius, static.location, static.half_extents)
            if penetration > 0.0:
                self._record_contact("STATIC", penetration, apply_velocity)
                body.location = _v_add(body.location, _v_mul(normal, penetration))
                if apply_velocity:
                    self._resolve_velocity_against_static(body, static, normal)
            return

        delta = _v_sub(body.location, static.location)
        distance = _v_len(delta)
        penetration = body.radius + static.radius - distance
        if penetration > 0.0:
            self._record_contact("STATIC", penetration, apply_velocity)
            normal = _v_normalized(delta)
            body.location = _v_add(body.location, _v_mul(normal, penetration))
            if apply_velocity:
                self._resolve_velocity_against_static(body, static, normal)

    @staticmethod
    def _sphere_aabb_contact(
        sphere_location: Sequence[float],
        radius: float,
        box_location: Sequence[float],
        half_extents: Sequence[float],
    ) -> Tuple[List[float], float]:
        local = _v_sub(sphere_location, box_location)
        closest_local = [
            _clamp(local[axis], -half_extents[axis], half_extents[axis])
            for axis in range(3)
        ]
        delta = _v_sub(local, closest_local)
        distance_sq = _v_len_sq(delta)
        if distance_sq > _EPS:
            distance = math.sqrt(distance_sq)
            return _v_mul(delta, 1.0 / distance), max(0.0, radius - distance)

        distances_to_face = [half_extents[axis] - abs(local[axis]) for axis in range(3)]
        axis = min(range(3), key=distances_to_face.__getitem__)
        sign = 1.0 if local[axis] >= 0.0 else -1.0
        normal = [0.0, 0.0, 0.0]
        normal[axis] = sign
        return normal, radius + max(0.0, distances_to_face[axis])

    @staticmethod
    def _combined_friction(first: BodyState, second: BodyState) -> float:
        return math.sqrt(max(0.0, first.friction * second.friction))

    @staticmethod
    def _combined_restitution(first: BodyState, second: BodyState) -> float:
        return max(first.restitution, second.restitution)

    def _resolve_velocity_against_static(self, body: BodyState, static: BodyState, normal: Sequence[float]) -> None:
        normal_speed = _v_dot(body.linear_velocity, normal)
        if normal_speed >= 0.0:
            return
        self.last_step_stats["velocity_impulses"] = self.last_step_stats.get("velocity_impulses", 0) + 1
        restitution = self._combined_restitution(body, static)
        body.linear_velocity = _v_sub(body.linear_velocity, _v_mul(normal, (1.0 + restitution) * normal_speed))
        self._apply_static_friction(body, static, normal)
        body.sleeping = False
        body.sleep_timer = 0.0

    def _apply_static_friction(self, body: BodyState, static: BodyState, normal: Sequence[float]) -> None:
        normal_component = _v_mul(normal, _v_dot(body.linear_velocity, normal))
        tangent = _v_sub(body.linear_velocity, normal_component)
        tangent_speed = _v_len(tangent)
        if tangent_speed <= _EPS:
            return
        friction = self._combined_friction(body, static)
        reduction = min(tangent_speed, friction * (1.0 + abs(_v_dot(self.gravity, normal))) * 0.01)
        body.linear_velocity = _v_sub(body.linear_velocity, _v_mul(_v_normalized(tangent), reduction))

    def _solve_dynamic_dynamic(self, first: BodyState, second: BodyState, apply_velocity: bool) -> None:
        delta = _v_sub(second.location, first.location)
        distance = _v_len(delta)
        penetration = first.radius + second.radius - distance
        if penetration <= 0.0:
            return

        self._record_contact("DYNAMIC", penetration, apply_velocity)
        normal = _v_normalized(delta, fallback=(1.0, 0.0, 0.0))
        inverse_mass_sum = first.inv_mass + second.inv_mass
        if inverse_mass_sum <= _EPS:
            return

        first.location = _v_sub(first.location, _v_mul(normal, penetration * first.inv_mass / inverse_mass_sum))
        second.location = _v_add(second.location, _v_mul(normal, penetration * second.inv_mass / inverse_mass_sum))

        if not apply_velocity:
            return

        relative_velocity = _v_sub(second.linear_velocity, first.linear_velocity)
        normal_speed = _v_dot(relative_velocity, normal)
        if normal_speed >= 0.0:
            return

        self.last_step_stats["velocity_impulses"] = self.last_step_stats.get("velocity_impulses", 0) + 1
        restitution = self._combined_restitution(first, second)
        normal_impulse_magnitude = -(1.0 + restitution) * normal_speed / inverse_mass_sum
        normal_impulse = _v_mul(normal, normal_impulse_magnitude)
        first.linear_velocity = _v_sub(first.linear_velocity, _v_mul(normal_impulse, first.inv_mass))
        second.linear_velocity = _v_add(second.linear_velocity, _v_mul(normal_impulse, second.inv_mass))

        relative_velocity = _v_sub(second.linear_velocity, first.linear_velocity)
        tangent = _v_sub(relative_velocity, _v_mul(normal, _v_dot(relative_velocity, normal)))
        tangent_speed = _v_len(tangent)
        if tangent_speed > _EPS:
            tangent_direction = _v_mul(tangent, 1.0 / tangent_speed)
            tangent_impulse_magnitude = -tangent_speed / inverse_mass_sum
            max_friction = self._combined_friction(first, second) * normal_impulse_magnitude
            tangent_impulse_magnitude = _clamp(tangent_impulse_magnitude, -max_friction, max_friction)
            tangent_impulse = _v_mul(tangent_direction, tangent_impulse_magnitude)
            first.linear_velocity = _v_sub(first.linear_velocity, _v_mul(tangent_impulse, first.inv_mass))
            second.linear_velocity = _v_add(second.linear_velocity, _v_mul(tangent_impulse, second.inv_mass))

        first.sleeping = False
        second.sleeping = False
        first.sleep_timer = 0.0
        second.sleep_timer = 0.0

    def _update_sleeping(self, dt: float) -> None:
        if not self.sleep_enabled:
            for body in self.dynamic_bodies:
                body.sleeping = False
                body.sleep_timer = 0.0
            return

        for body in self.dynamic_bodies:
            if (
                _v_len(body.linear_velocity) <= self.sleep_linear_threshold
                and _v_len(body.angular_velocity) <= self.sleep_angular_threshold
            ):
                body.sleep_timer += dt
                if body.sleep_timer >= self.sleep_time:
                    body.sleeping = True
                    body.linear_velocity = [0.0, 0.0, 0.0]
                    body.angular_velocity = [0.0, 0.0, 0.0]
            else:
                body.sleep_timer = 0.0
                body.sleeping = False
