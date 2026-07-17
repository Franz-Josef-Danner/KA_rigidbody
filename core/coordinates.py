"""Coordinate conversion between Blender (right-handed Z-up) and Jolt (right-handed Y-up)."""

from __future__ import annotations

import math
from typing import Iterable, Sequence, Tuple

Vec3 = Tuple[float, float, float]
QuatWXYZ = Tuple[float, float, float, float]
QuatXYZW = Tuple[float, float, float, float]

_SQRT_HALF = math.sqrt(0.5)
# -90 degrees around X: Blender (x, y, z) -> Jolt (x, z, -y)
_BASIS: QuatWXYZ = (_SQRT_HALF, -_SQRT_HALF, 0.0, 0.0)
_BASIS_INV: QuatWXYZ = (_SQRT_HALF, _SQRT_HALF, 0.0, 0.0)


def blender_vec_to_jolt(value: Sequence[float]) -> Vec3:
    return (float(value[0]), float(value[2]), -float(value[1]))


def jolt_vec_to_blender(value: Sequence[float]) -> Vec3:
    return (float(value[0]), -float(value[2]), float(value[1]))


def quat_multiply_wxyz(a: Sequence[float], b: Sequence[float]) -> QuatWXYZ:
    aw, ax, ay, az = map(float, a)
    bw, bx, by, bz = map(float, b)
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_conjugate_wxyz(q: Sequence[float]) -> QuatWXYZ:
    w, x, y, z = map(float, q)
    return (w, -x, -y, -z)


def quat_normalize_wxyz(q: Sequence[float]) -> QuatWXYZ:
    w, x, y, z = map(float, q)
    length = math.sqrt(w * w + x * x + y * y + z * z)
    if length <= 1.0e-20:
        return (1.0, 0.0, 0.0, 0.0)
    inv = 1.0 / length
    return (w * inv, x * inv, y * inv, z * inv)


def quat_rotate_vector_wxyz(q: Sequence[float], value: Sequence[float]) -> Vec3:
    qn = quat_normalize_wxyz(q)
    pure = (0.0, float(value[0]), float(value[1]), float(value[2]))
    rotated = quat_multiply_wxyz(quat_multiply_wxyz(qn, pure), quat_conjugate_wxyz(qn))
    return (rotated[1], rotated[2], rotated[3])


def blender_quat_to_jolt(q_wxyz: Sequence[float]) -> QuatXYZW:
    q = quat_normalize_wxyz(q_wxyz)
    converted = quat_multiply_wxyz(quat_multiply_wxyz(_BASIS, q), _BASIS_INV)
    converted = quat_normalize_wxyz(converted)
    return (converted[1], converted[2], converted[3], converted[0])


def jolt_quat_to_blender(q_xyzw: Sequence[float]) -> QuatWXYZ:
    q = quat_normalize_wxyz((float(q_xyzw[3]), float(q_xyzw[0]), float(q_xyzw[1]), float(q_xyzw[2])))
    converted = quat_multiply_wxyz(quat_multiply_wxyz(_BASIS_INV, q), _BASIS)
    return quat_normalize_wxyz(converted)


def add_vec3(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return (float(a[0]) + float(b[0]), float(a[1]) + float(b[1]), float(a[2]) + float(b[2]))


def subtract_vec3(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return (float(a[0]) - float(b[0]), float(a[1]) - float(b[1]), float(a[2]) - float(b[2]))


def scale_vec3(value: Sequence[float], factor: float) -> Vec3:
    return (float(value[0]) * factor, float(value[1]) * factor, float(value[2]) * factor)


def length_vec3(value: Sequence[float]) -> float:
    return math.sqrt(sum(float(component) * float(component) for component in value[:3]))
