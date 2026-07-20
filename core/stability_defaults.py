"""Shared scale-aware rigid-body stability defaults."""

from __future__ import annotations

import math

LOW_FRICTION_CONTACT_DEFAULT = 0.20
LEGACY_BODY_FRICTION_DEFAULT = 0.50
PENETRATION_SLOP_DEFAULT = 0.001
LEGACY_PENETRATION_SLOP_DEFAULT = 0.005


def matches_legacy_default(value: float, legacy_default: float) -> bool:
    """Return True only for untouched legacy values, preserving user edits."""
    return math.isclose(
        float(value),
        float(legacy_default),
        rel_tol=0.0,
        abs_tol=max(1.0e-9, abs(float(legacy_default)) * 1.0e-6),
    )
