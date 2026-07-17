"""Deterministic hashing and comparison helpers for physics caches."""

from __future__ import annotations

import hashlib
import json
import math
from typing import Any, Dict, Iterable, Tuple


def _quantize(value: Any, precision: int) -> Any:
    if isinstance(value, float):
        if not math.isfinite(value):
            return str(value)
        return round(value, precision)
    if isinstance(value, dict):
        return {str(key): _quantize(value[key], precision) for key in sorted(value, key=str)}
    if isinstance(value, (list, tuple)):
        return [_quantize(item, precision) for item in value]
    return value


def frames_digest(frames: Dict[str, Dict], precision: int = 9) -> str:
    """Return a stable digest independent of dictionary insertion order."""
    ordered = {
        str(frame): {
            str(name): _quantize(snapshot[name], precision)
            for name in sorted(snapshot)
        }
        for frame, snapshot in sorted(frames.items(), key=lambda item: int(item[0]))
    }
    encoded = json.dumps(ordered, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _iter_components(value: Any, path: str = "") -> Iterable[Tuple[str, float]]:
    if isinstance(value, dict):
        for key in sorted(value):
            child = f"{path}.{key}" if path else str(key)
            yield from _iter_components(value[key], child)
    elif isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            yield from _iter_components(item, f"{path}[{index}]")
    elif isinstance(value, (int, float)):
        yield path, float(value)


def compare_frames(reference: Dict[str, Dict], candidate: Dict[str, Dict], tolerance: float = 1.0e-6) -> Dict[str, Any]:
    """Compare two frame dictionaries and report the largest numeric deviation."""
    missing_frames = sorted(set(reference) ^ set(candidate), key=lambda item: int(item))
    max_error = 0.0
    max_error_path = None
    compared_values = 0
    structural_errors = []

    for frame in sorted(set(reference) & set(candidate), key=lambda item: int(item)):
        ref_objects = reference[frame]
        new_objects = candidate[frame]
        missing_objects = sorted(set(ref_objects) ^ set(new_objects))
        if missing_objects:
            structural_errors.append({"frame": int(frame), "objects": missing_objects[:20]})
        for name in sorted(set(ref_objects) & set(new_objects)):
            ref_values = dict(_iter_components(ref_objects[name], f"frame:{frame}/{name}"))
            new_values = dict(_iter_components(new_objects[name], f"frame:{frame}/{name}"))
            if set(ref_values) != set(new_values):
                structural_errors.append({"frame": int(frame), "object": name, "fields": sorted(set(ref_values) ^ set(new_values))[:20]})
                continue
            for path in sorted(ref_values):
                error = abs(ref_values[path] - new_values[path])
                compared_values += 1
                if error > max_error:
                    max_error = error
                    max_error_path = path

    match = not missing_frames and not structural_errors and max_error <= max(0.0, float(tolerance))
    return {
        "match": bool(match),
        "tolerance": float(tolerance),
        "max_error": float(max_error),
        "max_error_path": max_error_path,
        "compared_values": compared_values,
        "missing_frames": missing_frames[:20],
        "structural_errors": structural_errors[:20],
        "reference_digest": frames_digest(reference),
        "candidate_digest": frames_digest(candidate),
    }
