"""Fast binary transform cache storage for KA Rigid Dynamics."""

from __future__ import annotations

import array
import gzip
import hashlib
import json
import os
import struct
import sys
import tempfile
import zlib
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple

CACHE_VERSION = 3
CACHE_FILENAME = "ka_rigid_cache.karc"
LEGACY_CACHE_FILENAME = "ka_rigid_cache.json.gz"
_MAGIC = b"KARD045\0"
_LEGACY_MAGIC = b"KARD044\0"
_HEADER = struct.Struct("<8sII")
_FLOATS_PER_TRANSFORM = 7  # location xyz + rotation wxyz; scale is per-body metadata


def normalize_directory(path: str) -> str:
    path = os.path.abspath(os.path.expanduser(path))
    os.makedirs(path, exist_ok=True)
    return path


def cache_file_path(directory: str) -> str:
    return os.path.join(normalize_directory(directory), CACHE_FILENAME)


def legacy_cache_file_path(directory: str) -> str:
    return os.path.join(normalize_directory(directory), LEGACY_CACHE_FILENAME)


def _ordered_frame_numbers(frames: Mapping[str, Mapping]) -> List[str]:
    return sorted((str(key) for key in frames), key=lambda value: int(value))


def _body_names(frames: Mapping[str, Mapping], frame_numbers: Iterable[str]) -> List[str]:
    ordered: List[str] = []
    seen = set()
    for frame in frame_numbers:
        for name in frames.get(frame, {}):
            if name not in seen:
                seen.add(name)
                ordered.append(str(name))
    return ordered


def _array_bytes(values: array.array) -> bytes:
    if values.typecode != "f":
        values = array.array("f", values)
    elif sys.byteorder != "little":
        values = array.array("f", values)
    if sys.byteorder != "little":
        values.byteswap()
    return values.tobytes()


def _encode_frames(frames: Mapping[str, Mapping]) -> Tuple[List[str], List[str], Dict[str, List[float]], bytes]:
    frame_numbers = _ordered_frame_numbers(frames)
    names = _body_names(frames, frame_numbers)
    first = frames.get(frame_numbers[0], {}) if frame_numbers else {}
    scales = {name: list(first.get(name, {}).get("scale", (1.0, 1.0, 1.0))) for name in names}
    previous = {name: first.get(name, {}) for name in names}
    values = array.array("f")
    identity_rotation = (1.0, 0.0, 0.0, 0.0)
    for frame in frame_numbers:
        snapshot = frames.get(frame, {})
        for name in names:
            transform = snapshot.get(name) or previous.get(name) or {}
            location = transform.get("location", (0.0, 0.0, 0.0))
            rotation = transform.get("rotation", identity_rotation)
            values.extend(float(v) for v in (*location[:3], *rotation[:4]))
            previous[name] = transform
    return frame_numbers, names, scales, _array_bytes(values)


def _direct_block(block: Mapping[str, Any]) -> Tuple[List[str], List[str], Dict[str, List[float]], bytes]:
    frame_numbers = [str(value) for value in block.get("frame_numbers", [])]
    names = [str(value) for value in block.get("body_names", [])]
    scales = {
        str(name): [float(value) for value in values[:3]]
        for name, values in dict(block.get("body_scales", {})).items()
    }
    values = block.get("values")
    raw = block.get("raw")
    if isinstance(values, array.array):
        raw_bytes = _array_bytes(values)
    elif isinstance(raw, (bytes, bytearray, memoryview)):
        raw_bytes = bytes(raw)
    else:
        raise ValueError("Invalid direct KA Rigid cache frame block")
    expected = len(frame_numbers) * len(names) * _FLOATS_PER_TRANSFORM * 4
    if len(raw_bytes) != expected:
        raise ValueError(
            f"Invalid direct KA Rigid cache frame block: expected {expected} bytes, got {len(raw_bytes)}"
        )
    return frame_numbers, names, scales, raw_bytes


def _decode_frames(raw: bytes, frame_numbers: Sequence[str], names: Sequence[str], scales: Mapping[str, Sequence[float]]) -> Dict[str, Dict]:
    values = array.array("f")
    values.frombytes(raw)
    if sys.byteorder != "little":
        values.byteswap()
    expected = len(frame_numbers) * len(names) * _FLOATS_PER_TRANSFORM
    if len(values) != expected:
        raise ValueError(f"Corrupt KA Rigid cache transform block: expected {expected} floats, got {len(values)}")
    frames: Dict[str, Dict] = {}
    cursor = 0
    for frame in frame_numbers:
        snapshot: Dict[str, Dict] = {}
        for name in names:
            snapshot[str(name)] = {
                "location": [float(v) for v in values[cursor:cursor + 3]],
                "rotation": [float(v) for v in values[cursor + 3:cursor + 7]],
                "scale": list(scales.get(str(name), (1.0, 1.0, 1.0))),
            }
            cursor += _FLOATS_PER_TRANSFORM
        frames[str(frame)] = snapshot
    return frames


def decode_direct_frame_block(block: Mapping[str, Any]) -> Dict[str, Dict]:
    """Decode a backend-direct frame block only when a caller actually needs dictionaries."""
    frame_numbers, names, scales, raw = _direct_block(block)
    return _decode_frames(raw, frame_numbers, names, scales)


def direct_frame_block_digest(block: Mapping[str, Any]) -> str:
    """Hash the compact frame stream without constructing per-frame Python dictionaries."""
    frame_numbers, names, scales, raw = _direct_block(block)
    digest = hashlib.sha256()
    digest.update(json.dumps(frame_numbers, separators=(",", ":"), ensure_ascii=True).encode("utf-8"))
    digest.update(json.dumps(names, separators=(",", ":"), ensure_ascii=True).encode("utf-8"))
    digest.update(json.dumps(scales, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8"))
    digest.update(raw)
    return digest.hexdigest()


def write_cache(directory: str, payload: Dict[str, Any]) -> str:
    directory = normalize_directory(directory)
    final_path = cache_file_path(directory)
    data = dict(payload)
    frames = data.pop("frames", {})
    data.pop("_first_snapshot", None)
    direct = data.pop("_binary_frame_block", None)
    if isinstance(direct, Mapping):
        frame_numbers, names, scales, raw = _direct_block(direct)
        source = "backend-direct"
    else:
        frame_numbers, names, scales, raw = _encode_frames(frames)
        source = "dictionary-encoded"
    data["cache_version"] = CACHE_VERSION
    data["binary_frames"] = {
        "format": "float32-location3-rotation4",
        "frame_numbers": frame_numbers,
        "body_names": names,
        "body_scales": scales,
        "compression": "zlib-1",
        "source": source,
    }
    metadata = json.dumps(data, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    compressed = zlib.compress(raw, level=1)
    handle, temporary_path = tempfile.mkstemp(prefix="ka_rigid_", suffix=".tmp", dir=directory)
    os.close(handle)
    try:
        with open(temporary_path, "wb") as stream:
            stream.write(_HEADER.pack(_MAGIC, len(metadata), len(compressed)))
            stream.write(metadata)
            stream.write(compressed)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, final_path)
        legacy = legacy_cache_file_path(directory)
        if os.path.isfile(legacy):
            os.remove(legacy)
    finally:
        if os.path.exists(temporary_path):
            os.remove(temporary_path)
    return final_path


def _read_binary(path: str) -> Dict[str, Any]:
    with open(path, "rb") as stream:
        header = stream.read(_HEADER.size)
        if len(header) != _HEADER.size:
            raise ValueError("Truncated KA Rigid cache header")
        magic, metadata_size, compressed_size = _HEADER.unpack(header)
        if magic not in {_MAGIC, _LEGACY_MAGIC}:
            raise ValueError("Unsupported KA Rigid cache signature")
        metadata = json.loads(stream.read(metadata_size).decode("utf-8"))
        compressed = stream.read(compressed_size)
    version = int(metadata.get("cache_version", -1))
    if version not in {2, CACHE_VERSION}:
        raise ValueError("Unsupported KA Rigid Dynamics cache version")
    frame_info = metadata.pop("binary_frames", {})
    raw = zlib.decompress(compressed)
    metadata["frames"] = _decode_frames(
        raw,
        list(frame_info.get("frame_numbers", [])),
        list(frame_info.get("body_names", [])),
        dict(frame_info.get("body_scales", {})),
    )
    return metadata


def _read_legacy(path: str) -> Dict[str, Any]:
    with gzip.open(path, "rt", encoding="utf-8") as stream:
        payload = json.load(stream)
    if int(payload.get("cache_version", -1)) != 1:
        raise ValueError("Unsupported legacy KA Rigid Dynamics cache version")
    return payload


def read_cache(directory: str) -> Dict[str, Any]:
    path = cache_file_path(directory)
    if os.path.isfile(path):
        return _read_binary(path)
    legacy = legacy_cache_file_path(directory)
    if os.path.isfile(legacy):
        return _read_legacy(legacy)
    raise FileNotFoundError(path)


def remove_cache(directory: str) -> bool:
    removed = False
    for path in (cache_file_path(directory), legacy_cache_file_path(directory)):
        if os.path.exists(path):
            os.remove(path)
            removed = True
    return removed
