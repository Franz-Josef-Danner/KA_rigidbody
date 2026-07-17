"""Structured diagnostic logging without a Blender dependency."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import os
import threading
from typing import Any, Dict, Optional

_LOG_LOCK = threading.Lock()


def _json_default(value: Any) -> str:
    try:
        return str(value)
    except Exception:
        return "<unserializable>"


def write_diagnostic(
    enabled: bool,
    path: Optional[str],
    component: str,
    event: str,
    *,
    level: str = "INFO",
    data: Optional[Dict[str, Any]] = None,
) -> None:
    """Append one structured line to the log and mirror it to Blender's console."""
    if not enabled or not path:
        return

    timestamp = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
    payload = data or {}
    serialized = json.dumps(payload, ensure_ascii=False, sort_keys=True, default=_json_default)
    line = f"{timestamp} | {level.upper()} | {component} | {event} | {serialized}"
    print(f"KA Rigid Dynamics | {line}")

    try:
        directory = os.path.dirname(os.path.abspath(path))
        os.makedirs(directory, exist_ok=True)
        with _LOG_LOCK:
            with open(path, "a", encoding="utf-8") as stream:
                stream.write(line)
                stream.write("\n")
    except Exception as exc:
        print(f"KA Rigid Dynamics | LOG_WRITE_FAILED | {exc}")
