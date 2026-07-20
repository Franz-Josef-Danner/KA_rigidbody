"""Solver-only mass conditioning that preserves authored impactor ratios."""

from __future__ import annotations

from typing import Any, Dict, Mapping, MutableMapping, Optional, Sequence


def condition_dynamic_mass_ratios(
    bodies: Sequence[MutableMapping[str, Any]],
    constraints: Optional[Sequence[Mapping[str, Any]]] = None,
    *,
    enabled: bool = True,
    limit: float = 5000.0,
    absolute_floor: float = 0.001,
) -> Dict[str, Any]:
    """Raise tiny solver masses without scaling unrelated impactors and targets.

    When an authored Dynamic-Dynamic bond graph exists, conditioning is applied
    independently inside each connected dynamic bond component. Unbonded
    projectiles therefore retain their authored mass relative to a bonded
    structure. Scenes without Dynamic-Dynamic bonds keep the legacy global
    conditioning behaviour for loose piles.
    """
    dynamic = [
        body for body in bodies
        if str(body.get("body_type", "")) == "DYNAMIC" and not body.get("skip_simulation")
    ]
    summary: Dict[str, Any] = {
        "enabled": bool(enabled),
        "mode": "DISABLED",
        "largest_mass": 0.0,
        "mass_floor": 0.0,
        "adjusted_bodies": 0,
        "ratio_before": 0.0,
        "ratio_after": 0.0,
        "component_count": 0,
        "conditioned_components": 0,
        "max_component_ratio_before": 0.0,
        "max_component_ratio_after": 0.0,
    }
    if not dynamic:
        return summary

    absolute_floor = max(1.0e-6, float(absolute_floor))
    limit = max(10.0, float(limit))
    by_id: Dict[str, MutableMapping[str, Any]] = {}
    by_name: Dict[str, MutableMapping[str, Any]] = {}
    for index, body in enumerate(dynamic):
        stable_id = str(body.get("stable_id") or body.get("name") or f"dynamic:{index}")
        by_id[stable_id] = body
        by_name[str(body.get("name", stable_id))] = body

    parent = {stable_id: stable_id for stable_id in by_id}

    def find(value: str) -> str:
        root = value
        while parent[root] != root:
            root = parent[root]
        while parent[value] != value:
            next_value = parent[value]
            parent[value] = root
            value = next_value
        return root

    def union(first: str, second: str) -> None:
        root_a = find(first)
        root_b = find(second)
        if root_a == root_b:
            return
        if root_a < root_b:
            parent[root_b] = root_a
        else:
            parent[root_a] = root_b

    dynamic_edges = 0
    for constraint in constraints or ():
        if not bool(constraint.get("enabled", True)):
            continue
        if str(constraint.get("constraint_type", "")) != "BREAKABLE_FIXED":
            continue
        first_id = str(constraint.get("body_a", ""))
        second_id = str(constraint.get("body_b", ""))
        first = by_id.get(first_id) or by_name.get(str(constraint.get("body_a_name", "")))
        second = by_id.get(second_id) or by_name.get(str(constraint.get("body_b_name", "")))
        if first is None or second is None:
            continue
        first_key = str(first.get("stable_id") or first.get("name"))
        second_key = str(second.get("stable_id") or second.get("name"))
        if first_key not in parent or second_key not in parent:
            continue
        union(first_key, second_key)
        dynamic_edges += 1

    if dynamic_edges:
        components: Dict[str, list[MutableMapping[str, Any]]] = {}
        for stable_id, body in by_id.items():
            components.setdefault(find(stable_id), []).append(body)
        groups = list(components.values())
        summary["mode"] = "BONDED_COMPONENTS"
    else:
        groups = [dynamic]
        summary["mode"] = "GLOBAL"

    all_before = [max(1.0e-12, float(body.get("mass", 0.0))) for body in dynamic]
    summary["largest_mass"] = max(all_before)
    summary["ratio_before"] = max(all_before) / min(all_before)
    summary["component_count"] = len(groups)

    if not enabled:
        summary["ratio_after"] = summary["ratio_before"]
        return summary

    adjusted = 0
    maximum_floor = 0.0
    conditioned_components = 0
    max_ratio_before = 0.0
    max_ratio_after = 0.0
    for group in groups:
        masses = [max(1.0e-12, float(body.get("mass", 0.0))) for body in group]
        largest = max(masses)
        smallest = min(masses)
        ratio_before = largest / smallest
        max_ratio_before = max(max_ratio_before, ratio_before)
        # A singleton has no internal mass-ratio problem. Only the absolute
        # small-body floor applies, so a heavy unbonded impactor cannot inflate
        # every mass in an unrelated bonded structure.
        component_floor = absolute_floor
        if len(group) > 1:
            component_floor = max(absolute_floor, largest / limit)
        component_adjusted = 0
        for body in group:
            mass = max(0.0, float(body.get("mass", 0.0)))
            if mass + 1.0e-12 >= component_floor:
                continue
            body["mass"] = float(component_floor)
            adjustments = body.setdefault("stability_adjustments", [])
            if "mass_ratio_clamped" not in adjustments:
                adjustments.append("mass_ratio_clamped")
            adjusted += 1
            component_adjusted += 1
        after = [max(1.0e-12, float(body.get("mass", 0.0))) for body in group]
        max_ratio_after = max(max_ratio_after, max(after) / min(after))
        maximum_floor = max(maximum_floor, component_floor)
        if component_adjusted:
            conditioned_components += 1

    all_after = [max(1.0e-12, float(body.get("mass", 0.0))) for body in dynamic]
    summary.update({
        "mass_floor": maximum_floor,
        "adjusted_bodies": adjusted,
        "ratio_after": max(all_after) / min(all_after),
        "conditioned_components": conditioned_components,
        "max_component_ratio_before": max_ratio_before,
        "max_component_ratio_after": max_ratio_after,
    })
    return summary
