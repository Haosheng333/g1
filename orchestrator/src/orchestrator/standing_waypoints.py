"""Resolve the standing waypoints a route names, BEFORE preflight. No ROS.

`fetch_deliver` drives to STANDING poses -- where the robot itself stood when
an operator pressed A in `record_waypoints.py` -- not to object centres from
semantic.yaml. Those records live in <maps_dir>/<map_name>/standing_waypoints.yaml
and carry the identity of the map they were measured in.

This module asks g1_slam (the owner of that file format and of the goal
mapping) to look the names up and to check the map identity against the live
manifest. Every failure comes back as a blocker string, so the node can report a
missing record or a map mismatch in the same list as a missing service, and a
real run refuses before the first drive. Nothing here publishes or moves.
"""
from __future__ import annotations


def resolve_standing_waypoints(maps_dir: str, map_name: str, wanted: dict,
                               map_frame: str = "map", robot_frame: str = "pelvis"):
    """-> (resolved, blockers).

    `wanted` maps a leg name ("pickup", "delivery") to the waypoint name the
    request carried. `resolved` maps the same leg names to (key, record,
    goal_frame) for every name that was found; `blockers` lists the reason for
    every one that was not, in the preflight blocker format.
    """
    resolved, blockers = {}, []
    try:
        from g1_slam import standing_goal
    except ImportError as exc:
        blockers.append(
            "standing_waypoints (slam): g1_slam is not importable (%s); the DIP "
            "workspace overlay must be sourced" % (exc,))
        return resolved, blockers

    for leg, name in wanted.items():
        name = (name or "").strip()
        if not name:
            blockers.append("standing_waypoints (slam): the %s leg names no standing "
                            "waypoint" % leg)
            continue
        try:
            resolved[leg] = standing_goal.resolve(maps_dir, map_name, name,
                                                  map_frame, robot_frame)
        except Exception as exc:  # noqa: BLE001 - every reason becomes a blocker
            blockers.append("standing_waypoints (slam): %s waypoint %r: %s"
                            % (leg, name, exc))
    return resolved, blockers
