"""Standing-pose waypoints: read / write (no rospy, so it is unit-testable).

WHAT THIS IS, AND WHY IT IS NOT semantic.yaml
`semantic_store` holds OBJECT centres. `/navigation/navigate` looks an object up
there and then stands `approach_distance` away from it. These records are the
opposite: each one is **where the robot itself stood**, recorded by driving it
there. Sending it as a goal must therefore use navigation's EXACT-pose interface
(`/move_base_simple/goal`) and apply **no further approach offset** -- offsetting
a standing pose by another approach distance would park the robot a metre away
from the spot that was deliberately chosen.

Two different meanings, two different files, two different goal paths. Do not
merge them.

FILE LAYOUT, one file per saved map:

    <maps_dir>/<map_name>/standing_waypoints.yaml

    map:                        # IDENTITY of the map these coordinates belong to
      name: g1_map_20261016_101500
      created_at: '20261016_101500'
      map_frame: map
      robot_frame: pelvis
    waypoints:
      handover_01:
        x: 1.234                # metres, in the z-UP `map` frame
        y: -0.567
        yaw: 1.571              # radians, robot heading in `map`
        recorded_at: '2026-10-16T10:22:31'
        stamp: 1792252951.284   # ROS time of the TF the pose came from
        map_frame: map
        robot_frame: pelvis
        cloud_snapshot: handover_01_20261016_102231.pcd   # optional, relative

WHY THE IDENTITY BLOCK EXISTS. x/y/yaw mean nothing without the map they were
measured in. `map.name` + `map.created_at` come from the map's own
`manifest.yaml`, so re-recording against a different or re-made map is REFUSED
rather than silently appended -- which would mix two coordinate origins in one
file.

Writes are ATOMIC (temp file + fsync + rename) and always rewrite the whole
file from the records already on disk, so an existing record is never lost and a
crash never leaves a half-written file.
"""
import math
import os
import re
import tempfile

import yaml

HEADER = """\
# STANDING poses recorded by driving the robot there (rosrun g1_slam record_waypoints.py).
#
# Each entry is WHERE THE ROBOT STOOD, not where an object is. Navigate to one with
#   rosrun g1_slam goto_waypoint.py _name:=<name>
# which publishes the pose to /move_base_simple/goal -- navigation's EXACT-pose
# interface. No approach-distance offset is applied, and none must be: the pose is
# already the chosen standing spot. Object centres live in semantic.yaml instead.
#
#   x, y   metres in the z-UP `map` frame
#   yaw    radians, the robot's heading in `map`
# The `map:` block identifies which saved map these coordinates belong to; a
# mismatch is refused rather than appended.
"""

NAME_RE = re.compile(r"^[a-z0-9][a-z0-9_-]{0,39}$")

#: Keys every record carries. `cloud_snapshot` is optional.
RECORD_KEYS = ("x", "y", "yaw", "recorded_at", "stamp", "map_frame", "robot_frame")


class StoreError(ValueError):
    """The file, a record, or the requested operation is not usable."""


class MapMismatch(StoreError):
    """The file belongs to a different map than the one now being recorded in."""


def normalize_name(name):
    n = str(name).strip().lower()
    if not NAME_RE.match(n):
        raise StoreError(
            "bad waypoint name %r: use 1-40 chars of a-z 0-9 _ - (lower case)" % (name,))
    return n


def identity(name, created_at, map_frame="map", robot_frame="pelvis"):
    """The map-identity block. `name` and `created_at` come from manifest.yaml."""
    if not str(name).strip():
        raise StoreError("map identity needs a non-empty name")
    return {"name": str(name).strip(), "created_at": str(created_at).strip(),
            "map_frame": str(map_frame), "robot_frame": str(robot_frame)}


def same_map(a, b):
    """True when two identity blocks describe the same saved map."""
    if not a or not b:
        return False
    return (str(a.get("name", "")) == str(b.get("name", ""))
            and str(a.get("created_at", "")) == str(b.get("created_at", "")))


def describe_mismatch(on_disk, live):
    return ("the file holds poses for map %r (created %s) but the robot is localized "
            "to map %r (created %s). Coordinates from two different maps must never "
            "share a file: record into that map's own file, or pass a different "
            "--file." % (on_disk.get("name"), on_disk.get("created_at"),
                         live.get("name"), live.get("created_at")))


def load(path):
    """-> (identity_or_None, {name: record}). Raises StoreError if malformed."""
    with open(path, encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    if not isinstance(data, dict):
        raise StoreError("%s: top level must be a mapping" % path)

    ident = data.get("map")
    if ident is not None and not isinstance(ident, dict):
        raise StoreError("%s: 'map' must be a mapping" % path)

    raw = data.get("waypoints") or {}
    if not isinstance(raw, dict):
        raise StoreError("%s: 'waypoints' must be a mapping name -> record" % path)

    out = {}
    for name, value in raw.items():
        key = normalize_name(name)
        if key in out:
            raise StoreError("%s: waypoint %r appears twice (names are case-insensitive)"
                             % (path, name))
        if not isinstance(value, dict):
            raise StoreError("%s: waypoint %r is not a mapping" % (path, name))
        try:
            rec = {"x": float(value["x"]), "y": float(value["y"]),
                   "yaw": float(value["yaw"]),
                   "recorded_at": str(value.get("recorded_at", "")),
                   "stamp": float(value.get("stamp", 0.0)),
                   "map_frame": str(value.get("map_frame", "")),
                   "robot_frame": str(value.get("robot_frame", ""))}
        except (KeyError, TypeError, ValueError) as exc:
            raise StoreError("%s: waypoint %r is malformed (%s)" % (path, name, exc))
        if not all(math.isfinite(rec[k]) for k in ("x", "y", "yaw")):
            raise StoreError("%s: waypoint %r has a non-finite coordinate" % (path, name))
        snap = value.get("cloud_snapshot")
        if snap:
            rec["cloud_snapshot"] = str(snap)
        out[key] = rec
    return ident, out


def save(path, ident, waypoints):
    """Rewrite the whole file atomically. Never partially written."""
    body = {}
    if ident:
        body["map"] = {k: ident[k] for k in ("name", "created_at", "map_frame",
                                             "robot_frame") if k in ident}
    body["waypoints"] = {}
    for name in sorted(waypoints):
        rec = waypoints[name]
        item = {"x": round(float(rec["x"]), 4),
                "y": round(float(rec["y"]), 4),
                "yaw": round(float(rec["yaw"]), 4),
                "recorded_at": rec.get("recorded_at", ""),
                "stamp": round(float(rec.get("stamp", 0.0)), 3),
                "map_frame": rec.get("map_frame", ""),
                "robot_frame": rec.get("robot_frame", "")}
        if rec.get("cloud_snapshot"):
            item["cloud_snapshot"] = rec["cloud_snapshot"]
        body["waypoints"][name] = item

    text = HEADER + yaml.safe_dump(body, default_flow_style=False, sort_keys=False)
    directory = os.path.dirname(os.path.abspath(path))
    if not os.path.isdir(directory):
        raise StoreError("directory %s does not exist" % directory)
    fd, tmp = tempfile.mkstemp(dir=directory, prefix=".standing_waypoints.", suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            fh.write(text)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.remove(tmp)
        raise


def next_name(waypoints, prefix="handover"):
    """A unique name: <prefix>_01, _02, ... skipping any already taken."""
    prefix = normalize_name(prefix)
    n = 1
    while True:
        candidate = "%s_%02d" % (prefix, n)
        if candidate not in waypoints:
            return candidate
        n += 1


def add(path, ident, x, y, yaw, recorded_at, stamp, name=None, prefix="handover",
        cloud_snapshot=None, replace=False):
    """Append one standing pose, preserving everything already on disk.

    Refuses a map mismatch, a duplicate name without `replace`, and any
    non-finite coordinate. Returns (name, record).
    """
    for label, value in (("x", x), ("y", y), ("yaw", yaw)):
        if not math.isfinite(float(value)):
            raise StoreError("non-finite %s for a standing pose" % label)

    on_disk_ident, waypoints = (load(path) if os.path.exists(path) else (None, {}))
    if on_disk_ident and not same_map(on_disk_ident, ident):
        raise MapMismatch(describe_mismatch(on_disk_ident, ident))

    key = normalize_name(name) if name else next_name(waypoints, prefix)
    if key in waypoints and not replace:
        raise StoreError("waypoint %r already exists at (%.3f, %.3f); pass replace=True "
                         "to overwrite" % (key, waypoints[key]["x"], waypoints[key]["y"]))

    record = {"x": float(x), "y": float(y), "yaw": float(yaw),
              "recorded_at": str(recorded_at), "stamp": float(stamp),
              "map_frame": ident.get("map_frame", "map"),
              "robot_frame": ident.get("robot_frame", "pelvis")}
    if cloud_snapshot:
        record["cloud_snapshot"] = str(cloud_snapshot)
    waypoints[key] = record
    save(path, ident, waypoints)
    return key, record


def get(path, name):
    """One record, with the file's map identity. Raises KeyError if absent."""
    ident, waypoints = load(path)
    key = normalize_name(name)
    if key not in waypoints:
        raise KeyError("no standing waypoint %r in %s (have: %s)"
                       % (key, path, ", ".join(sorted(waypoints)) or "(none)"))
    return ident, key, waypoints[key]


def remove(path, name):
    ident, waypoints = load(path)
    key = normalize_name(name)
    if key not in waypoints:
        raise KeyError("no standing waypoint %r" % key)
    del waypoints[key]
    save(path, ident, waypoints)


def empty_file_text(ident=None):
    body = {}
    if ident:
        body["map"] = dict(ident)
    body["waypoints"] = {}
    return HEADER + yaml.safe_dump(body, default_flow_style=False, sort_keys=False)
