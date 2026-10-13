"""semantic.yaml read / write (no rospy: unit-testable).  Coordinates are in the z-UP `map` frame.

One file per saved map:  <maps_dir>/<map_name>/semantic.yaml   (small, hand-editable, version-controlled)
Written ATOMICALLY (temp file + rename) so a crash never leaves a half-written file the navigation node
would fail to parse.
"""
import math
import os
import re
import tempfile

import yaml

HEADER = """\
# Named objects for "navigate to the <class_name>" (/navigation/navigate).
# Coordinates are in the z-UP `map` frame.  Survey them on the real map, do not guess:
#   python3 tools/survey_semantic.py add table      (then click the object in rviz, Fixed Frame = map)
#   x, y         object position [m]
#   yaw          orientation of the object [rad] (informational)
#   approach_yaw OPTIONAL [rad]: stand on this side of the object (direction from the object towards
#                where the robot should stand).  If omitted the robot stands on the side it arrives from.
"""

NAME_RE = re.compile(r"^[a-z0-9][a-z0-9 _-]{0,39}$")


def normalize_name(name):
    n = str(name).strip().lower()
    if not NAME_RE.match(n):
        raise ValueError("bad object name %r: use 1-40 chars of a-z 0-9 space _ - (lower case)" % (name,))
    return n


def load(path):
    """-> {name: {x, y, yaw, approach_yaw|None}}; raises ValueError on a malformed file."""
    with open(path, encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    if not isinstance(data, dict):
        raise ValueError("%s: top level must be a mapping" % path)
    wps = data.get("waypoints") or {}
    if not isinstance(wps, dict):
        raise ValueError("%s: 'waypoints' must be a mapping name -> {x, y, ...}" % path)
    out = {}
    for name, v in wps.items():
        try:
            key = normalize_name(name)
            entry = {"x": float(v["x"]), "y": float(v["y"]), "yaw": float(v.get("yaw", 0.0)),
                     "approach_yaw": None if v.get("approach_yaw") is None else float(v["approach_yaw"])}
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("%s: waypoint %r is malformed (%s)" % (path, name, exc))
        if not all(math.isfinite(entry[k]) for k in ("x", "y", "yaw")):
            raise ValueError("%s: waypoint %r has a non-finite number" % (path, name))
        if key in out:
            raise ValueError("%s: waypoint %r appears twice (names are case-insensitive)" % (path, name))
        out[key] = entry
    return out


def save(path, waypoints):
    body = {"waypoints": {}}
    for name in sorted(waypoints):
        w = waypoints[name]
        item = {"x": round(float(w["x"]), 3), "y": round(float(w["y"]), 3), "yaw": round(float(w.get("yaw", 0.0)), 3)}
        if w.get("approach_yaw") is not None:
            item["approach_yaw"] = round(float(w["approach_yaw"]), 3)
        body["waypoints"][name] = item
    text = HEADER + yaml.safe_dump(body, default_flow_style=None, sort_keys=False)
    d = os.path.dirname(os.path.abspath(path))
    fd, tmp = tempfile.mkstemp(dir=d, prefix=".semantic.", suffix=".tmp")
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


def empty_file_text():
    return HEADER + "waypoints: {}\n"


def add(path, name, x, y, yaw=0.0, approach_yaw=None, replace=False):
    key = normalize_name(name)
    for v in (x, y, yaw) + (() if approach_yaw is None else (approach_yaw,)):
        if not math.isfinite(float(v)):
            raise ValueError("non-finite coordinate for %r" % name)
    wps = load(path) if os.path.exists(path) else {}
    if key in wps and not replace:
        raise ValueError("%r already exists at (%.2f, %.2f); pass --replace to overwrite" % (key, wps[key]["x"], wps[key]["y"]))
    wps[key] = {"x": float(x), "y": float(y), "yaw": float(yaw),
                "approach_yaw": None if approach_yaw is None else float(approach_yaw)}
    save(path, wps)
    return wps[key]


def remove(path, name):
    key = normalize_name(name)
    wps = load(path)
    if key not in wps:
        raise KeyError("no waypoint %r" % key)
    del wps[key]
    save(path, wps)
