#!/usr/bin/env python3
"""Is this saved map USABLE for navigation?  No ROS, no robot - run it right after mapping and again after surveying.

    python3 tools/check_saved_map.py lab                          # slam/maps/lab
    python3 tools/check_saved_map.py lab --require table,sofa     # the objects the routes navigate to must exist
    python3 tools/check_saved_map.py navigation/sim/sim_room --grid-only --start 1,1

Checks, with PASS/FAIL per line (exit code 1 if anything failed):
  * every file is there and readable; PCD point counts agree with manifest.yaml; the grid has free space and walls
  * the standable free space is ONE connected region (a split map means drift or a door the robot cannot pass)
  * the mapping start cell (camera_init origin = where the robot stood) is standable
  * every named object (and each --require name) has a standable, REACHABLE approach pose, found with the same
    spiral search and A* the navigation node uses
"""
import argparse
import math
import os
import sys

import numpy as np
from scipy import ndimage

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(REPO, "slam", "src"))
sys.path.insert(0, os.path.join(REPO, "navigation", "scripts"))

from g1_slam import grid_utils as gu  # noqa: E402
from g1_slam import map_store as ms  # noqa: E402
from costmap import Costmap  # noqa: E402
from global_planner import AStarPlanner  # noqa: E402
from semantic_goal import find_approach_pose  # noqa: E402

MIN_MAIN_FRACTION = 0.90


def run_checks(path, require=(), approach=0.6, start=None, grid_only=False):
    """-> list of (ok, name, detail)."""
    out = []

    def add(ok, name, detail=""):
        out.append((bool(ok), name, detail))

    problems, info = ms.verify_map_dir(path, require_pcd=not grid_only)
    if problems:
        add(False, "map folder is complete and consistent", "; ".join(problems))
        if not os.path.isfile(os.path.join(path, "map.yaml")) or not os.path.isfile(os.path.join(path, "map.pgm")):
            return out
    else:
        add(True, "map folder is complete and consistent",
            "%s points, %.1f m^2 free" % (info.get("num_points", "n/a"), info["free_area_m2"]))
    try:
        grid, res, ox, oy = gu.load_map_yaml(os.path.join(path, "map.yaml"))
    except Exception as exc:  # noqa: BLE001
        add(False, "grid loads", str(exc))
        return out
    tiers = gu.compose_tiers(grid, res, danger_m=0.25, risk_m=0.50, hole_fill_iterations=3)
    cm = Costmap.from_grid(tiers, res, ox, oy)
    standable = (~cm.blocked) & (cm.dist >= cm.cfg["standable_clearance"])
    lab, n = ndimage.label(standable)
    if n == 0:
        add(False, "standable free space exists", "none: the robot cannot stand anywhere")
        return out
    sizes = ndimage.sum(standable, lab, range(1, n + 1))
    main = int(np.argmax(sizes)) + 1
    frac = float(sizes.max() / sizes.sum())
    add(frac >= MIN_MAIN_FRACTION, "standable space is one connected region",
        "%.0f%% in the largest of %d regions (need >= %.0f%%)" % (100 * frac, n, 100 * MIN_MAIN_FRACTION))

    if start is None:
        man = info.get("manifest") or {}
        start = tuple(man.get("mapping_start_xy_map", (0.0, 0.0)))
    sx, sy = cm.world_to_map(start[0], start[1])
    start_ok = cm.in_bounds(sx, sy) and lab[sy, sx] == main
    add(start_ok, "mapping start (%.2f, %.2f) is in the main standable region" % start,
        "" if start_ok else "the cell where the robot started mapping is not standable or is cut off")

    sem = info.get("semantic")
    if sem is None:
        from g1_slam import semantic_store as ss
        try:
            sem = ss.load(os.path.join(path, "semantic.yaml"))
        except Exception as exc:  # noqa: BLE001
            add(False, "semantic.yaml loads", str(exc))
            return out
    planner = AStarPlanner()
    for name in sorted(set(sem) | {r.strip().lower() for r in require if r.strip()}):
        if name not in sem:
            add(False, "object '%s' is surveyed" % name, "missing from semantic.yaml: run tools/survey_semantic.py add %s" % name)
            continue
        w = sem[name]
        mx, my = cm.world_to_map(w["x"], w["y"])
        if not cm.in_bounds(mx, my):
            add(False, "object '%s' lies inside the map" % name, "(%.2f, %.2f) is outside the grid" % (w["x"], w["y"]))
            continue
        pose = find_approach_pose(cm, planner, (w["x"], w["y"]), approach, start, approach_yaw=w["approach_yaw"])
        if pose is None:
            add(False, "object '%s' has a reachable approach pose" % name,
                "no standable, reachable cell near %.2f m from (%.2f, %.2f)" % (approach, w["x"], w["y"]))
        else:
            d = math.hypot(pose[0] - w["x"], pose[1] - w["y"])
            add(True, "object '%s' has a reachable approach pose" % name,
                "stand at (%.2f, %.2f) facing %.2f rad, %.2f m from the object" % (pose[0], pose[1], pose[2], d))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("map", help="map name under --maps-dir, or a path to a map folder")
    ap.add_argument("--maps-dir", default=os.path.join(REPO, "slam", "maps"))
    ap.add_argument("--require", default="", help="comma separated object names that must be surveyed")
    ap.add_argument("--approach", type=float, default=0.6, help="approach distance to test [m]")
    ap.add_argument("--start", default="", help="x,y in the map frame (default: manifest mapping_start_xy_map)")
    ap.add_argument("--grid-only", action="store_true", help="skip PCD/manifest checks (simulator maps have none)")
    a = ap.parse_args(argv)
    path = a.map if os.path.isdir(a.map) else os.path.join(a.maps_dir, a.map)
    if not os.path.isdir(path):
        print("[FAIL] map folder %s does not exist" % path)
        return 1
    start = tuple(float(v) for v in a.start.split(",")) if a.start else None
    results = run_checks(path, [r for r in a.require.split(",") if r], a.approach, start, a.grid_only)
    for ok, name, detail in results:
        print("[%s] %s%s" % ("PASS" if ok else "FAIL", name, ("  -- " + detail) if detail else ""))
    bad = [r for r in results if not r[0]]
    print("\nRESULT: %s" % ("ALL PASS (%d checks)" % len(results) if not bad else "FAILED (%d of %d)" % (len(bad), len(results))))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
