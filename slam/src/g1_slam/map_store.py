"""Saving and verifying a SLAM map folder (no rospy: unit-testable).

    <maps_dir>/<name>/map.pcd       canonical cloud, camera_init frame (z-DOWN)  - the source of truth
                     map_zup.pcd    same cloud in the z-UP `map` frame
                     map.pgm/.yaml  2D occupancy grid (pcd_to_map), z-UP `map` frame
                     manifest.yaml  what was saved, from the REAL session
                     semantic.yaml  named objects (surveyed with tools/survey_semantic.py)
    <maps_dir>/current -> <name>

save_map() builds the folder in a hidden STAGING directory, verifies it, and only then renames it into place and
re-points `current`.  A failure at any step removes the staging directory and leaves `current` untouched, so a
half-written map can never be picked up by localization, and an honest error goes back to the operator.
"""
import math
import os
import shutil
import time

import numpy as np
import yaml

from g1_slam import cloud_utils as cu
from g1_slam import grid_utils as gu
from g1_slam import semantic_store as ss

REQUIRED_FILES = ("map.pcd", "map_zup.pcd", "map.pgm", "map.yaml", "manifest.yaml", "semantic.yaml")
MANIFEST_KEYS = ("map_name", "created_at", "slam", "sensor_config", "pcd_file", "pcd_frame", "map_frame",
                 "grid_file", "resolution", "floor_z_camera_init", "num_points", "voxel",
                 "mapping_start_xy_map")
MIN_FREE_CELLS = 400            # 1 m^2 at 0.05 m
MIN_OBSTACLE_CELLS = 40         # a real room has walls


class MapError(RuntimeError):
    pass


def unique_name(maps_dir, base):
    base = (base or "").strip() or time.strftime("g1_map_%Y%m%d_%H%M%S")
    if os.sep in base or base.startswith(".") or base == "current":
        raise MapError("bad map name %r" % base)
    name, k = base, 2
    while os.path.exists(os.path.join(maps_dir, name)):
        name = "%s_%d" % (base, k)
        k += 1
    return name


def set_current(maps_dir, name):
    link, tmp = os.path.join(maps_dir, "current"), os.path.join(maps_dir, ".current.tmp")
    if os.path.lexists(tmp):
        os.remove(tmp)
    os.symlink(name, tmp)           # relative link
    os.replace(tmp, link)


def grid_stats(grid, resolution):
    free = int((grid == gu.WALKABLE).sum())
    occ = int((grid == gu.OBSTACLE).sum())
    unk = int((grid == gu.UNKNOWN).sum())
    return {"free_cells": free, "obstacle_cells": occ, "unknown_cells": unk,
            "free_area_m2": free * resolution * resolution, "shape": tuple(grid.shape)}


def verify_map_dir(path, require_pcd=True):
    """-> (problems:list[str], info:dict).  Empty problems = usable."""
    problems, info = [], {}
    names = [f for f in REQUIRED_FILES if require_pcd or not f.endswith(".pcd")]
    if not require_pcd:
        names = [f for f in names if f != "manifest.yaml"]
    for f in names:
        p = os.path.join(path, f)
        if not os.path.isfile(p):
            problems.append("missing %s" % f)
        elif os.path.getsize(p) == 0:
            problems.append("%s is empty" % f)
    if problems:
        return problems, info
    try:
        grid, res, ox, oy = gu.load_map_yaml(os.path.join(path, "map.yaml"))
    except Exception as exc:  # noqa: BLE001
        return ["map.yaml/map.pgm unreadable: %s" % exc], info
    st = grid_stats(grid, res)
    info.update(st, resolution=res, origin=(ox, oy))
    if not (0.01 <= res <= 0.5):
        problems.append("resolution %.3f m is not plausible" % res)
    if st["free_cells"] < MIN_FREE_CELLS:
        problems.append("only %d free cells (%.1f m^2): the grid is almost empty" % (st["free_cells"], st["free_area_m2"]))
    if st["obstacle_cells"] < MIN_OBSTACLE_CELLS:
        problems.append("only %d obstacle cells: no walls were mapped" % st["obstacle_cells"])
    if require_pcd:
        try:
            pts = cu.read_pcd_xyz(os.path.join(path, "map.pcd"))
            zup = cu.read_pcd_xyz(os.path.join(path, "map_zup.pcd"))
        except Exception as exc:  # noqa: BLE001
            return problems + ["PCD unreadable: %s" % exc], info
        info["num_points"] = int(len(pts))
        if len(pts) == 0 or not np.isfinite(pts).all():
            problems.append("map.pcd has no points or non-finite points")
        if len(zup) != len(pts):
            problems.append("map_zup.pcd has %d points but map.pcd has %d" % (len(zup), len(pts)))
        try:
            with open(os.path.join(path, "manifest.yaml"), encoding="utf-8") as fh:
                man = yaml.safe_load(fh) or {}
            miss = [k for k in MANIFEST_KEYS if k not in man]
            if miss:
                problems.append("manifest.yaml lacks %s" % ", ".join(miss))
            elif int(man["num_points"]) != len(pts):
                problems.append("manifest says %s points, map.pcd has %d" % (man["num_points"], len(pts)))
            info["manifest"] = man
        except Exception as exc:  # noqa: BLE001
            problems.append("manifest.yaml unreadable: %s" % exc)
    try:
        info["semantic"] = ss.load(os.path.join(path, "semantic.yaml"))
    except Exception as exc:  # noqa: BLE001
        problems.append("semantic.yaml unusable: %s" % exc)
    return problems, info


def save_map(maps_dir, name, cloud, floor_down, grid_runner, voxel, resolution, rpy, obs_h, sensor="Livox Mid-360",
             slam="FAST-LIO2 (Livox Mid-360)"):
    """Write + verify + commit one map.  `grid_runner(pcd_zup_path, out_dir, floor_up)` must write map.pgm and
    map.yaml into out_dir (the real one runs `rosrun g1_slam pcd_to_map`).  Returns (final_dir, info)."""
    os.makedirs(maps_dir, exist_ok=True)
    stage = os.path.join(maps_dir, ".%s.staging" % name)
    final = os.path.join(maps_dir, name)
    if os.path.exists(final):
        raise MapError("map %r already exists" % name)
    if os.path.exists(stage):
        shutil.rmtree(stage)
    os.makedirs(stage)
    try:
        cu.write_pcd(os.path.join(stage, "map.pcd"), cloud)                                  # camera_init, z-down
        R = cu.rpy_to_R(*rpy)                                                                # map <- camera_init
        cu.write_pcd(os.path.join(stage, "map_zup.pcd"), cu.apply_T(cu.make_T(R, [0, 0, 0]), cloud))
        grid_runner(os.path.join(stage, "map_zup.pcd"), stage, -float(floor_down))           # roll pi flips z
        for f in ("map.pgm", "map.yaml"):
            if not os.path.isfile(os.path.join(stage, f)):
                raise MapError("the grid step did not write %s" % f)
        manifest = {
            "map_name": name, "created_at": time.strftime("%Y%m%d_%H%M%S"), "slam": slam, "sensor_config": sensor,
            "pcd_file": "map.pcd", "pcd_frame": "camera_init (z-down)", "map_frame": "map (z-up)",
            "map_to_saved_origin_rpy": [float(v) for v in rpy], "grid_file": "map.yaml",
            "resolution": float(resolution), "floor_z_camera_init": float(floor_down),
            "obstacle_height_range_above_floor": [float(obs_h[0]), float(obs_h[1])],
            "num_points": int(len(cloud)), "voxel": float(voxel),
            # camera_init's origin is where this mapping session started == where the robot stood, and it is
            # (0, 0) in the z-up map frame (the map<-camera_init rotation is about x only).  A known standable,
            # reachable cell: tools/check_saved_map.py plans from here.
            "mapping_start_xy_map": [0.0, 0.0],
        }
        with open(os.path.join(stage, "manifest.yaml"), "w", encoding="utf-8") as fh:
            yaml.safe_dump(manifest, fh, default_flow_style=False, sort_keys=False)
        with open(os.path.join(stage, "semantic.yaml"), "w", encoding="utf-8") as fh:
            fh.write(ss.empty_file_text())
        problems, info = verify_map_dir(stage)
        if problems:
            raise MapError("verification failed: " + "; ".join(problems))
        os.rename(stage, final)
    except BaseException:
        shutil.rmtree(stage, ignore_errors=True)
        raise
    set_current(maps_dir, name)
    return final, info
