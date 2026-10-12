#!/usr/bin/env python3
from __future__ import annotations
from pathlib import Path
import math
import numpy as np
import yaml

DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config" / "arm_kinematics.yaml"

def load_config(path=DEFAULT_CONFIG):
    with Path(path).open("r", encoding="utf-8") as f: return yaml.safe_load(f)

def compute_grasp_and_pregrasp(object_xyz, arm, config=None):
    if arm not in ("left","right"): raise ValueError("arm must be 'left' or 'right'.")
    p=np.asarray(object_xyz,float)
    if p.shape != (3,): raise ValueError("object_xyz must contain exactly [x, y, z].")
    if not np.all(np.isfinite(p)): raise ValueError("object_xyz must contain finite values.")
    config=load_config() if config is None else config
    g=config["grasp_geometry"]; sign=float(g["arm_sign"][arm]); x,y,z=p
    grasp_rz=math.atan2(y+sign*float(g["grasp_lateral_offset"]),x)
    pre_rz=math.atan2(y+sign*float(g["pregrasp_lateral_offset"]),x)
    approach=pre_rz+math.radians(float(g["approach_angle_deg"])); d=float(g["pregrasp_distance"])
    grasp=np.array([x,y,z,0.0,grasp_rz])
    pre=np.array([x-d*math.cos(approach),y-d*math.sin(approach),z,0.0,pre_rz])
    return grasp,pre
