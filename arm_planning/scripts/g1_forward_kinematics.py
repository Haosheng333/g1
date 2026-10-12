#!/usr/bin/env python3
from __future__ import annotations
from pathlib import Path
from typing import Any
import math
import numpy as np
import yaml

DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config" / "arm_kinematics.yaml"

def load_config(path: str | Path = DEFAULT_CONFIG) -> dict[str, Any]:
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Configuration file not found: {path}")
    with path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict):
        raise ValueError("YAML configuration is empty or invalid.")
    for arm in ("left", "right"):
        if arm not in cfg or len(cfg[arm].get("joint_names", [])) != 5:
            raise ValueError(f"The '{arm}' arm must contain exactly five joints.")
    return cfg

def rot_x(a):
    c,s=math.cos(a),math.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]],float)
def rot_y(a):
    c,s=math.cos(a),math.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]],float)
def rot_z(a):
    c,s=math.cos(a),math.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]],float)
def rpy_matrix(rpy):
    r,p,y=np.asarray(rpy,float); return rot_z(y) @ rot_y(p) @ rot_x(r)
def axis_angle_matrix(axis, angle):
    axis=np.asarray(axis,float); n=np.linalg.norm(axis)
    if n < 1e-12: raise ValueError("A joint axis cannot be zero.")
    x,y,z=axis/n; c,s,v=math.cos(angle),math.sin(angle),1-math.cos(angle)
    return np.array([[x*x*v+c,x*y*v-z*s,x*z*v+y*s],[y*x*v+z*s,y*y*v+c,y*z*v-x*s],[z*x*v-y*s,z*y*v+x*s,z*z*v+c]],float)
def make_transform(xyz, rotation):
    T=np.eye(4); T[:3,:3]=rotation; T[:3,3]=np.asarray(xyz,float); return T
def origin_transform(xyz,rpy): return make_transform(xyz,rpy_matrix(rpy))
def joint_transform(axis,angle): return make_transform(np.zeros(3),axis_angle_matrix(axis,angle))
def wrap_to_pi(a): return (a+math.pi)%(2*math.pi)-math.pi

def matrix_to_xyz_rpy(T):
    T=np.asarray(T,float)
    if T.shape != (4,4): raise ValueError("Expected a 4-by-4 transformation matrix.")
    R=T[:3,:3]; p=math.atan2(-R[2,0],math.hypot(R[0,0],R[1,0]))
    if abs(math.cos(p)) > 1e-9:
        r=math.atan2(R[2,1],R[2,2]); y=math.atan2(R[1,0],R[0,0])
    else:
        r=math.atan2(-R[0,1],R[1,1]); y=0.0
    return np.array([T[0,3],T[1,3],T[2,3],wrap_to_pi(r),wrap_to_pi(p),wrap_to_pi(y)])

def forward_kinematics(joints, arm, config=None, check_limits=True):
    config=load_config() if config is None else config
    if arm not in ("left","right"): raise ValueError("arm must be 'left' or 'right'.")
    q=np.asarray(joints,float)
    if q.shape != (5,): raise ValueError("joints must contain exactly five values.")
    if not np.all(np.isfinite(q)): raise ValueError("All joint values must be finite.")
    a=config[arm]; lower=np.asarray(a["lower_limits"],float); upper=np.asarray(a["upper_limits"],float)
    if check_limits and np.any((q<lower)|(q>upper)): raise ValueError(f"{arm} arm joint values exceed the limits.")
    T=np.eye(4)
    for xyz,rpy,axis,angle in zip(a["origins_xyz"],a["origins_rpy"],a["axes"],q):
        T = T @ origin_transform(xyz,rpy) @ joint_transform(axis,angle)
    return T @ origin_transform(a["grasp_xyz"],a["grasp_rpy"])

def pose_5d(joints, arm, config=None):
    full=matrix_to_xyz_rpy(forward_kinematics(joints,arm,config)); return full[[0,1,2,3,5]]

if __name__ == "__main__":
    np.set_printoptions(precision=6, suppress=True)
    for side in ("left", "right"):
        q = np.zeros(5)
        print(side + " transform:")
        print(forward_kinematics(q, side))
        print(side + " pose:", pose_5d(q, side))
        print()
