#!/usr/bin/env python3
"""Collision-aware multi-start numerical IK fallback for verified Cartesian targets."""
from __future__ import annotations
from dataclasses import dataclass
import math
import numpy as np
from scipy.optimize import least_squares
from g1_forward_kinematics import pose_5d
from g1_collision_probe import G1CollisionProbe


@dataclass
class CollisionAwareIKResult:
    success: bool
    message: str
    joints: np.ndarray | None
    position_error_m: float | None
    rx_error_rad: float | None
    rz_error_rad: float | None
    attempts: int


def wrap(value):
    return math.atan2(math.sin(value),math.cos(value))


def pose_errors(actual,target):
    position=float(np.linalg.norm(actual[:3]-target[:3]))
    return position,abs(wrap(float(actual[3]-target[3]))),abs(wrap(float(actual[4]-target[4])))


def solve_collision_aware_ik(target_pose,arm,config,probe,initial_joints=None,max_attempts=48,seed=42,max_position_error_m=.01,max_rx_error_rad=.10,max_rz_error_rad=.15):
    target=np.asarray(target_pose,float); lower=np.asarray(config[arm]['lower_limits'],float); upper=np.asarray(config[arm]['upper_limits'],float); rng=np.random.default_rng(seed)
    seeds=[]
    if initial_joints is not None: seeds.append(np.clip(np.asarray(initial_joints,float),lower,upper))
    midpoint=(lower+upper)/2; seeds.extend([midpoint,np.zeros(5).clip(lower,upper)])
    # Structured posture variants first, then random seeds.
    base=seeds[0] if seeds else midpoint
    for dyaw in (-1.0,-.6,-.3,.3,.6,1.0):
        s=base.copy(); s[2]=np.clip(s[2]+dyaw,lower[2],upper[2]); seeds.append(s)
    for droll in (-.8,-.4,.4,.8):
        s=base.copy(); s[1]=np.clip(s[1]+droll,lower[1],upper[1]); seeds.append(s)
    while len(seeds)<max_attempts: seeds.append(rng.uniform(lower,upper))

    best=None; checked=0
    def residual(q):
        pose=pose_5d(q,arm,config)
        # Position in centimetre-like scale; circular orientation residuals.
        return np.r_[10.0*(pose[:3]-target[:3]),wrap(float(pose[3]-target[3])),wrap(float(pose[4]-target[4])),0.01*(q-base)]

    for start in seeds[:max_attempts]:
        result=least_squares(residual,start,bounds=(lower,upper),max_nfev=300,xtol=1e-9,ftol=1e-9,gtol=1e-9)
        q=np.asarray(result.x,float); checked+=1; actual=pose_5d(q,arm,config); pe,rxe,rze=pose_errors(actual,target)
        if pe>max_position_error_m or rxe>max_rx_error_rad or rze>max_rz_error_rad: continue
        blocking=[item for item in probe.report(q) if item['blocking']]
        if blocking: continue
        score=pe+0.02*rxe+0.02*rze+0.001*np.linalg.norm(q-base)
        if best is None or score<best[0]: best=(score,q,pe,rxe,rze)
        if pe<.005 and rxe<.06 and rze<.10: break
    if best is None: return CollisionAwareIKResult(False,'no collision-free IK solution met pose thresholds',None,None,None,None,checked)
    _,q,pe,rxe,rze=best
    return CollisionAwareIKResult(True,'success',q,pe,rxe,rze,checked)
