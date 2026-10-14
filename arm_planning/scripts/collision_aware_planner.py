#!/usr/bin/env python3
"""Collision-aware A* with nearest-free goal bridging and exact-goal validation."""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import argparse, itertools, json, math, sys
import numpy as np
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parent))
from collision_mask import BooleanGridCollisionMask
from joint_space_astar import JointGrid, interpolate_joint_path, states_to_five_joint_path
from edge_checked_astar import astar_edge_checked
from g1_collision_probe import G1CollisionProbe


@dataclass
class CollisionAwarePlanResult:
    success: bool
    message: str
    path: np.ndarray | None
    selected_grid_state: tuple[int,int,int,int] | None
    exact_goal_cell_blocked: bool
    bridge_waypoints: int
    nodes_expanded: int
    used_fallback: bool


def blocking_collisions(probe, q):
    return [item for item in probe.report(q) if item['blocking']]


def validate_dense_path(path, probe):
    failures=[]
    for index,q in enumerate(path):
        hits=blocking_collisions(probe,q)
        if hits: failures.append({'waypoint':index,'joints_rad':q.tolist(),'blocking_collisions':hits})
    return failures


def nearby_states(goal, counts, radius):
    goal=np.asarray(goal,dtype=int); candidates=[]
    ranges=[range(max(0,int(v)-radius),min(int(counts[i]),int(v)+radius+1)) for i,v in enumerate(goal)]
    for state in itertools.product(*ranges):
        delta=np.abs(np.asarray(state)-goal)
        candidates.append((int(delta.sum()),int(delta.max()),state))
    candidates.sort(key=lambda item:(item[0],item[1],item[2]))
    return [state for _,_,state in candidates]


def bridge_path(grid,state,q_goal,q_start_wrist,probe,max_step_rad=math.radians(1.0)):
    q_cell=np.r_[grid.state_to_joints(state),q_start_wrist]
    coarse=np.vstack((q_cell,q_goal))
    dense=interpolate_joint_path(coarse,max_step_rad)
    failures=validate_dense_path(dense,probe)
    return dense,failures


def plan_collision_aware(q_start,q_goal,arm,config,mask_path,project_root='.',resolution_deg=4.0,search_radius=3,max_expansions=1_000_000):
    q_start=np.asarray(q_start,dtype=float); q_goal=np.asarray(q_goal,dtype=float); arm_cfg=config[arm]
    grid=JointGrid(arm_cfg['lower_limits'],arm_cfg['upper_limits'],resolution_deg)
    mask=BooleanGridCollisionMask(mask_path,grid.counts)
    probe=G1CollisionProbe(project_root,arm,config)
    if blocking_collisions(probe,q_start): return CollisionAwarePlanResult(False,'exact start configuration is colliding',None,None,False,0,0,False)
    if blocking_collisions(probe,q_goal): return CollisionAwarePlanResult(False,'exact IK goal is colliding',None,None,False,0,0,False)
    goal_state=grid.joints_to_state(q_goal); exact_cell_blocked=bool(mask(goal_state)); selected=None; selected_bridge=None
    for state in nearby_states(goal_state,grid.counts,search_radius):
        if mask(state): continue
        bridge,failures=bridge_path(grid,state,q_goal,q_start[4],probe)
        if not failures:
            selected=state; selected_bridge=bridge; break
    if selected is None: return CollisionAwarePlanResult(False,f'no free grid goal with collision-free exact-goal bridge within radius {search_radius}',None,None,exact_cell_blocked,0,0,False)
    # Edge-checked full 4D A*: every neighbour transition is sampled at <=1 degree.
    start_state=grid.joints_to_state(q_start)
    states,expanded,message=astar_edge_checked(
        grid,start_state,selected,mask,probe,q_start[4],
        movable_axes=(0,1,2,3),move_weights=(1.0,1.0,3.0,1.0),
        max_expansions=max_expansions,
    )
    if states is None:
        return CollisionAwarePlanResult(False,f'edge-checked A* failed: {message}',None,selected,exact_cell_blocked,len(selected_bridge),expanded,True)
    q_grid_goal=np.r_[grid.state_to_joints(selected),q_start[4]]
    astar_path=states_to_five_joint_path(grid,states,q_start,q_grid_goal)
    combined=np.vstack((astar_path,selected_bridge[1:]))
    combined[-1]=q_goal
    failures=validate_dense_path(combined,probe)
    if failures:
        first=failures[0]
        return CollisionAwarePlanResult(False,f"dense path collision at waypoint {first['waypoint']}: {first['blocking_collisions']}",None,selected,exact_cell_blocked,len(selected_bridge),expanded,True)
    if np.max(np.abs(np.diff(combined,axis=0)))>math.radians(1.0)+1e-9:
        return CollisionAwarePlanResult(False,'combined path exceeds one-degree waypoint step',None,selected,exact_cell_blocked,len(selected_bridge),expanded,True)
    return CollisionAwarePlanResult(True,'success',combined,selected,exact_cell_blocked,len(selected_bridge),expanded,True)


def main():
    p=argparse.ArgumentParser(); p.add_argument('--arm',choices=['left','right'],required=True); p.add_argument('--start',nargs=5,type=float,required=True); p.add_argument('--goal',nargs=5,type=float,required=True); p.add_argument('--config',type=Path,default=Path('config/arm_kinematics.yaml')); p.add_argument('--project-root',type=Path,default=Path('.')); p.add_argument('--mask',type=Path); p.add_argument('--search-radius',type=int,default=3); p.add_argument('--output',type=Path,required=True); a=p.parse_args()
    cfg=yaml.safe_load(a.config.read_text(encoding='utf-8')); mask=a.mask or Path('collision_masks')/f'{a.arm}_static_collision_mask.npy'
    result=plan_collision_aware(a.start,a.goal,a.arm,cfg,mask,a.project_root,search_radius=a.search_radius)
    if result.success: np.save(a.output,result.path)
    print(json.dumps({'success':result.success,'message':result.message,'waypoints':0 if result.path is None else len(result.path),'selected_grid_state':result.selected_grid_state,'exact_goal_cell_blocked':result.exact_goal_cell_blocked,'bridge_waypoints':result.bridge_waypoints,'nodes_expanded':result.nodes_expanded,'used_fallback':result.used_fallback,'output':str(a.output) if result.success else None},indent=2)); raise SystemExit(0 if result.success else 2)
if __name__=='__main__': main()
