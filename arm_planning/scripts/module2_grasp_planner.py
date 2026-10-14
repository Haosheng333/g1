#!/usr/bin/env python3
"""Orchestrator-facing grasp sequence planner for the G1 23-DOF arm."""
from __future__ import annotations
from dataclasses import dataclass, asdict
from pathlib import Path
import argparse, json, math
import numpy as np

from module2_planner import Module2Planner
from collision_aware_planner import plan_collision_aware
from grasp_geometry import compute_grasp_and_pregrasp


@dataclass
class GraspSegment:
    name: str
    target_pose: list[float]
    joint_path: list[list[float]]
    position_error_m: float
    rx_error_rad: float
    rz_error_rad: float


@dataclass
class PlanGraspResult:
    success: bool
    message: str
    arm: str
    object_xyz: list[float]
    joint_names: list[str]
    pregrasp_pose: list[float]
    grasp_pose: list[float]
    lift_pose: list[float]
    retract_pose: list[float]
    segments: dict[str, dict]
    final_joints: list[float]


def wrapped_interpolate(a, b, fraction):
    delta=math.atan2(math.sin(b-a),math.cos(b-a))
    return a+fraction*delta


def interpolate_pose(start, end, max_position_step=0.015, max_angle_step=math.radians(3.0)):
    start=np.asarray(start,float); end=np.asarray(end,float)
    linear=np.max(np.abs(end[:3]-start[:3]))/max_position_step
    angular=max(abs(math.atan2(math.sin(end[3]-start[3]),math.cos(end[3]-start[3]))),abs(math.atan2(math.sin(end[4]-start[4]),math.cos(end[4]-start[4]))))/max_angle_step
    steps=max(1,int(math.ceil(max(linear,angular))))
    poses=[]
    for i in range(1,steps+1):
        f=i/steps; pose=start+(end-start)*f
        pose[3]=wrapped_interpolate(start[3],end[3],f); pose[4]=wrapped_interpolate(start[4],end[4],f); poses.append(pose)
    return poses


class Module2GraspPlanner:
    def __init__(self, point_planner: Module2Planner | None = None, **planner_kwargs):
        self.planner=point_planner or Module2Planner(**planner_kwargs)

    def _failure(self, arm, object_xyz, message, poses=None):
        poses=poses or {}
        names=self.planner.config.get(arm,{}).get('joint_names',[]) if arm in ('left','right') else []
        return PlanGraspResult(False,message,arm,list(object_xyz) if np.asarray(object_xyz).shape==(3,) else [],list(names),poses.get('pregrasp',[]),poses.get('grasp',[]),poses.get('lift',[]),poses.get('retract',[]),{},[])

    def compute_task_poses(self, arm, object_xyz):
        # Use the already-tested canonical geometry function. Do not duplicate its equations here.
        grasp,pre=compute_grasp_and_pregrasp(object_xyz,arm,self.planner.config)
        geometry=self.planner.config['grasp_geometry']
        lift=grasp.copy(); lift[2]+=float(geometry['lift_height'])
        retract=lift.copy(); retract[0]=float(geometry['retract_target_x']); retract[4]=math.atan2(retract[1],retract[0])
        return {'pregrasp':pre,'grasp':grasp,'lift':lift,'retract':retract}


    def staging_candidates(self, nominal, mode='pregrasp'):
        nominal=np.asarray(nominal,float); candidates=[]
        # Staging orientation is flexible; final grasp orientation is never changed.
        yaw_offsets=np.radians([0,15,-15,30,-30,45,-45])
        if mode=='pregrasp':
            xyz_offsets=[(dx,dy,dz) for dz in (0.0,0.03,0.06,0.09) for dx in (0.0,0.04,-0.03) for dy in (0.0,0.04,-0.04)]
        else:
            xyz_offsets=[(dx,dy,dz) for dz in (0.0,0.03,-0.03) for dx in (0.0,0.04,-0.04,0.08) for dy in (0.0,0.04,-0.04)]
        ws=self.planner.config['training_workspace']
        for dx,dy,dz in xyz_offsets:
            for dyaw in yaw_offsets:
                q=nominal.copy(); q[:3]+=[dx,dy,dz]; q[4]=math.atan2(math.sin(q[4]+dyaw),math.cos(q[4]+dyaw))
                if ws['x_min']<=q[0]<=ws['x_max'] and ws['y_min']<=q[1]<=ws['y_max'] and ws['z_min']<=q[2]<=ws['z_max']:
                    candidates.append(q)
        candidates.sort(key=lambda q:(float(np.linalg.norm(q[:3]-nominal[:3])),abs(math.atan2(math.sin(q[4]-nominal[4]),math.cos(q[4]-nominal[4])))))
        return candidates

    def pregrasp_candidates(self, nominal):
        return self.staging_candidates(nominal,'pregrasp')

    def choose_pregrasp(self, arm, current, nominal, grasp):
        failures=[]
        for index,candidate in enumerate(self.pregrasp_candidates(nominal),start=1):
            result=self.planner.plan_to_pose(arm,current,candidate)
            if not result.success:
                failures.append(f'candidate {index}: {result.message}')
                continue
            # Require the candidate to have a viable first approach waypoint.
            first_waypoint=interpolate_pose(candidate,grasp)[0]
            probe=self.planner.plan_to_pose(arm,result.verified_goal_joints,first_waypoint)
            if probe.success:
                return candidate,result,None
            failures.append(f'candidate {index} approach: {probe.message}')
        return None,None,'; '.join(failures[-5:])

    def _cartesian_segment(self, arm, start_joints, start_pose, end_pose, name):
        current=np.asarray(start_joints,float); paths=[]; last_result=None
        for waypoint_index,pose in enumerate(interpolate_pose(start_pose,end_pose),start=1):
            result=self.planner.plan_to_pose(arm,current,pose)
            if not result.success: return None,f'{name} waypoint {waypoint_index} failed: {result.message}'
            part=np.asarray(result.joint_path,float); paths.append(part if not paths else part[1:])
            current=np.asarray(result.verified_goal_joints,float); last_result=result
        path=np.vstack(paths) if paths else np.asarray([start_joints],float)
        return GraspSegment(name,np.asarray(end_pose,float).tolist(),path.tolist(),float(last_result.position_error_m),float(last_result.rx_error_rad),float(last_result.rz_error_rad)),None

    def plan_grasp_from_point(self, arm, current_joints, object_xyz):
        if arm not in ('left','right'): return self._failure(str(arm),object_xyz,"arm must be 'left' or 'right'")
        try: current=np.asarray(current_joints,float); obj=np.asarray(object_xyz,float)
        except (TypeError,ValueError): return self._failure(arm,object_xyz,'current_joints and object_xyz must be numeric')
        if current.shape!=(5,) or not np.all(np.isfinite(current)): return self._failure(arm,obj,'current_joints must contain five finite values')
        if obj.shape!=(3,) or not np.all(np.isfinite(obj)): return self._failure(arm,object_xyz,'object_xyz must contain three finite values')
        poses=self.compute_task_poses(arm,obj); pose_lists={k:v.tolist() for k,v in poses.items()}
        # Select a collision-free staging pose near the nominal pregrasp.
        chosen_pre,pre,error=self.choose_pregrasp(arm,current,poses['pregrasp'],poses['grasp'])
        if error: return self._failure(arm,obj,'no viable pregrasp candidate: '+error,pose_lists)
        poses['pregrasp']=chosen_pre; pose_lists['pregrasp']=chosen_pre.tolist()
        seg_pre=GraspSegment('to_pregrasp',chosen_pre.tolist(),pre.joint_path,float(pre.position_error_m),float(pre.rx_error_rad),float(pre.rz_error_rad))
        q=np.asarray(pre.verified_goal_joints,float)
        approach,error=self._cartesian_segment(arm,q,chosen_pre,poses['grasp'],'approach')
        if error: return self._failure(arm,obj,error,pose_lists)
        q=np.asarray(approach.joint_path[-1],float)
        lift,error=self._cartesian_segment(arm,q,poses['grasp'],poses['lift'],'lift')
        if error: return self._failure(arm,obj,error,pose_lists)
        q=np.asarray(lift.joint_path[-1],float)
        # Retract is also a flexible staging target. Try nearby collision-free
        # endpoints and retain the first complete Cartesian segment that passes.
        retract=None; retract_errors=[]; selected_retract=None
        for candidate in self.staging_candidates(poses['retract'],'retract'):
            candidate_segment,candidate_error=self._cartesian_segment(arm,q,poses['lift'],candidate,'retract')
            if candidate_error is None:
                retract=candidate_segment; selected_retract=candidate; break
            retract_errors.append(candidate_error)
        if retract is None:
            return self._failure(arm,obj,'no viable retract candidate: '+'; '.join(retract_errors[-5:]),pose_lists)
        poses['retract']=selected_retract; pose_lists['retract']=selected_retract.tolist()
        segments={s.name:asdict(s) for s in (seg_pre,approach,lift,retract)}
        return PlanGraspResult(True,'success',arm,obj.tolist(),list(self.planner.config[arm]['joint_names']),pose_lists['pregrasp'],pose_lists['grasp'],pose_lists['lift'],pose_lists['retract'],segments,retract.joint_path[-1])


def main():
    p=argparse.ArgumentParser(); p.add_argument('--arm',choices=['left','right'],required=True); p.add_argument('--current-joints',nargs=5,type=float,required=True); p.add_argument('--object-xyz',nargs=3,type=float,required=True); p.add_argument('--output-json',type=Path); a=p.parse_args()
    result=Module2GraspPlanner().plan_grasp_from_point(a.arm,a.current_joints,a.object_xyz); text=json.dumps(asdict(result),indent=2); print(text)
    if a.output_json: a.output_json.write_text(text,encoding='utf-8')
    raise SystemExit(0 if result.success else 2)
if __name__=='__main__': main()
