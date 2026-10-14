from pathlib import Path
import sys, math
import numpy as np, yaml
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT/'scripts'))
from joint_space_astar import JointGrid, astar_search, plan_joint_path
CFG=yaml.safe_load((ROOT/'config'/'arm_kinematics.yaml').read_text(encoding='utf-8'))


def limits(arm='left'):
    return np.asarray(CFG[arm]['lower_limits']),np.asarray(CFG[arm]['upper_limits'])


def test_grid_round_trip_within_half_cell():
    lo,hi=limits(); grid=JointGrid(lo,hi,4.0); q=(lo[:4]+hi[:4])/2; recovered=grid.state_to_joints(grid.joints_to_state(q)); assert np.max(np.abs(recovered-q))<=grid.resolution/2+1e-12


def test_start_equals_goal():
    lo,hi=limits(); q=(lo+hi)/2; result=plan_joint_path(q,q,lo,hi); assert result.success; assert np.allclose(result.path[0],q); assert np.allclose(result.path[-1],q)


def test_unblocked_plan_hits_exact_goal_and_one_degree_steps():
    lo,hi=limits(); start=np.array([0.,0.,0.,0.,0.]); goal=np.array([.20,-.15,.25,.30,.18]); result=plan_joint_path(start,goal,lo,hi); assert result.success; assert np.allclose(result.path[0],start); assert np.allclose(result.path[-1],goal); assert np.max(np.abs(np.diff(result.path,axis=0)))<=math.radians(1)+1e-9


def test_blocked_goal_fails():
    lo,hi=limits(); grid=JointGrid(lo,hi,4); start=grid.joints_to_state([0,0,0,0]); goal=grid.joints_to_state([.2,.2,.2,.2]); path,_,message=astar_search(grid,start,goal,lambda state: state==goal); assert path is None; assert 'goal state is blocked' in message


def test_fallback_routes_around_stage_obstacle():
    lo,hi=limits(); start_q=np.array([0.,0.,0.,0.,0.]); goal_q=np.array([.14,0.,.14,0.,0.]); grid=JointGrid(lo,hi,4); start=grid.joints_to_state(start_q); goal=grid.joints_to_state(goal_q); stage1_goal=(goal[0],goal[1],start[2],goal[3])
    # Block the staged intermediate goal, but leave alternate full-4D states free.
    result=plan_joint_path(start_q,goal_q,lo,hi,is_blocked=lambda s: s==stage1_goal); assert result.success; assert result.used_fallback
