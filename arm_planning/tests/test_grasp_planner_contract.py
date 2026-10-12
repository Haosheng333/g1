from pathlib import Path
import sys, numpy as np
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT/'scripts'))
from module2_grasp_planner import Module2GraspPlanner, interpolate_pose

class Stub:
    config={'grasp_geometry':{'pregrasp_distance':.12,'approach_angle_deg':40.,'grasp_lateral_offset':.17,'pregrasp_lateral_offset':.07,'arm_sign':{'left':-1.,'right':1.},'lift_height':.10,'retract_target_x':.15},'left':{'joint_names':['a']*5},'right':{'joint_names':['b']*5}}

def test_task_poses_have_lift_and_retract():
    p=Module2GraspPlanner.__new__(Module2GraspPlanner); p.planner=Stub(); poses=p.compute_task_poses('left',[.3,.15,0.]); assert np.isclose(poses['lift'][2]-poses['grasp'][2],.1); assert np.isclose(poses['retract'][0],.15)
def test_interpolation_ends_exactly():
    end=np.array([.3,.1,.1,0.,.2]); assert np.allclose(interpolate_pose(np.zeros(5),end)[-1],end)
def test_invalid_arm_fails_without_segments():
    p=Module2GraspPlanner.__new__(Module2GraspPlanner); p.planner=Stub(); r=p.plan_grasp_from_point('middle',[0]*5,[.3,0,0]); assert not r.success and r.segments=={}
