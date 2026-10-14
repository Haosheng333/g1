from pathlib import Path
import sys, numpy as np, yaml
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT/'scripts'))
from module2_grasp_planner import Module2GraspPlanner
from grasp_geometry import compute_grasp_and_pregrasp
class Stub:
    config=yaml.safe_load((ROOT/'config'/'arm_kinematics.yaml').read_text(encoding='utf-8'))
def test_planner_uses_canonical_grasp_geometry():
    planner=Module2GraspPlanner.__new__(Module2GraspPlanner); planner.planner=Stub()
    for arm,obj in [('left',[.3,.15,0.]),('right',[.3,-.15,0.])]:
        poses=planner.compute_task_poses(arm,obj); grasp,pre=compute_grasp_and_pregrasp(obj,arm,Stub.config)
        assert np.allclose(poses['grasp'],grasp); assert np.allclose(poses['pregrasp'],pre)
        assert np.allclose(poses['grasp'][:3],obj)
