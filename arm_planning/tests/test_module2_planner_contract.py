from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT/'scripts'))
from module2_planner import Module2Planner


def bare_planner():
    planner=object.__new__(Module2Planner)
    planner.config={
        'left':{'joint_names':['a','b','c','d','e'],'lower_limits':[-1]*5,'upper_limits':[1]*5},
        'right':{'joint_names':['f','g','h','i','j'],'lower_limits':[-1]*5,'upper_limits':[1]*5},
    }
    return planner


def test_rejects_invalid_arm_without_path():
    result=bare_planner().plan_to_pose('middle',[0]*5,[0]*5)
    assert not result.success and result.joint_path==[]


def test_rejects_bad_current_shape_without_path():
    result=bare_planner().plan_to_pose('left',[0]*4,[0]*5)
    assert not result.success and result.joint_path==[]


def test_rejects_nonfinite_target_without_path():
    result=bare_planner().plan_to_pose('left',[0]*5,[0,0,np.nan,0,0])
    assert not result.success and result.joint_path==[]
