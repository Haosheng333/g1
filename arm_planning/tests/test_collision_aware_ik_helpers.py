from pathlib import Path
import sys, numpy as np
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT/'scripts'))
from collision_aware_ik import wrap, pose_errors
def test_wrap_crosses_pi(): assert abs(wrap(-np.pi+.01-(np.pi-.01))-.02)<1e-9
def test_pose_errors():
    p,r,y=pose_errors(np.array([.01,0,0,.02,-.03]),np.zeros(5)); assert np.isclose(p,.01) and np.isclose(r,.02) and np.isclose(y,.03)
