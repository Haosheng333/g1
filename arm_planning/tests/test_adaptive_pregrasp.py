from pathlib import Path
import sys,numpy as np
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT/'scripts'))
from module2_grasp_planner import Module2GraspPlanner
class Stub:
    config={'training_workspace':{'x_min':.1,'x_max':.4,'y_min':-.3,'y_max':.3,'z_min':-.15,'z_max':.2}}
def test_candidates_include_nominal_and_raised_options():
    p=Module2GraspPlanner.__new__(Module2GraspPlanner); p.planner=Stub(); nominal=np.array([.23,.05,0,0,.2]); c=p.pregrasp_candidates(nominal)
    assert np.allclose(c[0],nominal); assert any(q[2]>.05 for q in c)
