from pathlib import Path
import sys,numpy as np
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT/'scripts'))
from module2_grasp_planner import Module2GraspPlanner
class Stub:
    config={'training_workspace':{'x_min':.1,'x_max':.4,'y_min':-.3,'y_max':.3,'z_min':-.15,'z_max':.2}}
def test_staging_candidates_include_yaw_variants_and_nominal_first():
    p=Module2GraspPlanner.__new__(Module2GraspPlanner); p.planner=Stub(); nominal=np.array([.23,.05,0.,0.,.2]); c=p.staging_candidates(nominal,'pregrasp')
    assert np.allclose(c[0],nominal); assert len({round(float(q[4]),4) for q in c})>1
def test_retract_candidates_stay_inside_workspace():
    p=Module2GraspPlanner.__new__(Module2GraspPlanner); p.planner=Stub(); c=p.staging_candidates(np.array([.15,.15,.1,0,.7]),'retract')
    assert c and all(.1<=q[0]<=.4 and -.3<=q[1]<=.3 and -.15<=q[2]<=.2 for q in c)
