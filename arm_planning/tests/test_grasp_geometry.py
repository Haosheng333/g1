from pathlib import Path
import sys, numpy as np, pytest
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT/"scripts"))
from grasp_geometry import compute_grasp_and_pregrasp, load_config
CFG=load_config(ROOT/"config"/"arm_kinematics.yaml")
def test_shapes_and_position():
 p=np.array([.35,.10,.05]); g,pre=compute_grasp_and_pregrasp(p,"left",CFG); assert g.shape==(5,) and pre.shape==(5,); assert np.allclose(g[:3],p)
def test_distance_and_height():
 p=np.array([.35,.10,.05])
 for arm in ("left","right"):
  g,pre=compute_grasp_and_pregrasp(p,arm,CFG); assert np.linalg.norm(g[:3]-pre[:3])==pytest.approx(.12); assert pre[2]==pytest.approx(g[2]); assert g[3]==0 and pre[3]==0
def test_arm_sign_changes_yaw():
 p=np.array([.35,.10,.05]); l,_=compute_grasp_and_pregrasp(p,"left",CFG); r,_=compute_grasp_and_pregrasp(p,"right",CFG); assert l[4] != pytest.approx(r[4])
def test_invalid():
 with pytest.raises(ValueError): compute_grasp_and_pregrasp([.3,.1],"left",CFG)
 with pytest.raises(ValueError): compute_grasp_and_pregrasp([.3,.1,.2],"middle",CFG)
