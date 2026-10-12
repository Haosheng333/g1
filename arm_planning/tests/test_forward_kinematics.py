from pathlib import Path
import sys, numpy as np, pytest
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT/"scripts"))
from g1_forward_kinematics import forward_kinematics, load_config, pose_5d
CFG=load_config(ROOT/"config"/"arm_kinematics.yaml")
def test_transform():
 T=forward_kinematics(np.zeros(5),"left",CFG); assert T.shape==(4,4); assert np.allclose(T[3],[0,0,0,1]); assert np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-10)
def test_pose():
 for arm in ("left","right"): assert pose_5d(np.zeros(5),arm,CFG).shape==(5,)
def test_mirror():
 l=pose_5d(np.zeros(5),"left",CFG); r=pose_5d(np.zeros(5),"right",CFG); assert np.isclose(l[0],r[0],atol=2e-3); assert np.isclose(l[1],-r[1],atol=2e-3); assert np.isclose(l[2],r[2],atol=2e-3)
def test_limits():
 for arm in ("left","right"): forward_kinematics(np.asarray(CFG[arm]["lower_limits"]),arm,CFG); forward_kinematics(np.asarray(CFG[arm]["upper_limits"]),arm,CFG)
def test_bad_limit():
 q=np.asarray(CFG["left"]["upper_limits"],float); q[0]+=.01
 with pytest.raises(ValueError): forward_kinematics(q,"left",CFG)
def test_bad_arm():
 with pytest.raises(ValueError): forward_kinematics(np.zeros(5),"middle",CFG)
