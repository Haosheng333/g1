from pathlib import Path
import sys, yaml, numpy as np
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT/'scripts')); sys.path.insert(0,str(ROOT/'scripts'))
from vectorized_g1_fk import batch_fk_pose_5d
from g1_forward_kinematics import pose_5d
CFG=yaml.safe_load((ROOT/'config'/'arm_kinematics.yaml').read_text())
def test_vectorized_matches_scalar():
    rng=np.random.default_rng(7)
    for arm in ('left','right'):
        lo=np.asarray(CFG[arm]['lower_limits']); hi=np.asarray(CFG[arm]['upper_limits']); q=rng.uniform(lo,hi,size=(32,5)); fast=batch_fk_pose_5d(q,arm,CFG); slow=np.array([pose_5d(x,arm,CFG) for x in q]); assert np.allclose(fast,slow,atol=2e-6)
