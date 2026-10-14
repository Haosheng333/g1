from pathlib import Path
import sys, yaml, numpy as np, pytest
torch=pytest.importorskip("torch")
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT/'scripts'))
from torch_g1_fk import TorchG1FK
from g1_forward_kinematics import pose_5d
CFG=yaml.safe_load((ROOT/'config'/'arm_kinematics.yaml').read_text())
def test_torch_fk_matches_scalar_and_has_gradients():
    rng=np.random.default_rng(4)
    for arm in ('left','right'):
        lo=np.asarray(CFG[arm]['lower_limits']); hi=np.asarray(CFG[arm]['upper_limits']); q=rng.uniform(lo,hi,size=(16,5)).astype(np.float32); qt=torch.tensor(q,requires_grad=True); fast=TorchG1FK(arm,CFG)(qt); slow=np.array([pose_5d(x,arm,CFG) for x in q]); assert np.allclose(fast.detach().numpy(),slow,atol=3e-6); fast[:,:3].square().mean().backward(); assert qt.grad is not None; assert torch.isfinite(qt.grad).all(); assert qt.grad.abs().sum()>0
