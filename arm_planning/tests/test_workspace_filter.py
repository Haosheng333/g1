from pathlib import Path
import sys, numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from generate_ik_dataset import inside_workspace
import yaml
CFG = yaml.safe_load((ROOT / "config" / "arm_kinematics.yaml").read_text())

def test_inside_global_workspace():
    assert inside_workspace(np.array([0.25, 0.00, 0.00, 0.0, 0.0]), "left", CFG)

def test_rejects_behind_or_far_or_wrong_height():
    assert not inside_workspace(np.array([-0.10, 0.00, 0.00, 0.0, 0.0]), "left", CFG)
    assert not inside_workspace(np.array([0.45, 0.00, 0.00, 0.0, 0.0]), "left", CFG)
    assert not inside_workspace(np.array([0.25, 0.31, 0.00, 0.0, 0.0]), "left", CFG)
    assert not inside_workspace(np.array([0.25, 0.00, 0.21, 0.0, 0.0]), "left", CFG)
