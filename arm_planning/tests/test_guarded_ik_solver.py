from pathlib import Path
import pytest
pytest.importorskip("torch")
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from guarded_ik_solver import wrapped_difference, pose_in_workspace
import yaml

CFG = yaml.safe_load((ROOT / "config" / "arm_kinematics.yaml").read_text(encoding="utf-8"))


def test_wrapped_difference_crosses_pi_correctly():
    error = wrapped_difference(-np.pi + 0.01, np.pi - 0.01)
    assert abs(error - 0.02) < 1e-9


def test_workspace_guard():
    assert pose_in_workspace([0.25, 0.0, 0.0, 0.0, 0.0], CFG)
    assert not pose_in_workspace([0.41, 0.0, 0.0, 0.0, 0.0], CFG)
    assert not pose_in_workspace([0.25, 0.31, 0.0, 0.0, 0.0], CFG)
    assert not pose_in_workspace([0.25, 0.0, 0.21, 0.0, 0.0], CFG)
