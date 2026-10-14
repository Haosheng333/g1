from pathlib import Path
import pytest
pytest.importorskip("trimesh")
pytest.importorskip("fcl")
import sys,yaml
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT/'scripts'))
from g1_collision_probe import G1CollisionProbe
CFG=yaml.safe_load((ROOT/'config'/'arm_kinematics.yaml').read_text(encoding='utf-8'))
def test_shoulder_yaw_torso_allowance_only():
    probe=object.__new__(G1CollisionProbe); probe.cfg=CFG
    assert probe.allowance('shoulder_yaw','torso')==0.001
    assert probe.allowance('hand','torso')==0.0
