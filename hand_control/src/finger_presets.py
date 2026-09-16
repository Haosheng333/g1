"""
finger_presets.py — the single source of truth for grip geometry.

Both the real Modbus backend (revo_hand_link.py) and the MuJoCo sim backend
(hand_sim_link.py) consume FingerPose objects from here, so there is exactly
one place to tune "how open is pre_grasp" / "how hard does grasp squeeze" —
config/finger_presets.yaml. Nothing in scripts/ or src/ should hardcode a
raw position number.
"""

import os
from dataclasses import dataclass
from typing import Dict, List

import yaml

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_YAML = os.path.normpath(os.path.join(_THIS_DIR, "..", "..", "config", "finger_presets.yaml"))


@dataclass(frozen=True)
class FingerPose:
    """One target pose for all five fingers: [thumb, index, middle, ring, pinky]."""
    positions: List[int]              # 0-1000 per finger
    speed: int = 500                  # 0-1000
    force_limit: int = 300            # 0-1000
    settle_tolerance: int = 25        # raw units; "arrived" if |fb - target| <= this
    settle_timeout_s: float = 2.0


def load_presets(path: str = None) -> Dict[str, FingerPose]:
    """Load {name: FingerPose} from YAML. Resolution order:
    explicit `path` arg > HAND_CONTROL_PRESETS_YAML env var >
    config/finger_presets.yaml next to this package (works whether or not
    a ROS/catkin workspace is sourced, as long as the repo layout is intact).
    """
    resolved = path or os.environ.get("HAND_CONTROL_PRESETS_YAML", _DEFAULT_YAML)
    with open(resolved) as f:
        raw = yaml.safe_load(f)
    return {name: FingerPose(**cfg) for name, cfg in raw.items()}
