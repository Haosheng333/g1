"""
finger_presets.py - single source of truth for grip geometry.

Both backends (revo_hand_link.py for the real hand, hand_sim_link.py for MuJoCo)
consume FingerPose objects from here, so there is exactly one place to tune the hand:
config/finger_presets.yaml. Nothing in scripts/ or src/ hardcodes a position number.

CONVENTION (same as the BrainCo SDK, used everywhere in this package, no inversion):
    per-mille, 0 = fully OPEN, 1000 = fully CLOSED
    finger order: [thumb, thumb_aux, index, middle, ring, pinky]
"""

import os
from dataclasses import dataclass
from typing import Dict, List

import yaml

NUM_FINGERS = 6
# The orchestrator contract: /hand_{side}/pre_grasp, /hand_{side}/grasp, /hand_{side}/release
REQUIRED_PRESETS = ("pre_grasp", "grasp", "release")

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_YAML = os.path.normpath(os.path.join(_THIS_DIR, "..", "..", "config", "finger_presets.yaml"))


@dataclass(frozen=True)
class FingerPose:
    positions: List[int]              # 6 values, 0 (open) .. 1000 (closed)
    settle_tolerance: int = 40        # "arrived" if |feedback - target| <= this
    settle_timeout_s: float = 3.0     # give up waiting for arrival after this long
    ramp_steps: int = 1               # real hand: reach the target in N sub-steps (gentle closing)
    ramp_dt: float = 0.05             # seconds between sub-steps
    require_settle: bool = False      # True: service FAILS if the fingers did not arrive
    expect_contact: bool = False      # True: service FAILS if no finger was stopped short (= grasped air)
    min_contact_fingers: int = 1      # how many fingers must be stopped short to count as "holding"

    def __post_init__(self):
        pos = self.positions
        if not isinstance(pos, (list, tuple)) or len(pos) != NUM_FINGERS:
            raise ValueError("positions must be a list of %d values, got %r" % (NUM_FINGERS, pos))
        clean = [int(p) for p in pos]
        if any(p < 0 or p > 1000 for p in clean):
            raise ValueError("positions must be within 0..1000, got %r" % (pos,))
        object.__setattr__(self, "positions", clean)
        if self.settle_tolerance <= 0 or self.settle_timeout_s <= 0:
            raise ValueError("settle_tolerance and settle_timeout_s must be > 0")
        if self.ramp_steps < 1 or self.ramp_dt < 0:
            raise ValueError("ramp_steps must be >= 1 and ramp_dt >= 0")
        if not 1 <= self.min_contact_fingers <= NUM_FINGERS:
            raise ValueError("min_contact_fingers must be within 1..%d" % NUM_FINGERS)


def load_presets(path: str = None) -> Dict[str, FingerPose]:
    """Load {name: FingerPose} from YAML. Resolution order:
    explicit `path` > HAND_CONTROL_PRESETS_YAML env var > config/finger_presets.yaml next to the package.
    Fails loudly (ValueError) on a missing required preset, a bad value, or an unknown key,
    so a typo is caught at startup instead of moving the hand.
    """
    resolved = path or os.environ.get("HAND_CONTROL_PRESETS_YAML", _DEFAULT_YAML)
    with open(resolved) as f:
        raw = yaml.safe_load(f) or {}

    missing = [n for n in REQUIRED_PRESETS if n not in raw]
    if missing:
        raise ValueError("%s is missing required preset(s): %s" % (resolved, ", ".join(missing)))

    presets = {}
    for name, cfg in raw.items():
        try:
            presets[name] = FingerPose(**cfg)
        except (TypeError, ValueError) as exc:
            raise ValueError("preset '%s' in %s is invalid: %s" % (name, resolved, exc))
    return presets
