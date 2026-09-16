"""
hand_sim_link.py — MuJoCo stand-in for RevoHandLink.

Same interface as revo_hand_link.RevoHandLink: connect / close / write_pose /
read_positions / wait_until_settled. hand_node.py picks whichever backend
based on the ~sim param; nothing else about the node changes.

Works with either the placeholder 5-finger model or the real Revo2 URDF-
derived model (config/revo2_hand_real/{side}_hand.xml) — the joint/actuator
NAMES differ between them, so pass the right `joint_names`/`actuator_names`
for whichever model_path you're loading (see HAND_JOINTS_REAL below).
"""

import asyncio
import threading
import time
from typing import List, Optional

import mujoco

from hand_control.finger_presets import FingerPose

# --- Placeholder 5-finger stand-in model (config/revo2_hand_sim.xml) ---
PLACEHOLDER_JOINTS = [f"finger{i}_joint" for i in range(5)]
PLACEHOLDER_ACTUATORS = [f"finger{i}_act" for i in range(5)]

# --- Real Revo2 URDF-derived model (config/revo2_hand_real/{side}_hand.xml) ---
# Order matches finger_presets.yaml's 6-value convention:
# [thumb_metacarpal(rotation), thumb_proximal(flex), index, middle, ring, pinky]
def real_hand_joint_names(side: str) -> List[str]:
    return [
        f"{side}_thumb_metacarpal_joint",
        f"{side}_thumb_proximal_joint",
        f"{side}_index_proximal_joint",
        f"{side}_middle_proximal_joint",
        f"{side}_ring_proximal_joint",
        f"{side}_pinky_proximal_joint",
    ]


def real_hand_actuator_names(side: str) -> List[str]:
    return [
        f"{side}_thumb_metacarpal_act",
        f"{side}_thumb_proximal_act",
        f"{side}_index_proximal_act",
        f"{side}_middle_proximal_act",
        f"{side}_ring_proximal_act",
        f"{side}_pinky_proximal_act",
    ]


def real_hand_finger_bodies(side: str) -> List[str]:
    """The link whose contact should freeze each corresponding actuator
    during a compliant close. Order matches real_hand_joint_names/actuator_names."""
    return [
        f"{side}_thumb_distal_link",
        f"{side}_thumb_distal_link",
        f"{side}_index_distal_link",
        f"{side}_middle_distal_link",
        f"{side}_ring_distal_link",
        f"{side}_pinky_distal_link",
    ]


class RevoHandSimLink:
    def __init__(self, model_path: str, joint_names: List[str], actuator_names: List[str],
                 finger_bodies: Optional[List[str]] = None,
                 render: bool = False, physics_hz: Optional[float] = None,
                 close_ramp_seconds: float = 1.5):
        """finger_bodies: optional, one body name per actuator. When given,
        write_pose() closes gradually and freezes each finger's actuator the
        instant its body registers ANY contact (self-collision is disabled
        at the model level, so any contact here is necessarily with a
        foreign body — the floor or a grasped object — never another part
        of the hand itself). Without this, position actuators jump straight
        to the target regardless of what's in the way, which is fine for an
        empty hand but will fling a grasped object rather than hold it.
        """
        self._render = render
        self._model = mujoco.MjModel.from_xml_path(model_path)
        self._data = mujoco.MjData(self._model)
        self._dt = self._model.opt.timestep
        self._physics_hz = physics_hz or (1.0 / self._dt)
        self._close_ramp_steps = max(1, int(close_ramp_seconds / self._dt))

        self._act_ids = [
            mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in actuator_names
        ]
        joint_ids = [mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in joint_names]
        self._joint_qpos_adr = [self._model.jnt_qposadr[jid] for jid in joint_ids]
        # Per-joint [min, max] radians, read from the model instead of a shared constant,
        # since the real hand's 6 joints each have a different range.
        self._joint_ranges = [tuple(self._model.jnt_range[jid]) for jid in joint_ids]

        self._finger_body_ids = None
        if finger_bodies is not None:
            self._finger_body_ids = [
                mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, name) for name in finger_bodies
            ]

        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._viewer = None

    async def connect(self):
        self._running = True
        self._thread = threading.Thread(target=self._physics_loop, daemon=True)
        self._thread.start()
        await asyncio.sleep(0.05)

    async def close(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._viewer is not None:
            self._viewer.close()

    async def write_pose(self, pose: FingerPose):
        targets = [self._raw_to_angle(p, r) for p, r in zip(pose.positions, self._joint_ranges)]
        if self._finger_body_ids is None:
            with self._lock:
                for aid, t in zip(self._act_ids, targets):
                    self._data.ctrl[aid] = t
            return
        await self._compliant_close(targets)

    async def _compliant_close(self, targets: List[float]):
        """Ramp every actuator toward its target over close_ramp_seconds,
        freezing each one individually the instant its finger body touches
        anything. Runs in chunks with a yield between them so it doesn't
        block the asyncio loop for the full ~1.5s straight."""
        n = len(self._act_ids)
        with self._lock:
            starts = [self._data.ctrl[aid] for aid in self._act_ids]
        frozen = [False] * n
        chunk = 20
        for step in range(0, self._close_ramp_steps, chunk):
            frac = min(1.0, step / (self._close_ramp_steps * 0.8))
            with self._lock:
                for i, aid in enumerate(self._act_ids):
                    if not frozen[i]:
                        self._data.ctrl[aid] = starts[i] + frac * (targets[i] - starts[i])
                for _ in range(chunk):
                    mujoco.mj_step(self._model, self._data)
                for i, bid in enumerate(self._finger_body_ids):
                    if frozen[i]:
                        continue
                    for c in range(self._data.ncon):
                        con = self._data.contact[c]
                        b1, b2 = self._model.geom_bodyid[con.geom1], self._model.geom_bodyid[con.geom2]
                        if bid == b1 or bid == b2:
                            frozen[i] = True
                            break
            if all(frozen):
                break
            await asyncio.sleep(0)

    async def read_positions(self) -> List[int]:
        with self._lock:
            angles = [self._data.qpos[adr] for adr in self._joint_qpos_adr]
        return [self._angle_to_raw(a, r) for a, r in zip(angles, self._joint_ranges)]

    async def wait_until_settled(self, pose: FingerPose) -> bool:
        deadline = time.monotonic() + pose.settle_timeout_s
        while time.monotonic() < deadline:
            fb = await self.read_positions()
            if all(abs(fb[i] - pose.positions[i]) <= pose.settle_tolerance
                   for i in range(len(pose.positions))):
                return True
            await asyncio.sleep(0.02)
        return False

    def _physics_loop(self):
        if self._render:
            from mujoco import viewer as mj_viewer
            self._viewer = mj_viewer.launch_passive(self._model, self._data)
        period = 1.0 / self._physics_hz
        next_t = time.perf_counter()
        while self._running:
            with self._lock:
                mujoco.mj_step(self._model, self._data)
            if self._viewer is not None:
                self._viewer.sync()
            next_t += period
            sleep_for = next_t - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_t = time.perf_counter()

    @staticmethod
    def _raw_to_angle(raw: int, joint_range) -> float:
        lo, hi = joint_range
        raw = max(0, min(1000, raw))
        # raw=1000 -> lo (open), raw=0 -> hi (closed) -- matches finger_presets.yaml convention
        return hi - (hi - lo) * (raw / 1000.0)

    @staticmethod
    def _angle_to_raw(angle: float, joint_range) -> int:
        lo, hi = joint_range
        angle = max(lo, min(hi, angle))
        return int(round(1000 * (hi - angle) / (hi - lo))) if hi > lo else 0

