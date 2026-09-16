"""
revo_hand_link.py — REAL Revo2 hand backend using BrainCo's official bc_stark_sdk.

This REPLACES the earlier placeholder version that wrote raw, guessed Modbus
register addresses — do not use that old version against real hardware.

Every function/argument used below was confirmed by directly inspecting the
installed bc_stark_sdk package (its compiled main_mod, via Python's
`inspect` and the bundled .pyi type stub), not guessed from memory:
  - main_mod.auto_detect(scan_all, port, protocol) -> list[DetectedDevice]
  - main_mod.init_from_detected(device)            -> DeviceContext   (await)
  - ctx.get_hand_type(slave_id)                    -> HandType        (await)
  - ctx.set_finger_positions(slave_id, positions)                     (await)
  - ctx.get_finger_positions(slave_id)             -> list[int]       (await)
  - main_mod.close_device_handler(ctx)                                (await)

IMPORTANT — convention mismatch, handled internally:
The real SDK's own position range is documented as
    0 = fully open, 1000 = fully closed
which is the OPPOSITE of the 1000=open/0=closed convention already used
throughout the rest of this project (finger_presets.yaml, hand_sim_link.py).
Rather than flip every other file (and risk breaking the already-validated
simulation), this module inverts internally: real = 1000 - ours. So
FingerPose.positions keeps meaning the same thing everywhere else in the
codebase — only this file needs to know about the SDK's own convention.

Finger order is [Thumb, ThumbAux, Index, Middle, Ring, Pinky] per
bc_stark_sdk's FingerId enum — matching the
[thumb_metacarpal, thumb_proximal, index, middle, ring, pinky] order
already used in finger_presets.yaml.

NOT YET VALIDATED AGAINST REAL HARDWARE — the connect()/device-matching
logic in particular (picking the right hand when one is plugged in) is the
part most likely to need adjusting once you can see real auto_detect()
output. Test connect() + a single write_pose() call manually before wiring
this into hand_node.py's Trigger services.
"""

import asyncio
import inspect
import time
from typing import List, Optional

from hand_control.finger_presets import FingerPose

try:
    from bc_stark_sdk import main_mod as stark
except ImportError:
    stark = None


def _to_real(our_positions: List[int]) -> List[int]:
    """Our convention (1000=open, 0=closed) -> SDK convention (0=open, 1000=closed)."""
    return [max(0, min(1000, 1000 - p)) for p in our_positions]


def _from_real(real_positions: List[int]) -> List[int]:
    """Inverse of _to_real."""
    return [max(0, min(1000, 1000 - p)) for p in real_positions]


class RevoHandLink:
    """Same interface as hand_control.hand_sim_link.RevoHandSimLink:
    connect(), close(), write_pose(pose), read_positions(), wait_until_settled(pose).
    """

    def __init__(self, side: str, port: Optional[str] = None):
        """side: "left" or "right". port: optionally force a specific COM
        port (e.g. "COM3") instead of auto-detecting across all ports.
        """
        if stark is None:
            raise RuntimeError("bc_stark_sdk not installed: pip install bc-stark-sdk")
        self._side = side
        self._port = port
        self._ctx = None
        self._slave_id = None

    async def connect(self):
        # The compiled SDK returns an asyncio Future here (not a coroutine),
        # so test for any awaitable rather than asyncio.iscoroutine().
        devices = stark.auto_detect(scan_all=True, port=self._port, protocol=None)
        if inspect.isawaitable(devices):
            devices = await devices
        devices = list(devices or [])

        if not devices:
            raise RuntimeError(
                f"No Stark/Revo2 device detected on {self._port or 'any port'}. "
                "Check the hand is powered, the USB-RS485 adapter is plugged in, "
                "and the CH340 driver is installed (verify in Device Manager first)."
            )

        # Single hand plugged in: just use it, but still report its actual
        # side so a side-mismatch is visible rather than silently wrong.
        if len(devices) == 1:
            device = devices[0]
            self._ctx = await stark.init_from_detected(device)
            self._slave_id = device.slave_id
            hand_type = await self._ctx.get_hand_type(self._slave_id)
            detected_side = "left" if hand_type == stark.HandType.Left else "right"
            if detected_side != self._side:
                raise RuntimeError(
                    f"hand_node.py was started with side='{self._side}' but the "
                    f"connected hand reports itself as '{detected_side}'. "
                    f"Either restart with the correct _side param, or check you "
                    f"plugged in the hand you meant to."
                )
            return

        # Multiple devices detected (e.g. both hands plugged in): find the
        # one matching the requested side, closing the ones we don't use.
        for device in devices:
            ctx = await stark.init_from_detected(device)
            hand_type = await ctx.get_hand_type(device.slave_id)
            detected_side = "left" if hand_type == stark.HandType.Left else "right"
            if detected_side == self._side:
                self._ctx = ctx
                self._slave_id = device.slave_id
                return
            await stark.close_device_handler(ctx)

        raise RuntimeError(
            f"Detected {len(devices)} device(s) but none matched side='{self._side}'. "
            f"Detected slave_ids: {[d.slave_id for d in devices]}"
        )

    async def close(self):
        if self._ctx is not None:
            await stark.close_device_handler(self._ctx)
            self._ctx = None

    async def write_pose(self, pose: FingerPose):
        real_positions = _to_real(pose.positions)
        await self._ctx.set_finger_positions(self._slave_id, real_positions)

    async def read_positions(self) -> List[int]:
        real_positions = await self._ctx.get_finger_positions(self._slave_id)
        return _from_real(list(real_positions))

    async def wait_until_settled(self, pose: FingerPose) -> bool:
        deadline = time.monotonic() + pose.settle_timeout_s
        while time.monotonic() < deadline:
            fb = await self.read_positions()
            if all(abs(fb[i] - pose.positions[i]) <= pose.settle_tolerance
                   for i in range(len(pose.positions))):
                return True
            await asyncio.sleep(0.02)
        return False
