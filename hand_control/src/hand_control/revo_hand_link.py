"""
revo_hand_link.py - REAL Revo2 hand backend using BrainCo's bc_stark_sdk.

SDK calls used (as confirmed by inspecting the installed bc_stark_sdk package):
  main_mod.auto_detect(scan_all, port, protocol) -> list[DetectedDevice]
  main_mod.init_from_detected(device)            -> DeviceContext        (await)
  ctx.get_hand_type(slave_id)                    -> HandType             (await)
  ctx.set_finger_positions(slave_id, positions)                          (await)
  ctx.get_finger_positions(slave_id)             -> list[int]            (await)
  main_mod.close_device_handler(ctx)                                     (await)

Convention: positions are passed to the SDK UNCHANGED - 0 = open, 1000 = closed,
order [thumb, thumb_aux, index, middle, ring, pinky]. No inversion anywhere.

Same interface as hand_sim_link.RevoHandSimLink:
  connect / close / reconnect / write_pose / read_positions / wait_until_settled
"""

import asyncio
import time
from typing import List, Optional

from hand_control.finger_presets import FingerPose

try:
    from bc_stark_sdk import main_mod as stark
except ImportError:
    stark = None


def _clamp(positions: List[float]) -> List[int]:
    return [max(0, min(1000, int(round(p)))) for p in positions]


def _lerp(a: List[int], b: List[int], t: float) -> List[int]:
    return _clamp([x + (y - x) * t for x, y in zip(a, b)])


class RevoHandLink:
    def __init__(self, side: str, port: Optional[str] = None):
        """side: "left" or "right". port: force a serial port (e.g. "/dev/ttyUSB0");
        None = auto-detect across all ports."""
        if stark is None:
            raise RuntimeError("bc_stark_sdk not installed: pip install bc-stark-sdk")
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right', got %r" % (side,))
        self._side = side
        self._port = port
        self._ctx = None
        self._slave_id = None
        self._last_cmd: Optional[List[int]] = None

    # ------------------------------------------------------------------ connection
    async def _side_of(self, ctx, slave_id) -> str:
        hand_type = await ctx.get_hand_type(slave_id)
        return "left" if hand_type == stark.HandType.Left else "right"

    async def connect(self):
        devices = stark.auto_detect(scan_all=True, port=self._port, protocol=None)
        if asyncio.iscoroutine(devices):
            devices = await devices

        if not devices:
            raise RuntimeError(
                "No Stark/Revo2 device detected on %s. Check the hand is powered, the USB-RS485 "
                "adapter is plugged in, you are in the 'dialout' group, and no other program "
                "(e.g. BrainCo's PC software) is holding the port." % (self._port or "any port")
            )

        # Open each detected device, keep the one whose reported side matches; close the rest.
        # A single device with the WRONG side is an error (never silently drive the wrong hand).
        seen = []
        for device in devices:
            ctx = await stark.init_from_detected(device)
            detected = await self._side_of(ctx, device.slave_id)
            seen.append((device.slave_id, detected))
            if detected == self._side:
                self._ctx = ctx
                self._slave_id = device.slave_id
                self._last_cmd = None
                return
            await stark.close_device_handler(ctx)

        raise RuntimeError(
            "Detected %d device(s) but none is a '%s' hand. Detected (slave_id, side): %s. "
            "Restart with the correct side, or check which hand is plugged in."
            % (len(devices), self._side, seen)
        )

    async def close(self):
        ctx, self._ctx = self._ctx, None
        if ctx is not None:
            try:
                await stark.close_device_handler(ctx)
            except Exception:
                pass

    async def reconnect(self):
        await self.close()
        await self.connect()

    # ------------------------------------------------------------------ commands
    async def _set(self, positions: List[int]):
        await self._ctx.set_finger_positions(self._slave_id, positions)
        self._last_cmd = list(positions)

    async def write_pose(self, pose: FingerPose):
        """Move to pose.positions. If pose.ramp_steps > 1 the move is split into sub-steps
        (starting from where the fingers actually are), so closing is gradual."""
        target = _clamp(pose.positions)
        if pose.ramp_steps <= 1:
            await self._set(target)
            return

        try:
            start = _clamp(await self.read_positions())
        except Exception:
            start = self._last_cmd or target
        for i in range(1, pose.ramp_steps + 1):
            await self._set(_lerp(start, target, i / float(pose.ramp_steps)))
            if i < pose.ramp_steps:
                await asyncio.sleep(pose.ramp_dt)

    async def read_positions(self) -> List[int]:
        return [int(p) for p in await self._ctx.get_finger_positions(self._slave_id)]

    async def wait_until_settled(self, pose: FingerPose) -> bool:
        deadline = time.monotonic() + pose.settle_timeout_s
        while time.monotonic() < deadline:
            fb = await self.read_positions()
            if all(abs(fb[i] - pose.positions[i]) <= pose.settle_tolerance
                   for i in range(len(pose.positions))):
                return True
            await asyncio.sleep(0.02)
        return False
