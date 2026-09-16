#!/usr/bin/env python3
"""
Dev test for the REAL Revo2 hand via bc_stark_sdk — no ROS needed.

Run this on Windows with the SAME Python that has bc_stark_sdk installed
(the brainco-hand-sdk venv/Python 3.11), e.g.:
    py -3.11 scripts/test_real_hand.py left

Start simple: this only calls connect(), reads the current position, then
does ONE gentle open command. Watch the hand closely. Don't run pre_grasp/
grasp presets against real hardware until this basic connect+read+one-move
works and looks safe.
"""
import asyncio
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from hand_control.revo_hand_link import RevoHandLink
from hand_control.finger_presets import FingerPose


async def main():
    side = sys.argv[1] if len(sys.argv) > 1 else "left"
    print(f"Connecting to '{side}' hand (auto-detecting port)...")
    link = RevoHandLink(side=side)
    await link.connect()
    print("Connected.")

    positions = await link.read_positions()
    print(f"Current finger positions (our convention, 1000=open/0=closed): {positions}")

    print("Sending a gentle 'mostly open' command in 3 seconds — watch the hand now...")
    await asyncio.sleep(3)
    gentle_open = FingerPose(positions=[900, 900, 900, 900, 900, 900], speed=200, force_limit=150,
                              settle_tolerance=50, settle_timeout_s=3.0)
    await link.write_pose(gentle_open)
    settled = await link.wait_until_settled(gentle_open)
    fb = await link.read_positions()
    print(f"After open command: feedback={fb} settled={settled}")

    await link.close()
    print("Disconnected cleanly.")


if __name__ == "__main__":
    asyncio.run(main())
