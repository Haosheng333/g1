#!/usr/bin/env python3
"""
Dev test for the REAL Revo2 hand: run the grasp presets from
config/finger_presets.yaml end to end — pre_grasp -> grasp -> (hold) -> release.
No ROS needed.

    python3 scripts/test_real_grasp.py left            # or right
    python3 scripts/test_real_grasp.py left --hold 10  # hold the grasp longer

Only run this after scripts/test_real_hand.py (connect + one gentle open)
has been shown to work. Each step waits for Enter before moving the hand:
put the bottle in the palm when asked, and keep your own fingers clear.
"""
import argparse
import asyncio
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from hand_control.finger_presets import load_presets
from hand_control.revo_hand_link import RevoHandLink


async def step(link, presets, name):
    pose = presets[name]
    await link.write_pose(pose)
    settled = await link.wait_until_settled(pose)
    fb = await link.read_positions()
    print(f"{name:10s} target={pose.positions} feedback={fb} settled={settled}")
    return fb


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("side", nargs="?", default="left", choices=["left", "right"])
    ap.add_argument("--hold", type=float, default=5.0, help="seconds to hold the grasp")
    args = ap.parse_args()

    presets = load_presets()
    link = RevoHandLink(side=args.side)
    print(f"Connecting to '{args.side}' hand...")
    await link.connect()
    print(f"Connected. Current positions: {await link.read_positions()}")

    input("Enter -> pre_grasp (hand opens; nothing in the hand yet) ")
    await step(link, presets, "pre_grasp")

    input("Place the bottle in the palm, then Enter -> grasp ")
    await step(link, presets, "grasp")

    print(f"Holding for {args.hold:.0f}s - check the bottle is held firmly, then try a gentle tug...")
    await asyncio.sleep(args.hold)
    print(f"Positions after hold: {await link.read_positions()}")

    input("Enter -> release (be ready to catch the bottle) ")
    await step(link, presets, "release")

    await link.close()
    print("Disconnected cleanly.")


if __name__ == "__main__":
    asyncio.run(main())
