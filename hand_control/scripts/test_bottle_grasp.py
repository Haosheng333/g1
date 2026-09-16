#!/usr/bin/env python3
"""
Dev test: does the hand actually pick up (stably hold) a bottle?
Not a ROS test — exercises hand_control.hand_sim_link.RevoHandSimLink
directly against the scene file that has a bottle in it
(config/revo2_hand_real/{side}_hand_scene.xml).
"""
import asyncio
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from hand_control.finger_presets import load_presets
from hand_control.hand_sim_link import (
    RevoHandSimLink, real_hand_joint_names, real_hand_actuator_names, real_hand_finger_bodies,
)

SIDE = "left"
MODEL_PATH = os.path.join(os.path.dirname(__file__), "..", "config", "revo2_hand_real", f"{SIDE}_hand_scene.xml")


async def main():
    presets = load_presets()
    render = os.environ.get("RENDER") == "1"
    link = RevoHandSimLink(
        MODEL_PATH,
        joint_names=real_hand_joint_names(SIDE),
        actuator_names=real_hand_actuator_names(SIDE),
        finger_bodies=real_hand_finger_bodies(SIDE),  # enables compliant closing
        render=render,
    )
    await link.connect()

    for name in ["pre_grasp", "grasp"]:
        pose = presets[name]
        await link.write_pose(pose)
        settled = await link.wait_until_settled(pose)
        fb = await link.read_positions()
        print(f"{name:10s} target={pose.positions} feedback={fb} settled={settled}")

    print("Holding grasp for 5s to check the bottle stays put...")
    import mujoco
    bottle_bid = mujoco.mj_name2id(link._model, mujoco.mjtObj.mjOBJ_BODY, "bottle")
    start_pos = link._data.xpos[bottle_bid].copy()
    await asyncio.sleep(5.0)
    end_pos = link._data.xpos[bottle_bid].copy()
    drift = ((end_pos - start_pos) ** 2).sum() ** 0.5
    print(f"bottle drift over 5s: {drift:.4f} m ({'STABLE' if drift < 0.05 else 'SLIPPED/DROPPED'})")

    await link.close()


if __name__ == "__main__":
    asyncio.run(main())
