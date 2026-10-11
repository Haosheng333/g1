#!/usr/bin/env python3
"""ON-ROBOT check of the motor-index table used by g1_joint_state_bridge.py (mode=dds).

    python3 tools/verify_joint_bridge.py enp2s0

Reads rt/lowstate with unitree_sdk2py (needs the robot's network interface) and, for every joint in
G1_MOTOR_INDEX, records how far its slot moves while YOU move the robot by hand.  The robot should be
in damping/zero-torque mode, hanging or supported, and nobody commands it from this script (read only).

Steps it asks for:  1) hold still 3 s (noise floor)  2) rotate the WAIST left/right by ~30 deg
3) bend the LEFT elbow  4) bend the RIGHT elbow.
PASS only if the slot that moved most is the table's slot for that joint.  Exit 0 = table confirmed.
"""
import ast
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
BRIDGE = os.path.join(HERE, "..", "navigation", "scripts", "g1_joint_state_bridge.py")


def load_table():
    with open(BRIDGE) as f:
        tree = ast.parse(f.read())
    for node in tree.body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == "G1_MOTOR_INDEX":
            return ast.literal_eval(node.value)
    sys.exit("G1_MOTOR_INDEX not found")


def main():
    iface = sys.argv[1] if len(sys.argv) > 1 else ""
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    except ImportError as exc:
        sys.exit("unitree_sdk2py is not importable here: %s" % exc)
    ChannelFactoryInitialize(0, iface) if iface else ChannelFactoryInitialize(0)
    holder = {"low": None}
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(lambda m: holder.__setitem__("low", m), 10)
    t0 = time.time()
    while holder["low"] is None:
        if time.time() - t0 > 10:
            sys.exit("no rt/lowstate within 10 s: wrong interface or robot not on the network")
        time.sleep(0.05)
    n = len(holder["low"].motor_state)
    print("rt/lowstate has %d motor slots" % n)

    def record(seconds):
        lo, hi = [1e9] * n, [-1e9] * n
        end = time.time() + seconds
        while time.time() < end:
            ms = holder["low"].motor_state
            for i in range(n):
                q = float(ms[i].q)
                lo[i], hi[i] = min(lo[i], q), max(hi[i], q)
            time.sleep(0.01)
        return [h - l for l, h in zip(lo, hi)]

    table = load_table()
    ok = True
    input("\nHold the robot STILL, then press Enter (3 s noise floor)... ")
    noise = record(3.0)
    print("noise floor: max slot jitter %.4f rad" % max(noise))

    for label, joint in (("rotate the WAIST left/right ~30 deg", "waist_yaw_joint"),
                         ("bend the LEFT elbow", "left_elbow_joint"),
                         ("bend the RIGHT elbow", "right_elbow_joint")):
        input("\n%s, then press Enter and move for 5 s... " % label.capitalize())
        span = record(5.0)
        net = [s - nz for s, nz in zip(span, noise)]
        best = max(range(n), key=lambda i: net[i])
        exp = table[joint]
        good = best == exp and net[best] > 0.15
        ok &= good
        print("  %-18s table slot %2d | slot that moved most: %2d (%.2f rad) | %s"
              % (joint, exp, best, net[best], "PASS" if good else "FAIL"))
    print("\nTABLE CONFIRMED for waist + both elbows" if ok else
          "\nTABLE WRONG or nothing moved: do NOT use mode=dds; use mode=relay, and fix G1_MOTOR_INDEX")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
