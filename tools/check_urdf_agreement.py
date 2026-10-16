#!/usr/bin/env python3
"""Do the two vendored G1 URDFs agree on where torso_link sits relative to pelvis?  No ROS, no robot.

    python3 tools/check_urdf_agreement.py           # prints both numbers
    python3 tools/check_urdf_agreement.py --strict  # exit 1 if they differ by more than --tol (default 0.001 m)

slam/urdf/g1_23dof_mode_10.urdf        feeds robot_state_publisher, so TF  pelvis -> torso_link  (and the Mid-360 pose)
arm_planning/urdf/g1_23dof.urdf        feeds the arm planner, whose base frame is torso_link
At zero waist yaw the translation pelvis -> torso_link is the origin of waist_yaw_joint.  A disagreement here is a
systematic error between where the LiDAR/TF thinks the torso is and where the arm planner thinks it is.
Which one is right is decided by MEASURING the robot (see docs/FRAME_CONTRACT.md); this tool only reports.
"""
import argparse
import os
import sys
import xml.etree.ElementTree as ET

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
URDFS = {"slam": os.path.join(REPO, "slam", "urdf", "g1_23dof_mode_10.urdf"),
         "arm_planning": os.path.join(REPO, "arm_planning", "urdf", "g1_23dof.urdf")}


def pelvis_to_torso(path):
    """-> (xyz, rpy) of the joint whose parent is pelvis and child is torso_link."""
    for j in ET.parse(path).getroot().iter("joint"):
        if j.find("parent").get("link") == "pelvis" and j.find("child").get("link") == "torso_link":
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            return xyz, rpy, j.get("name"), j.get("type")
    raise ValueError("%s has no pelvis -> torso_link joint" % path)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--strict", action="store_true")
    ap.add_argument("--tol", type=float, default=0.001)
    args = ap.parse_args(argv)
    got = {k: pelvis_to_torso(p) for k, p in URDFS.items() if os.path.isfile(p)}
    for k, (xyz, rpy, name, typ) in got.items():
        print("%-13s %s (%s): pelvis -> torso_link at zero waist yaw = (%.4f, %.4f, %.4f) rpy=%s" % (k, name, typ, *xyz, rpy))
    if len(got) < 2:
        print("[SKIP] only %d URDF found" % len(got))
        return 0
    (s_xyz, _, _, _), (p_xyz, _, _, _) = got["slam"], got["arm_planning"]
    diff = [pv - sv for sv, pv in zip(s_xyz, p_xyz)]
    worst = max(abs(d) for d in diff)
    ok = worst <= args.tol
    print("[%s] arm_planning minus slam: dx=%.4f dy=%.4f dz=%.4f m (largest %.4f m, tolerance %.4f m)"
          % ("PASS" if ok else "DIFFER", *diff, worst, args.tol))
    if not ok:
        print("       -> measure the real robot, then correct the wrong URDF (docs/FRAME_CONTRACT.md, 'torso_link').")
    return 1 if (args.strict and not ok) else 0


if __name__ == "__main__":
    sys.exit(main())
