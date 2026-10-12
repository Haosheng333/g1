"""Offline consistency check of the DDS motor table in g1_joint_state_bridge.py (no ROS, no SDK).

    cd navigation/test && python3 -m unittest test_joint_bridge_table -v

This proves the table is internally consistent with the vendored 23-DoF URDF and with the documented
unitree_sdk2py G1JointIndex layout.  It CANNOT prove the robot's firmware uses that layout: run
tools/verify_joint_bridge.py on the real robot for that.
"""
import ast
import os
import unittest
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
BRIDGE = os.path.join(HERE, "..", "scripts", "g1_joint_state_bridge.py")
URDF = os.path.join(HERE, "..", "..", "slam", "urdf", "g1_23dof_mode_10.urdf")

# unitree_sdk2py.g1 G1JointIndex (29-DoF slot layout; the 23-DoF robot leaves some slots unused)
SDK_LAYOUT = {
    "left_hip_pitch": 0, "left_hip_roll": 1, "left_hip_yaw": 2, "left_knee": 3, "left_ankle_pitch": 4,
    "left_ankle_roll": 5, "right_hip_pitch": 6, "right_hip_roll": 7, "right_hip_yaw": 8, "right_knee": 9,
    "right_ankle_pitch": 10, "right_ankle_roll": 11, "waist_yaw": 12, "left_shoulder_pitch": 15,
    "left_shoulder_roll": 16, "left_shoulder_yaw": 17, "left_elbow": 18, "left_wrist_roll": 19,
    "right_shoulder_pitch": 22, "right_shoulder_roll": 23, "right_shoulder_yaw": 24, "right_elbow": 25,
    "right_wrist_roll": 26,
}


def bridge_table():
    with open(BRIDGE) as f:
        tree = ast.parse(f.read())
    for node in tree.body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == "G1_MOTOR_INDEX":
            return ast.literal_eval(node.value)
    raise AssertionError("G1_MOTOR_INDEX not found")


class TestMotorTable(unittest.TestCase):
    def test_table_covers_exactly_the_urdf_revolute_joints(self):
        root = ET.parse(URDF).getroot()
        urdf = {j.get("name") for j in root.iter("joint") if j.get("type") == "revolute"}
        self.assertEqual(set(bridge_table()), urdf)

    def test_indices_are_unique_and_in_lowstate_range(self):
        idx = list(bridge_table().values())
        self.assertEqual(len(idx), len(set(idx)))
        self.assertTrue(all(0 <= i < 29 for i in idx))

    def test_matches_documented_sdk_layout(self):
        table = {k.replace("_joint", ""): v for k, v in bridge_table().items()}
        self.assertEqual(table, SDK_LAYOUT)

    def test_waist_yaw_is_slot_12(self):
        self.assertEqual(bridge_table()["waist_yaw_joint"], 12)


if __name__ == "__main__":
    unittest.main()
