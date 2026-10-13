"""Frame contract (docs/FRAME_CONTRACT.md): base_link is an identity alias of pelvis with exactly one publisher.

    cd navigation/test && python3 -m unittest test_frame_contract -v      (no ROS, no robot)
"""
import glob
import importlib.util
import os
import re
import shutil
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "slam", "scripts"))
sys.path.insert(0, os.path.join(REPO, "slam", "src"))

import slam_health_check as shc  # noqa: E402


def _read(*p):
    with open(os.path.join(REPO, *p), encoding="utf-8", errors="replace") as fh:
        return fh.read()


def _tool(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(REPO, "tools", name + ".py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class TestAlias(unittest.TestCase):
    def test_alias_is_on_by_default_and_exactly_identity(self):
        text = re.sub(r"<!--.*?-->", "", _read("system_bringup", "launch", "navigation.launch"), flags=re.S)
        root = ET.fromstring(text)
        args = {a.get("name"): a.get("default") for a in root.iter("arg")}
        self.assertEqual(args["with_base_link"], "true")
        nodes = [n for n in root.iter("node") if n.get("type") == "static_transform_publisher"]
        self.assertEqual(len(nodes), 1)
        self.assertEqual(nodes[0].get("args").split(), ["0", "0", "0", "0", "0", "0", "pelvis", "base_link"])

    def test_g1_slam_never_publishes_base_link(self):
        for lf in glob.glob(os.path.join(REPO, "slam", "launch", "*.launch")):
            self.assertNotIn("base_link", re.sub(r"<!--.*?-->", "", _read(lf), flags=re.S), lf)
        cfg = _read("slam", "config", "g1_tf.yaml")
        self.assertRegex(cfg, r'(?m)^base_link_alias:\s*""\s*$')
        self.assertNotIn("base_link", _read("slam", "urdf", "g1_23dof_mode_10.urdf"))

    def test_docs_do_not_still_say_it_is_unresolved_or_off(self):
        for rel in (("CLAUDE.md",), ("slam", "README.md"), ("navigation", "README.md"),
                    ("slam", "config", "g1_tf.yaml"), ("system_bringup", "launch", "navigation.launch")):
            text = _read(*rel)
            self.assertNotRegex(text, r"(?i)base_link[^\n]{0,80}(unresolved|off by default)", rel)


class TestHealthCheckAllowsOnlyTheAlias(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.pkg = os.path.join(self.tmp.name, "slam")
        shutil.copytree(os.path.join(REPO, "slam"), self.pkg, ignore=shutil.ignore_patterns("__pycache__", "maps"))

    def tearDown(self):
        self.tmp.cleanup()

    def add_urdf_joint(self, parent):
        p = os.path.join(self.pkg, "urdf", "g1_23dof_mode_10.urdf")
        s = open(p).read()
        s = s.replace("</robot>", '<link name="base_link"/><joint name="bl" type="fixed"><parent link="%s"/>'
                      '<child link="base_link"/><origin xyz="0 0 0" rpy="0 0 0"/></joint></robot>' % parent)
        open(p, "w").write(s)

    def tf_status(self):
        r = shc.Results(True)
        shc.check_tf_single_parent(r, self.pkg, None)
        return r.records[0]

    def test_clean_tree_passes(self):
        self.assertEqual(self.tf_status()["status"], "PASS")

    def test_base_link_under_pelvis_is_accepted(self):
        self.add_urdf_joint("pelvis")
        self.assertEqual(self.tf_status()["status"], "PASS", self.tf_status())

    def test_base_link_under_anything_else_fails(self):
        self.add_urdf_joint("torso_link")
        rec = self.tf_status()
        self.assertEqual(rec["status"], "FAIL")
        self.assertIn("identity", rec["reason"])

    def test_non_empty_alias_in_g1_tf_yaml_is_rejected_with_a_readable_reason(self):
        p = os.path.join(self.pkg, "config", "g1_tf.yaml")
        s = re.sub(r'(?m)^base_link_alias:.*$', 'base_link_alias: "pelvis"', open(p).read())
        open(p, "w").write(s)
        r = shc.Results(True)
        shc.check_g1_tf_config(r, self.pkg, False)
        rec = r.records[-1]
        self.assertEqual(rec["status"], "FAIL")
        self.assertIn("system_bringup", rec["reason"])
        self.assertIn("'pelvis'", rec["reason"])


class TestUrdfEvidence(unittest.TestCase):
    def test_reported_offsets_match_the_contract_doc(self):
        """If someone fixes a URDF, this fails on purpose: update docs/FRAME_CONTRACT.md with the new numbers."""
        tool = _tool("check_urdf_agreement")
        doc = _read("docs", "FRAME_CONTRACT.md")
        for key, path in tool.URDFS.items():
            xyz, rpy, name, typ = tool.pelvis_to_torso(path)
            self.assertEqual((name, typ), ("waist_yaw_joint", "revolute"))
            self.assertIn("%.4f" % xyz[2], doc, "%s torso z %.4f is not in docs/FRAME_CONTRACT.md" % (key, xyz[2]))
            self.assertEqual(rpy, [0.0, 0.0, 0.0])

    def test_tool_runs_and_strict_mode_reflects_the_difference(self):
        tool = _tool("check_urdf_agreement")
        a = tool.pelvis_to_torso(tool.URDFS["slam"])[0][2]
        b = tool.pelvis_to_torso(tool.URDFS["arm_planning"])[0][2]
        self.assertEqual(tool.main([]), 0)                          # report only
        self.assertEqual(tool.main(["--strict"]), 0 if abs(a - b) <= 0.001 else 1)


if __name__ == "__main__":
    unittest.main()
