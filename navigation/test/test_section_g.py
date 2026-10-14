"""Section G conformance (items 27-29 + "not defined in the document"), and cross-module ownership.

    cd navigation/test && python3 -m unittest test_section_g -v

27  ONE navigation stack          no AMCL / move_base / DWAPlannerROS / rtabmap anywhere in the launches or packages
28  ONE publisher per TF frame    the TF tree is map -> camera_init -> pelvis (+ optional base_link); no odom
29  acceptance is testable        a simulator with real obstacles + tools/nav_acceptance_test.py exist
--  joint states are not faked    the bridge refuses zeros unless asked; package.xml / CMakeLists / srv / rviz exist
Ownership (alignment with other modules): other packages must not publish what navigation owns, and the
interface the orchestrator calls must stay exactly as documented.  If a teammate adds a conflicting publisher,
this test fails in CI instead of the robot misbehaving.
"""
import glob
import os
import re
import unittest
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
OWN = ("navigation", "slam", "system_bringup")
OTHERS = ("arm_control", "arm_planning", "hand_control", "handover", "orchestrator", "perception")


def _slurp(path):
    with open(path, encoding="utf-8", errors="replace") as fh:
        return fh.read()


def read(*p):
    return _slurp(os.path.join(REPO, *p))


def strip_comments(text):
    return re.sub(r"<!--.*?-->", "", text, flags=re.S)


def launch_files():
    out = []
    for pkg in OWN:
        out += glob.glob(os.path.join(REPO, pkg, "**", "*.launch"), recursive=True)
    return [f for f in out if "g1_ws/src/fast_lio" not in f and os.path.getsize(f) > 0]


def py_files(pkgs):
    out = []
    for pkg in pkgs:
        out += glob.glob(os.path.join(REPO, pkg, "**", "*.py"), recursive=True)
    return [f for f in out if "/test" not in f and "g1_ws/src/fast_lio" not in f]


class Item27OneStack(unittest.TestCase):
    def test_no_second_navigation_stack(self):
        for lf in launch_files():
            text = strip_comments(_slurp(lf))
            for pkg_name in ("amcl", "move_base", "rtabmap_ros", "dwa_local_planner"):
                self.assertNotRegex(text, r'pkg="%s"' % pkg_name, lf)
            self.assertNotIn("DWAPlannerROS", text, lf)

    def test_exactly_one_navigation_node(self):
        nodes = []
        for lf in launch_files():
            root = ET.fromstring(strip_comments(_slurp(lf)))
            nodes += [(os.path.basename(lf), n.get("type")) for n in root.iter("node")
                      if n.get("type") == "navigation_node.py"]
        self.assertEqual(nodes, [("navigation.launch", "navigation_node.py")])   # sim.launch and system_bringup include it

    def test_move_base_goal_topic_is_only_the_rviz_click(self):
        src = read("navigation", "scripts", "navigation_node.py")
        self.assertIn("/move_base_simple/goal", src)
        self.assertNotIn("/move_base/", src)


class Item28OneTfPublisher(unittest.TestCase):
    def test_static_tf_publishers_are_only_the_optional_base_link(self):
        found = []
        for lf in launch_files():
            root = ET.fromstring(strip_comments(_slurp(lf)))
            for n in root.iter("node"):
                if n.get("type") == "static_transform_publisher":
                    found.append((os.path.basename(lf), (n.get("args") or "").split()[-2:], n.get("if")))
        base = [f for f in found if f[1] and f[1][-1] == "base_link"]
        self.assertEqual(len(base), 1, found)
        self.assertEqual(base[0][1], ["pelvis", "base_link"])
        self.assertEqual(base[0][2], "$(arg with_base_link)")          # guarded, off by default
        for f in found:
            self.assertNotIn("odom", " ".join(f[1]), f)                # no odom frame at all

    def test_no_odom_frame_in_navigation_code(self):
        for f in py_files(("navigation",)):
            for i, line in enumerate(_slurp(f).splitlines(), 1):
                code = line.split("#")[0]
                self.assertNotRegex(code, r"""["']odom["']""", "%s:%d" % (f, i))

    def test_pelvis_is_the_robot_frame(self):
        cfg = read("navigation", "config", "navigation.yaml")
        self.assertRegex(cfg, r"robot_frame:\s*pelvis")
        self.assertRegex(cfg, r"global_frame:\s*map")


class Item29Testable(unittest.TestCase):
    def test_simulator_with_obstacles_and_acceptance_script(self):
        self.assertTrue(os.path.isfile(os.path.join(REPO, "navigation", "scripts", "sim_world.py")))
        self.assertTrue(os.path.isfile(os.path.join(REPO, "navigation", "sim", "sim_room", "map.pgm")))
        self.assertTrue(os.path.isfile(os.path.join(REPO, "tools", "nav_acceptance_test.py")))
        self.assertIn("ground-truth collisions", read("tools", "nav_acceptance_test.py"))


class NotDefinedInDocument(unittest.TestCase):
    def test_joint_bridge_never_fabricates_silently(self):
        src = read("navigation", "scripts", "g1_joint_state_bridge.py")
        self.assertIn("allow_fake_zeros", src)
        self.assertEqual(src.count("[0.0] * len("), 1)                 # zeros exist only in run_fake()
        self.assertLess(src.index("def run_fake"), src.index("[0.0] * len("))

    def test_package_files_exist(self):
        for rel in (("navigation", "package.xml"), ("navigation", "CMakeLists.txt"),
                    ("navigation", "srv", "NavigateToObject.srv"), ("navigation", "rviz", "navigation.rviz"),
                    ("system_bringup", "package.xml"), ("system_bringup", "CMakeLists.txt")):
            self.assertGreater(os.path.getsize(os.path.join(REPO, *rel)), 0, rel)


class Ownership(unittest.TestCase):
    """Alignment with the other modules."""
    OWNED_TOPICS = ("/locomotion/cmd_vel", "/navigation/", "/joint_states", "/tf", "/tf_static", "/map",
                    "/slam/", "/obstacle_cloud")
    #: navigation's INPUTS: clients publish these, navigation subscribes (documented in navigation_node.py)
    CLIENT_INPUTS = ("/navigation/cancel",)

    def test_other_modules_do_not_publish_what_navigation_owns(self):
        bad = []
        for f in py_files(OTHERS):
            if os.path.getsize(f) == 0:
                continue
            text = _slurp(f)
            for m in re.finditer(r"Publisher\(\s*[\"']([^\"']+)[\"']", text):
                if m.group(1) not in self.CLIENT_INPUTS and any(m.group(1).startswith(t) for t in self.OWNED_TOPICS):
                    bad.append("%s publishes %s" % (os.path.relpath(f, REPO), m.group(1)))
            if re.search(r"TransformBroadcaster|StaticTransformBroadcaster", text):
                bad.append("%s broadcasts TF" % os.path.relpath(f, REPO))
        self.assertEqual(bad, [], "\n" + "\n".join(bad))

    def test_only_one_publisher_of_each_robot_topic_inside_navigation(self):
        owners = {"/locomotion/cmd_vel": {"cmd_vel_gate.py"}, "/joint_states": {"g1_joint_state_bridge.py"},
                  "/navigation/cmd_vel": {"navigation_node.py"}}
        seen = {t: set() for t in owners}
        for f in py_files(("navigation",)):
            text = _slurp(f)
            for m in re.finditer(r"Publisher\(\s*(?:p\(\s*[\"']~\w+[\"']\s*,\s*)?[\"'](/[^\"']+)[\"']", text):
                if m.group(1) in seen:
                    seen[m.group(1)].add(os.path.basename(f))
        self.assertEqual(seen, owners)

    def test_orchestrator_contract_is_unchanged(self):
        srv = read("navigation", "srv", "NavigateToObject.srv")
        self.assertRegex(srv, r"string class_name\s+float32 approach_distance\s+---\s+bool success\s+string message")
        node = read("navigation", "scripts", "navigation_node.py")
        self.assertIn("/navigation/navigate", node)
        self.assertIn("/navigation/status", node)
        self.assertRegex(node, r'Subscriber\("/navigation/cancel", Empty')      # the orchestrator publishes this
        self.assertRegex(node, r"req\.approach_distance if req\.approach_distance > 0 else")   # 0.0 = use the default
        self.assertRegex(node, r'PLANNING, FOLLOWING, RECOVERING, SUCCESS, FAILURE = \(\s*"PLANNING", "FOLLOWING", '
                               r'"RECOVERING", "SUCCESS", "FAILURE"\)')

    def test_dds_nodes_use_the_same_domain_as_arm_control(self):
        arm = read("arm_control", "src", "arm_control", "arm_controller.py")
        self.assertRegex(arm, r"ChannelFactoryInitialize\(0")
        for rel in (("navigation", "scripts", "g1_joint_state_bridge.py"),
                    ("navigation", "scripts", "locomotion_bridge.py")):
            self.assertRegex(read(*rel), r"ChannelFactoryInitialize\(0")


if __name__ == "__main__":
    unittest.main()
