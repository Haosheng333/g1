"""Static checks of every launch file in this repository (no ROS needed).

    cd navigation/test && python3 -m unittest test_launch_files -v

Catches what only shows up at roslaunch time: XML that is not well-formed (e.g. `--` inside a comment),
$(arg x) never declared, $(find pkg)/file that does not exist, <node type=...> with no such script,
<include> passing an arg the included file does not declare, and nodes missing from CMake install.
Packages outside this repo (livox_ros_driver2, realsense2_camera, ...) are skipped.
"""
import glob
import os
import re
import unittest
import xml.etree.ElementTree as ET

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
# ROS package name -> directory. Two packages are named differently from their
# directory (g1_slam/slam, g1_navigation/navigation); the rest match.
PKGS = {
    "g1_slam": "slam",
    "g1_navigation": "navigation",
    "system_bringup": "system_bringup",
    "perception": "perception",
    "arm_planning": "arm_planning",
    "arm_control": "arm_control",
    "hand_control": "hand_control",
    "handover": "handover",
    "orchestrator": "orchestrator",
}
LAUNCHES = sorted(
    lf for d in sorted(set(PKGS.values()))
    for lf in glob.glob(os.path.join(REPO, d, "launch", "*.launch"))
)


def _pkg_path(pkg, rel=""):
    return os.path.join(REPO, PKGS[pkg], rel)


class TestLaunchFiles(unittest.TestCase):
    def test_found_launch_files(self):
        self.assertGreaterEqual(len(LAUNCHES), 10)

    def test_every_launch_file(self):
        problems = []
        for lf in LAUNCHES:
            name = os.path.relpath(lf, REPO)
            if os.path.getsize(lf) == 0:
                continue                                  # placeholder owned by another team
            try:
                root = ET.parse(lf).getroot()
            except ET.ParseError as exc:
                problems.append("%s: not well-formed XML (%s)" % (name, exc))
                continue
            text = open(lf).read()
            declared = {a.get("name") for a in root.iter("arg") if a.get("name")}
            for m in re.finditer(r"\$\(arg (\w+)\)", text):
                if m.group(1) not in declared:
                    problems.append("%s: $(arg %s) is never declared" % (name, m.group(1)))
            for m in re.finditer(r"\$\(find (\w+)\)/([^\"\s)$]*)", text):
                pkg, rel = m.groups()
                if pkg in PKGS and rel and not os.path.exists(_pkg_path(pkg, rel)):
                    problems.append("%s: $(find %s)/%s does not exist" % (name, pkg, rel))
            for node in root.iter("node"):
                pkg, typ = node.get("pkg"), node.get("type")
                if pkg in PKGS and not os.path.isfile(_pkg_path(pkg, os.path.join("scripts", typ))):
                    if not os.path.isfile(_pkg_path(pkg, os.path.join("src", typ))):
                        problems.append("%s: node %s/%s has no script" % (name, pkg, typ))
            for inc in root.iter("include"):
                m = re.match(r"\$\(find (\w+)\)/(.*)", inc.get("file") or "")
                if not m or m.group(1) not in PKGS:
                    continue
                target = _pkg_path(m.group(1), m.group(2))
                if not os.path.isfile(target):
                    problems.append("%s: includes missing %s" % (name, m.group(2)))
                    continue
                if os.path.getsize(target) == 0:
                    continue
                sub_args = {a.get("name") for a in ET.parse(target).getroot().iter("arg")}
                for a in inc.findall("arg"):
                    if a.get("name") not in sub_args:
                        problems.append("%s: passes arg %s that %s does not declare"
                                        % (name, a.get("name"), m.group(2)))
        self.assertEqual(problems, [], "\n" + "\n".join(problems))

    def test_scripts_are_installed_by_cmake(self):
        missing = []
        for pkg in ("navigation", "slam"):
            cmake = open(os.path.join(REPO, pkg, "CMakeLists.txt")).read()
            for script in glob.glob(os.path.join(REPO, pkg, "scripts", "*.py")):
                if "scripts/" + os.path.basename(script) not in cmake:
                    missing.append("%s/scripts/%s" % (pkg, os.path.basename(script)))
        self.assertEqual(missing, [])

    def test_no_move_base_or_duplicate_tf_in_navigation_launches(self):
        for lf in glob.glob(os.path.join(REPO, "navigation", "launch", "*.launch")) + \
                [os.path.join(REPO, "system_bringup", "launch", "navigation.launch")]:
            text = re.sub(r"<!--.*?-->", "", open(lf).read(), flags=re.S)
            for bad in ("move_base", "amcl", "rtabmap"):
                self.assertNotIn(bad, text.replace("move_base_simple", ""), lf)


if __name__ == "__main__":
    unittest.main()
