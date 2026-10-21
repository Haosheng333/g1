"""Repository-level contract for the optional voice-control package."""

from __future__ import annotations

import os
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class TestVoiceControlRepositoryIntegration(unittest.TestCase):
    def test_package_is_visible_from_catkin_workspace(self):
        link = os.path.join(REPO, "g1_ws", "src", "g1_voice_control")
        self.assertTrue(os.path.islink(link), "g1_ws/src/g1_voice_control must be a symlink")
        self.assertEqual(os.readlink(link), "../../voice_control")
        self.assertTrue(os.path.isfile(os.path.join(link, "package.xml")))

    def test_full_system_keeps_voice_disabled_by_default(self):
        launch = ET.parse(os.path.join(REPO, "system_bringup", "launch", "full_system.launch"))
        root = launch.getroot()
        args = {node.get("name"): node.get("default") for node in root.findall("arg")}
        self.assertEqual(args.get("voice_control"), "false")
        includes = [node for node in root.findall("include") if "g1_voice_control" in node.get("file", "")]
        self.assertEqual(len(includes), 1)
        self.assertEqual(includes[0].get("if"), "$(arg voice_control)")

    def test_system_bringup_declares_voice_runtime_dependency(self):
        package = ET.parse(os.path.join(REPO, "system_bringup", "package.xml"))
        dependencies = {node.text.strip() for node in package.getroot().findall("exec_depend")}
        self.assertIn("g1_voice_control", dependencies)

    def test_offline_workflow_runs_voice_tests(self):
        workflow = Path(REPO, ".github", "workflows", "offline-tests.yml").read_text(
            encoding="utf-8"
        )
        self.assertIn("voice_control/src", workflow)
        self.assertIn("working-directory: voice_control", workflow)


if __name__ == "__main__":
    unittest.main()
