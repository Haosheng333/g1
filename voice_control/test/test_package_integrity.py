import os
import unittest
import xml.etree.ElementTree as ET


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class TestPackageIntegrity(unittest.TestCase):
    def test_launch_files_are_well_formed_xml(self):
        ET.parse(os.path.join(PACKAGE_ROOT, "launch", "voice_control.launch"))

    def test_ros_transport_test_is_executable(self):
        path = os.path.join(PACKAGE_ROOT, "test", "test_ros_transport.py")
        self.assertTrue(os.access(path, os.X_OK), f"rostest node is not executable: {path}")

    def test_model_download_helper_is_installed(self):
        path = os.path.join(PACKAGE_ROOT, "scripts", "download_model.sh")
        self.assertTrue(os.access(path, os.X_OK), f"model helper is not executable: {path}")
        with open(os.path.join(PACKAGE_ROOT, "CMakeLists.txt"), encoding="utf-8") as stream:
            cmake = stream.read()
        self.assertIn("scripts/download_model.sh", cmake)


if __name__ == "__main__":
    unittest.main()
