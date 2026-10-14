"""Importing `arm_control` must not import unitree_sdk2py.

catkin generates `arm_control.srv` into the same package namespace, so anything
that only wants MoveArmJoints must not be forced to install the robot's DDS
SDK. These tests run the imports in SUBPROCESSES so a blocked module cannot
leak into sibling tests in the same interpreter.

No control logic, gain, joint index or timing is exercised or changed here;
this is purely about WHEN arm_controller is imported.

    cd arm_control && python3 -m unittest discover -s test -v
"""
import ast
import os
import subprocess
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "src")
INIT = os.path.join(SRC, "arm_control", "__init__.py")

#: The public API. Pinned so it cannot silently shrink when __init__ changes.
EXPECTED_ALL = [
    "ArmController",
    "Execute_Waypoint_Path",
    "Get_Measured",
    "JointIndex",
    "Mode",
    "Run_Arm_Controller",
    "Set_Target",
    "Wait_Until_Arrived",
    "left_target",
    "right_target",
]

#: The SDK the control loop needs but a srv consumer must not.
HEAVY = ("unitree_sdk2py",)


def run_snippet(body, blocked=HEAVY):
    """Run `body` in a fresh interpreter with `blocked` modules unimportable."""
    preamble = "import sys\n"
    for name in blocked:
        preamble += "sys.modules[%r] = None\n" % (name,)
    preamble += "sys.path.insert(0, %r)\n" % (SRC,)
    return subprocess.run(
        [sys.executable, "-c", preamble + body],
        capture_output=True, text=True, timeout=120,
    )


class ImportsWithoutTheRobotSdk(unittest.TestCase):
    def test_importing_the_package_succeeds(self):
        proc = run_snippet("import arm_control\nprint('IMPORTED')\n")
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertIn("IMPORTED", proc.stdout)

    def test_the_controller_module_is_not_pulled_in(self):
        proc = run_snippet(
            "import arm_control\n"
            "assert 'arm_control.arm_controller' not in sys.modules, 'eagerly imported'\n"
            "print('LAZY')\n"
        )
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertIn("LAZY", proc.stdout)

    def test_public_api_is_still_advertised(self):
        proc = run_snippet("import arm_control\nprint(','.join(arm_control.__all__))\n")
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertEqual(proc.stdout.strip().split(","), EXPECTED_ALL)

    def test_dir_still_lists_the_public_api(self):
        proc = run_snippet(
            "import arm_control\n"
            "names = dir(arm_control)\n"
            "missing = [n for n in arm_control.__all__ if n not in names]\n"
            "assert not missing, missing\n"
            "print('DIR_OK')\n"
        )
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertIn("DIR_OK", proc.stdout)

    def test_the_regression_this_guards_would_be_caught(self):
        """Importing the controller directly must still fail without the SDK."""
        proc = run_snippet("import arm_control.arm_controller\n")
        self.assertNotEqual(proc.returncode, 0, "blocking is not working; tests are vacuous")

    def test_attribute_access_still_reports_the_missing_sdk(self):
        """The failure is deferred, not hidden."""
        proc = run_snippet(
            "import arm_control\n"
            "try:\n"
            "    arm_control.JointIndex\n"
            "except ImportError as exc:\n"
            "    print('DEFERRED:%s' % exc)\n"
            "else:\n"
            "    print('UNEXPECTED_SUCCESS')\n"
        )
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertIn("DEFERRED:", proc.stdout)

    def test_unknown_attribute_raises_attributeerror_not_importerror(self):
        proc = run_snippet(
            "import arm_control\n"
            "try:\n"
            "    arm_control.no_such_name\n"
            "except AttributeError:\n"
            "    print('ATTRIBUTE_ERROR')\n"
        )
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertIn("ATTRIBUTE_ERROR", proc.stdout)


class InitSourceStaysLazy(unittest.TestCase):
    def test_no_module_level_import_of_the_controller(self):
        with open(INIT) as f:
            tree = ast.parse(f.read())
        for node in tree.body:
            if isinstance(node, ast.ImportFrom):
                self.assertNotIn(
                    "arm_controller", node.module or "",
                    "arm_control/__init__.py must not import arm_controller at module level",
                )
            if isinstance(node, ast.Import):
                for alias in node.names:
                    self.assertNotIn("arm_controller", alias.name)

    def test_module_getattr_is_defined(self):
        with open(INIT) as f:
            tree = ast.parse(f.read())
        names = [n.name for n in tree.body if isinstance(n, ast.FunctionDef)]
        self.assertIn("__getattr__", names)
        self.assertIn("__dir__", names)


class ControlBehaviourIsUnchanged(unittest.TestCase):
    """The controller source itself must not have been touched by this change."""

    def test_joint_order_is_still_the_specification_order(self):
        controller = os.path.join(SRC, "arm_control", "arm_controller.py")
        with open(controller) as f:
            source = f.read()
        start = source.index("left_joints = [")
        block = source[start:source.index("]", start)]
        expected = ["ShoulderPitch", "ShoulderRoll", "ShoulderYaw", "Elbow", "WristRoll"]
        self.assertEqual([s for s in expected if ("Left" + s) in block], expected)

    def test_controller_still_imports_the_sdk_itself(self):
        """The lazy shim must not have moved the SDK import into a function."""
        controller = os.path.join(SRC, "arm_control", "arm_controller.py")
        with open(controller) as f:
            tree = ast.parse(f.read())
        top_level = []
        for node in tree.body:
            if isinstance(node, ast.ImportFrom) and node.module:
                top_level.append(node.module)
        self.assertTrue(
            any("unitree_sdk2py" in m for m in top_level),
            "arm_controller.py should still import the SDK at module level",
        )


class WithTheRobotSdkAvailable(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        probe = subprocess.run(
            [sys.executable, "-c", "import numpy, unitree_sdk2py"],
            capture_output=True, text=True,
        )
        if probe.returncode != 0:
            raise unittest.SkipTest("numpy/unitree_sdk2py not installed here")

    def test_attribute_access_resolves_and_caches(self):
        proc = run_snippet(
            "import arm_control\n"
            "first = arm_control.JointIndex\n"
            "assert 'arm_control.arm_controller' in sys.modules\n"
            "assert arm_control.__dict__['JointIndex'] is first, 'not cached'\n"
            "print('RESOLVED')\n",
            blocked=(),
        )
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertIn("RESOLVED", proc.stdout)


if __name__ == "__main__":
    unittest.main()
