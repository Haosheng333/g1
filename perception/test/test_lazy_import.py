"""Importing `perception` must not import ultralytics, torch or the YOLO weights.

catkin generates `perception.srv` into the same package namespace, so anything
that only wants the service type must not be forced to install the detector
stack. These tests run the imports in SUBPROCESSES so a blocked module cannot
leak into sibling tests in the same interpreter.

    cd perception && python3 -m unittest discover -s test -v
"""
import ast
import os
import subprocess
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "src")
INIT = os.path.join(SRC, "perception", "__init__.py")

#: The public API. Pinned so it cannot silently shrink when __init__ changes.
EXPECTED_ALL = [
    "CameraIntrinsics",
    "Detection",
    "Extrinsics",
    "ObjectPositionResult",
    "backproject_pixel",
    "bbox_center",
    "handle_get_object_position",
    "median_nonzero_depth",
    "optical_to_base",
    "push_along_ray",
    "run_detection_pipeline",
]

#: Third-party modules the algorithm needs but a srv consumer must not.
HEAVY = ("ultralytics", "torch")


def run_snippet(body, blocked=HEAVY):
    """Run `body` in a fresh interpreter with `blocked` modules unimportable."""
    preamble = "import sys\n"
    for name in blocked:
        # None in sys.modules makes `import name` raise ImportError.
        preamble += "sys.modules[%r] = None\n" % (name,)
    preamble += "sys.path.insert(0, %r)\n" % (SRC,)
    proc = subprocess.run(
        [sys.executable, "-c", preamble + body],
        capture_output=True, text=True, timeout=120,
    )
    return proc


class ImportsWithoutTheDetectorStack(unittest.TestCase):
    def test_importing_the_package_succeeds(self):
        proc = run_snippet("import perception\nprint('IMPORTED')\n")
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertIn("IMPORTED", proc.stdout)

    def test_the_algorithm_module_is_not_pulled_in(self):
        proc = run_snippet(
            "import perception\n"
            "assert 'perception.m1_perception' not in sys.modules, 'eagerly imported'\n"
            "assert 'ultralytics' not in [m for m in sys.modules if sys.modules[m] is not None]\n"
            "print('LAZY')\n"
        )
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertIn("LAZY", proc.stdout)

    def test_public_api_is_still_advertised(self):
        proc = run_snippet(
            "import perception\n"
            "print(','.join(perception.__all__))\n"
        )
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertEqual(proc.stdout.strip().split(","), EXPECTED_ALL)

    def test_dir_still_lists_the_public_api(self):
        proc = run_snippet(
            "import perception\n"
            "names = dir(perception)\n"
            "missing = [n for n in perception.__all__ if n not in names]\n"
            "assert not missing, missing\n"
            "print('DIR_OK')\n"
        )
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertIn("DIR_OK", proc.stdout)

    def test_the_regression_this_guards_would_be_caught(self):
        """Importing the algorithm module directly must still fail without the deps.

        If this ever passes, the blocking in run_snippet stopped working and the
        tests above would be vacuous.
        """
        proc = run_snippet("import perception.m1_perception\n")
        self.assertNotEqual(proc.returncode, 0, "blocking is not working; tests are vacuous")

    def test_attribute_access_still_reports_the_missing_dependency(self):
        """The failure is deferred, not hidden: using the API still errors clearly."""
        proc = run_snippet(
            "import perception\n"
            "try:\n"
            "    perception.Extrinsics\n"
            "except ImportError as exc:\n"
            "    print('DEFERRED:%s' % exc)\n"
            "else:\n"
            "    print('UNEXPECTED_SUCCESS')\n"
        )
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertIn("DEFERRED:", proc.stdout)

    def test_unknown_attribute_raises_attributeerror_not_importerror(self):
        proc = run_snippet(
            "import perception\n"
            "try:\n"
            "    perception.no_such_name\n"
            "except AttributeError:\n"
            "    print('ATTRIBUTE_ERROR')\n"
        )
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertIn("ATTRIBUTE_ERROR", proc.stdout)


class InitSourceStaysLazy(unittest.TestCase):
    """Static guard: no top-level import of the algorithm module."""

    def test_no_module_level_import_of_the_algorithm(self):
        with open(INIT) as f:
            tree = ast.parse(f.read())
        for node in tree.body:
            if isinstance(node, ast.ImportFrom):
                self.assertNotIn(
                    "m1_perception", node.module or "",
                    "perception/__init__.py must not import m1_perception at module level",
                )
            if isinstance(node, ast.Import):
                for alias in node.names:
                    self.assertNotIn("m1_perception", alias.name)

    def test_module_getattr_is_defined(self):
        with open(INIT) as f:
            tree = ast.parse(f.read())
        names = [n.name for n in tree.body if isinstance(n, ast.FunctionDef)]
        self.assertIn("__getattr__", names)
        self.assertIn("__dir__", names)


class WithTheDetectorStackAvailable(unittest.TestCase):
    """When the deps ARE installed, the public API must resolve as before."""

    @classmethod
    def setUpClass(cls):
        probe = subprocess.run(
            [sys.executable, "-c", "import numpy, ultralytics"],
            capture_output=True, text=True,
        )
        if probe.returncode != 0:
            raise unittest.SkipTest("numpy/ultralytics not installed here")

    def test_attribute_access_resolves_and_caches(self):
        proc = run_snippet(
            "import perception\n"
            "first = perception.Extrinsics\n"
            "assert 'perception.m1_perception' in sys.modules\n"
            "assert perception.__dict__['Extrinsics'] is first, 'not cached'\n"
            "print('RESOLVED')\n",
            blocked=(),
        )
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertIn("RESOLVED", proc.stdout)

    def test_star_import_still_works(self):
        proc = run_snippet(
            "ns = {}\n"
            "exec('from perception import *', ns)\n"
            "import perception\n"
            "missing = [n for n in perception.__all__ if n not in ns]\n"
            "assert not missing, missing\n"
            "print('STAR_OK')\n",
            blocked=(),
        )
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertIn("STAR_OK", proc.stdout)


if __name__ == "__main__":
    unittest.main()
