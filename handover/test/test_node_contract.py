"""The monitor detects and delegates. It must never actuate anything itself."""
import ast
import io
import os
import re
import tokenize
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
NODE = os.path.join(HERE, "..", "scripts", "auto_release_monitor.py")


def executable_source(path):
    """Source with comments and all docstrings removed.

    Needed because the module docstring legitimately names the services the
    node deliberately does not call.
    """
    with open(path) as f:
        raw = f.read()
    kept = [t for t in tokenize.generate_tokens(io.StringIO(raw).readline)
            if t.type != tokenize.COMMENT]
    stripped = tokenize.untokenize(kept)
    tree = ast.parse(stripped)
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
            doc = ast.get_docstring(node, clean=False)
            if doc:
                stripped = stripped.replace(doc, "")
    return stripped


class NeverActuates(unittest.TestCase):
    def setUp(self):
        self.code = executable_source(NODE)

    def test_code_never_references_a_hand_or_arm_endpoint(self):
        for forbidden in (
            "/hand_left", "/hand_right", "/hand_%s",            # hand_control services
            "move_arm_joints",                                 # arm execution
            "plan_to_pose", "plan_grasp_from_point",           # arm planning
            "arm_sdk", "lowcmd",                               # Unitree DDS command topics
            "cmd_vel", "enable_motion",                        # locomotion
            "/navigation/",                                    # navigation
        ):
            self.assertNotIn(
                forbidden, self.code,
                "auto_release_monitor.py must not reference %r; the orchestrator actuates"
                % forbidden,
            )

    def test_the_only_service_client_is_the_orchestrator_route(self):
        proxies = sorted(set(re.findall(r"rospy\.ServiceProxy\(\s*([A-Za-z_\.\"/]+)", self.code)))
        self.assertEqual(proxies, ["RELEASE_ROUTE_SERVICE"])
        constant = re.search(r'RELEASE_ROUTE_SERVICE\s*=\s*"([^"]+)"', self.code)
        self.assertEqual(constant.group(1), "/orchestrator/run_auto_release")

    def test_only_arm_and_disarm_are_advertised(self):
        """Four names, but only two operations: legacy plus side-qualified."""
        advertised = sorted(set(
            re.findall(r'rospy\.Service\(\s*"([^"]+)"', self.code)
            + re.findall(r'rospy\.Service\(\s*"([^"]+)"\s*%', self.code)))
        for name in advertised:
            self.assertTrue(
                name.startswith("/auto_release_monitor/"),
                "unexpected service %r" % name)
            self.assertTrue(
                name.endswith(("/arm", "/disarm")),
                "the monitor may only advertise arm and disarm, got %r" % name)
        self.assertIn("/auto_release_monitor/arm", advertised)
        self.assertIn("/auto_release_monitor/disarm", advertised)

    def test_publishes_only_diagnostics(self):
        published = sorted(set(re.findall(r'rospy\.Publisher\(\s*"([^"]+)"', self.code)))
        self.assertEqual(published, [
            "/auto_release_monitor/event",
            "/auto_release_monitor/ready",
            "/auto_release_monitor/state",
        ])

    def test_orchestrator_service_type_is_imported_lazily(self):
        """A top-level import would stop the node starting before orchestrator is built."""
        tree = ast.parse(self.code)
        top_level = []
        for node in tree.body:
            if isinstance(node, ast.ImportFrom) and node.module:
                top_level.append(node.module)
        self.assertNotIn("orchestrator.srv", top_level)
        self.assertIn("orchestrator.srv", self.code, "it should still be imported, just lazily")


if __name__ == "__main__":
    unittest.main()
