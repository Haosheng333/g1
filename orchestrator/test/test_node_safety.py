"""Node-level safety: dry-run by default, zero motion when blocked, no robotics here.

rospy and the generated srv module are stubbed, so this runs with plain python3,
no ROS master and no catkin build.
"""
import ast
import io
import os
import re
import sys
import tokenize
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
# scripts/ AFTER src/, so the orchestrator package always wins over any
# same-named module that may sit in scripts/.
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

NODE_SOURCE = os.path.join(HERE, "..", "scripts", "orchestrator_node.py")
PKG_SRC = os.path.join(HERE, "..", "src", "orchestrator")


# --------------------------------------------------------------------- stubs
class RunRouteResponse(object):
    _defaults = {
        "success": False, "message": "", "steps_completed": [], "steps_skipped": [],
        "blockers": [], "final_step": "", "motion_issued": False, "dry_run": False,
        "object_state": "NONE", "transfer_verified": False,
    }

    def __init__(self, **kw):
        for name, default in self._defaults.items():
            setattr(self, name, list(default) if isinstance(default, list) else default)
        for name, value in kw.items():
            if name not in self._defaults:
                raise TypeError("unknown response field %r" % (name,))
            setattr(self, name, value)


def _install_stubs():
    params = {}

    rospy = types.ModuleType("rospy")
    rospy.ROSException = type("ROSException", (Exception,), {})
    rospy.ServiceException = type("ServiceException", (Exception,), {})
    rospy.get_param = lambda name, default=None: params.get(name, default)
    rospy.set_param = lambda name, value: params.__setitem__(name, value)
    rospy.init_node = lambda *a, **k: None
    rospy.spin = lambda *a, **k: None
    rospy.Service = lambda *a, **k: None
    rospy.Subscriber = lambda *a, **k: None
    rospy.Publisher = lambda *a, **k: types.SimpleNamespace(publish=lambda *_a, **_k: None)
    def _raising_proxy(*a, **k):
        raise AssertionError(
            "this test must not construct a ServiceProxy (%r)" % (a,))
    rospy.ServiceProxy = _raising_proxy
    rospy._raising_proxy = _raising_proxy
    rospy.wait_for_service = lambda *a, **k: None
    rospy.wait_for_message = lambda *a, **k: None
    rospy.is_shutdown = lambda: False
    rospy.Rate = lambda hz: types.SimpleNamespace(sleep=lambda: None)
    rospy.Duration = lambda s: s
    rospy.get_master = lambda: types.SimpleNamespace(getPublishedTopics=lambda p: [1, 2, []])
    rospy.Time = types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_sec=lambda: 0.0))
    for lvl in ("loginfo", "logwarn", "logerr", "logfatal", "logwarn_throttle",
                "logerr_throttle", "loginfo_throttle"):
        setattr(rospy, lvl, lambda *a, **k: None)
    rospy._params = params
    sys.modules["rospy"] = rospy

    for pkg, names in (
        ("std_msgs", ("Bool", "String", "Empty")),
        ("sensor_msgs", ("JointState",)),
    ):
        mod = types.ModuleType(pkg)
        msg = types.ModuleType("%s.msg" % pkg)
        for name in names:
            setattr(msg, name, lambda data=None, **k: types.SimpleNamespace(data=data))
        mod.msg = msg
        sys.modules[pkg] = mod
        sys.modules["%s.msg" % pkg] = msg

    srvs = types.ModuleType("std_srvs")
    srv = types.ModuleType("std_srvs.srv")
    srv.SetBool = object
    srv.Trigger = object
    srvs.srv = srv
    sys.modules["std_srvs"] = srvs
    sys.modules["std_srvs.srv"] = srv

    osrv = types.ModuleType("orchestrator.srv")
    osrv.RunRoute = object
    osrv.RunRouteResponse = RunRouteResponse
    sys.modules["orchestrator.srv"] = osrv
    return rospy


ROSPY = _install_stubs()

import orchestrator_node as on  # noqa: E402


class StubbedRospy(unittest.TestCase):
    """Re-assert THIS module's rospy stub.

    Sibling test files install their own rospy stub at import time, and whichever
    is imported last wins for the whole process. So every test here reinstates
    the stub it needs instead of trusting module import order.
    """

    def setUp(self):
        self._saved = sys.modules.get("rospy")
        sys.modules["rospy"] = ROSPY
        on.rospy = ROSPY
        ROSPY.ServiceProxy = ROSPY._raising_proxy

    def tearDown(self):
        on.pf.run_preflight = ORIGINAL_RUN_PREFLIGHT
        if self._saved is not None:
            sys.modules["rospy"] = self._saved

from orchestrator import routes as r  # noqa: E402

#: Captured BEFORE any test patches it, so restoring cannot resurrect a stub.
ORIGINAL_RUN_PREFLIGHT = on.pf.run_preflight


class PreflightPatch(StubbedRospy):
    """Base class that restores both the rospy stub and the real preflight."""


def request(route="", target="table", start_step="", dry_run=False, event_token="",
            destination="", label=""):
    return types.SimpleNamespace(
        route=route, target=target, destination=destination, label=label,
        start_step=start_step, dry_run=dry_run, event_token=event_token,
    )


def node(enable_motion=False, blockers=None, side="right"):
    """A node with preflight stubbed to return `blockers`."""
    ROSPY._params.clear()
    ROSPY._params["~enable_motion"] = enable_motion
    ROSPY._params["~side"] = side
    instance = on.OrchestratorNode()

    import orchestrator.preflight as pf

    result = pf.PreflightResult(checks=[
        pf.Check("cap%d" % i, "owner", False, text) for i, text in enumerate(blockers or [])
    ])
    instance.probe = object()
    on.pf.run_preflight = lambda route, probe, side: result
    return instance


class DryRunIsTheDefault(PreflightPatch):
    def test_motion_disabled_forces_a_dry_run_even_when_not_requested(self):
        n = node(enable_motion=False)
        response = n.run("navigate_report", request(dry_run=False))
        self.assertTrue(response.dry_run)
        self.assertFalse(response.motion_issued)
        self.assertIn("DRY RUN", response.message)

    def test_a_request_can_force_a_dry_run_on_a_motion_enabled_node(self):
        n = node(enable_motion=True)
        response = n.run("navigate_report", request(dry_run=True))
        self.assertTrue(response.dry_run)
        self.assertFalse(response.motion_issued)

    def test_no_request_field_can_turn_motion_on(self):
        """The srv has dry_run only; there is deliberately no force-motion field."""
        with open(os.path.join(HERE, "..", "srv", "RunRoute.srv")) as f:
            request_block = f.read().split("---")[0]
        # Field NAMES only; comments legitimately discuss what is not offered.
        names = []
        for line in request_block.splitlines():
            line = line.split("#")[0].strip()
            if line:
                names.append(line.split()[1])
        self.assertEqual(names, ["route", "target", "destination", "label", "start_step",
                                 "dry_run", "event_token"])
        for forbidden in ("force", "enable_motion", "allow_motion", "execute"):
            self.assertNotIn(forbidden, names)

    def test_a_dry_run_traverses_every_step(self):
        n = node(enable_motion=False)
        response = n.run("navigate_report", request())
        self.assertTrue(response.success, response.message)
        self.assertEqual(list(response.steps_completed), r.NAVIGATE_REPORT.step_names)

    def test_a_dry_run_reports_the_blockers_a_real_run_would_refuse_on(self):
        n = node(enable_motion=False, blockers=["localization (slam): not localized"])
        response = n.run("navigate_report", request())
        self.assertTrue(response.success)                 # the rehearsal itself worked
        self.assertEqual(len(response.blockers), 1)
        self.assertIn("a real run would be refused", response.message)

    def test_a_dry_run_of_the_blocked_grasp_route_still_traverses_it(self):
        n = node(enable_motion=False, blockers=r.GRASP.blockers())
        response = n.run("grasp", request())
        self.assertTrue(response.success)
        self.assertEqual(list(response.steps_completed), r.GRASP.step_names)
        self.assertFalse(response.motion_issued)


class BlockedRealRunIssuesNoMotion(PreflightPatch):
    def test_blocked_real_run_executes_nothing(self):
        n = node(enable_motion=True, blockers=["arm_planning (arm_planning): NOT_READY"])
        response = n.run("grasp", request())
        self.assertFalse(response.success)
        self.assertFalse(response.motion_issued)
        self.assertFalse(response.dry_run)
        self.assertEqual(list(response.steps_completed), [])
        self.assertIn("NOT_READY", response.message)
        self.assertIn("refusing to move", response.message)

    def test_the_complete_blocker_list_is_returned(self):
        blockers = ["a (x): one", "b (y): two", "c (z): three"]
        n = node(enable_motion=True, blockers=blockers)
        response = n.run("deliver", request())
        self.assertEqual(len(response.blockers), 3)

    def test_a_blocked_real_run_never_constructs_a_service_proxy(self):
        """The stub ServiceProxy raises, so reaching it would fail this test."""
        n = node(enable_motion=True, blockers=["x (y): z"])
        response = n.run("navigate_report", request())
        self.assertFalse(response.success)


class RequestValidation(PreflightPatch):
    def test_a_mismatched_route_field_is_refused(self):
        n = node()
        response = n.run("grasp", request(route="deliver"))
        self.assertFalse(response.success)
        self.assertIn("does not match", response.message)

    def test_a_matching_route_field_is_accepted(self):
        n = node()
        response = n.run("navigate_report", request(route="navigate_report"))
        self.assertTrue(response.success, response.message)

    def test_an_empty_route_field_is_accepted(self):
        n = node()
        self.assertTrue(n.run("navigate_report", request(route="")).success)

    def test_an_invalid_side_is_fatal_at_startup(self):
        ROSPY._params.clear()
        ROSPY._params["~side"] = "middle"
        with self.assertRaises(ValueError):
            on.OrchestratorNode()

    def test_a_second_route_is_refused_while_one_is_running(self):
        n = node()
        n.lock.acquire("grasp (dry_run=True)")
        response = n.run("deliver", request())
        self.assertFalse(response.success)
        self.assertIn("already running", response.message)
        n.lock.release()

    def test_the_lock_is_released_after_a_route(self):
        n = node()
        n.run("navigate_report", request())
        self.assertFalse(n.lock.busy)


def executable_source(path):
    with open(path) as f:
        raw = f.read()
    kept = [t for t in tokenize.generate_tokens(io.StringIO(raw).readline)
            if t.type != tokenize.COMMENT]
    stripped = tokenize.untokenize(kept)
    tree = ast.parse(stripped)
    for n in ast.walk(tree):
        if isinstance(n, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
            doc = ast.get_docstring(n, clean=False)
            if doc:
                stripped = stripped.replace(doc, "")
    return stripped


class NoRoboticsInsideOrchestrator(unittest.TestCase):
    """The orchestrator composes interfaces; it implements none of them."""

    def _all_sources(self):
        paths = [NODE_SOURCE]
        for name in sorted(os.listdir(PKG_SRC)):
            if name.endswith(".py"):
                paths.append(os.path.join(PKG_SRC, name))
        return {p: executable_source(p) for p in paths}

    def test_no_kinematics_or_tf_maths(self):
        for path, code in self._all_sources().items():
            for forbidden in (
                "numpy", "scipy", "torch",                       # no maths libraries at all
                "tf2_ros", "tf.Transform", "lookupTransform",    # no TF work
                "quaternion", "euler_from", "math.cos", "math.sin",
                "inverse_kinematics",                            # no IK implemented here
                "costmap", "Costmap", "astar", "dwa",            # no navigation algorithms
                "cv_bridge", "YOLO", "ultralytics",              # no perception
                "unitree", "arm_sdk", "lowcmd",                  # no direct hardware
            ):
                self.assertNotIn(
                    forbidden, code, "%s must not contain %r" % (os.path.basename(path), forbidden)
                )

    def test_planning_is_delegated_never_implemented(self):
        """The orchestrator may CALL the planner; it may not BE one.

        `plan_to_pose` used to be a forbidden substring, which stopped working
        once the adapters were wired to the real /arm/plan_to_pose service. The
        intent is unchanged: no planning function may be defined here, and every
        mention must be a service proxy or the service name.
        """
        code = executable_source(NODE_SOURCE)
        for forbidden in ("def plan_to_pose", "def plan_grasp_from_point",
                          "def execute_grasp_from_point", "def _execute_grasp_pipeline",
                          "def _solve_ik", "def _ik", "def forward_kinematics",
                          "interp_joint_waypoints"):
            self.assertNotIn(forbidden, code,
                             "the orchestrator must not define %r" % forbidden)
        # Every occurrence must be the service path, the proxy attribute, the
        # generated type, or a label inside a message string -- never a local
        # definition or a local call.
        allowed = ("/arm/plan_to_pose", "/arm/plan_grasp_from_point",
                   "/g1_arm_simple/plan_grasp_from_point",      # arm_grasp, fetch_deliver
                   "self._plan_to_pose", "self._plan_grasp_from_point",
                   "PlanToPose", "PlanGraspFromPoint",
                   '"plan_to_pose', '"plan_grasp_from_point')
        for line in code.splitlines():
            if "plan_to_pose" not in line and "plan_grasp_from_point" not in line:
                continue
            self.assertTrue(any(a in line for a in allowed),
                            "unexpected planning reference: %s" % line.strip())

    def test_the_arm_services_are_called_not_reimplemented(self):
        """Positive check: the three downstream services are actually wired."""
        code = executable_source(NODE_SOURCE)
        for service in ("/arm/plan_to_pose", "/arm/plan_grasp_from_point",
                        "/arm/move_arm_joints"):
            self.assertIn(service, code, "%s is not wired up" % service)

    def test_the_pure_modules_do_not_import_ros(self):
        for name in sorted(os.listdir(PKG_SRC)):
            if not name.endswith(".py"):
                continue
            code = executable_source(os.path.join(PKG_SRC, name))
            self.assertNotIn("import rospy", code, name)

    def test_only_the_six_routes_are_advertised(self):
        """run_deliver_task is the full-task entry point (spec 0.4 / 6.2);
        run_fetch_deliver is the spoken task on arm_grasp + handover_node.

        Each composes its steps IN-PROCESS under one execution lock; none may
        appear as a service call from inside the node, which would deadlock
        against that same lock.
        """
        code = executable_source(NODE_SOURCE)
        advertised = sorted(set(re.findall(r'"(/orchestrator/run_[a-z_]+)"', code)))
        self.assertEqual(advertised, [
            "/orchestrator/run_auto_release",
            "/orchestrator/run_deliver",
            "/orchestrator/run_deliver_task",
            "/orchestrator/run_fetch_deliver",
            "/orchestrator/run_grasp",
            "/orchestrator/run_navigate_report",
        ])

    def test_no_route_calls_another_route_service(self):
        """A composed route must not re-enter the orchestrator over ROS."""
        code = executable_source(NODE_SOURCE)
        proxies = re.findall(r'ServiceProxy\(\s*"([^"]+)"', code)
        self.assertEqual([p for p in proxies if p.startswith("/orchestrator/")], [],
                         "the node builds a client for its own route service")

    def test_perception_results_are_never_frame_converted(self):
        """frame_id must be passed through, never recomputed."""
        code = executable_source(NODE_SOURCE)
        self.assertIn("response.frame_id", code)
        for forbidden in ("transformPoint", "do_transform", "base_link\" ==", "pelvis\" =="):
            self.assertNotIn(forbidden, code)


if __name__ == "__main__":
    unittest.main()
