"""Offline test of arm_planning_node's planner-result -> service-response layer.

rospy, std_msgs and the generated arm_planning.srv module are stubbed (the same
approach as navigation/test/test_navigation_node.py), so this runs with plain
python3 and no catkin build, no ROS master, no torch.

Covers the hard-failure rule of doc section 12.1: when the planner reports
failure, or returns a path that does not hold up, the response must carry no
executable path at all.
"""
import math
import os
import re
import sys
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))


# --------------------------------------------------------------------- stubs
class _Response(object):
    """Stand-in for a generated srv response: keyword init, attribute access."""

    _defaults = {}

    def __init__(self, **kw):
        for name, default in self._defaults.items():
            setattr(self, name, list(default) if isinstance(default, list) else default)
        for name, value in kw.items():
            if name not in self._defaults:
                raise TypeError("unknown response field %r" % (name,))
            setattr(self, name, value)


class PlanToPoseResponse(_Response):
    _defaults = {
        "success": False, "message": "", "joint_names": [], "waypoint_count": 0,
        "flattened_joint_path": [], "verified_goal_joints": [],
        "position_error_m": 0.0, "rx_error_rad": 0.0, "rz_error_rad": 0.0,
        "nodes_expanded": 0, "exact_goal_cell_blocked": False,
    }


class PlanGraspFromPointResponse(_Response):
    _defaults = {
        "success": False, "message": "", "joint_names": [], "segment_names": [],
        "segment_waypoint_counts": [], "flattened_segment_paths": [], "final_joints": [],
        "pregrasp_pose": [], "grasp_pose": [], "lift_pose": [], "retract_pose": [],
    }


class ConvertObservationResponse(object):
    """Mirrors ConvertObservation.srv's response block."""

    def __init__(self, success=False, message="", planner_point=(), planner_frame="",
                 waist_yaw_rad=0.0, transform_age_s=0.0, convention_offset=(),
                 used_latest_transform=False):
        self.success = success
        self.message = message
        self.planner_point = list(planner_point)
        self.planner_frame = planner_frame
        self.waist_yaw_rad = waist_yaw_rad
        self.transform_age_s = transform_age_s
        self.convention_offset = list(convention_offset)
        self.used_latest_transform = used_latest_transform


def _install_stubs():
    rospy = types.ModuleType("rospy")
    rospy.get_param = lambda name, default=None: default
    rospy.init_node = lambda *a, **k: None
    rospy.spin = lambda *a, **k: None
    rospy.signal_shutdown = lambda *a, **k: None
    rospy.Service = lambda *a, **k: None
    rospy.Publisher = lambda *a, **k: types.SimpleNamespace(publish=lambda *_a, **_k: None)
    for lvl in ("loginfo", "logwarn", "logerr", "logfatal"):
        setattr(rospy, lvl, lambda *a, **k: None)
    sys.modules["rospy"] = rospy

    std_msgs = types.ModuleType("std_msgs")
    msg = types.ModuleType("std_msgs.msg")
    msg.Bool = lambda data=False: types.SimpleNamespace(data=data)
    msg.String = lambda data="": types.SimpleNamespace(data=data)
    std_msgs.msg = msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = msg

    srv = types.ModuleType("arm_planning.srv")
    srv.PlanToPose = object
    srv.PlanGraspFromPoint = object
    srv.ConvertObservation = object
    srv.PlanToPoseResponse = PlanToPoseResponse
    srv.PlanGraspFromPointResponse = PlanGraspFromPointResponse
    srv.ConvertObservationResponse = ConvertObservationResponse
    sys.modules["arm_planning.srv"] = srv

    # tf2_ros is imported lazily by ArmPlanningNode.__init__ for the frame
    # conversion. This test exercises the response layer only, but the stub
    # keeps the module importable without a ROS install.
    tf2 = types.ModuleType("tf2_ros")
    tf2.Buffer = lambda *a, **k: None
    tf2.TransformListener = lambda *a, **k: None
    sys.modules.setdefault("tf2_ros", tf2)


_install_stubs()

import arm_planning_node as apn  # noqa: E402


# ------------------------------------------------------------------- fixtures
class FakePoseResult(object):
    def __init__(self, **kw):
        self.success = True
        self.message = "success"
        self.arm = "left"
        self.joint_names = ["left_shoulder_pitch_joint"]
        self.joint_path = [[0.0] * 5, [0.1] * 5]
        self.target_pose = [0.3, 0.15, 0.0, 0.0, 0.0]
        self.verified_goal_joints = [0.1] * 5
        self.position_error_m = 0.00464
        self.rx_error_rad = 0.001
        self.rz_error_rad = 0.002
        self.selected_grid_state = None
        self.exact_goal_cell_blocked = False
        self.nodes_expanded = 42
        self.used_fallback = False
        self.__dict__.update(kw)


class FakeGraspResult(object):
    def __init__(self, **kw):
        self.success = True
        self.message = "success"
        self.arm = "right"
        self.object_xyz = [0.3, -0.15, 0.0]
        self.joint_names = ["right_shoulder_pitch_joint"]
        self.pregrasp_pose = [0.2, -0.1, 0.05, 0.0, -0.26]
        self.grasp_pose = [0.3, -0.15, 0.0, 0.0, 0.0]
        self.lift_pose = [0.3, -0.15, 0.1, 0.0, 0.0]
        self.retract_pose = [0.15, -0.15, 0.1, 0.0, 0.0]
        self.segments = {
            "to_pregrasp": {"joint_path": [[0.0] * 5, [0.1] * 5]},
            "approach": {"joint_path": [[0.2] * 5]},
            "lift": {"joint_path": [[0.3] * 5, [0.4] * 5, [0.5] * 5]},
            "retract": {"joint_path": [[0.6] * 5]},
        }
        self.final_joints = [0.6] * 5
        self.__dict__.update(kw)


def _bare_node():
    """An ArmPlanningNode with no assets loaded, for conversion only."""
    node = apn.ArmPlanningNode.__new__(apn.ArmPlanningNode)
    node._blockers = []
    node._planner = None
    node._grasp_planner = None
    node._assets = None
    return node


# ----------------------------------------------------------------------- tests
class PoseResponseConversion(unittest.TestCase):
    def setUp(self):
        self.node = _bare_node()

    def test_success_carries_flat_path_and_count(self):
        r = self.node._to_pose_response(FakePoseResult())
        self.assertTrue(r.success)
        self.assertEqual(r.waypoint_count, 2)
        self.assertEqual(len(r.flattened_joint_path), 10)
        self.assertEqual(r.flattened_joint_path[:5], [0.0] * 5)
        self.assertEqual(r.nodes_expanded, 42)
        self.assertEqual(r.verified_goal_joints, [0.1] * 5)

    def test_failure_carries_no_path(self):
        r = self.node._to_pose_response(
            FakePoseResult(success=False, message="target outside trained workspace",
                           joint_path=[], verified_goal_joints=[])
        )
        self.assertFalse(r.success)
        self.assertEqual(r.message, "target outside trained workspace")
        self.assertEqual(r.flattened_joint_path, [])
        self.assertEqual(r.waypoint_count, 0)

    def test_failure_discards_any_path_the_planner_still_returned(self):
        """Doc 12.1: a partial path on failure is never a recoverable command."""
        r = self.node._to_pose_response(
            FakePoseResult(success=False, message="dense path collision",
                           joint_path=[[0.0] * 5, [0.1] * 5])
        )
        self.assertFalse(r.success)
        self.assertEqual(r.flattened_joint_path, [])
        self.assertEqual(r.waypoint_count, 0)

    def test_unmeasured_errors_become_nan(self):
        r = self.node._to_pose_response(
            FakePoseResult(success=False, message="bad request", joint_path=[],
                           position_error_m=None, rx_error_rad=None, rz_error_rad=None)
        )
        self.assertTrue(math.isnan(r.position_error_m))
        self.assertTrue(math.isnan(r.rx_error_rad))
        self.assertTrue(math.isnan(r.rz_error_rad))

    def test_success_with_empty_path_is_downgraded_to_failure(self):
        r = self.node._to_pose_response(FakePoseResult(joint_path=[]))
        self.assertFalse(r.success)
        self.assertIn("empty path", r.message)

    def test_malformed_path_is_downgraded_to_failure(self):
        r = self.node._to_pose_response(FakePoseResult(joint_path=[[0.0] * 5, [0.1] * 4]))
        self.assertFalse(r.success)
        self.assertIn("malformed", r.message)
        self.assertEqual(r.flattened_joint_path, [])


class GraspResponseConversion(unittest.TestCase):
    def setUp(self):
        self.node = _bare_node()

    def test_success_carries_four_ordered_segments(self):
        r = self.node._to_grasp_response(FakeGraspResult())
        self.assertTrue(r.success)
        self.assertEqual(r.segment_names, ["to_pregrasp", "approach", "lift", "retract"])
        self.assertEqual(r.segment_waypoint_counts, [2, 1, 3, 1])
        self.assertEqual(len(r.flattened_segment_paths), 7 * 5)
        self.assertEqual(r.final_joints, [0.6] * 5)

    def test_failure_carries_no_segments(self):
        r = self.node._to_grasp_response(
            FakeGraspResult(success=False, message="no viable pregrasp candidate",
                            segments={}, final_joints=[])
        )
        self.assertFalse(r.success)
        self.assertEqual(r.segment_names, [])
        self.assertEqual(r.flattened_segment_paths, [])
        self.assertEqual(r.segment_waypoint_counts, [])

    def test_missing_segment_on_success_is_downgraded_to_failure(self):
        segments = FakeGraspResult().segments
        del segments["lift"]
        r = self.node._to_grasp_response(FakeGraspResult(segments=segments))
        self.assertFalse(r.success)
        self.assertIn("malformed segments", r.message)
        self.assertIn("lift", r.message)
        self.assertEqual(r.flattened_segment_paths, [])

    def test_poses_are_reported_even_on_failure(self):
        """Staging poses help diagnose a rejected grasp; they are not a path."""
        r = self.node._to_grasp_response(
            FakeGraspResult(success=False, message="no viable retract candidate",
                            segments={}, final_joints=[])
        )
        self.assertFalse(r.success)
        self.assertEqual(len(r.grasp_pose), 5)


class NotReadyBehaviour(unittest.TestCase):
    """A node without assets must refuse, not raise and not plan."""

    def setUp(self):
        self.node = _bare_node()
        self.node._blockers = ["missing python dependencies: torch (needed by guarded_ik_solver)"]

    def test_pose_request_is_refused_with_the_blocker_list(self):
        req = types.SimpleNamespace(
            arm="left", current_joints=[0.0] * 5, target_pose=[0.3, 0.15, 0.0, 0.0, 0.0]
        )
        r = self.node.handle_plan_to_pose(req)
        self.assertFalse(r.success)
        self.assertTrue(r.message.startswith("NOT_READY"))
        self.assertIn("torch", r.message)

    def test_grasp_request_is_refused_with_the_blocker_list(self):
        req = types.SimpleNamespace(
            arm="right", current_joints=[0.0] * 5, object_xyz=[0.3, -0.15, 0.0]
        )
        r = self.node.handle_plan_grasp_from_point(req)
        self.assertFalse(r.success)
        self.assertTrue(r.message.startswith("NOT_READY"))

    def test_malformed_request_is_rejected_before_the_readiness_check(self):
        """A bad request gets the specific reason, not a generic NOT_READY."""
        req = types.SimpleNamespace(arm="middle", current_joints=[0.0] * 5, target_pose=[0.0] * 5)
        r = self.node.handle_plan_to_pose(req)
        self.assertFalse(r.success)
        self.assertNotIn("NOT_READY", r.message)
        self.assertIn("left", r.message)

    def test_planner_exception_becomes_a_failure_response(self):
        class Boom(object):
            def plan_to_pose(self, **kw):
                raise RuntimeError("fcl exploded")

        self.node._blockers = []
        self.node._grasp_planner = object()      # marks the node ready
        self.node._planner = Boom()
        import threading
        self.node._lock = threading.Lock()
        req = types.SimpleNamespace(
            arm="left", current_joints=[0.0] * 5, target_pose=[0.3, 0.15, 0.0, 0.0, 0.0]
        )
        r = self.node.handle_plan_to_pose(req)
        self.assertFalse(r.success)
        self.assertIn("RuntimeError", r.message)
        self.assertIn("fcl exploded", r.message)


class NodeNeverCommandsTheRobot(unittest.TestCase):
    """Doc 2.1 / 12.1: this package plans and nothing else.

    Checked against executable code only: docstrings and comments are stripped
    first, because the module docstring legitimately *names* the execution
    service it deliberately does not call.
    """

    @staticmethod
    def _executable_source(path):
        """Source with comments and every docstring removed."""
        import ast
        import io
        import tokenize

        with open(path) as f:
            raw = f.read()
        out = []
        for tok in tokenize.generate_tokens(io.StringIO(raw).readline):
            if tok.type != tokenize.COMMENT:
                out.append(tok)
        stripped = tokenize.untokenize(out)
        tree = ast.parse(stripped)
        docstrings = set()
        for node in ast.walk(tree):
            if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
                doc = ast.get_docstring(node, clean=False)
                if doc:
                    docstrings.add(doc)
        for doc in docstrings:
            stripped = stripped.replace(doc, "")
        return stripped

    def test_code_constructs_no_service_client_or_command_publisher(self):
        code = self._executable_source(
            os.path.join(HERE, "..", "scripts", "arm_planning_node.py")
        )
        for forbidden in (
            "ServiceProxy",      # no calling out to any other node
            "actionlib",         # no FollowJointTrajectory client
            "move_arm_joints",   # no arm_control execution
            "arm_sdk",           # no Unitree DDS command topic
            "lowcmd",
            "cmd_vel",
        ):
            self.assertNotIn(
                forbidden, code, "arm_planning_node.py code must not reference %r" % forbidden
            )

    def test_only_the_three_specified_services_are_advertised(self):
        code = self._executable_source(
            os.path.join(HERE, "..", "scripts", "arm_planning_node.py")
        )
        advertised = sorted(set(re.findall(r'rospy\.Service\(\s*"([^"]+)"', code)))
        # /arm/convert_observation was added for boundary 3: it converts an
        # observation into the planner's frame and still commands nothing.
        self.assertEqual(advertised, ["/arm/convert_observation",
                                      "/arm/plan_grasp_from_point", "/arm/plan_to_pose"])


if __name__ == "__main__":
    unittest.main()
