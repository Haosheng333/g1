#!/usr/bin/env python3
"""Grasp-and-deliver sequencing over other modules' published interfaces.

    /orchestrator/run_navigate_report  orchestrator/RunRoute
    /orchestrator/run_grasp            orchestrator/RunRoute
    /orchestrator/run_deliver          orchestrator/RunRoute
    /orchestrator/run_auto_release     orchestrator/RunRoute

This package contains no robotics of its own. No IK, no TF maths, no path
planning, no perception, no arm control, no hand control, no navigation: each
of those stays in its owning package and is reached only through its
interface.

SAFETY MODEL

  * Execution is DRY-RUN unless the node is launched with enable_motion:=true.
    A dry run uses DryRunBackend, which constructs no robot service client of
    any kind, and still traverses every step so the order and the resume logic
    are exercised with the robot powered off.
  * dry_run=true in a request forces a dry run even on a motion-enabled node.
    There is deliberately no field that can force motion ON.
  * Preflight runs BEFORE any step. It collects every blocker, and a single
    blocker means zero steps execute and zero motion is issued.
  * cancel_navigation and disable_motion run in the sequencer's finally block
    on every exit path.
  * The full grasp route stays NOT_READY: it requires the unresolved
    perception -> arm_planning frame contract (see capabilities.FRAME_CONTRACT).
"""
from __future__ import annotations

import threading

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import SetBool, Trigger

from orchestrator import capabilities as caps
from orchestrator import handover_state as hs
from orchestrator import nav_guard as ng
from orchestrator import preflight as pf
from orchestrator import routes as routes_mod
from orchestrator import sequencer as seq
from orchestrator.execution_lock import Busy, ExecutionLock
from orchestrator.srv import RunRoute, RunRouteResponse

#: Latched Bool flags preflight reads. Subscribed once at startup; a flag that
#: never arrives stays None, which preflight reports as WAIT rather than false.
READY_FLAGS = (
    "/slam/manager/state",
    "/arm/planner/ready",
    "/auto_release_monitor/ready",
)

#: Joint name slam needs for camera_init -> pelvis (slam/config/g1_tf.yaml).
WAIST_JOINT = "waist_yaw_joint"


class RosProbe(pf.Probe):
    """The only part of preflight that touches ROS."""

    def __init__(self, service_timeout_s: float = 1.0):
        self.service_timeout_s = float(service_timeout_s)
        self._flags = {}
        self._joint_stamp = None
        self._joint_has_waist = False
        for topic in READY_FLAGS:
            rospy.Subscriber(topic, Bool, self._make_flag_cb(topic), queue_size=1)
        rospy.Subscriber("/joint_states", JointState, self._on_joint_states, queue_size=5)

    def _make_flag_cb(self, topic):
        def cb(msg):
            self._flags[topic] = bool(msg.data)
        return cb

    def _on_joint_states(self, msg):
        self._joint_stamp = rospy.Time.now().to_sec()
        self._joint_has_waist = WAIST_JOINT in msg.name

    # --- Probe -------------------------------------------------------------------
    def has_service(self, name):
        try:
            rospy.wait_for_service(name, timeout=self.service_timeout_s)
            return True
        except rospy.ROSException:
            return False

    def flag(self, topic):
        return self._flags.get(topic)

    def has_publisher(self, topic):
        try:
            published = rospy.get_master().getPublishedTopics("")[2]
        except Exception as exc:  # noqa: BLE001 - no master is itself a blocker
            rospy.logwarn_throttle(10.0, "[orchestrator] master query failed: %s" % (exc,))
            return False
        return any(entry[0] == topic for entry in published)

    def fresh(self, topic, max_age_s):
        if topic != "/joint_states":
            return "" if self.has_publisher(topic) else "%s has no publisher" % topic
        if self._joint_stamp is None:
            return "/joint_states not received (WAIT; EXTERNAL topic owned by robot-control)"
        age = rospy.Time.now().to_sec() - self._joint_stamp
        if age > float(max_age_s):
            return "/joint_states is stale (%.2fs > %.2fs)" % (age, max_age_s)
        if not self._joint_has_waist:
            return "/joint_states carries no %s, so slam cannot publish the robot pose" % WAIST_JOINT
        return ""


class RosBackend(seq.Backend):
    """Real actions. Built ONLY for a motion run whose preflight passed.

    Every proxy is created in __init__, so the mere existence of this object
    means the run is a real one. A dry run never constructs it.
    """

    def __init__(self, side: str, timeouts: dict):
        self.side = side
        self.timeouts = timeouts
        self._nav_status = None
        self._nav_status_lock = threading.Lock()

        rospy.Subscriber("/navigation/status", String, self._on_nav_status, queue_size=10)
        self._nav_cancel_pub = rospy.Publisher("/navigation/cancel", Empty, queue_size=1)

        from g1_navigation.srv import NavigateToObject
        from perception.srv import GetObjectPosition

        self._navigate = rospy.ServiceProxy("/navigation/navigate", NavigateToObject)
        self._enable_motion = rospy.ServiceProxy("/navigation/enable_motion", SetBool)
        self._detect = rospy.ServiceProxy("/perception/get_object_position", GetObjectPosition)
        self._hand = {
            name: rospy.ServiceProxy("/hand_%s/%s" % (side, name), Trigger)
            for name in ("pre_grasp", "grasp", "release")
        }
        self._arm_handover = rospy.ServiceProxy("/auto_release_monitor/arm", Trigger)
        self._disarm_handover = rospy.ServiceProxy("/auto_release_monitor/disarm", Trigger)

    def _on_nav_status(self, msg):
        with self._nav_status_lock:
            self._nav_status = msg.data

    # --- actions -----------------------------------------------------------------
    def await_localization(self, ctx):
        # Preflight already confirmed /slam/manager/state is True; this step
        # exists so a resume starting here still re-checks it.
        timeout = self.timeouts.get("localization", 30.0)
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            try:
                msg = rospy.wait_for_message("/slam/manager/state", Bool, timeout=1.0)
            except rospy.ROSException:
                continue
            if msg.data:
                return seq.StepResult(True, "localized")
        return seq.StepResult(False, "WAIT: not localized within %.0fs" % timeout)

    def enable_motion(self, ctx):
        response = self._enable_motion(True)
        return seq.StepResult(
            bool(response.success),
            "motion gate: %s" % response.message,
            motion_issued=bool(response.success),
        )

    def navigate(self, ctx):
        """Call /navigation/navigate, then apply navigation's saw_active guard."""
        guard = ng.NavGuard(self.timeouts.get("navigate", 1800.0))
        guard.start(rospy.Time.now().to_sec())
        response = self._navigate(class_name=ctx.target, approach_distance=0.0)
        if not response.success:
            return seq.StepResult(False, "navigate(%r) rejected: %s" % (ctx.target, response.message))

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            with self._nav_status_lock:
                status = self._nav_status
            verdict = guard.update(rospy.Time.now().to_sec(), status)
            if verdict != ng.PENDING:
                break
            rate.sleep()
        return seq.StepResult(
            verdict == ng.SUCCESS, "navigation %s: %s" % (verdict, guard.reason), motion_issued=True
        )

    def detect_object(self, ctx):
        response = self._detect(label=ctx.target, index=0)
        if not response.success:
            return seq.StepResult(False, "perception: %s" % response.message)
        # Stored verbatim. No frame conversion happens here or anywhere in this
        # package; frame_id is reported exactly as perception sent it.
        ctx.detection = {
            "x": response.x, "y": response.y, "z": response.z,
            "frame_id": response.frame_id, "label": ctx.target,
        }
        return seq.StepResult(True, "detected %s at (%.3f, %.3f, %.3f) in %s" % (
            ctx.target, response.x, response.y, response.z, response.frame_id))

    def report(self, ctx):
        rospy.loginfo("[orchestrator] detection: %s", ctx.detection)
        return seq.StepResult(True, "reported: %s" % (ctx.detection,))

    def to_planner_frame(self, ctx):
        # Unreachable: FRAME_CONTRACT is an unresolved capability, so preflight
        # refuses the route before any step runs. Kept explicit so the reason
        # travels with the step rather than living only in a comment.
        return seq.StepResult(
            False, "NOT_READY: " + caps.capability(caps.FRAME_CONTRACT).unresolved
        )

    def plan_grasp(self, ctx):
        return seq.StepResult(False, "unreachable: route blocked before planning")

    def execute_segment(self, ctx, step):
        return seq.StepResult(False, "unreachable: route blocked before arm motion")

    def open_gripper(self, ctx):
        return self._hand_call("pre_grasp")

    def close_gripper(self, ctx):
        return self._hand_call("grasp")

    def release_hand(self, ctx):
        return self._hand_call("release")

    def _hand_call(self, preset):
        response = self._hand[preset]()
        return seq.StepResult(
            bool(response.success),
            "hand %s: %s" % (preset, response.message),
            motion_issued=True,
        )

    def arm_handover(self, ctx):
        response = self._arm_handover()
        return seq.StepResult(bool(response.success), "handover arm: %s" % response.message)

    def disarm_handover(self, ctx):
        response = self._disarm_handover()
        return seq.StepResult(bool(response.success), "handover disarm: %s" % response.message)

    # --- cleanup -----------------------------------------------------------------
    def cancel_navigation(self):
        try:
            self._nav_cancel_pub.publish(Empty())
        except Exception as exc:  # noqa: BLE001 - cleanup must never raise
            rospy.logerr("[orchestrator] cancel_navigation failed: %s", exc)

    def disable_motion(self):
        try:
            self._enable_motion(False)
        except Exception as exc:  # noqa: BLE001 - cleanup must never raise
            rospy.logerr("[orchestrator] disable_motion failed: %s", exc)


class OrchestratorNode:
    def __init__(self) -> None:
        self.enable_motion = bool(rospy.get_param("~enable_motion", False))
        self.side = str(rospy.get_param("~side", "right"))
        self.timeouts = dict(rospy.get_param("~timeouts", {}) or {})
        self.retry = seq.RetryPolicy(
            attempts=dict(rospy.get_param("~retries", {}) or {}),
            default_attempts=int(rospy.get_param("~default_attempts", 1)),
        )
        self.lock = ExecutionLock()
        # Task-owned handover record. Deliberately NOT guarded by self.lock:
        # see orchestrator/src/orchestrator/handover_state.py for why the
        # release callback must not touch the execution lock.
        self.handover = hs.HandoverState(
            max_awaiting_sec=float(rospy.get_param("~max_awaiting_pull_sec", 120.0)))
        self.probe = RosProbe(float(rospy.get_param("~service_timeout_s", 1.0)))
        self.status_pub = rospy.Publisher("/orchestrator/status", String, queue_size=1, latch=True)

        if self.side not in ("left", "right"):
            rospy.logfatal("[orchestrator] ~side must be left or right, got %r", self.side)
            raise ValueError("~side must be 'left' or 'right'")

        mode = "MOTION ENABLED" if self.enable_motion else "dry-run (enable_motion is false)"
        rospy.logwarn("[orchestrator] %s; arm side %s", mode, self.side)
        self.status_pub.publish(String(data="idle; %s" % mode))

    # ------------------------------------------------------------------ handler
    def make_handler(self, route_name: str):
        def handler(req):
            return self.run(route_name, req)
        return handler

    def handle_release_request(self, req) -> RunRouteResponse:
        """/orchestrator/run_auto_release: VALIDATE and RECORD, never actuate.

        This callback deliberately does NOT acquire the execution lock and does
        NOT run a route. A handover task already holds that lock while waiting
        for the pull, so trying to run a route here would be refused by the very
        task this request is meant to serve. Instead the validated event is
        recorded against the active task, which consumes it under its existing
        ownership. Nothing here opens a hand or moves an arm.
        """
        if req.route and req.route != "auto_release":
            return RunRouteResponse(
                success=False,
                message="request route %r does not match this service's route "
                        "'auto_release'" % (req.route,),
                dry_run=True,
            )

        validation = self.handover.request_release(
            rospy.Time.now().to_sec(), source="auto_release_monitor")
        snap = self.handover.snapshot()

        if validation.ok:
            rospy.logwarn("[orchestrator] %s", validation.reason)
        else:
            rospy.logwarn("[orchestrator] release request %s", validation.reason)

        return RunRouteResponse(
            success=validation.ok,
            message=validation.reason,
            blockers=[] if validation.ok else [validation.reason],
            final_step="await_pull",
            # This path issues no motion under any circumstances.
            motion_issued=False,
            dry_run=not snap.release_authorized,
        )

    def run(self, route_name: str, req) -> RunRouteResponse:
        route = routes_mod.resolve_route(route_name)

        if req.route and req.route != route_name:
            return RunRouteResponse(
                success=False,
                message="request route %r does not match this service's route %r"
                % (req.route, route_name),
                dry_run=True,
            )

        # Dry run unless the node was launched for motion AND the caller did not
        # ask for a dry run. Nothing can force motion on from a request.
        dry_run = bool(req.dry_run) or not self.enable_motion

        try:
            held = self.lock.hold("%s (dry_run=%s)" % (route_name, dry_run))
        except Busy as exc:
            # Non-blocking: refuse at once rather than queue behind a route that
            # may be navigating for minutes.
            return RunRouteResponse(success=False, message=str(exc), dry_run=dry_run)

        with held:
            return self._execute(route, req, dry_run)

    def _execute(self, route, req, dry_run: bool) -> RunRouteResponse:
        self.status_pub.publish(
            String(data="%s: preflight (%s)" % (route.name, "dry-run" if dry_run else "MOTION"))
        )

        result = pf.run_preflight(route, self.probe, self.side)
        blockers = result.blockers
        if blockers:
            rospy.logwarn("[orchestrator] %s", result.summary())

        if dry_run:
            # Traverse the route regardless, so the rehearsal is useful even
            # when hardware is absent, but report the blockers that a real run
            # would have refused on.
            backend = seq.DryRunBackend()
            ctx = seq.RouteContext(route=route.name, target=req.target, side=self.side, dry_run=True)
            run = seq.run_route(route, backend, ctx, req.start_step, self.retry, blockers=[])
            message = "DRY RUN: %s" % run.message
            if blockers:
                message += " | a real run would be refused: %d blocker(s)" % len(blockers)
            response = RunRouteResponse(
                success=run.success, message=message,
                steps_completed=run.steps_completed, steps_skipped=run.steps_skipped,
                blockers=blockers, final_step=run.final_step,
                motion_issued=False, dry_run=True,
            )
            self.status_pub.publish(String(data="%s: dry run finished" % route.name))
            return response

        if blockers:
            # Real run, not ready: zero steps, zero motion, complete list.
            self.status_pub.publish(String(data="%s: NOT_READY" % route.name))
            return RunRouteResponse(
                success=False,
                message="NOT_READY: refusing to move; %d blocker(s)" % len(blockers),
                blockers=blockers, motion_issued=False, dry_run=False,
            )

        try:
            backend = RosBackend(self.side, self.timeouts)
        except Exception as exc:  # noqa: BLE001 - a missing srv module is a blocker
            rospy.logerr("[orchestrator] backend construction failed: %s", exc)
            return RunRouteResponse(
                success=False,
                message="cannot build the execution backend: %s: %s" % (type(exc).__name__, exc),
                blockers=["backend: %s" % exc], motion_issued=False, dry_run=False,
            )

        ctx = seq.RouteContext(route=route.name, target=req.target, side=self.side, dry_run=False)
        self.status_pub.publish(String(data="%s: RUNNING" % route.name))
        run = seq.run_route(route, backend, ctx, req.start_step, self.retry, blockers=[])
        self.status_pub.publish(
            String(data="%s: %s" % (route.name, "done" if run.success else "FAILED"))
        )
        return RunRouteResponse(
            success=run.success, message=run.message,
            steps_completed=run.steps_completed, steps_skipped=run.steps_skipped,
            blockers=run.blockers, final_step=run.final_step,
            motion_issued=run.motion_issued, dry_run=False,
        )


def main() -> None:
    rospy.init_node("orchestrator")
    node = OrchestratorNode()
    for route_name, service in (
        ("navigate_report", "/orchestrator/run_navigate_report"),
        ("grasp", "/orchestrator/run_grasp"),
        ("deliver", "/orchestrator/run_deliver"),
    ):
        rospy.Service(service, RunRoute, node.make_handler(route_name))

    # Not a route: the monitor's request is validated and recorded for the task
    # that already owns the execution lock. See handle_release_request.
    rospy.Service("/orchestrator/run_auto_release", RunRoute, node.handle_release_request)
    rospy.loginfo("[orchestrator] routes up: %s", ", ".join(sorted(routes_mod.ROUTES)))
    rospy.spin()


if __name__ == "__main__":
    main()
