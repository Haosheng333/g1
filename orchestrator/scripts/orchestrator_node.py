#!/usr/bin/env python3
"""Grasp-and-deliver sequencing over other modules' published interfaces.

    /orchestrator/run_navigate_report  orchestrator/RunRoute
    /orchestrator/run_grasp            orchestrator/RunRoute
    /orchestrator/run_deliver          orchestrator/RunRoute
    /orchestrator/run_deliver_task     orchestrator/RunRoute
    /orchestrator/run_fetch_deliver    orchestrator/RunRoute   <- the spoken task
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

TWO REAL BACKENDS, ONE PER STACK

  * RosBackend      arm_planning / arm_control / hand_control / the DIP tactile
                    monitor: the grasp, deliver and deliver_task routes.
  * ArmGraspBackend arm_grasp (g1_arm_simple) / g1_hands on PC2 / handover_node
                    / STANDING waypoints: the fetch_deliver route, i.e. the
                    modules that have actually run on this robot.

  Both share RosCommon (localization, motion gate, navigation status,
  perception). The node picks the backend from the route, and a backend is
  constructed only for a real run whose preflight passed, so a dry run still
  builds no robot client of any kind.
"""
from __future__ import annotations

import math
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
from orchestrator import standing_waypoints as sw
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

#: Three delivery steps need a pose that exists nowhere in the repository. A
#: search of arm_planning/{config,scripts}, arm_control and hand_control/config
#: for home|rest|stow|neutral|park|tuck returns nothing, and the grasp retract
#: is object-relative (it starts at the lift pose), so it cannot be replayed
#: after a handover. Inventing joint angles here would be a guess, so the step
#: fails with this instead.
_MISSING_POSE = (
    "NOT_READY: no %s pose is defined anywhere in this repository, so this step "
    "cannot be planned. It needs arm_planning to publish a validated %s "
    "(reachable, inside the trained workspace, collision-free). The grasp retract "
    "path is NOT a substitute: it starts at the lift pose and inherits y and z "
    "from the grasped object. See docs/DELIVERY_FLOW.md.")


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


class RosCommon(seq.Backend):
    """Adapters every real route needs: localization, gate, navigation, perception.

    Constructed ONLY for a motion run whose preflight passed; a dry run never
    builds one. Subclasses add one manipulation stack each.
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

    def _on_nav_status(self, msg):
        with self._nav_status_lock:
            self._nav_status = msg.data

    # --- injected clock (the shared await_pull loop uses these) -----------------
    def now(self):
        return rospy.Time.now().to_sec()

    def sleep(self, seconds):
        rospy.sleep(float(seconds))

    def aborted(self):
        return rospy.is_shutdown()

    # --- shared actions ------------------------------------------------------------
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
        """Drive to the PICKUP waypoint (ctx.target)."""
        return self._drive_to(ctx.target, "pickup")

    def navigate_to_destination(self, ctx):
        """Drive to the DELIVERY waypoint (ctx.destination).

        A separate action so the delivery leg cannot silently reuse the pickup
        waypoint, which is what happened while RunRoute carried only `target`.
        """
        if not ctx.destination:
            return seq.StepResult(
                False,
                "no delivery destination: RunRoute.destination is empty. The full task "
                "navigates twice and this leg has no waypoint; refusing rather than "
                "re-driving to the pickup waypoint %r." % (ctx.target,))
        return self._drive_to(ctx.destination, "delivery")

    def _drive_to(self, waypoint, leg):
        """Call /navigation/navigate, then apply navigation's saw_active guard."""
        guard = ng.NavGuard(self.timeouts.get("navigate", 1800.0))
        guard.start(rospy.Time.now().to_sec())
        response = self._navigate(class_name=waypoint, approach_distance=0.0)
        if not response.success:
            return seq.StepResult(False, "navigate(%s=%r) rejected: %s"
                                  % (leg, waypoint, response.message))
        return self._await_nav_terminal(
            guard, "navigation to the %s waypoint %r" % (leg, waypoint))

    def _await_nav_terminal(self, guard, what):
        """Poll /navigation/status through the saw_active guard to a verdict."""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            with self._nav_status_lock:
                status = self._nav_status
            verdict = guard.update(rospy.Time.now().to_sec(), status)
            if verdict != ng.PENDING:
                break
            rate.sleep()
        return seq.StepResult(
            verdict == ng.SUCCESS,
            "%s %s: %s" % (what, verdict, guard.reason),
            motion_issued=True)

    def detect_object(self, ctx):
        # `label` is set by routes whose `target` is a waypoint name; the older
        # routes use `target` as the label as well.
        label = ctx.label or ctx.target
        response = self._detect(label=label, index=0)
        if not response.success:
            return seq.StepResult(False, "perception: %s" % response.message)
        # Stored verbatim. No frame conversion happens here or anywhere in this
        # package; frame_id is reported exactly as perception sent it.
        ctx.detection = {
            "x": response.x, "y": response.y, "z": response.z,
            "frame_id": response.frame_id, "label": label,
            # RECEIPT time, not observation time: GetObjectPosition.srv carries
            # no stamp. Recorded so to_planner_frame can pass an honest upper
            # bound and let arm_planning apply its waist-motion guard.
            "received_at": rospy.Time.now(),
        }
        return seq.StepResult(True, "detected %s at (%.3f, %.3f, %.3f) in %s" % (
            label, response.x, response.y, response.z, response.frame_id))

    def report(self, ctx):
        rospy.loginfo("[orchestrator] detection: %s", ctx.detection)
        return seq.StepResult(True, "reported: %s" % (ctx.detection,))

    def record_held_object(self, ctx):
        """The ONLY writer of held-object state."""
        label = ctx.label or ctx.target or "object"
        ctx.handover.begin(ctx.side, label, release_authorized=True)
        ctx.held_label = label
        return seq.StepResult(
            True, "recorded %r held in the %s hand" % (ctx.held_label, ctx.side))

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


class RosBackend(RosCommon):
    """arm_planning / arm_control / hand_control / DIP tactile monitor.

    Every proxy is created in __init__, so the mere existence of this object
    means the run is a real one. A dry run never constructs it.
    """

    def __init__(self, side: str, timeouts: dict):
        super(RosBackend, self).__init__(side, timeouts)

        # Measured arm joints, needed as `current_joints` by both planner
        # services. Captured here rather than in RosProbe, which only tracks
        # freshness. /joint_states is an EXTERNAL topic owned by robot-control:
        # if it does not carry the five arm joints, every planning step reports
        # NOT_READY with that exact reason instead of guessing a posture.
        self._joint_positions = {}
        self._joint_lock = threading.Lock()
        rospy.Subscriber("/joint_states", JointState, self._on_arm_joint_states,
                         queue_size=5)

        from arm_control.srv import MoveArmJoints
        from arm_planning.srv import ConvertObservation, PlanGraspFromPoint, PlanToPose
        from handover.srv import ArmMonitor

        self._hand = {
            name: rospy.ServiceProxy("/hand_%s/%s" % (side, name), Trigger)
            for name in ("pre_grasp", "grasp", "release")
        }
        self._arm_monitor = rospy.ServiceProxy("/auto_release_monitor/arm", ArmMonitor)
        self._disarm_monitor = rospy.ServiceProxy("/auto_release_monitor/disarm", Trigger)
        # Real downstream planner and executor. Both are implemented production
        # nodes (arm_planning/scripts/arm_planning_node.py,
        # arm_control/scripts/arm_control_node.py); they were simply never wired.
        self._plan_to_pose = rospy.ServiceProxy("/arm/plan_to_pose", PlanToPose)
        self._plan_grasp_from_point = rospy.ServiceProxy(
            "/arm/plan_grasp_from_point", PlanGraspFromPoint)
        self._move_arm_joints = rospy.ServiceProxy("/arm/move_arm_joints", MoveArmJoints)
        # Boundary 3. The TF maths lives in arm_planning; this is a pass-through.
        self._convert_observation = rospy.ServiceProxy(
            "/arm/convert_observation", ConvertObservation)
        #: name -> [x, y, z, rx, rz] in torso_link, from ~poses. Ships EMPTY.
        self.poses = dict(rospy.get_param("~poses", {}) or {})

    # --- measured joints ---------------------------------------------------------
    def _on_arm_joint_states(self, msg):
        with self._joint_lock:
            for name, position in zip(msg.name, msg.position):
                self._joint_positions[name] = float(position)

    def _current_joints(self, arm: str):
        """(joints, reason). `joints` is 5 finite radians in the planner's order.

        Never substitutes a default posture: planning from a guessed start
        configuration would return a path that does not begin where the arm is.
        """
        try:
            from arm_planning.plan_conversion import expected_joint_names
            names = expected_joint_names(arm)
        except Exception as exc:  # noqa: BLE001 - a missing helper is a blocker
            return None, "cannot resolve the %s arm joint names: %s" % (arm, exc)
        with self._joint_lock:
            have = dict(self._joint_positions)
        missing = [n for n in names if n not in have]
        if missing:
            return None, (
                "NOT_READY: /joint_states carries no %s, so the planner has no "
                "current_joints. That topic is owned by robot-control / integration; "
                "this package will not substitute a guessed posture." % ", ".join(missing))
        joints = [have[n] for n in names]
        if not all(math.isfinite(v) for v in joints):
            return None, "/joint_states reported a non-finite value for %s: %s" % (arm, joints)
        return joints, ""

    # --- configured poses --------------------------------------------------------
    def _configured_pose(self, name: str):
        """(pose, reason). A 5-vector [x,y,z,rx,rz] in torso_link, or None.

        Reads ~poses/<name>. The shipped config is EMPTY on purpose, so every
        pose-dependent step stays NOT_READY until arm_planning publishes a
        validated pose. Nothing here invents one.
        """
        raw = self.poses.get(name)
        if raw in (None, "", [], {}):
            return None, _MISSING_POSE % (name, "%s pose" % name)
        try:
            pose = [float(v) for v in raw]
        except (TypeError, ValueError):
            return None, "~poses/%s is not a list of numbers: %r" % (name, raw)
        if len(pose) != 5:
            return None, (
                "~poses/%s must be exactly 5 values [x, y, z, rx, rz] in torso_link "
                "(metres, radians), got %d" % (name, len(pose)))
        if not all(math.isfinite(v) for v in pose):
            return None, "~poses/%s contains a non-finite value: %r" % (name, pose)
        return pose, ""

    # --- task-state steps ------------------------------------------------------
    def require_held_object(self, ctx):
        """Never invents a held object; a delivery cannot start without a grasp."""
        validation = ctx.handover.require_held_object(ctx.side)
        return seq.StepResult(validation.ok, validation.reason)

    def settle_handover(self, ctx):
        delay = float(self.timeouts.get("handover_settle_sec", 1.0))
        self.sleep(delay)
        return seq.StepResult(True, "settled %.2fs after reaching the handover pose" % delay)

    def settle_recipient(self, ctx):
        delay = float(self.timeouts.get("recipient_settle_sec", 1.5))
        self.sleep(delay)
        return seq.StepResult(True, "waited %.2fs for the recipient to secure the object" % delay)

    def arm_monitor(self, ctx):
        """Create the session token, arm the monitor with it, then await the pull.

        Order matters. The token is created here, the monitor must confirm it
        stored that exact value, and only then does the task enter
        AWAITING_PULL -- so there is no window in which a release could be
        validated against a monitor that is not yet armed, and exactly one
        token exists for the session.
        """
        token = hs.new_session_token()
        # ARMING first, so a request arriving in the handshake window carries a
        # token the task already knows and is QUEUED rather than lost.
        ctx.handover.begin_arming(self.now(), token)
        ctx.session_token = token
        try:
            response = self._arm_monitor(side=ctx.side, session_token=token)
        except Exception as exc:  # noqa: BLE001 - arming must fail safe
            ctx.handover.abandon("arm call raised")
            return seq.StepResult(
                False, "monitor arm call raised %s: %s; abandoned, no release possible"
                % (type(exc).__name__, exc))
        if not response.success:
            # Abandon drops any queued event and clears the token, so nothing
            # can be released against a monitor that is not armed.
            ctx.handover.abandon("monitor refused to arm")
            return seq.StepResult(False, "monitor refused to arm: %s" % response.message)
        echoed = (getattr(response, "session_token", "") or "")
        if echoed != token:
            ctx.handover.abandon("token echo mismatch")
            return seq.StepResult(
                False,
                "monitor echoed token %r but was given %r; abandoned rather than await a "
                "pull it cannot identify" % (echoed, token))
        ctx.handover.mark_awaiting_pull(self.now(), token)
        return seq.StepResult(True, "monitor armed for the %s hand; AWAITING_PULL with "
                                    "token %s" % (ctx.side, token))

    def disarm_monitor(self, ctx):
        response = self._disarm_monitor()
        return seq.StepResult(bool(response.success), "monitor disarm: %s" % response.message)

    def confirm_released(self, ctx):
        """Interpret the release result per hand_control's ACTUAL contract.

        hand_control's `release` preset sets neither require_settle nor
        expect_contact, so a success means the open command was accepted -- not
        that the fingers moved and not that the object transferred. So this
        records RELEASE_COMMANDED, never a verified transfer.
        """
        if ctx.release_outcome == "ok":
            # Only a command acknowledgement is available. hand_control exposes
            # no settled flag and no feedback positions, and its release preset
            # leaves require_settle absent, so success does not mean the fingers
            # moved. mark_release_settled is deliberately NOT called from a
            # configuration flag; see handover_state.
            ctx.handover.mark_release_commanded()
            return seq.StepResult(
                True,
                "release command ACKNOWLEDGED only. Not settling, not transfer: "
                "hand_control's release preset sets neither require_settle nor "
                "expect_contact, so a success response does not establish that the "
                "fingers moved. Retraction stays refused; see retraction_allowed.")
        ctx.handover.mark_release_uncertain(ctx.release_outcome)
        return seq.StepResult(
            False,
            "release outcome UNCERTAIN (%s): the object may still be in the hand or "
            "falling. Retraction is refused and a human must inspect."
            % (ctx.release_outcome or "unknown",))

    # --- pose-dependent steps ----------------------------------------------------
    #
    # The plan+execute WIRING is implemented below and calls the real
    # /arm/plan_to_pose and /arm/move_arm_joints. What is still missing is the
    # POSE ITSELF: ~poses ships empty, so _configured_pose returns NOT_READY and
    # these steps refuse exactly as before. Supplying a number here would not
    # resolve the capability either -- arm_planning owns validating that a pose
    # is reachable, inside the trained workspace and collision-free, and the
    # handover_pose / stow_pose / home_pose capabilities stay `unresolved` so
    # preflight refuses the route regardless of this config.

    def plan_handover_pose(self, ctx):
        return self._plan_pose(ctx, "handover", "handover_pose")

    def execute_retract_home(self, ctx):
        """Retract to home. Refused unless the release outcome permits motion."""
        allowed, reason = ctx.handover.retraction_allowed()
        if not allowed:
            return seq.StepResult(False, reason)
        planned = self._plan_pose(ctx, "home", "home_pose")
        if not planned.ok:
            return planned
        moved = self._execute_path(ctx, "home_pose")
        if not moved.ok:
            return moved
        # End of delivery: close the task record only after the arm is home.
        ctx.handover.mark_done()
        return moved

    def plan_stow(self, ctx):
        """Spec 2.4: plan the stow/carry recovery path BEFORE the grasp moves.

        Planned from the end of retract (`plan_grasp`'s own final_joints), so it
        is validated while the arm is still parked and the hand still open --
        never discovered to be unplannable after the hand has closed on the
        object.
        """
        if not ctx.grasp_final_joints:
            return seq.StepResult(
                False,
                "cannot plan the stow path: plan_grasp did not report final_joints, so "
                "the posture the stow would start from is unknown. Spec 2.4 requires "
                "this path to be validated before the grasp starts moving.")
        return self._plan_pose(ctx, "stow", "stow", start_joints=ctx.grasp_final_joints)

    def disarm_monitor_cleanup(self):
        try:
            self._disarm_monitor()
        except Exception as exc:  # noqa: BLE001 - cleanup must never raise
            rospy.logerr("[orchestrator] disarm_monitor_cleanup failed: %s", exc)

    def to_planner_frame(self, ctx):
        """Hand the observation to arm_planning's /arm/convert_observation.

        This package performs NO TF mathematics: it passes the point, the frame
        perception labelled it with, and the best timestamp it has, and stores
        whatever comes back. The 10 mm torso_link convention correction and the
        TF lookup both happen in arm_planning, which owns the planner's frame.

        TIMESTAMP LIMITATION, passed on rather than hidden:
        perception/GetObjectPosition.srv has NO stamp field, so the orchestrator
        cannot know when the observation was taken. It sends its own RECEIPT
        time with stamp_is_observation_time=false, which makes the conversion
        apply a waist-motion guard and refuse instead of pairing an old
        observation with a newer waist pose.
        """
        if not ctx.detection:
            return seq.StepResult(
                False, "no detection to convert: detect_object did not run or failed")
        point = [ctx.detection.get("x"), ctx.detection.get("y"), ctx.detection.get("z")]
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in point):
            return seq.StepResult(
                False, "perception returned a non-finite position %s; refusing to route "
                       "it to the planner" % (point,))
        source = ctx.detection.get("frame_id") or ""
        if not source:
            return seq.StepResult(
                False, "perception reported no frame_id, so the observation cannot be "
                       "converted")
        try:
            response = self._convert_observation(
                source_frame=source, point=point,
                # Receipt time, NOT observation time -- see the docstring.
                stamp=ctx.detection.get("received_at") or rospy.Time.now(),
                stamp_is_observation_time=False,
                max_waist_delta_rad=float(self.timeouts.get("max_waist_delta_rad", 0.0)))
        except Exception as exc:  # noqa: BLE001
            return seq.StepResult(False, "/arm/convert_observation raised %s: %s"
                                  % (type(exc).__name__, exc))
        if not response.success:
            return seq.StepResult(False, "convert_observation: %s" % response.message)
        try:
            converted = [float(v) for v in response.planner_point]
        except (TypeError, ValueError):
            return seq.StepResult(False, "convert_observation returned a non-numeric point")
        if len(converted) != 3 or not all(math.isfinite(v) for v in converted):
            return seq.StepResult(
                False, "convert_observation returned an unusable point: %s" % (converted,))
        ctx.planner_target = converted
        ctx.planner_frame = response.planner_frame
        return seq.StepResult(
            True,
            "converted %s %s -> %s %s (waist %.4f rad, convention offset %s, "
            "latest_transform=%s)"
            % (source, [round(v, 4) for v in point], response.planner_frame,
               [round(v, 4) for v in converted], response.waist_yaw_rad,
               list(response.convention_offset), response.used_latest_transform))

    # --- planning and execution: REAL calls to arm_planning / arm_control -------
    #
    # Both downstream services are implemented production nodes that were never
    # wired up. These adapters call them and carry their outputs verbatim: the
    # flattened N x 5 path, the planner's own joint_names, the arm side and the
    # failure message. Nothing here re-plans, rounds, re-orders or substitutes a
    # successful result.

    def _store_segment(self, ctx, segment, flat, count, joint_names, arm):
        ctx.plan[segment] = {
            "flat": list(flat), "count": int(count),
            "joint_names": list(joint_names), "arm": arm,
        }

    def _check_joint_names(self, arm, joint_names):
        """The planner's joint order must be the one we think it is."""
        try:
            from arm_planning.plan_conversion import expected_joint_names
            expected = expected_joint_names(arm)
        except Exception as exc:  # noqa: BLE001
            return "cannot verify joint order: %s" % (exc,)
        if list(joint_names) != list(expected):
            return (
                "planner returned joint order %s but this arm's order is %s; refusing "
                "to execute a path whose column meaning is unclear"
                % (list(joint_names), expected))
        return ""

    def plan_grasp(self, ctx):
        """Call /arm/plan_grasp_from_point and keep all four segments."""
        if not ctx.planner_target:
            return seq.StepResult(
                False,
                "NOT_READY: no planner-frame target. to_planner_frame must convert the "
                "detection into torso_link first, and that conversion is unimplemented "
                "(frame_contract). plan_grasp will not plan from a perception frame the "
                "planner does not accept.")
        joints, reason = self._current_joints(self.side)
        if joints is None:
            return seq.StepResult(False, reason)
        try:
            response = self._plan_grasp_from_point(
                arm=self.side, current_joints=joints, object_xyz=list(ctx.planner_target))
        except Exception as exc:  # noqa: BLE001
            return seq.StepResult(False, "/arm/plan_grasp_from_point raised %s: %s"
                                  % (type(exc).__name__, exc))
        if not response.success:
            # The planner's own reason, verbatim: it names which segment failed.
            return seq.StepResult(False, "plan_grasp_from_point: %s" % response.message)

        bad = self._check_joint_names(self.side, response.joint_names)
        if bad:
            return seq.StepResult(False, bad)

        names = list(response.segment_names)
        counts = [int(c) for c in response.segment_waypoint_counts]
        flat = list(response.flattened_segment_paths)
        if len(names) != len(counts):
            return seq.StepResult(False, "planner returned %d segment names but %d counts"
                                  % (len(names), len(counts)))
        if sum(counts) * 5 != len(flat):
            return seq.StepResult(
                False, "planner path length %d does not match %d waypoints x 5 joints"
                % (len(flat), sum(counts)))
        offset = 0
        for name, count in zip(names, counts):
            if count <= 0:
                return seq.StepResult(False, "planner segment %r has %d waypoints"
                                      % (name, count))
            block = flat[offset:offset + count * 5]
            if not all(math.isfinite(v) for v in block):
                return seq.StepResult(False, "planner segment %r contains a non-finite "
                                      "joint value" % (name,))
            self._store_segment(ctx, name, block, count, response.joint_names, self.side)
            offset += count * 5
        # Needed by plan_stow: the stow path starts where retract ends, and must
        # be planned BEFORE the grasp moves (spec 2.4), so it cannot use the
        # live joints.
        ctx.grasp_final_joints = [float(v) for v in response.final_joints]
        return seq.StepResult(True, "planned %d segments for the %s arm (%s), %d waypoints"
                              % (len(names), self.side, ", ".join(names), sum(counts)))

    def _execute_path(self, ctx, segment):
        """Send one stored segment to /arm/move_arm_joints, unmodified."""
        stored = ctx.plan.get(segment)
        if not stored:
            return seq.StepResult(
                False,
                "no planned path for segment %r: its planning step did not run or did "
                "not succeed. Refusing to move without the plan it was supposed to "
                "execute." % (segment,))
        if stored["arm"] != self.side:
            return seq.StepResult(False, "segment %r was planned for the %s arm but this "
                                  "run is %s" % (segment, stored["arm"], self.side))
        bad = self._check_joint_names(self.side, stored["joint_names"])
        if bad:
            return seq.StepResult(False, bad)
        try:
            response = self._move_arm_joints(
                arm=stored["arm"], path_flat=stored["flat"],
                timeout_per_waypoint=float(self.timeouts.get("waypoint_sec", 0.0)))
        except Exception as exc:  # noqa: BLE001
            # motion_issued: the command left this process; the arm's state is
            # now unknown, so the route must not be retried blindly.
            return seq.StepResult(False, "/arm/move_arm_joints raised %s: %s"
                                  % (type(exc).__name__, exc), motion_issued=True)
        detail = "%s: %s" % (segment, response.message)
        return seq.StepResult(bool(response.success), detail, motion_issued=True)

    def execute_segment(self, ctx, step):
        """Execute the segment this step names. `execute_<segment>`."""
        segment = step.name[len("execute_"):] if step.name.startswith("execute_") else step.name
        return self._execute_path(ctx, segment)

    def _plan_pose(self, ctx, pose_name, segment, start_joints=None):
        """Plan to a CONFIGURED pose via /arm/plan_to_pose. NOT_READY if unset.

        `start_joints` overrides the live measurement, which is what spec 2.4
        needs: a recovery path planned before the grasp moves must start from
        where the grasp will END, not from where the arm is now.
        """
        pose, reason = self._configured_pose(pose_name)
        if pose is None:
            return seq.StepResult(False, reason)
        if start_joints is not None:
            if len(start_joints) != 5 or not all(math.isfinite(v) for v in start_joints):
                return seq.StepResult(
                    False, "cannot plan %s from %r: need 5 finite joint values"
                    % (pose_name, start_joints))
            joints, reason = list(start_joints), ""
        else:
            joints, reason = self._current_joints(self.side)
        if joints is None:
            return seq.StepResult(False, reason)
        try:
            response = self._plan_to_pose(
                arm=self.side, current_joints=joints, target_pose=pose)
        except Exception as exc:  # noqa: BLE001
            return seq.StepResult(False, "/arm/plan_to_pose raised %s: %s"
                                  % (type(exc).__name__, exc))
        if not response.success:
            return seq.StepResult(False, "plan_to_pose(%s): %s" % (pose_name, response.message))
        bad = self._check_joint_names(self.side, response.joint_names)
        if bad:
            return seq.StepResult(False, bad)
        count = int(response.waypoint_count)
        flat = list(response.flattened_joint_path)
        if count <= 0 or len(flat) != count * 5:
            return seq.StepResult(
                False, "plan_to_pose(%s) returned %d waypoints and %d values; a malformed "
                "path is never a recoverable command" % (pose_name, count, len(flat)))
        if not all(math.isfinite(v) for v in flat):
            return seq.StepResult(False, "plan_to_pose(%s) returned a non-finite joint value"
                                  % (pose_name,))
        self._store_segment(ctx, segment, flat, count, response.joint_names, self.side)
        return seq.StepResult(
            True, "planned %s pose for the %s arm: %d waypoints, position_error_m=%s"
            % (pose_name, self.side, count, response.position_error_m))

    def open_gripper(self, ctx):
        return self._hand_call("pre_grasp")

    def close_gripper(self, ctx):
        return self._hand_call("grasp")

    def release_hand(self, ctx):
        """Send the open command. Interpretation is confirm_released's job.

        Refuses unless a validated pull was consumed in this session, so a
        resume cannot open the hand. A failure marks the outcome UNCERTAIN here
        rather than relying on confirm_released, which fail-fast never reaches.
        """
        permitted, reason = ctx.handover.release_permitted()
        if not permitted:
            return seq.StepResult(False, reason)
        try:
            result = self._hand_call("release")
        except Exception as exc:  # noqa: BLE001
            ctx.release_outcome = "raised %s: %s" % (type(exc).__name__, exc)
            ctx.handover.mark_release_uncertain(ctx.release_outcome)
            return seq.StepResult(False, ctx.release_outcome, motion_issued=True)
        if not result.ok:
            ctx.release_outcome = result.detail
            ctx.handover.mark_release_uncertain(result.detail)
            return result
        ctx.release_outcome = "ok"
        return result

    def _hand_call(self, preset):
        response = self._hand[preset]()
        return seq.StepResult(
            bool(response.success),
            "hand %s: %s" % (preset, response.message),
            motion_issued=True,
        )

class ArmGraspBackend(RosCommon):
    """arm_grasp (g1_arm_simple) + g1_hands on PC2 + handover_node + standing poses.

    The fetch_deliver route. Every action is one service call or one topic wait
    against a node that has already been exercised on this robot; nothing is
    planned, interpolated or converted here. Built only for a real run whose
    preflight passed.
    """

    ARM_NODE = "/g1_arm_simple"
    HANDOVER_DONE = "done"
    HANDOVER_ERROR_PREFIX = "error:"

    def __init__(self, side: str, timeouts: dict):
        super(ArmGraspBackend, self).__init__(side, timeouts)

        from arm_grasp.srv import ExecuteGraspFromPoint, PlanGraspFromPoint
        from geometry_msgs.msg import PoseStamped
        from g1_slam import standing_goal

        self._standing_goal = standing_goal
        # Navigation's EXACT-pose interface. The publisher exists from the start
        # of the run so the connection is up long before the first drive.
        self._goal_pub = rospy.Publisher(standing_goal.GOAL_TOPIC, PoseStamped, queue_size=1)
        # Delegated to arm_grasp: planning (no motion) and the whole grasp pipeline.
        self._plan_grasp_from_point_arm_grasp = rospy.ServiceProxy(
            "/g1_arm_simple/plan_grasp_from_point", PlanGraspFromPoint)
        self._execute_grasp = rospy.ServiceProxy(
            "/g1_arm_simple/execute_grasp_from_point", ExecuteGraspFromPoint)
        self._release_arm_control = rospy.ServiceProxy(
            "/g1_arm_simple/release_arm_control", Trigger)
        # handover_node: start (blocks until armed) and abort (disarm, arm stays).
        self._handover_start = rospy.ServiceProxy("/handover/start", Trigger)
        self._handover_abort = rospy.ServiceProxy("/handover/abort", Trigger)
        self._handover_state = None
        self._handover_state_lock = threading.Lock()
        rospy.Subscriber("/handover/state", String, self._on_handover_state, queue_size=5)
        #: True from a successful /handover/start until /handover/state settles,
        #: so cleanup knows whether a monitor may still be armed.
        self._handover_open = False

    def _on_handover_state(self, msg):
        with self._handover_state_lock:
            self._handover_state = msg.data

    def _current_handover_state(self):
        with self._handover_state_lock:
            return self._handover_state

    # --- standing-pose navigation --------------------------------------------------
    def navigate_standing(self, ctx):
        return self._drive_to_standing(ctx, "pickup", ctx.target)

    def navigate_standing_destination(self, ctx):
        if not ctx.destination:
            return seq.StepResult(
                False, "no delivery destination: RunRoute.destination is empty; refusing "
                       "rather than re-driving to the pickup pose %r" % (ctx.target,))
        return self._drive_to_standing(ctx, "delivery", ctx.destination)

    def _drive_to_standing(self, ctx, leg, name):
        """Publish the resolved standing pose as an exact goal; guard the status."""
        resolved = ctx.standing.get(leg)
        if not resolved:
            return seq.StepResult(
                False, "the %s standing waypoint %r was not resolved before the route "
                       "started; refusing to guess a pose" % (leg, name))
        key, record, frame = resolved
        goal = self._standing_goal.build_goal(record, frame, rospy.Time.now())
        guard = ng.NavGuard(self.timeouts.get("navigate", 1800.0))
        # Give a freshly started navigation_node a moment to subscribe; the
        # publisher itself was created when this backend was built.
        deadline = rospy.Time.now() + rospy.Duration(2.0)
        while (self._goal_pub.get_num_connections() == 0 and rospy.Time.now() < deadline
               and not rospy.is_shutdown()):
            rospy.sleep(0.05)
        if self._goal_pub.get_num_connections() == 0:
            return seq.StepResult(
                False, "nothing subscribes to %s; is navigation_node running?"
                % self._standing_goal.GOAL_TOPIC)
        guard.start(rospy.Time.now().to_sec())
        self._goal_pub.publish(goal)
        rospy.loginfo("[orchestrator] standing pose %s -> x=%.3f y=%.3f yaw=%.3f in %s "
                      "(exact goal, no approach offset)", key, record["x"], record["y"],
                      record["yaw"], frame)
        return self._await_nav_terminal(
            guard, "drive to the %s standing pose %r" % (leg, key))

    def settle_chassis(self, ctx):
        delay = float(self.timeouts.get("nav_settle_sec", 1.5))
        self.sleep(delay)
        return seq.StepResult(True, "chassis settled %.2fs after the drive" % delay)

    # --- arm_grasp -----------------------------------------------------------------
    def _target_xyz(self, ctx):
        point = [ctx.detection.get(k) for k in ("x", "y", "z")]
        if not ctx.detection or not all(
                isinstance(v, (int, float)) and math.isfinite(v) for v in point):
            return None, ("no usable detection: detect_object did not run or returned a "
                          "non-finite point %s" % (point,))
        return point, ""

    def plan_grasp_point(self, ctx):
        """Plan the whole grasp in arm_grasp, moving nothing. Its reason, verbatim."""
        point, reason = self._target_xyz(ctx)
        if point is None:
            return seq.StepResult(False, reason)
        try:
            # Positional (arm, x, y, z, waypoint-count override, planner
            # override): both overrides at 0 = that node's own defaults.
            response = self._plan_grasp_from_point_arm_grasp(
                ctx.side, point[0], point[1], point[2], 0, 0)
        except Exception as exc:  # noqa: BLE001
            return seq.StepResult(False, "arm_grasp planning raised %s: %s"
                                  % (type(exc).__name__, exc))
        if not response.success:
            return seq.StepResult(False, "arm_grasp plan: %s" % response.message)
        ctx.plan["arm_grasp"] = {
            "pre_waypoints": int(response.pre_waypoint_count),
            "target_waypoints": int(response.target_waypoint_count),
            "lift": bool(list(response.lift_joint_angles)),
            "retract": bool(list(response.retract_joint_angles)),
            "home": bool(list(response.home_joint_angles)),
        }
        summary = ctx.plan["arm_grasp"]
        return seq.StepResult(
            True, "arm_grasp planned the %s arm to (%.3f, %.3f, %.3f): %d pre + %d target "
                  "waypoints, lift=%s retract=%s home=%s | %s"
            % (ctx.side, point[0], point[1], point[2], summary["pre_waypoints"],
               summary["target_waypoints"], summary["lift"], summary["retract"],
               summary["home"], response.message))

    def execute_grasp_point(self, ctx):
        """The whole grasp pipeline inside arm_grasp, as grasp_point.py drives it."""
        if "arm_grasp" not in ctx.plan:
            return seq.StepResult(
                False, "refusing to execute a grasp that was not planned in this run")
        point, reason = self._target_xyz(ctx)
        if point is None:
            return seq.StepResult(False, reason)
        try:
            response = self._execute_grasp(
                arm=ctx.side, x=point[0], y=point[1], z=point[2], wait=True,
                timeout=float(self.timeouts.get("grasp_move_sec", 6.0)))
        except Exception as exc:  # noqa: BLE001 - the command left this process
            return seq.StepResult(
                False, "arm_grasp execute raised %s: %s; the arm's state is unknown and "
                       "it keeps SDK control" % (type(exc).__name__, exc),
                motion_issued=True)
        if not response.success:
            return seq.StepResult(
                False, "arm_grasp grasp FAILED: %s. The arm may be extended and keeps SDK "
                       "control; `rosrun arm_grasp grasp_point.py home` parks it"
                % response.message, motion_issued=True)
        return seq.StepResult(
            True, "grasped with the %s arm; arm_grasp ran pregrasp, pre_grasp, approach, "
                  "grasp_5f, lift, retract, home: %s" % (ctx.side, response.message),
            motion_issued=True)

    def release_arm_control(self, ctx):
        """Hand the arms back to the robot's own controller, as main_process does.

        Best effort, never fatal: arm_grasp refuses unless both arms are at home,
        and main_process then simply walks with the SDK holding the arm. The
        outcome is reported either way.
        """
        try:
            response = self._release_arm_control()
        except Exception as exc:  # noqa: BLE001
            return seq.StepResult(
                True, "arm hand-back skipped (%s: %s); the arm node keeps control"
                % (type(exc).__name__, exc))
        if not response.success:
            return seq.StepResult(
                True, "arm hand-back refused by arm_grasp, SDK keeps the arms: %s"
                % response.message)
        return seq.StepResult(True, "arms handed back to the robot's controller: %s"
                              % response.message, motion_issued=True)

    # --- handover_node -------------------------------------------------------------
    def handover_start(self, ctx):
        validation = ctx.handover.require_held_object(ctx.side)
        if not validation.ok:
            return seq.StepResult(False, validation.reason)
        try:
            response = self._handover_start()
        except Exception as exc:  # noqa: BLE001
            return seq.StepResult(False, "/handover/start raised %s: %s; the arm's state is "
                                         "unknown" % (type(exc).__name__, exc),
                                  motion_issued=True)
        if not response.success:
            return seq.StepResult(False, "handover_node: %s" % response.message,
                                  motion_issued=True)
        self._handover_open = True
        return seq.StepResult(True, "handover_node: %s" % response.message, motion_issued=True)

    def await_handover(self, ctx):
        """Wait for handover_node to report `done`: pull detected, hand open, arm home.

        The monitor is g1_hands' tactile auto_release_monitor, and the release
        itself runs inside handover_node (/handover/run_auto_release). This step
        only observes. A timeout aborts the handover (monitor disarmed, hand
        still closed, arm left at the pose) and FAILS; nothing is released.
        """
        timeout = float(self.timeouts.get("handover_sec", 120.0))
        started = self.now()
        deadline = started + timeout
        while not self.aborted():
            state = self._current_handover_state() or ""
            if state == self.HANDOVER_DONE:
                self._handover_open = False
                # handover_node's release is a hand-open command acknowledged by
                # the hand service, then GoHomeArm. That is a COMMANDED release;
                # object transfer stays unverifiable, exactly as for `deliver`.
                ctx.handover.mark_release_commanded()
                ctx.handover.mark_done()
                return seq.StepResult(
                    True, "handover done after %.1fs: pull detected, hand released, %s arm "
                          "homed by handover_node" % (self.now() - started, ctx.side))
            if state.startswith(self.HANDOVER_ERROR_PREFIX):
                self._handover_open = False
                ctx.handover.mark_release_uncertain(state)
                return seq.StepResult(
                    False, "handover_node reported %r; the object may still be in the hand "
                           "or falling, a human must inspect" % state)
            if self.now() >= deadline:
                self._abort_handover("timeout")
                ctx.handover.abandon("handover timeout")
                return seq.StepResult(
                    False, "TIMEOUT: nobody pulled the object within %.0fs; monitor disarmed, "
                           "hand still closed, arm left at the handover pose. Release by hand "
                           "with `rosrun handover handover_cli.py release`" % timeout)
            self.sleep(0.1)
        return seq.StepResult(False, "CANCELLED while awaiting the handover; nothing released")

    def _abort_handover(self, why):
        try:
            response = self._handover_abort()
            rospy.logwarn("[orchestrator] handover aborted (%s): %s", why, response.message)
        except Exception as exc:  # noqa: BLE001 - cleanup must never raise
            rospy.logerr("[orchestrator] /handover/abort failed (%s): %s", why, exc)
        self._handover_open = False

    def disarm_monitor_cleanup(self):
        """A route must never exit leaving g1_hands' monitor armed."""
        if self._handover_open:
            self._abort_handover("route exit")


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
            max_awaiting_sec=float(rospy.get_param("~max_awaiting_pull_sec", 120.0)),
            require_event_token=bool(rospy.get_param("~require_event_token", True)))
        self.probe = RosProbe(float(rospy.get_param("~service_timeout_s", 1.0)))
        self.status_pub = rospy.Publisher("/orchestrator/status", String, queue_size=1, latch=True)
        # fetch_deliver: where the standing waypoints live, and the perception
        # label used when the request carries none.
        self.maps_dir = str(rospy.get_param("~maps_dir", "") or "")
        self.map_name = str(rospy.get_param("~map_name", "current") or "current")
        self.default_label = str(rospy.get_param("~default_label", "bottle") or "bottle")

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
            rospy.Time.now().to_sec(), source="auto_release_monitor",
            event_token=getattr(req, "event_token", "") or "")
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
            object_state=snap.object_state,
            transfer_verified=False,
        )

    @staticmethod
    def _destination(req) -> str:
        """The delivery waypoint, or "" when the request carries none."""
        return (getattr(req, "destination", "") or "").strip()

    def _label(self, req) -> str:
        """The perception label: the request's, else ~default_label."""
        return (getattr(req, "label", "") or "").strip() or self.default_label

    def _maps_dir(self) -> str:
        if self.maps_dir:
            return self.maps_dir
        import os
        from rospkg import RosPack
        return os.path.join(RosPack().get_path("g1_slam"), "maps")

    def _route_blockers(self, route, req) -> tuple:
        """Request-specific blockers, found BEFORE preflight: (standing, blockers).

        fetch_deliver names standing waypoints and relies on handover_node being
        launched for the same arm. Both are checked here so a dry run reports
        them and a real run refuses before the first drive.
        """
        standing, blockers = {}, []
        if route.name not in routes_mod.ROUTES_USING_STANDING_WAYPOINTS:
            return standing, blockers
        try:
            maps_dir = self._maps_dir()
        except Exception as exc:  # noqa: BLE001
            return standing, ["standing_waypoints (slam): cannot locate g1_slam/maps: %s" % exc]
        standing, blockers = sw.resolve_standing_waypoints(
            maps_dir, self.map_name,
            {"pickup": req.target, "delivery": self._destination(req)})
        handover_arm = str(rospy.get_param("/handover/arm", "") or "").strip().lower()
        if handover_arm and handover_arm != self.side:
            blockers.append(
                "handover_node (handover): launched for the %s arm but this orchestrator "
                "grasps with the %s; pass the same side to both launches"
                % (handover_arm, self.side))
        return standing, blockers

    @staticmethod
    def _make_backend(route, side, timeouts):
        """The real backend for `route`. Only ever called for a motion run."""
        if route.name in routes_mod.ROUTES_USING_STANDING_WAYPOINTS:
            return ArmGraspBackend(side, timeouts)
        return RosBackend(side, timeouts)

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

        # A route with a delivery leg needs its own waypoint. Refused up front,
        # before the lock, so the task cannot drive to the pickup point twice.
        destination = self._destination(req)
        if route_name in routes_mod.ROUTES_NEEDING_DESTINATION and not destination:
            return RunRouteResponse(
                success=False, dry_run=dry_run,
                message="route %r needs both `target` (pickup waypoint / object label) "
                        "and `destination` (delivery waypoint); destination is empty"
                        % route_name)

        try:
            held = self.lock.hold("%s (dry_run=%s)" % (route_name, dry_run))
        except Busy as exc:
            # Non-blocking: refuse at once rather than queue behind a route that
            # may be navigating for minutes.
            return RunRouteResponse(success=False, message=str(exc), dry_run=dry_run)

        with held:
            return self._execute(route, req, dry_run)

    def _execute(self, route, req, dry_run: bool) -> RunRouteResponse:
        destination = self._destination(req)
        label = self._label(req)
        self.status_pub.publish(
            String(data="%s: preflight (%s)" % (route.name, "dry-run" if dry_run else "MOTION"))
        )

        result = pf.run_preflight(route, self.probe, self.side)
        standing, extra = self._route_blockers(route, req)
        blockers = list(result.blockers) + list(extra)
        if blockers:
            rospy.logwarn("[orchestrator] %s", result.summary())
            for text in extra:
                rospy.logwarn("[orchestrator] blocker: %s", text)

        if dry_run:
            # Traverse the route regardless, so the rehearsal is useful even
            # when hardware is absent, but report the blockers that a real run
            # would have refused on.
            backend = seq.DryRunBackend()
            ctx = seq.RouteContext(route=route.name, target=req.target,
                                   destination=destination, label=label,
                                   standing=standing, side=self.side,
                                   dry_run=True, handover=self.handover)
            run = seq.run_route(route, backend, ctx, req.start_step, self.retry, blockers=[])
            message = "DRY RUN: %s" % run.message
            if blockers:
                message += " | a real run would be refused: %d blocker(s)" % len(blockers)
            response = RunRouteResponse(
                success=run.success, message=message,
                steps_completed=run.steps_completed, steps_skipped=run.steps_skipped,
                blockers=blockers, final_step=run.final_step,
                motion_issued=False, dry_run=True,
                object_state=self.handover.object_state, transfer_verified=False,
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
            backend = self._make_backend(route, self.side, self.timeouts)
            backend.await_pull_timeout_sec = float(
                self.timeouts.get("await_pull_sec", 120.0))
        except Exception as exc:  # noqa: BLE001 - a missing srv module is a blocker
            rospy.logerr("[orchestrator] backend construction failed: %s", exc)
            return RunRouteResponse(
                success=False,
                message="cannot build the execution backend: %s: %s" % (type(exc).__name__, exc),
                blockers=["backend: %s" % exc], motion_issued=False, dry_run=False,
            )

        ctx = seq.RouteContext(route=route.name, target=req.target,
                               destination=destination, label=label,
                               standing=standing, side=self.side,
                               dry_run=False, handover=self.handover)
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
            object_state=self.handover.object_state,
            # Only a hand contract that actually establishes transfer may set this.
            transfer_verified=False,
        )


def main() -> None:
    rospy.init_node("orchestrator")
    node = OrchestratorNode()
    for route_name, service in (
        ("navigate_report", "/orchestrator/run_navigate_report"),
        ("grasp", "/orchestrator/run_grasp"),
        ("deliver", "/orchestrator/run_deliver"),
        # The single full-task entry point (spec 0.4 / 6.2): grasp then deliver,
        # composed in-process under ONE execution lock. It never calls
        # run_grasp or run_deliver, which would deadlock against its own lock.
        ("deliver_task", "/orchestrator/run_deliver_task"),
        # The spoken task on the modules that have run on this robot: standing
        # waypoints, perception, arm_grasp, handover_node. See routes.FETCH_DELIVER.
        ("fetch_deliver", "/orchestrator/run_fetch_deliver"),
    ):
        rospy.Service(service, RunRoute, node.make_handler(route_name))

    # Not a route: the monitor's request is validated and recorded for the task
    # that already owns the execution lock. See handle_release_request.
    rospy.Service("/orchestrator/run_auto_release", RunRoute, node.handle_release_request)
    rospy.loginfo("[orchestrator] routes up: %s", ", ".join(sorted(routes_mod.ROUTES)))
    rospy.spin()


if __name__ == "__main__":
    main()
