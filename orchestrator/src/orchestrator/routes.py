"""Declarative routes. Pure data plus start_step resume; no ROS, no robotics.

A route is an ordered tuple of Steps. Each Step names the capability it needs
and the backend action that performs it, and says whether it can move the
robot. The sequencer walks the list; it never decides what a step means.

Doc section 13.3 fixes the grasp order, including where the gripper acts:

    to_pregrasp -> open gripper -> approach -> close gripper -> lift -> retract
"""
from __future__ import annotations

from dataclasses import dataclass

from orchestrator import capabilities as caps


@dataclass(frozen=True)
class Step:
    name: str
    action: str                 # backend method to call
    capability: str = ""        # "" = needs nothing beyond the route's own requirements
    motion: bool = False        # True = this step can physically move the robot
    description: str = ""


@dataclass(frozen=True)
class Route:
    name: str
    steps: tuple
    #: Capabilities preflighted before ANY step runs.
    requires: tuple = ()
    description: str = ""

    @property
    def step_names(self) -> list:
        return [s.name for s in self.steps]

    @property
    def motion_steps(self) -> list:
        return [s.name for s in self.steps if s.motion]

    def blockers(self) -> list:
        """Unresolved-decision blockers that no runtime state can clear."""
        out = []
        for key in self.requires:
            cap = caps.capability(key)
            if cap.unresolved:
                out.append("%s (%s): %s" % (key, cap.owner, cap.unresolved))
        return out

    @property
    def ready_in_principle(self) -> bool:
        """False when the route depends on an unresolved integration decision."""
        return not self.blockers()


# --- navigate_report: the reduced route that can actually run today ------------------
#
# navigation -> perception -> report. The only motion is navigation, which is
# itself interlocked behind /navigation/enable_motion. Nothing is planned for
# the arm and nothing is grasped, so the unresolved frame contract does not
# apply: the perception result is reported, not turned into a planner target.

NAVIGATE_REPORT = Route(
    name="navigate_report",
    description="Navigate to a mapped waypoint, detect the object there, report its pose",
    requires=(caps.LOCALIZATION, caps.MAP, caps.JOINT_STATES, caps.NAVIGATION,
              caps.MOTION_GATE, caps.PERCEPTION),
    steps=(
        Step("await_localization", "await_localization", caps.LOCALIZATION,
             description="wait for slam to report a localized pose"),
        Step("enable_motion", "enable_motion", caps.MOTION_GATE, motion=True,
             description="open the navigation motion gate"),
        Step("navigate", "navigate", caps.NAVIGATION, motion=True,
             description="drive to the target waypoint, guarded by saw_active"),
        Step("detect_object", "detect_object", caps.PERCEPTION,
             description="call perception for the object pose, in its own frame"),
        Step("report", "report", description="return the detection verbatim; no conversion"),
    ),
)


# --- grasp: NOT_READY until the frame contract is resolved ---------------------------
#
# Every step is declared so a dry run still traverses the whole sequence and the
# blocker is reported against a real plan, but FRAME_CONTRACT is unresolved so
# this route can never execute for real.

GRASP = Route(
    name="grasp",
    description="Navigate to the object, plan the full grasp, then execute it segment by segment",
    requires=(caps.LOCALIZATION, caps.MAP, caps.JOINT_STATES, caps.NAVIGATION,
              caps.MOTION_GATE, caps.PERCEPTION, caps.ARM_PLANNING, caps.ARM_CONTROL,
              caps.HAND, caps.FRAME_CONTRACT, caps.STOW_POSE),
    steps=(
        Step("await_localization", "await_localization", caps.LOCALIZATION),
        Step("enable_motion", "enable_motion", caps.MOTION_GATE, motion=True),
        Step("navigate", "navigate", caps.NAVIGATION, motion=True,
             description="drive to the object's waypoint"),
        Step("detect_object", "detect_object", caps.PERCEPTION),
        Step("to_planner_frame", "to_planner_frame", caps.FRAME_CONTRACT,
             description="BLOCKED: perception frame -> torso_link, owned by no one yet"),
        # Everything below moves nothing until the whole grasp has been planned.
        Step("plan_grasp", "plan_grasp", caps.ARM_PLANNING,
             description="plan all four segments BEFORE any arm motion (doc 13.3)"),
        Step("execute_to_pregrasp", "execute_segment", caps.ARM_CONTROL, motion=True),
        Step("open_gripper", "open_gripper", caps.HAND, motion=True),
        Step("execute_approach", "execute_segment", caps.ARM_CONTROL, motion=True),
        Step("close_gripper", "close_gripper", caps.HAND, motion=True),
        Step("execute_lift", "execute_segment", caps.ARM_CONTROL, motion=True),
        Step("execute_retract", "execute_segment", caps.ARM_CONTROL, motion=True),
        Step("record_held_object", "record_held_object",
             description="the ONLY writer of held-object state; a delivery reads it"),
        Step("execute_stow", "execute_stow", caps.ARM_CONTROL, motion=True,
             description="bring the arm to the carry pose before walking"),
    ),
)


# --- deliver: the full handover sequence --------------------------------------------
#
# Framework section C/E order, with the gripper and settle points where the
# framework puts them. Every step is explicit so the sequencer can fail at a
# named place and so resume can refuse at the right one.
#
# `require_held_object` is first on purpose: a delivery may never start, or
# resume, without a grasp having recorded a held object. `arm_monitor` comes
# strictly after `execute_handover_pose` and `settle_handover`, so the detector
# is never armed while the arm is moving. `await_pull` consumes the validated
# event while this route holds the execution lock.

DELIVER = Route(
    name="deliver",
    description="Carry the held object to a waypoint, hand it over on a verified pull, retract",
    # Every known requirement is declared here so preflight refuses the whole
    # route up front. Discovering a missing home pose at execute_retract_home
    # would mean finding out AFTER the hand has opened.
    requires=(caps.LOCALIZATION, caps.MAP, caps.JOINT_STATES, caps.NAVIGATION,
              caps.MOTION_GATE, caps.ARM_PLANNING, caps.ARM_CONTROL, caps.HAND,
              caps.TACTILE_HANDOVER, caps.FRAME_CONTRACT,
              caps.HANDOVER_POSE, caps.RELEASE_COMPLETION, caps.HOME_POSE),
    steps=(
        Step("require_held_object", "require_held_object",
             description="refuse unless a grasp recorded a held object; never invent one"),
        Step("await_localization", "await_localization", caps.LOCALIZATION),
        Step("enable_motion", "enable_motion", caps.MOTION_GATE, motion=True),
        Step("navigate", "navigate", caps.NAVIGATION, motion=True,
             description="drive to the delivery waypoint, hand still closed"),
        Step("plan_handover_pose", "plan_handover_pose", caps.ARM_PLANNING,
             description="plan the whole move to the handover pose BEFORE any arm motion"),
        Step("execute_handover_pose", "execute_segment", caps.ARM_CONTROL, motion=True),
        Step("settle_handover", "settle_handover",
             description="wait out the arm's deceleration before arming the detector"),
        Step("arm_monitor", "arm_monitor", caps.TACTILE_HANDOVER,
             description="only now: AWAITING_PULL, and a fresh session token is issued"),
        Step("await_pull", "await_pull", caps.TACTILE_HANDOVER,
             description="consume the validated event while holding the execution lock"),
        Step("disarm_monitor", "disarm_monitor", caps.TACTILE_HANDOVER,
             description="disarm BEFORE releasing, per the framework order"),
        Step("release_hand", "release_hand", caps.HAND, motion=True),
        Step("confirm_released", "confirm_released", caps.HAND,
             description="interpret the service result; success is a command ack, not transfer"),
        Step("settle_recipient", "settle_recipient",
             description="let the recipient secure the object before the arm moves"),
        Step("execute_retract_home", "execute_retract_home", caps.ARM_CONTROL, motion=True,
             description="refused while the release outcome is UNCERTAIN"),
    ),
)


# The former `auto_release` route is deleted. Its two steps now live inside
# `deliver` (`disarm_monitor` then `release_hand`, the framework's order -- the
# old route had them reversed), so there is exactly one motion path and no
# route that can open the hand on its own. /orchestrator/run_auto_release is
# bound to the validation callback, which records an event and moves nothing.


ROUTES = {r.name: r for r in (NAVIGATE_REPORT, GRASP, DELIVER)}


def resolve_route(name: str) -> Route:
    try:
        return ROUTES[name]
    except KeyError:
        raise KeyError(
            "unknown route %r (known: %s)" % (name, ", ".join(sorted(ROUTES)))
        )


def plan_steps(route: Route, start_step: str = ""):
    """Split `route` at `start_step` -> (skipped, to_run).

    An empty start_step runs everything. An unknown one raises rather than
    silently running the whole route, because a mistyped resume point must not
    quietly repeat motion that has already happened.
    """
    if not start_step:
        return [], list(route.steps)
    names = route.step_names
    if start_step not in names:
        raise KeyError(
            "unknown start_step %r for route %r (steps: %s)"
            % (start_step, route.name, ", ".join(names))
        )
    index = names.index(start_step)
    return list(route.steps[:index]), list(route.steps[index:])
