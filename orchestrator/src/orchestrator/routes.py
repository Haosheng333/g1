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
# blocker is reported against a real plan. The frame conversion is now
# implemented (caps.PLANNER_FRAME_CONVERSION, a service), but
# caps.PERCEPTION_EXTRINSICS is still unresolved, so this route cannot execute
# for real: a correct base->planner transform cannot repair a point computed
# with the wrong camera extrinsics.

GRASP = Route(
    name="grasp",
    description="Navigate to the object, plan the full grasp, then execute it segment by segment",
    requires=(caps.LOCALIZATION, caps.MAP, caps.JOINT_STATES, caps.NAVIGATION,
              caps.MOTION_GATE, caps.PERCEPTION, caps.ARM_PLANNING, caps.ARM_CONTROL,
              caps.HAND, caps.PLANNER_FRAME_CONVERSION, caps.PERCEPTION_EXTRINSICS,
              caps.STOW_POSE),
    steps=(
        Step("await_localization", "await_localization", caps.LOCALIZATION),
        Step("enable_motion", "enable_motion", caps.MOTION_GATE, motion=True),
        Step("navigate", "navigate", caps.NAVIGATION, motion=True,
             description="drive to the object's waypoint"),
        Step("detect_object", "detect_object", caps.PERCEPTION),
        Step("to_planner_frame", "to_planner_frame", caps.PLANNER_FRAME_CONVERSION,
             description="call arm_planning's /arm/convert_observation: base_link -> "
                         "the planner's torso_link, incl. the 10 mm convention offset"),
        # Everything below moves nothing until the whole grasp AND the stow
        # recovery path have been planned and validated (spec 2.4).
        Step("plan_grasp", "plan_grasp", caps.ARM_PLANNING,
             description="plan all four segments BEFORE any arm motion (doc 13.3)"),
        Step("plan_stow", "plan_stow", caps.STOW_POSE,
             description="spec 2.4: plan the stow/carry recovery path from the END of "
                         "retract BEFORE the grasp starts moving, never after the hand "
                         "has closed"),
        Step("execute_to_pregrasp", "execute_segment", caps.ARM_CONTROL, motion=True),
        Step("open_gripper", "open_gripper", caps.HAND, motion=True),
        Step("execute_approach", "execute_segment", caps.ARM_CONTROL, motion=True),
        Step("close_gripper", "close_gripper", caps.HAND, motion=True),
        Step("execute_lift", "execute_segment", caps.ARM_CONTROL, motion=True),
        Step("execute_retract", "execute_segment", caps.ARM_CONTROL, motion=True),
        Step("record_held_object", "record_held_object",
             description="the ONLY writer of held-object state; a delivery reads it"),
        Step("execute_stow", "execute_segment", caps.ARM_CONTROL, motion=True,
             description="execute the stow path planned before the grasp began"),
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
    # No frame conversion and no perception here: a delivery navigates to a
    # waypoint and plans to a CONFIGURED pose already in the planner's frame, so
    # neither PLANNER_FRAME_CONVERSION nor PERCEPTION_EXTRINSICS applies. They
    # reach deliver_task through the grasp phase, where an observation IS
    # converted.
    requires=(caps.LOCALIZATION, caps.MAP, caps.JOINT_STATES, caps.NAVIGATION,
              caps.MOTION_GATE, caps.ARM_PLANNING, caps.ARM_CONTROL, caps.HAND,
              caps.TACTILE_HANDOVER,
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


# --- deliver_task: the ONE command the framework asks for ----------------------------
#
# WHY THIS EXISTS. Spec 0.4 / 6.2 want a single task command covering navigation
# to the table, perception, grasp planning and execution, carrying to point B,
# handover and return to a stowed pose. Before this, `grasp` and `deliver` were
# two separate services and `deliver` begins with `require_held_object`, so the
# full task needed two calls with a human deciding in between. There was no
# full-task entry point.
#
# HOW IT COMPOSES. In-process: the grasp steps followed by the delivery steps,
# walked by ONE sequencer under ONE execution lock. It deliberately does NOT
# call /orchestrator/run_grasp or /orchestrator/run_deliver, because a route
# holding the execution lock calling a public route service would hit its own
# lock and be refused.
#
# STEP NAMES ARE DISAMBIGUATED. `await_localization`, `enable_motion` and
# `navigate` appear in both phases. `plan_steps` resolves `start_step` with
# `names.index()`, which takes the FIRST match, so leaving the names duplicated
# would make a resume at "navigate" silently restart the grasp drive instead of
# the delivery drive. The delivery-phase copies are therefore renamed; the
# `action` each dispatches to is unchanged.
#
# KNOWN CONTRACT GAP, deliberately not papered over: RunRoute.srv carries ONE
# `target` field, and this route needs TWO destinations -- the object/table
# waypoint and the delivery point B. RouteContext has no `destination`, so the
# delivery-phase navigate would reuse the same target. Inventing a convention
# (splitting `target` on a separator, say) would be fabricating an interface, so
# the route is registered with the gap recorded here and in
# docs/DATA_PIPELINE_VALIDATION.md. It changes nothing operationally today: the
# route inherits every unresolved capability from both phases, so preflight
# refuses it before any step runs.


def _rename(step: Step, new_name: str, action: str = "") -> Step:
    """A delivery-phase copy of `step` under a non-colliding name."""
    return Step(name=new_name, action=action or step.action, capability=step.capability,
                motion=step.motion, description=step.description)


def _deliver_phase_step(step: Step) -> Step:
    """Rename a colliding step, and point the delivery drive at `destination`.

    The delivery navigate dispatches to its OWN action so it reads
    ctx.destination instead of ctx.target; `navigate` keeps reading the pickup
    target. One implementation serves both.
    """
    if step.name == "navigate":
        return _rename(step, "navigate_to_recipient", action="navigate_to_destination")
    if step.name in _DELIVER_PHASE_RENAMES:
        return _rename(step, _DELIVER_PHASE_RENAMES[step.name])
    return step


_DELIVER_PHASE_RENAMES = {
    "await_localization": "deliver_await_localization",
    "enable_motion": "deliver_enable_motion",
    "navigate": "navigate_to_recipient",
}

#: Routes whose delivery leg needs a second waypoint. `run` refuses these with
#: an empty `destination` rather than silently driving to the pickup waypoint.
ROUTES_NEEDING_DESTINATION = ("deliver_task", "fetch_deliver")

DELIVER_TASK = Route(
    name="deliver_task",
    description=(
        "ONE command for the whole framework task: navigate, detect, plan and execute "
        "the grasp, stow, carry to the delivery point, hand over on a verified pull, "
        "retract home"
    ),
    # The union of both phases, so preflight refuses the whole task up front
    # rather than discovering a missing home pose after the hand has opened.
    requires=tuple(dict.fromkeys(GRASP.requires + DELIVER.requires)),
    steps=GRASP.steps + tuple(
        _deliver_phase_step(s) for s in DELIVER.steps
    ),
)


# --- fetch_deliver: the spoken task, on the modules that have run on this robot ---
#
#   "go to the desk, pick up a can, go to the chair, and pass me, over"
#
# WHY A SEPARATE ROUTE. `deliver_task` is built on arm_planning / arm_control /
# hand_control and stays NOT_READY on unresolved poses and extrinsics. What has
# actually been exercised on the G1 is a different set of interfaces:
#
#   navigation   STANDING waypoints (slam/maps/<map>/standing_waypoints.yaml),
#                driven through /move_base_simple/goal with NO approach offset
#   perception   /perception/get_object_position, already in torso_link, which
#                is the frame arm_grasp plans in -- so no frame conversion
#   arm_grasp    /g1_arm_simple/{plan,execute}_grasp_from_point: NN IK + voxel
#                A*, pregrasp, hand pre_grasp, approach, grasp_5f, lift, retract,
#                home -- all inside that node, exactly as grasp_point.py drives it
#   handover     handover_node (/handover/start, /handover/state): the held arm
#                to the handover pose, g1_hands' tactile monitor armed, and on a
#                detected pull: release, settle, GoHomeArm
#
# This is g1_intellect main_process's navigate -> grasp -> navigate -> handover
# plan, sequenced here through DIP's own interfaces. The orchestrator still
# implements none of it: every step is one service call or one topic wait.
#
# ORDER, and why:
#   * the whole grasp is PLANNED (plan_grasp_point, no motion) before anything
#     reaches for the object, so an unreachable point fails with the planner's
#     own reason while the arm is still parked;
#   * record_held_object follows a successful execute_grasp: it is the ONLY
#     writer of held-object state, and the handover steps read it;
#   * hand_back_arms (release_arm_control) comes BEFORE the delivery drive, as
#     main_process's @walking_task does: with the arm back at home holding the
#     can, the G1's own controller should own the arms while it walks. The arm
#     node refuses the hand-back if an arm is not home; that is reported, and
#     the robot then walks with the SDK holding the arm, as main_process does;
#   * handover_start only after the chassis has settled, so the arm does not
#     plan from a rocking base; await_handover then waits for the human pull
#     with the execution lock held, and a timeout releases NOTHING.
#
# `target` and `destination` are STANDING waypoint names; `label` is the
# perception class. The node resolves both waypoints against the active map
# before preflight, so a missing record or a map mismatch is a blocker, not a
# surprise after the first drive.

FETCH_DELIVER = Route(
    name="fetch_deliver",
    description=(
        "Spoken task: drive to the pickup standing pose, detect and grasp the object "
        "with arm_grasp, drive to the recipient's standing pose, hand it over on a "
        "tactile pull through handover_node"
    ),
    requires=(caps.LOCALIZATION, caps.MAP, caps.JOINT_STATES, caps.NAVIGATION,
              caps.MOTION_GATE, caps.PERCEPTION, caps.ARM_GRASP, caps.REMOTE_HANDS,
              caps.HANDOVER_NODE),
    steps=(
        Step("await_localization", "await_localization", caps.LOCALIZATION),
        Step("enable_motion", "enable_motion", caps.MOTION_GATE, motion=True),
        Step("navigate_to_pickup", "navigate_standing", caps.NAVIGATION, motion=True,
             description="exact standing pose `target` via /move_base_simple/goal, "
                         "no approach offset, guarded by saw_active"),
        Step("settle_at_pickup", "settle_chassis",
             description="let the base stop rocking before the camera is read"),
        Step("detect_object", "detect_object", caps.PERCEPTION,
             description="/perception/get_object_position(label) in torso_link"),
        Step("plan_grasp_point", "plan_grasp_point", caps.ARM_GRASP,
             description="/g1_arm_simple/plan_grasp_from_point: the whole grasp planned, "
                         "no motion"),
        Step("execute_grasp", "execute_grasp_point", caps.ARM_GRASP, motion=True,
             description="/g1_arm_simple/execute_grasp_from_point: pregrasp, pre_grasp, "
                         "approach, grasp_5f, lift, retract, home"),
        Step("record_held_object", "record_held_object",
             description="the ONLY writer of held-object state"),
        Step("hand_back_arms", "release_arm_control", caps.ARM_GRASP, motion=True,
             description="arms back to the robot's own controller before walking"),
        Step("navigate_to_recipient", "navigate_standing_destination", caps.NAVIGATION,
             motion=True, description="exact standing pose `destination`"),
        Step("settle_at_recipient", "settle_chassis"),
        Step("handover_start", "handover_start", caps.HANDOVER_NODE, motion=True,
             description="/handover/start: held arm to the handover pose, tactile "
                         "monitor armed; returns once armed"),
        Step("await_handover", "await_handover", caps.HANDOVER_NODE,
             description="wait on /handover/state for done; a timeout disarms and "
                         "releases nothing"),
        Step("hand_back_arms_after_handover", "release_arm_control", caps.ARM_GRASP,
             motion=True, description="GoHomeArm has run inside handover_node; hand "
                                      "the arms back"),
    ),
)

#: Routes whose `target` / `destination` are STANDING waypoint names that the
#: node resolves against the active map before preflight.
ROUTES_USING_STANDING_WAYPOINTS = ("fetch_deliver",)

ROUTES = {r.name: r for r in (NAVIGATE_REPORT, GRASP, DELIVER, DELIVER_TASK, FETCH_DELIVER)}


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
