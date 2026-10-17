"""The capabilities a route can need, and which package owns each one.

Nothing here talks to ROS. `preflight` turns these declarations into a blocker
list using an injected probe, so the whole readiness decision is testable
offline.
"""
from __future__ import annotations

from dataclasses import dataclass

# --- capability keys ----------------------------------------------------------------

LOCALIZATION = "localization"
MAP = "map"
JOINT_STATES = "joint_states"
NAVIGATION = "navigation"
MOTION_GATE = "motion_gate"
PERCEPTION = "perception"
ARM_PLANNING = "arm_planning"
ARM_CONTROL = "arm_control"
HAND = "hand"
TACTILE_HANDOVER = "tactile_handover"
#: The base_link -> planner torso_link conversion. IMPLEMENTED (2026-10) and
#: owned by arm_planning: a service, so it is probed like any other.
PLANNER_FRAME_CONVERSION = "planner_frame_conversion"
#: perception's camera -> base_link extrinsics. A SEPARATE prerequisite, still
#: unresolved. A correct base->planner transform cannot repair a point that was
#: already computed with the wrong camera extrinsics.
PERCEPTION_EXTRINSICS = "perception_extrinsics"
#: Poses and evidence a delivery needs. These are known missing at STARTUP, so
#: they belong in preflight: discovering them at step time would mean finding
#: out there is no home pose only after the hand has already opened.
HANDOVER_POSE = "handover_pose"
STOW_POSE = "stow_pose"
HOME_POSE = "home_pose"
RELEASE_COMPLETION = "release_completion"


@dataclass(frozen=True)
class Capability:
    key: str
    owner: str
    #: Services that must exist. "{side}" is filled from the configured arm.
    services: tuple = ()
    #: Latched boolean topics that must currently be True.
    ready_flags: tuple = ()
    #: Topics that must simply have a publisher.
    topics: tuple = ()
    #: (topic, max_age_s) pairs that must be arriving and recent. The probe
    #: returns the reason it is not, so a per-topic rule (such as
    #: /joint_states needing waist_yaw_joint) stays with the probe.
    fresh_topics: tuple = ()
    #: Set for a capability that is blocked by an unresolved decision, not by
    #: anything that could come up at runtime.
    unresolved: str = ""


CAPABILITIES = {
    LOCALIZATION: Capability(
        key=LOCALIZATION, owner="slam",
        ready_flags=("/slam/manager/state",),
    ),
    MAP: Capability(
        key=MAP, owner="slam",
        topics=("/map",),
    ),
    JOINT_STATES: Capability(
        key=JOINT_STATES, owner="robot-control / integration",
        # EXTERNAL interface. slam's camera_init -> pelvis TF needs
        # waist_yaw_joint and treats samples older than 0.5 s as stale
        # (slam/config/g1_tf.yaml joint_states_timeout), so the same bound
        # applies here: without it there is no robot pose to navigate from.
        fresh_topics=(("/joint_states", 0.5),),
    ),
    NAVIGATION: Capability(
        key=NAVIGATION, owner="navigation",
        services=("/navigation/navigate",),
        topics=("/navigation/status",),
    ),
    MOTION_GATE: Capability(
        key=MOTION_GATE, owner="navigation",
        services=("/navigation/enable_motion",),
    ),
    PERCEPTION: Capability(
        key=PERCEPTION, owner="perception",
        services=("/perception/get_object_position",),
    ),
    ARM_PLANNING: Capability(
        key=ARM_PLANNING, owner="arm_planning",
        services=("/arm/plan_to_pose", "/arm/plan_grasp_from_point"),
        ready_flags=("/arm/planner/ready",),
    ),
    ARM_CONTROL: Capability(
        key=ARM_CONTROL, owner="arm_control",
        services=("/arm/move_arm_joints",),
    ),
    HAND: Capability(
        key=HAND, owner="hand_control",
        services=("/hand_{side}/pre_grasp", "/hand_{side}/grasp", "/hand_{side}/release"),
    ),
    TACTILE_HANDOVER: Capability(
        key=TACTILE_HANDOVER, owner="handover",
        # /arm is handover/ArmMonitor (side + session_token); /disarm stays
        # std_srvs/Trigger. Preflight only checks the names exist.
        services=("/auto_release_monitor/arm", "/auto_release_monitor/disarm"),
        ready_flags=("/auto_release_monitor/ready",),
    ),
    HANDOVER_POSE: Capability(
        key=HANDOVER_POSE, owner="arm_planning",
        unresolved=(
            "no handover pose is defined anywhere in this repository. A search of "
            "arm_planning/{config,scripts}, arm_control and hand_control/config for "
            "home|rest|stow|neutral|park|tuck returns nothing, and the only "
            "pose-defining config is grasp_geometry (7 grasp-relative keys) plus "
            "training_workspace. arm_planning must publish a pose that is reachable, "
            "inside the trained workspace, collision-free, and sensible to hand to a "
            "person"
        ),
    ),
    STOW_POSE: Capability(
        key=STOW_POSE, owner="arm_planning",
        unresolved=(
            "no stow/carry pose is defined, so there is no validated posture for "
            "walking while holding an object without swinging the arm into the body "
            "or the environment"
        ),
    ),
    HOME_POSE: Capability(
        key=HOME_POSE, owner="arm_planning",
        unresolved=(
            "no home pose is defined. The only candidate is arm_control's "
            "left_target/right_target = [0,0,0,0,0], which is a shared-memory "
            "initialisation seed rather than a validated home, and reaching it needs a "
            "collision-checked plan (Execute_Waypoint_Path does velocity limiting only). "
            "The grasp retract path is not a substitute: it starts at the lift pose and "
            "inherits y and z from the grasped object"
        ),
    ),
    RELEASE_COMPLETION: Capability(
        key=RELEASE_COMPLETION, owner="hand_control",
        unresolved=(
            "the hand service cannot establish that a release physically completed. "
            "/hand_{side}/release returns only success and a human-readable message; "
            "revo_hand_link.wait_until_settled does poll fresh read_positions() against "
            "the target within settle_tolerance (deadline settle_timeout_s), but the "
            "result is not exposed, the release preset leaves require_settle absent so "
            "it defaults false, and _execute therefore does not fail on a settling "
            "timeout -- it returns success with 'release commanded, did not settle'. "
            "Until hand_control exposes settling (a structured flag, or the feedback "
            "positions), retraction after a release stays refused. Object transfer "
            "remains separately unverifiable either way"
        ),
    ),
    PLANNER_FRAME_CONVERSION: Capability(
        key=PLANNER_FRAME_CONVERSION, owner="arm_planning",
        services=("/arm/convert_observation",),
        # NOT unresolved any more. The conversion is implemented in
        # arm_planning/src/arm_planning/frame_conversion.py and served by
        # /arm/convert_observation, so readiness is now a runtime fact: is the
        # service up and can it get a torso_link <- base_link transform.
        #
        # The 10 mm is resolved explicitly rather than assumed away. The live TF
        # torso_link (slam/urdf/g1_23dof_mode_10.urdf, waist_yaw z=0.044) and the
        # planner's torso_link (arm_planning/urdf/g1_23dof.urdf, z=0.054) are the
        # same NAME for datums 10 mm apart. pelvis -> shoulder_pitch is identical
        # in both (0.044+0.24778 == 0.054+0.23778 == 0.29178), so neither URDF is
        # wrong and neither was modified: a declared convention translation of
        # -0.010 m in z is applied after the TF lookup, exact at every waist
        # angle because the waist rotates about z.
    ),
    PERCEPTION_EXTRINSICS: Capability(
        key=PERCEPTION_EXTRINSICS, owner="perception",
        unresolved=(
            "perception's camera extrinsics are not yet verified on this robot. "
            "perception/src/perception/m1_perception.py now uses the torso_link -> "
            "camera_link mount calibrated on this robot in g1_intellect (g1_camera, "
            "where torso_link is labelled base_link), as a proper rotation, and "
            "reports points in torso_link, which carries the camera, so the result "
            "holds at every waist angle. What is open: (1) no DIP run has checked "
            "a detected point against a tape-measured object; (2) that calibration "
            "was tuned against g1_intellect's arm (arm_grasp), and it is unconfirmed "
            "whether its torso_link datum is the TF torso_link that "
            "/arm/convert_observation assumes before its -0.010 m planner offset. "
            "Measure one object with the camera up, then clear this"
        ),
    ),
}


def capability(key: str) -> Capability:
    try:
        return CAPABILITIES[key]
    except KeyError:
        raise KeyError("unknown capability %r" % (key,))


def resolve_services(cap: Capability, side: str) -> list:
    """Service names for `cap`, with "{side}" filled in."""
    return [name.format(side=side) for name in cap.services]
