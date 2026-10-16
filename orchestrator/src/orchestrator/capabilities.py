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
FRAME_CONTRACT = "frame_contract"
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
    FRAME_CONTRACT: Capability(
        key=FRAME_CONTRACT, owner="navigation + arm_planning",
        unresolved=(
            "the perception -> arm_planning frame conversion is not implemented. "
            "Navigation's half IS settled (docs/FRAME_CONTRACT.md): base_link is an "
            "identity alias of pelvis at EVERY waist angle -- the pelvis is the "
            "waist's parent, so it never rotates with the waist -- published once by "
            "system_bringup with with_base_link on by default. Two things still block "
            "a real run. (1) The two vendored URDFs place torso_link 10 mm apart (slam "
            "waist_yaw_joint z=0.044 vs arm_planning z=0.054), the size of "
            "arm_planning's own 0.01 m tolerance. That may be two different DEFINITIONS "
            "of the frame rather than a wrong URDF, so the TF torso_link must be shown "
            "to match the planner's convention before a target is routed through it; "
            "run tools/check_urdf_agreement.py. (2) Nobody has implemented the "
            "conversion: it is one TF lookup, torso_link <- base_link, which carries "
            "the live waist yaw, and it belongs in arm_planning, not here. "
            "perception's camera extrinsics are a SEPARATE requirement: they are "
            "hard-coded for a torso-mounted D435 that DOES rotate with the waist, so "
            "perception's output is only valid near zero waist yaw regardless of how "
            "base_link is defined. The zero-waist limitation belongs to those "
            "extrinsics, never to base_link"
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
