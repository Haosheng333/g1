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
        services=("/auto_release_monitor/arm", "/auto_release_monitor/disarm"),
        ready_flags=("/auto_release_monitor/ready",),
    ),
    FRAME_CONTRACT: Capability(
        key=FRAME_CONTRACT, owner="navigation + arm_planning",
        unresolved=(
            "the perception -> arm_planning frame contract is unresolved. perception "
            "reports object positions in base_link (documented as pelvis at zero waist "
            "yaw), arm_planning takes torso_link (doc 2.3, 13.4), slam refuses to "
            "publish pelvis -> base_link at all (g1_tf.yaml base_link_alias is empty, "
            "marked UNRESOLVED pending the navigation team), and the two vendored URDFs "
            "place torso_link 10 mm apart (slam waist_yaw_joint z=0.044 vs arm_planning "
            "z=0.054) which is the size of arm_planning's own 0.01 m tolerance. "
            "Converting a perception point into a planner target would be a guess, so "
            "no route that needs it can run"
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
