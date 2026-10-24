"""Turn a recorded STANDING pose into navigation's exact-pose goal. No rospy.

Shared by `scripts/goto_waypoint.py` and by the orchestrator's `fetch_deliver`
route, so there is exactly one place that knows how a standing record becomes
a `/move_base_simple/goal` and how the map identity is checked first.

WHY THE GOAL IS EXACT. `/navigation/navigate` looks an OBJECT up in
semantic.yaml and stands `approach_distance` away from it. A standing record is
the opposite: it is where the robot itself stood, so the pose goes through
unchanged -- no approach offset, no rounding, no frame maths beyond yaw ->
quaternion. Offsetting it again would park the robot a metre from the spot that
was deliberately chosen.

WHY THE IDENTITY IS CHECKED. x/y/yaw mean something only on the map they were
measured in. The record file carries the identity of that map; the live map's
`manifest.yaml` carries the identity of the map the robot is localized to. A
mismatch is refused, never guessed around.
"""
import math
import os

import yaml

from g1_slam import standing_store as store

# Navigation's terminal-state contract, mirrored rather than imported: g1_slam
# must NOT depend on g1_navigation, which already depends on g1_slam (that would
# be a dependency cycle). slam/test/test_waypoint_scripts.py reads navigation's
# own source and fails if these sets ever drift apart.
ACTIVE = ("PLANNING", "FOLLOWING", "RECOVERING")
TERMINAL = ("SUCCESS", "FAILURE")
GOAL_TOPIC = "/move_base_simple/goal"


def quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def build_goal(record, frame, stamp, pose_cls=None):
    """record -> PoseStamped in `frame`.

    The record's x/y/yaw go through UNCHANGED: no approach offset, no rounding.
    z is 0 because the pose is a standing spot on the floor and `map` is the
    z-UP navigation frame. `pose_cls` lets tests pass a stand-in; by default the
    real geometry_msgs type is imported here, lazily, so this module stays
    importable without ROS.
    """
    if pose_cls is None:
        from geometry_msgs.msg import PoseStamped
        pose_cls = PoseStamped
    goal = pose_cls()
    goal.header.frame_id = frame
    goal.header.stamp = stamp
    goal.pose.position.x = float(record["x"])
    goal.pose.position.y = float(record["y"])
    goal.pose.position.z = 0.0
    (goal.pose.orientation.x, goal.pose.orientation.y,
     goal.pose.orientation.z, goal.pose.orientation.w) = quaternion_from_yaw(
        float(record["yaw"]))
    return goal


def live_identity(map_dir, map_frame="map", robot_frame="pelvis"):
    """The identity block of the map in `map_dir`, from its manifest.yaml."""
    manifest = os.path.join(map_dir, "manifest.yaml")
    if not os.path.isfile(manifest):
        raise RuntimeError("no manifest at %s: cannot confirm which map is loaded"
                           % manifest)
    with open(manifest, encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    return store.identity(str(data.get("map_name", "")).strip(),
                          str(data.get("created_at", "")).strip(),
                          map_frame, robot_frame)


def waypoints_file(maps_dir, map_name):
    return os.path.join(maps_dir, map_name, "standing_waypoints.yaml")


def resolve(maps_dir, map_name, name, map_frame="map", robot_frame="pelvis"):
    """Look `name` up in the active map's standing_waypoints.yaml.

    -> (key, record, goal_frame). Raises with the reason when the file or the
    record is missing, or when the file belongs to a different map than the one
    in <maps_dir>/<map_name>/manifest.yaml. Pure file reading: nothing here
    publishes, moves or opens a gate.
    """
    map_dir = os.path.join(maps_dir, map_name)
    path = waypoints_file(maps_dir, map_name)
    if not os.path.isfile(path):
        raise store.StoreError(
            "no standing waypoints recorded for map %r (%s is missing); record one with "
            "`rosrun g1_slam record_waypoints.py`" % (map_name, path))
    file_ident, key, record = store.get(path, name)
    loaded = live_identity(map_dir, map_frame, robot_frame)
    if file_ident and not store.same_map(file_ident, loaded):
        raise store.MapMismatch(store.describe_mismatch(file_ident, loaded))
    return key, record, (record.get("map_frame") or map_frame)
