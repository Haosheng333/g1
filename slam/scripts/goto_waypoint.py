#!/usr/bin/env python3
"""Send the robot back to a recorded STANDING pose.

    rosrun g1_slam goto_waypoint.py _list:=true
    rosrun g1_slam goto_waypoint.py _name:=handover_01
    rosrun g1_slam goto_waypoint.py _name:=handover_01 _dry_run:=true

HOW THE GOAL IS SENT, and why it is this interface
The pose is published to **/move_base_simple/goal** (geometry_msgs/PoseStamped
in the `map` frame) -- navigation's EXACT-pose interface, the same one rviz's
"2D Nav Goal" and g1_navigation's own send_goal_node.py use
(navigation_node.goal_cb -> _accept({"kind": "pose", ...})).

NO APPROACH-DISTANCE OFFSET IS APPLIED, and none must be. `/navigation/navigate`
is the other interface: it looks a named OBJECT up in semantic.yaml and then
stands `approach_distance` away from it. A recorded standing pose is already the
spot the robot stood on, so offsetting it again would park the robot a metre
from the place that was deliberately chosen. These records never go through
/navigation/navigate.

MOTION GATES ARE UNTOUCHED. This script publishes a goal and watches the status;
it never calls /navigation/enable_motion. If the gate is closed the robot will
not move, and that is correct -- the gate is reported here so the reason is
obvious, not bypassed.

RESTART SAFETY. The record stores map-frame coordinates plus the identity of the
map they were measured in. After a restart, once the relocalizer has localized
to that SAME saved map, `map` means the same thing it did during recording, so
x/y/yaw still denote the same physical spot. The identity is re-checked before
the goal is sent; a different map is refused.
"""
import math
import os
import sys
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

from g1_slam import standing_store as store
# The goal mapping and the status contract live in g1_slam.standing_goal so the
# orchestrator's fetch_deliver route drives a standing pose through the very
# same code. Re-exported here under their old names.
from g1_slam.standing_goal import (  # noqa: F401 - re-exported
    ACTIVE, GOAL_TOPIC, TERMINAL, build_goal, live_identity, quaternion_from_yaw,
)


def _param(name, default):
    return rospy.get_param("~" + name, default)


class StatusWatcher(object):
    """Fold /navigation/status into one verdict, with the saw_active guard.

    The guard matters: /navigation/status is latched, so a SUCCESS left over
    from the previous goal is still on the topic when a new one is accepted. A
    terminal state only counts once an ACTIVE state has been seen first.
    """

    def __init__(self):
        self.latest = None
        self.lock = threading.Lock()
        rospy.Subscriber("/navigation/status", String, self._cb, queue_size=10)

    def _cb(self, msg):
        with self.lock:
            self.latest = msg.data

    def wait_terminal(self, timeout):
        saw_active = False
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self.lock:
                status = self.latest
            if status in ACTIVE:
                saw_active = True
            elif status in TERMINAL and saw_active:
                return status
            rate.sleep()
        return "TIMEOUT"


def resolve_paths():
    maps_dir = os.path.expanduser(str(_param("maps_dir", "")))
    if not maps_dir:
        from rospkg import RosPack
        maps_dir = os.path.join(RosPack().get_path("g1_slam"), "maps")
    map_name = str(_param("map_name", "current")).strip() or "current"
    map_dir = os.path.join(maps_dir, map_name)
    waypoints_file = os.path.expanduser(str(_param(
        "waypoints_file", os.path.join(map_dir, "standing_waypoints.yaml"))))
    return map_dir, waypoints_file


def do_list(waypoints_file):
    if not os.path.exists(waypoints_file):
        print("no records yet: %s" % waypoints_file)
        return 0
    ident, waypoints = store.load(waypoints_file)
    print("%s\nmap %r (created %s), %d record(s)\n"
          % (waypoints_file, (ident or {}).get("name"),
             (ident or {}).get("created_at"), len(waypoints)))
    if not waypoints:
        print("  (none)")
        return 0
    print("  %-16s %9s %9s %10s %8s  %s"
          % ("name", "x [m]", "y [m]", "yaw [rad]", "yaw [deg]", "recorded_at"))
    for key in sorted(waypoints):
        w = waypoints[key]
        print("  %-16s %+9.3f %+9.3f %+10.3f %+8.1f  %s"
              % (key, w["x"], w["y"], w["yaw"], math.degrees(w["yaw"]),
                 w.get("recorded_at", "")))
    return 0


def main():
    rospy.init_node("goto_waypoint", anonymous=True)
    map_frame = str(_param("map_frame", "map"))
    robot_frame = str(_param("robot_frame", "pelvis"))
    map_dir, waypoints_file = resolve_paths()

    if bool(_param("list", False)):
        return do_list(waypoints_file)

    name = str(_param("name", "")).strip()
    if not name:
        rospy.logfatal("[goto_waypoint] _name:= is required (or _list:=true)")
        return 2

    try:
        file_ident, key, record = store.get(waypoints_file, name)
    except (KeyError, store.StoreError, OSError) as exc:
        rospy.logfatal("[goto_waypoint] %s", exc)
        return 2

    # Map identity: the coordinates only mean something on the map they were
    # measured in. Refused, never guessed.
    try:
        loaded = live_identity(map_dir, map_frame, robot_frame)
    except RuntimeError as exc:
        rospy.logfatal("[goto_waypoint] %s", exc)
        return 2
    if file_ident and not store.same_map(file_ident, loaded):
        rospy.logfatal("[goto_waypoint] REFUSED: %s",
                       store.describe_mismatch(file_ident, loaded))
        return 2

    goal_frame = record.get("map_frame") or map_frame
    rospy.loginfo("[goto_waypoint] %s: x=%+.3f y=%+.3f yaw=%+.3f rad (%+.1f deg) in %s",
                  key, record["x"], record["y"], record["yaw"],
                  math.degrees(record["yaw"]), goal_frame)
    rospy.loginfo("[goto_waypoint] exact pose via %s; NO approach offset applied",
                  GOAL_TOPIC)

    if bool(_param("dry_run", False)):
        rospy.logwarn("[goto_waypoint] dry run: the goal was NOT published")
        return 0

    # Report the motion gate rather than touching it.
    gate = {"value": None}

    def on_gate(msg):
        gate["value"] = bool(msg.data)

    rospy.Subscriber("/navigation/motion_enabled", Bool, on_gate, queue_size=1)

    watcher = StatusWatcher()
    pub = rospy.Publisher(GOAL_TOPIC, PoseStamped, queue_size=1, latch=True)
    rospy.sleep(0.5)                            # let the latched connection form

    if gate["value"] is False:
        rospy.logwarn("[goto_waypoint] /navigation/motion_enabled is FALSE: the goal "
                      "will be planned but the robot will not move. This script does "
                      "not open the gate -- do that deliberately.")
    elif gate["value"] is None:
        rospy.logwarn("[goto_waypoint] no /navigation/motion_enabled publisher seen; "
                      "the gate state is unknown")

    pub.publish(build_goal(record, goal_frame, rospy.Time.now()))

    result = watcher.wait_terminal(float(_param("timeout", 600.0)))
    rospy.loginfo("[goto_waypoint] %s -> %s", key, result)
    return 0 if result == "SUCCESS" else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(0)
