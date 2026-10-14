"""arm_planning -- catkin package for the Module 2 arm planner.

`plan_conversion` holds every piece of logic that does NOT need ROS, torch,
scipy, trimesh or fcl, so it can be unit-tested offline. The ROS node in
scripts/arm_planning_node.py is a thin wrapper over it.

The planner algorithms themselves live in arm_planning/scripts/ as flat
modules (doc section 8.3) and are imported unchanged; this package adds the
ROS1 service surface around them and never modifies them.
"""
