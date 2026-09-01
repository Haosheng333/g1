#!/usr/bin/env python3
"""Configurable static TF that attaches the G1 base to FAST-LIO2's tree.

Part of the G1 TF framework. FAST-LIO2 broadcasts ``camera_init -> body``
(dynamic; ``body`` is the Mid-360 IMU frame). To keep every frame with exactly
one parent, this node hangs the robot base UNDER ``body``::

    camera_init --(FAST-LIO)--> body --(this node, static)--> base_link

so the configured transform is ``parent_frame -> child_frame`` =
``body -> base_link`` (base_link expressed in the Mid-360 IMU frame). The
values are a physical quantity that must be measured on the real robot; until
then this node ships DISABLED (config/g1_tf.yaml: ``enabled: false``) and
refuses to publish an identity guess.

Two ways to run:

* As a ROS node (normal launch via g1_tf.launch): configuration comes from
  private ROS params loaded from config/g1_tf.yaml. If enabled, it broadcasts
  exactly one static transform; if disabled it stays up but publishes nothing.

* Offline check (no ROS master needed)::

      g1_static_tf.py --check --config <path/to/g1_tf.yaml>

  Validates the configuration and exits 0 (PASS), 2 (WAIT - values still the
  unmeasured all-zero placeholder, transform disabled) or 1 (FAIL - malformed,
  or enabled while still an identity placeholder).
"""
import argparse
import math
import sys

REQUIRED_KEYS = ("enabled", "parent_frame", "child_frame", "translation", "rotation_rpy")
TRANSLATION_KEYS = ("x", "y", "z")
ROTATION_KEYS = ("roll", "pitch", "yaw")

# The tree is fixed: camera_init --(FAST-LIO)--> body --(this node)--> base_link.
# The parent must be exactly `body`; camera_init is the fixed world frame and is
# never an allowed parent here (base_link is rigid to the sensor, not the world).
REQUIRED_PARENT_FRAME = "body"
REQUIRED_CHILD_FRAME = "base_link"

# Exit codes mirror slam_health_check.py: PASS=0, FAIL=1, WAIT=2.
EXIT_PASS, EXIT_FAIL, EXIT_WAIT = 0, 1, 2


class ConfigError(ValueError):
    """Raised when a g1_tf configuration is structurally invalid."""


def validate_config(cfg):
    """Validate a config mapping. Return (translation, rotation) as float dicts.

    Raises ConfigError on any structural or type problem.
    """
    if not isinstance(cfg, dict):
        raise ConfigError("config root must be a mapping, got {}".format(type(cfg).__name__))

    missing = [k for k in REQUIRED_KEYS if k not in cfg]
    if missing:
        raise ConfigError("missing keys: {}".format(", ".join(missing)))

    if not isinstance(cfg["enabled"], bool):
        raise ConfigError("'enabled' must be a bool, got {!r}".format(cfg["enabled"]))

    for fk in ("parent_frame", "child_frame"):
        if not isinstance(cfg[fk], str) or not cfg[fk].strip():
            raise ConfigError("'{}' must be a non-empty string".format(fk))
    if cfg["parent_frame"] != REQUIRED_PARENT_FRAME:
        raise ConfigError("parent_frame must be {!r} (got {!r}); camera_init is the "
                          "fixed world frame and is not an allowed parent".format(
                              REQUIRED_PARENT_FRAME, cfg["parent_frame"]))
    if cfg["child_frame"] != REQUIRED_CHILD_FRAME:
        raise ConfigError("child_frame must be {!r} (got {!r})".format(
            REQUIRED_CHILD_FRAME, cfg["child_frame"]))

    trans, rot = cfg["translation"], cfg["rotation_rpy"]
    if not isinstance(trans, dict) or any(k not in trans for k in TRANSLATION_KEYS):
        raise ConfigError("'translation' must be a mapping with keys x, y, z")
    if not isinstance(rot, dict) or any(k not in rot for k in ROTATION_KEYS):
        raise ConfigError("'rotation_rpy' must be a mapping with keys roll, pitch, yaw")

    tvals, rvals = {}, {}
    for k in TRANSLATION_KEYS:
        try:
            tvals[k] = float(trans[k])
        except (TypeError, ValueError):
            raise ConfigError("translation.{} is not a number: {!r}".format(k, trans[k]))
    for k in ROTATION_KEYS:
        try:
            rvals[k] = float(rot[k])
        except (TypeError, ValueError):
            raise ConfigError("rotation_rpy.{} is not a number: {!r}".format(k, rot[k]))

    return tvals, rvals


def is_placeholder(tvals, rvals):
    """True while the transform is still the unmeasured all-zero identity."""
    return all(v == 0.0 for v in tvals.values()) and all(v == 0.0 for v in rvals.values())


def quaternion_from_rpy(roll, pitch, yaw):
    """(x, y, z, w) for an intrinsic yaw-pitch-roll rotation (tf2 convention)."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _summary(cfg, tvals, rvals):
    return ("{} -> {}  xyz=({:.4f}, {:.4f}, {:.4f})  rpy=({:.4f}, {:.4f}, {:.4f})  enabled={}".format(
        cfg["parent_frame"], cfg["child_frame"],
        tvals["x"], tvals["y"], tvals["z"],
        rvals["roll"], rvals["pitch"], rvals["yaw"], cfg["enabled"]))


def run_check(path):
    try:
        import yaml
        with open(path) as f:
            cfg = yaml.safe_load(f)
    except Exception as exc:
        print("FAIL: cannot read {}: {}".format(path, exc))
        return EXIT_FAIL

    try:
        tvals, rvals = validate_config(cfg)
    except ConfigError as exc:
        print("FAIL: {}: {}".format(path, exc))
        return EXIT_FAIL

    placeholder = is_placeholder(tvals, rvals)
    if cfg["enabled"] and placeholder:
        print("FAIL: static TF enabled but the transform is all zeros - identity is "
              "not a real measurement. Measure {} in the {} frame and fill in "
              "{}.".format(cfg["child_frame"], cfg["parent_frame"], path))
        return EXIT_FAIL
    if placeholder:
        print("WAIT: transform not yet measured (all zeros) and disabled - "
              "expected shipped state. " + _summary(cfg, tvals, rvals))
        return EXIT_WAIT
    print("PASS: " + _summary(cfg, tvals, rvals))
    return EXIT_PASS


def run_node():
    import rospy
    import tf2_ros
    from geometry_msgs.msg import TransformStamped

    rospy.init_node("g1_static_tf")

    cfg = {
        "enabled": rospy.get_param("~enabled", False),
        "parent_frame": rospy.get_param("~parent_frame", "body"),
        "child_frame": rospy.get_param("~child_frame", "base_link"),
        "translation": rospy.get_param("~translation", {"x": 0.0, "y": 0.0, "z": 0.0}),
        "rotation_rpy": rospy.get_param("~rotation_rpy", {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}),
    }

    try:
        tvals, rvals = validate_config(cfg)
    except ConfigError as exc:
        rospy.logfatal("g1_static_tf: invalid configuration: %s", exc)
        return EXIT_FAIL

    if not cfg["enabled"]:
        rospy.logwarn("g1_static_tf: DISABLED (config/g1_tf.yaml enabled: false). Not "
                      "publishing %s -> %s. Measure %s in the %s frame, fill in "
                      "g1_tf.yaml and set enabled: true.",
                      cfg["parent_frame"], cfg["child_frame"],
                      cfg["child_frame"], cfg["parent_frame"])
        rospy.spin()
        return EXIT_PASS

    if is_placeholder(tvals, rvals):
        rospy.logfatal("g1_static_tf: enabled but the transform is all zeros. Refusing "
                       "to publish an identity guess as %s -> %s.",
                       cfg["parent_frame"], cfg["child_frame"])
        return EXIT_FAIL

    q = quaternion_from_rpy(rvals["roll"], rvals["pitch"], rvals["yaw"])
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = cfg["parent_frame"]
    t.child_frame_id = cfg["child_frame"]
    t.transform.translation.x = tvals["x"]
    t.transform.translation.y = tvals["y"]
    t.transform.translation.z = tvals["z"]
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(t)
    rospy.loginfo("g1_static_tf: publishing static %s", _summary(cfg, tvals, rvals))
    rospy.spin()
    return EXIT_PASS


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", action="store_true",
                        help="offline: validate --config and exit (no ROS master)")
    parser.add_argument("--config", help="path to g1_tf.yaml (required with --check)")
    # roslaunch appends __name:=/__log:= etc. - ignore anything unrecognised.
    args, _unknown = parser.parse_known_args()

    if args.check:
        if not args.config:
            parser.error("--check requires --config")
        return run_check(args.config)
    return run_node()


if __name__ == "__main__":
    sys.exit(main())
