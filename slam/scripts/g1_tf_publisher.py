#!/usr/bin/env python3
"""Dynamic TF from FAST-LIO2's world frame to the G1's URDF root.

    camera_init --(FAST-LIO2 /laserMapping, dynamic)--> body
    camera_init --(THIS NODE,               dynamic)--> pelvis
    pelvis      --(robot_state_publisher, waist_yaw_joint)--> torso_link
    torso_link  --(robot_state_publisher, fixed)--> mid360_link, head_link, ...

Why this node exists
--------------------
The Mid-360 is bolted to ``torso_link``, and ``torso_link`` hangs off ``pelvis``
through ``waist_yaw_joint``, which is **revolute**. The sensor-to-robot
transform is therefore NOT static: freezing it costs roughly one degree of
heading per degree of waist rotation. This node evaluates the chain every
odometry cycle instead::

    camera_init -> pelvis = (camera_init -> body)        from /Odometry
                          . (body -> mid360_link)        FAST-LIO2 extrinsic, const
                          . (mid360_link -> pelvis)      URDF chain at waist_yaw

``body -> mid360_link`` is deliberately NOT broadcast: ``mid360_link`` already has
``torso_link`` as its parent from ``robot_state_publisher``, and ``body`` already
has ``camera_init`` from FAST-LIO2, so broadcasting it would give a frame two
parents and break TF. It is used as a math constant only, pinned by
``test/test_tf_transforms.py``.

External interfaces this node CONSUMES and does not provide
-----------------------------------------------------------
``/joint_states`` (``sensor_msgs/JointState``) carrying ``waist_yaw_joint`` is
owned by the robot-control / integration team. This node never synthesizes it.
While it is absent or stale, the node stays up, warns, and publishes **nothing** -
a missing transform is far safer than a confidently wrong heading.

``base_link`` is an unresolved integration decision pending the navigation team.
This node does not invent it and does not publish it.

Two ways to run
---------------
* As a ROS node (``g1_tf.launch``): private params from ``config/g1_tf.yaml``,
  plus ``~urdf_path`` passed as an already-resolved path by the launch file.
* Offline check, no ROS master::

      g1_tf_publisher.py --check --config <g1_tf.yaml> --urdf <model.urdf>

  Exits 0 (PASS), 2 (WAIT - valid but pending an external interface or an
  unresolved decision) or 1 (FAIL - malformed config, or URDF/constant drift).
"""
import argparse
import os
import sys

# Runs both as an installed ROS node (g1_slam on PYTHONPATH via catkin) and
# straight from the source tree for the offline --check.
try:
    from g1_slam import tf_chain
except ImportError:                                                  # pragma: no cover
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "src"))
    from g1_slam import tf_chain

REQUIRED_KEYS = (
    "world_frame",
    "robot_root_frame",
    "imu_frame",
    "lidar_frame",
    "odom_topic",
    "joint_states_topic",
    "waist_yaw_joint_name",
    "joint_states_timeout",
)

# The tree is fixed by FAST-LIO2 and the official URDF; the config names frames,
# it does not get to re-wire them.
REQUIRED_FRAMES = {
    "world_frame": tf_chain.WORLD_FRAME,
    "robot_root_frame": tf_chain.ROBOT_ROOT_FRAME,
    "imu_frame": tf_chain.IMU_FRAME,
    "lidar_frame": tf_chain.LIDAR_FRAME,
}

# Exit codes mirror slam_health_check.py: PASS=0, FAIL=1, WAIT=2.
EXIT_PASS, EXIT_FAIL, EXIT_WAIT = 0, 1, 2

DEFAULT_CONFIG = {
    "world_frame": tf_chain.WORLD_FRAME,
    "robot_root_frame": tf_chain.ROBOT_ROOT_FRAME,
    "imu_frame": tf_chain.IMU_FRAME,
    "lidar_frame": tf_chain.LIDAR_FRAME,
    "odom_topic": "/Odometry",
    "joint_states_topic": "/joint_states",
    "waist_yaw_joint_name": "waist_yaw_joint",
    "joint_states_timeout": 0.5,
    "base_link_alias": "",
    "warn_period": 5.0,
}


class ConfigError(ValueError):
    """Raised when a g1_tf configuration is structurally invalid."""


def validate_config(cfg):
    """Validate a config mapping and return it normalised. Raises ConfigError."""
    if not isinstance(cfg, dict):
        raise ConfigError("config root must be a mapping, got {}".format(type(cfg).__name__))

    missing = [k for k in REQUIRED_KEYS if k not in cfg]
    if missing:
        raise ConfigError("missing keys: {}".format(", ".join(missing)))

    out = dict(cfg)

    for key, expected in sorted(REQUIRED_FRAMES.items()):
        value = out[key]
        if not isinstance(value, str) or not value.strip():
            raise ConfigError("'{}' must be a non-empty string".format(key))
        if value != expected:
            raise ConfigError(
                "{}={!r} but the tree is fixed by FAST-LIO2 and the official URDF; "
                "it must be {!r}".format(key, value, expected))

    for key in ("odom_topic", "joint_states_topic", "waist_yaw_joint_name"):
        if not isinstance(out[key], str) or not out[key].strip():
            raise ConfigError("'{}' must be a non-empty string".format(key))

    try:
        out["joint_states_timeout"] = float(out["joint_states_timeout"])
    except (TypeError, ValueError):
        raise ConfigError("joint_states_timeout is not a number: {!r}".format(
            out["joint_states_timeout"]))
    if out["joint_states_timeout"] <= 0.0:
        raise ConfigError("joint_states_timeout must be > 0, got {}".format(
            out["joint_states_timeout"]))

    out["warn_period"] = float(out.get("warn_period", DEFAULT_CONFIG["warn_period"]))

    # An unresolved cross-team decision: empty means "not decided", which is the
    # only supported value. Anything else would be an invented transform.
    alias = out.get("base_link_alias", "")
    if alias is None:
        alias = ""
    if not isinstance(alias, str):
        raise ConfigError("base_link_alias must be a string (empty = unresolved)")
    if alias.strip():
        raise ConfigError(
            "base_link_alias={!r}: the pelvis -> base_link mapping is an unresolved "
            "integration decision pending the navigation team, and publishing a guessed "
            "transform is not supported. Leave it empty.".format(alias))
    out["base_link_alias"] = ""

    # ROS substitutions are expanded by roslaunch in launch XML, never inside a
    # plain YAML file loaded by rosparam. A literal "$(find ...)" here means the
    # node would receive the unexpanded string.
    for key, value in sorted(out.items()):
        if isinstance(value, str) and "$(" in value:
            raise ConfigError(
                "{}={!r} contains an unexpanded ROS substitution; YAML is not "
                "substituted - pass resolved paths from the launch file".format(key, value))

    return out


def _summary(cfg, urdf_path):
    return ("{} -> {} via {} -> {} at {}; odom={} joints={} ({})  urdf={}".format(
        cfg["world_frame"], cfg["robot_root_frame"], cfg["imu_frame"], cfg["lidar_frame"],
        cfg["waist_yaw_joint_name"], cfg["odom_topic"], cfg["joint_states_topic"],
        "timeout {}s".format(cfg["joint_states_timeout"]), urdf_path))


def run_check(config_path, urdf_path):
    """Offline validation of the config and the vendored URDF. No ROS master."""
    try:
        import yaml
        with open(config_path) as handle:
            raw = yaml.safe_load(handle)
    except Exception as exc:
        print("FAIL: cannot read {}: {}".format(config_path, exc))
        return EXIT_FAIL

    try:
        cfg = validate_config(raw)
    except ConfigError as exc:
        print("FAIL: {}: {}".format(config_path, exc))
        return EXIT_FAIL

    if not urdf_path:
        print("FAIL: --check requires --urdf (the launch file passes it as ~urdf_path)")
        return EXIT_FAIL
    if not os.path.isfile(urdf_path):
        print("FAIL: URDF not found at {}".format(urdf_path))
        return EXIT_FAIL

    try:
        problems = tf_chain.verify_against_urdf(urdf_path)
    except Exception as exc:
        print("FAIL: cannot parse URDF {}: {}".format(urdf_path, exc))
        return EXIT_FAIL
    if problems:
        print("FAIL: URDF no longer matches tf_chain constants: {}".format("; ".join(problems)))
        return EXIT_FAIL

    # Structurally sound. Two things are still outstanding by design, and both
    # are owned by other teams, so report WAIT rather than PASS.
    print("WAIT: config and URDF valid; runtime TF still needs {} carrying {} "
          "(owned by robot-control/integration), and pelvis -> base_link remains an "
          "unresolved navigation-team decision. {}".format(
              cfg["joint_states_topic"], cfg["waist_yaw_joint_name"],
              _summary(cfg, urdf_path)))
    return EXIT_WAIT


class PelvisTfPublisher(object):
    """Broadcasts camera_init -> pelvis whenever odometry and joint state agree."""

    def __init__(self, cfg, urdf_path):
        import rospy
        import tf2_ros

        self._rospy = rospy
        self.cfg = cfg
        self.urdf_path = urdf_path
        self.broadcaster = tf2_ros.TransformBroadcaster()

        self._waist_yaw = None
        self._waist_stamp = None
        self._published = 0

    def on_joint_states(self, msg):
        """Latch waist_yaw_joint. Never synthesized - absent means absent."""
        name = self.cfg["waist_yaw_joint_name"]
        try:
            index = list(msg.name).index(name)
        except ValueError:
            self._rospy.logwarn_throttle(
                self.cfg["warn_period"],
                "g1_tf_publisher: %s carries %d joint(s) but not %r; no TF published. "
                "This topic is owned by robot-control/integration.",
                self.cfg["joint_states_topic"], len(msg.name), name)
            return
        if index >= len(msg.position):
            self._rospy.logwarn_throttle(
                self.cfg["warn_period"],
                "g1_tf_publisher: %r present in %s but position[] is too short; "
                "no TF published.", name, self.cfg["joint_states_topic"])
            return

        self._waist_yaw = float(msg.position[index])
        stamp = msg.header.stamp
        self._waist_stamp = self._rospy.Time.now() if stamp.is_zero() else stamp

    def on_odometry(self, msg):
        """Compose the chain and broadcast, or stay silent and explain why."""
        if self._waist_yaw is None or self._waist_stamp is None:
            self._rospy.logwarn_throttle(
                self.cfg["warn_period"],
                "g1_tf_publisher: WAITING for %s carrying %r - publishing no "
                "%s -> %s. That topic is an external interface owned by "
                "robot-control/integration; this node will not fabricate it.",
                self.cfg["joint_states_topic"], self.cfg["waist_yaw_joint_name"],
                self.cfg["world_frame"], self.cfg["robot_root_frame"])
            return

        age = (msg.header.stamp - self._waist_stamp).to_sec()
        if abs(age) > self.cfg["joint_states_timeout"]:
            self._rospy.logwarn_throttle(
                self.cfg["warn_period"],
                "g1_tf_publisher: latest %r is %.3fs from this odometry stamp "
                "(timeout %.3fs); publishing no %s -> %s rather than a stale heading.",
                self.cfg["waist_yaw_joint_name"], age, self.cfg["joint_states_timeout"],
                self.cfg["world_frame"], self.cfg["robot_root_frame"])
            return

        if msg.child_frame_id and msg.child_frame_id != self.cfg["imu_frame"]:
            self._rospy.logwarn_throttle(
                self.cfg["warn_period"],
                "g1_tf_publisher: %s child_frame_id is %r, expected %r; the chain "
                "assumes odometry tracks the Mid-360 IMU.",
                self.cfg["odom_topic"], msg.child_frame_id, self.cfg["imu_frame"])
            return

        pose = msg.pose.pose
        try:
            world_to_body = tf_chain.transform_from_pose(
                (pose.position.x, pose.position.y, pose.position.z),
                (pose.orientation.x, pose.orientation.y,
                 pose.orientation.z, pose.orientation.w))
        except ValueError as exc:
            self._rospy.logwarn_throttle(
                self.cfg["warn_period"],
                "g1_tf_publisher: unusable odometry orientation (%s); no TF published.", exc)
            return

        world_to_pelvis = tf_chain.world_to_pelvis(world_to_body, self._waist_yaw)
        self.broadcaster.sendTransform(self._to_msg(world_to_pelvis, msg.header.stamp))

        if self._published == 0:
            self._rospy.loginfo("g1_tf_publisher: publishing %s -> %s (first transform sent)",
                                self.cfg["world_frame"], self.cfg["robot_root_frame"])
        self._published += 1

    def _to_msg(self, matrix, stamp):
        from geometry_msgs.msg import TransformStamped

        translation = tf_chain.translation_of(matrix)
        quaternion = tf_chain.quaternion_of(matrix)

        out = TransformStamped()
        out.header.stamp = stamp
        out.header.frame_id = self.cfg["world_frame"]
        out.child_frame_id = self.cfg["robot_root_frame"]
        out.transform.translation.x = translation[0]
        out.transform.translation.y = translation[1]
        out.transform.translation.z = translation[2]
        (out.transform.rotation.x, out.transform.rotation.y,
         out.transform.rotation.z, out.transform.rotation.w) = quaternion
        return out


def run_node():
    import rospy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState

    rospy.init_node("g1_tf_publisher")

    raw = {key: rospy.get_param("~" + key, default)
           for key, default in DEFAULT_CONFIG.items()}
    urdf_path = rospy.get_param("~urdf_path", "")

    try:
        cfg = validate_config(raw)
    except ConfigError as exc:
        rospy.logfatal("g1_tf_publisher: invalid configuration: %s", exc)
        return EXIT_FAIL

    if not urdf_path:
        rospy.logfatal("g1_tf_publisher: ~urdf_path is empty. g1_tf.launch must pass the "
                       "resolved URDF path (ROS substitutions are not expanded inside YAML).")
        return EXIT_FAIL
    if not os.path.isfile(urdf_path):
        rospy.logfatal("g1_tf_publisher: ~urdf_path does not exist: %s", urdf_path)
        return EXIT_FAIL

    try:
        problems = tf_chain.verify_against_urdf(urdf_path)
    except Exception as exc:
        rospy.logfatal("g1_tf_publisher: cannot parse URDF %s: %s", urdf_path, exc)
        return EXIT_FAIL
    if problems:
        rospy.logfatal("g1_tf_publisher: URDF no longer matches tf_chain constants, refusing "
                       "to publish a stale chain: %s", "; ".join(problems))
        return EXIT_FAIL

    node = PelvisTfPublisher(cfg, urdf_path)
    rospy.Subscriber(cfg["joint_states_topic"], JointState, node.on_joint_states, queue_size=10)
    rospy.Subscriber(cfg["odom_topic"], Odometry, node.on_odometry, queue_size=10)

    rospy.loginfo("g1_tf_publisher: %s", _summary(cfg, urdf_path))
    rospy.logwarn("g1_tf_publisher: no %s -> %s will be published until %s carries %r. "
                  "pelvis -> base_link stays unpublished (unresolved with navigation).",
                  cfg["world_frame"], cfg["robot_root_frame"],
                  cfg["joint_states_topic"], cfg["waist_yaw_joint_name"])
    rospy.spin()
    return EXIT_PASS


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", action="store_true",
                        help="offline: validate --config and --urdf, then exit (no ROS master)")
    parser.add_argument("--config", help="path to g1_tf.yaml (required with --check)")
    parser.add_argument("--urdf", help="path to the vendored URDF (required with --check)")
    # roslaunch appends __name:=/__log:= etc. - ignore anything unrecognised.
    args, _unknown = parser.parse_known_args()

    if args.check:
        if not args.config:
            parser.error("--check requires --config")
        return run_check(args.config, args.urdf)
    return run_node()


if __name__ == "__main__":
    sys.exit(main())
