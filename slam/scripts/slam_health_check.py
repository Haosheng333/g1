#!/usr/bin/env python3
import argparse
import ipaddress
import json
import os
import re
import subprocess
import sys

STATUS_PASS = "PASS"
STATUS_FAIL = "FAIL"
STATUS_WAIT = "WAIT"

EXIT_CODES = {STATUS_PASS: 0, STATUS_FAIL: 1, STATUS_WAIT: 2}
_SEVERITY = {STATUS_PASS: 0, STATUS_WAIT: 1, STATUS_FAIL: 2}

PACKAGE_NAME = "g1_slam"
EXPECTED_HOST_IP = "192.168.123.101"
EXPECTED_HOST_IFACE = "192.168.123.101/24"
EXPECTED_LIDAR_IP = "192.168.123.120"
LIVOX_NODE_NAME = "/livox_lidar_publisher2"
LIDAR_TOPIC = "/livox/lidar"
IMU_TOPIC = "/livox/imu"

FASTLIO_NODE_NAME = "/laserMapping"
ODOM_TOPIC = "/Odometry"
CLOUD_TOPIC = "/cloud_registered"
PATH_TOPIC = "/path"
# Official LiDAR-to-IMU extrinsic for the Mid-360, from fast_lio's own
# config/mid360.yaml. fastlio.yaml must preserve these exactly.
EXPECTED_EXTRINSIC_T = [-0.011, -0.02329, 0.04412]
EXPECTED_EXTRINSIC_R = [1, 0, 0, 0, 1, 0, 0, 0, 1]

MAP_SOURCE_PCD_REL = os.path.join("PCD", "scans.pcd")  # relative to the fast_lio package

# G1 TF framework (component "tf").
# The tree, with exactly one parent per frame:
#   camera_init --(FAST-LIO2 /laserMapping, dynamic)--> body
#   camera_init --(g1_tf_publisher,          dynamic)--> pelvis
#   pelvis      --(robot_state_publisher, waist_yaw_joint)--> torso_link
#   torso_link  --(robot_state_publisher, fixed)--> mid360_link, head_link, ...
TF_NODE_NAME = "/g1_tf_publisher"
ROBOT_STATE_PUBLISHER_NODE = "/robot_state_publisher"
ROBOT_STATE_PUBLISHER_PKG = "robot_state_publisher"
ROBOT_STATE_PUBLISHER_APT = "ros-noetic-robot-state-publisher"
TF_CONFIG_REL = os.path.join("config", "g1_tf.yaml")
TF_PUBLISHER_REL = os.path.join("scripts", "g1_tf_publisher.py")

FASTLIO_MAP_FRAME = "camera_init"       # FAST-LIO2 odometry origin
FASTLIO_BODY_FRAME = "body"             # FAST-LIO2 tracked (Mid-360 IMU) frame
ROBOT_ROOT_FRAME = "pelvis"             # official URDF root; what we broadcast
TORSO_FRAME = "torso_link"              # carries the Mid-360; child of waist_yaw_joint
LIDAR_FRAME = "mid360_link"             # Mid-360 LiDAR frame, per the URDF

# `base_link` is an UNRESOLVED integration decision pending the navigation team.
# Nothing in this package may publish or invent it; the checks below assert that.
UNRESOLVED_BASE_FRAME = "base_link"

# (parent, child) TF edges FAST-LIO2's laserMapping node broadcasts itself.
# Verified against src/laserMapping.cpp by check_fastlio_frame_contract().
FASTLIO_TF_EDGES = [(FASTLIO_MAP_FRAME, FASTLIO_BODY_FRAME)]

# What g1_tf_publisher broadcasts. Parent is FAST-LIO2's world frame (which
# already parents `body`, so `pelvis` becomes a second CHILD of camera_init, not
# a second parent of anything).
EXPECTED_TF_PARENT = FASTLIO_MAP_FRAME
EXPECTED_TF_CHILD = ROBOT_ROOT_FRAME

# Vendored official robot description. Provenance and these digests are recorded
# in urdf/SOURCE.md; both are re-verified by check_vendored_urdf().
URDF_REL = os.path.join("urdf", "g1_23dof_mode_10.urdf")
URDF_LICENSE_REL = os.path.join("urdf", "LICENSE")
URDF_SOURCE_REL = os.path.join("urdf", "SOURCE.md")
URDF_SHA256 = "9333e89c51614c92b416c2c22a28f1614284d02321548e10479cce73552bcf43"
URDF_LICENSE_SHA256 = "84aac59fd3246e3ddc49d1387644e8fcf43b0def4f0b9fe687f372e90446df2d"
URDF_UPSTREAM_COMMIT = "7d6075f7f58588b189b940130e3edab3c839b2df"
URDF_UPSTREAM_URL = "https://github.com/unitreerobotics/unitree_ros.git"

# External interface, owned by the robot-control / integration team. This package
# consumes it and never synthesizes it.
JOINT_STATES_TOPIC = "/joint_states"
WAIST_YAW_JOINT = "waist_yaw_joint"

REQUIRED_FILES = [
    "package.xml",
    "CMakeLists.txt",
    "config/mid360_config.json",
    "config/fastlio.yaml",
    "config/g1_tf.yaml",
    "launch/sensors.launch",
    "launch/fastlio_mapping.launch",
    "launch/slam_bringup.launch",
    "launch/g1_tf.launch",
    "scripts/slam_health_check.py",
    "scripts/save_map.py",
    "scripts/g1_tf_publisher.py",
    "setup.py",
    "src/g1_slam/__init__.py",
    "src/g1_slam/tf_chain.py",
    "test/test_tf_transforms.py",
    "urdf/g1_23dof_mode_10.urdf",
    "urdf/LICENSE",
    "urdf/SOURCE.md",
    "src/pcd_to_map.cpp",
    "launch/map_server.launch",
]

# Offline PCD -> 2D map converter (component "pcdmap").
PCD_TO_MAP_BIN = "pcd_to_map"
MAPS_GITIGNORE_PATTERNS = ("*.pcd", "*.pgm", "*.yaml")

# ROS1 map_server integration (component "mapserver").
MAP_SERVER_LAUNCH = "map_server.launch"
MAP_SERVER_NODE = "/map_server"
MAP_TOPIC = "/map"
MAP_METADATA_TOPIC = "/map_metadata"
# The served OccupancyGrid's default frame. pcd_to_map takes its pixel
# coordinates straight from the FAST-LIO2 PCD, whose points are in FAST-LIO2's
# camera_init (odometry origin) frame, so the map must be published there.
DEFAULT_MAP_FRAME = FASTLIO_MAP_FRAME

_node_initialized = False


class Results:
    def __init__(self, as_json):
        self.as_json = as_json
        self.records = []

    def add(self, name, status, reason):
        self.records.append({"check": name, "status": status, "reason": reason})
        if not self.as_json:
            print("[{}] {} - {}".format(status, name, reason))

    def overall(self):
        worst = STATUS_PASS
        for r in self.records:
            if _SEVERITY[r["status"]] > _SEVERITY[worst]:
                worst = r["status"]
        return worst


def hardware_status(require_hardware):
    return STATUS_FAIL if require_hardware else STATUS_WAIT


def run(cmd, timeout):
    try:
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               timeout=timeout)
        return proc.returncode, proc.stdout.decode("utf-8", "replace")
    except FileNotFoundError as exc:
        return 127, str(exc)
    except subprocess.TimeoutExpired:
        return None, "timed out after {}s".format(timeout)


def check_package_files(results):
    try:
        import rospkg
        pkg_path = rospkg.RosPack().get_path(PACKAGE_NAME)
    except Exception as exc:
        results.add("g1_slam manifest and required files", STATUS_FAIL,
                    "rospack could not resolve package '{}': {}".format(PACKAGE_NAME, exc))
        return None

    missing = [f for f in REQUIRED_FILES if not os.path.isfile(os.path.join(pkg_path, f))]
    if missing:
        results.add("g1_slam manifest and required files", STATUS_FAIL,
                     "missing files under {}: {}".format(pkg_path, ", ".join(missing)))
        return pkg_path

    results.add("g1_slam manifest and required files", STATUS_PASS,
                "all required files present under {}".format(pkg_path))
    return pkg_path


def check_livox_driver_package(results):
    try:
        import rospkg
        path = rospkg.RosPack().get_path("livox_ros_driver2")
    except Exception as exc:
        results.add("livox_ros_driver2 package resolution", STATUS_FAIL,
                     "package not resolvable: {}".format(exc))
        return None
    results.add("livox_ros_driver2 package resolution", STATUS_PASS,
                "resolved at {}".format(path))
    return path


def check_config_json(results, pkg_path):
    if pkg_path is None:
        results.add("mid360_config.json JSON validity", STATUS_FAIL,
                     "cannot check, package path unknown")
        results.add("expected host and LiDAR IP values", STATUS_FAIL,
                     "cannot check, package path unknown")
        return

    config_path = os.path.join(pkg_path, "config", "mid360_config.json")
    try:
        with open(config_path) as f:
            data = json.load(f)
    except Exception as exc:
        results.add("mid360_config.json JSON validity", STATUS_FAIL,
                     "{}: {}".format(config_path, exc))
        results.add("expected host and LiDAR IP values", STATUS_FAIL,
                     "cannot check, JSON invalid")
        return

    results.add("mid360_config.json JSON validity", STATUS_PASS,
                "{} parsed OK".format(config_path))

    try:
        host_info = data["MID360"]["host_net_info"]
        lidar_ip = data["lidar_configs"][0]["ip"]
        host_ips = {host_info[k] for k in
                    ("cmd_data_ip", "push_msg_ip", "point_data_ip", "imu_data_ip")}
    except (KeyError, IndexError, TypeError) as exc:
        results.add("expected host and LiDAR IP values", STATUS_FAIL,
                     "unexpected config structure: {}".format(exc))
        return

    if host_ips != {EXPECTED_HOST_IP} or lidar_ip != EXPECTED_LIDAR_IP:
        results.add("expected host and LiDAR IP values", STATUS_FAIL,
                     "host ip(s)={} lidar ip={} (expected host={} lidar={})".format(
                         sorted(host_ips), lidar_ip, EXPECTED_HOST_IP, EXPECTED_LIDAR_IP))
    else:
        results.add("expected host and LiDAR IP values", STATUS_PASS,
                     "host={} lidar={}".format(EXPECTED_HOST_IP, EXPECTED_LIDAR_IP))


def check_sensors_launch(results, pkg_path, timeout):
    if pkg_path is None:
        results.add("sensors.launch XML and ROS package/node resolution", STATUS_FAIL,
                     "cannot check, package path unknown")
        return

    launch_path = os.path.join(pkg_path, "launch", "sensors.launch")
    try:
        import xml.etree.ElementTree as ET
        ET.parse(launch_path)
    except Exception as exc:
        results.add("sensors.launch XML and ROS package/node resolution", STATUS_FAIL,
                     "XML parse error in {}: {}".format(launch_path, exc))
        return

    code, output = run(["roslaunch", "--nodes", PACKAGE_NAME, "sensors.launch"], timeout)
    if code != 0:
        last_line = output.strip().splitlines()[-1] if output.strip() else ""
        results.add("sensors.launch XML and ROS package/node resolution", STATUS_FAIL,
                     "roslaunch --nodes {} sensors.launch failed (code={}): {}".format(
                         PACKAGE_NAME, code, last_line))
        return

    if LIVOX_NODE_NAME not in output:
        results.add("sensors.launch XML and ROS package/node resolution", STATUS_FAIL,
                     "expected node {} not found in roslaunch --nodes output".format(LIVOX_NODE_NAME))
        return

    results.add("sensors.launch XML and ROS package/node resolution", STATUS_PASS,
                "XML valid, node {} resolves".format(LIVOX_NODE_NAME))


def check_fastlio_package(results):
    try:
        import rospkg
        path = rospkg.RosPack().get_path("fast_lio")
    except Exception as exc:
        results.add("fast_lio package resolution", STATUS_FAIL,
                     "package not resolvable: {}".format(exc))
        return None
    results.add("fast_lio package resolution", STATUS_PASS,
                "resolved at {}".format(path))
    return path


def check_fastlio_yaml(results, pkg_path):
    if pkg_path is None:
        results.add("fastlio.yaml validity and extrinsic values", STATUS_FAIL,
                     "cannot check, package path unknown")
        return

    config_path = os.path.join(pkg_path, "config", "fastlio.yaml")
    try:
        import yaml
        with open(config_path) as f:
            data = yaml.safe_load(f)
    except Exception as exc:
        results.add("fastlio.yaml validity and extrinsic values", STATUS_FAIL,
                     "{}: {}".format(config_path, exc))
        return

    try:
        lid_topic = data["common"]["lid_topic"]
        imu_topic = data["common"]["imu_topic"]
        extrinsic_t = data["mapping"]["extrinsic_T"]
        extrinsic_r = data["mapping"]["extrinsic_R"]
    except (KeyError, TypeError) as exc:
        results.add("fastlio.yaml validity and extrinsic values", STATUS_FAIL,
                     "unexpected config structure: {}".format(exc))
        return

    problems = []
    if lid_topic != LIDAR_TOPIC or imu_topic != IMU_TOPIC:
        problems.append("input topics lid={} imu={} (expected lid={} imu={})".format(
            lid_topic, imu_topic, LIDAR_TOPIC, IMU_TOPIC))
    if list(extrinsic_t) != EXPECTED_EXTRINSIC_T:
        problems.append("extrinsic_T={} (expected {})".format(extrinsic_t, EXPECTED_EXTRINSIC_T))
    if list(extrinsic_r) != EXPECTED_EXTRINSIC_R:
        problems.append("extrinsic_R={} (expected {})".format(extrinsic_r, EXPECTED_EXTRINSIC_R))

    if problems:
        results.add("fastlio.yaml validity and extrinsic values", STATUS_FAIL,
                     "; ".join(problems))
    else:
        results.add("fastlio.yaml validity and extrinsic values", STATUS_PASS,
                     "{} parsed OK, official Mid-360 extrinsic preserved".format(config_path))


def check_fastlio_launch(results, pkg_path, timeout):
    if pkg_path is None:
        results.add("fastlio_mapping.launch XML and ROS package/node resolution", STATUS_FAIL,
                     "cannot check, package path unknown")
        return

    launch_path = os.path.join(pkg_path, "launch", "fastlio_mapping.launch")
    try:
        import xml.etree.ElementTree as ET
        ET.parse(launch_path)
    except Exception as exc:
        results.add("fastlio_mapping.launch XML and ROS package/node resolution", STATUS_FAIL,
                     "XML parse error in {}: {}".format(launch_path, exc))
        return

    code, output = run(["roslaunch", "--nodes", PACKAGE_NAME, "fastlio_mapping.launch"], timeout)
    if code != 0:
        last_line = output.strip().splitlines()[-1] if output.strip() else ""
        results.add("fastlio_mapping.launch XML and ROS package/node resolution", STATUS_FAIL,
                     "roslaunch --nodes {} fastlio_mapping.launch failed (code={}): {}".format(
                         PACKAGE_NAME, code, last_line))
        return

    if FASTLIO_NODE_NAME not in output:
        results.add("fastlio_mapping.launch XML and ROS package/node resolution", STATUS_FAIL,
                     "expected node {} not found in roslaunch --nodes output".format(FASTLIO_NODE_NAME))
        return

    results.add("fastlio_mapping.launch XML and ROS package/node resolution", STATUS_PASS,
                "XML valid, node {} resolves".format(FASTLIO_NODE_NAME))


def check_maps_dir(results, pkg_path):
    name = "slam/maps directory exists and is writable"
    if pkg_path is None:
        results.add(name, STATUS_FAIL, "cannot check, package path unknown")
        return

    maps_dir = os.path.join(pkg_path, "maps")
    if not os.path.isdir(maps_dir):
        results.add(name, STATUS_FAIL, "missing directory: {}".format(maps_dir))
        return
    if not os.access(maps_dir, os.W_OK):
        results.add(name, STATUS_FAIL, "not writable: {}".format(maps_dir))
        return
    results.add(name, STATUS_PASS, "{} exists and is writable".format(maps_dir))


def check_save_map_script(results, pkg_path, timeout):
    name = "save_map.py script sanity (--help)"
    if pkg_path is None:
        results.add(name, STATUS_FAIL, "cannot check, package path unknown")
        return

    script_path = os.path.join(pkg_path, "scripts", "save_map.py")
    code, output = run([sys.executable, script_path, "--help"], timeout)
    if code != 0:
        last_line = output.strip().splitlines()[-1] if output.strip() else ""
        results.add(name, STATUS_FAIL, "save_map.py --help failed (code={}): {}".format(code, last_line))
        return
    results.add(name, STATUS_PASS, "save_map.py runs and parses arguments")


def check_map_source(results, fastlio_pkg_path, require_hardware):
    name = "accumulated FAST-LIO2 map available to save"
    if fastlio_pkg_path is None:
        results.add(name, hardware_status(require_hardware),
                     "fast_lio package path unknown, cannot locate PCD output")
        return

    source_path = os.path.join(fastlio_pkg_path, MAP_SOURCE_PCD_REL)
    if not os.path.isfile(source_path) or os.path.getsize(source_path) == 0:
        results.add(name, hardware_status(require_hardware),
                     "no non-empty map at {} yet (run fastlio_mapping.launch to completion "
                     "with pcd_save_en:true, then use save_map.py --name <name>)".format(source_path))
        return

    results.add(name, STATUS_PASS, "found accumulated map at {}".format(source_path))


def _write_pcd(path, points, binary):
    """Write a minimal x/y/z PCD (ASCII or uncompressed binary) for self-tests."""
    import struct
    n = len(points)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        "WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA {d}\n"
        .format(n=n, d="binary" if binary else "ascii"))
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        if binary:
            for x, y, z in points:
                f.write(struct.pack("<fff", x, y, z))
        else:
            for x, y, z in points:
                f.write("{:.6f} {:.6f} {:.6f}\n".format(x, y, z).encode("ascii"))


def _synthetic_cloud():
    """A 2x2 m room: ground plane at z=-0.8, walls at x in {0,2} at z=0.5."""
    pts = []
    y = 0.0
    while y <= 2.0 + 1e-9:
        x = 0.0
        while x <= 2.0 + 1e-9:
            pts.append((x, y, -0.8))          # floor -> free evidence
            x += 0.2
        pts.append((0.0, y, 0.5))             # wall -> occupied
        pts.append((2.0, y, 0.5))
        y += 0.1
    return pts


def check_maps_gitignore(results, pkg_path):
    name = "slam/maps/.gitignore ignores generated pcd/pgm/yaml"
    if pkg_path is None:
        results.add(name, STATUS_FAIL, "cannot check, package path unknown")
        return
    gi_path = os.path.join(pkg_path, "maps", ".gitignore")
    try:
        with open(gi_path) as f:
            lines = {ln.strip() for ln in f}
    except OSError as exc:
        results.add(name, STATUS_FAIL, "cannot read {}: {}".format(gi_path, exc))
        return
    missing = [p for p in MAPS_GITIGNORE_PATTERNS if p not in lines]
    if missing:
        results.add(name, STATUS_FAIL,
                     "missing pattern(s) {} in {}".format(missing, gi_path))
        return
    results.add(name, STATUS_PASS, "ignores {}".format(", ".join(MAPS_GITIGNORE_PATTERNS)))


def check_pcd_to_map_build(results, timeout):
    name = "pcd_to_map executable built and runnable"
    code, output = run(["rosrun", "g1_slam", PCD_TO_MAP_BIN, "--help"], max(timeout, 20))
    if code != 0:
        last_line = output.strip().splitlines()[-1] if output.strip() else ""
        results.add(name, STATUS_FAIL,
                     "`rosrun g1_slam pcd_to_map --help` failed (code={}): {} "
                     "(build the workspace with catkin_make)".format(code, last_line))
        return
    if "Usage:" not in output or "--resolution" not in output:
        results.add(name, STATUS_FAIL, "help text missing expected content")
        return
    results.add(name, STATUS_PASS, "runs and prints usage")


def check_pcd_to_map_cli_errors(results, timeout):
    name = "pcd_to_map rejects bad names/paths/params safely"
    import shutil
    import tempfile
    tmp = tempfile.mkdtemp(prefix="g1_pcdmap_err_")
    try:
        good_pcd = os.path.join(tmp, "ok.pcd")
        _write_pcd(good_pcd, _synthetic_cloud(), binary=False)
        cases = [
            (["--name", "x", "--output-dir", tmp], "missing --input"),
            (["--input", os.path.join(tmp, "nope.pcd"), "--name", "x", "--output-dir", tmp],
             "nonexistent input"),
            (["--input", good_pcd, "--name", "../evil", "--output-dir", tmp],
             "path-traversal --name"),
            (["--input", good_pcd, "--name", "a/b", "--output-dir", tmp],
             "--name with slash"),
            (["--input", good_pcd, "--name", ".hidden", "--output-dir", tmp],
             "--name leading dot"),
            (["--input", good_pcd, "--name", "x", "--output-dir", os.path.join(tmp, "missing")],
             "nonexistent --output-dir"),
            (["--input", good_pcd, "--name", "x", "--output-dir", tmp, "--resolution", "0"],
             "zero --resolution"),
            (["--input", good_pcd, "--name", "x", "--output-dir", tmp,
              "--z-min", "3", "--z-max", "1"], "inverted z range"),
            (["--input", good_pcd, "--name", "x", "--output-dir", tmp, "--negate", "2"],
             "bad --negate"),
        ]
        accepted = []
        for extra, desc in cases:
            code, _ = run(["rosrun", "g1_slam", PCD_TO_MAP_BIN] + extra, max(timeout, 15))
            if code == 0:
                accepted.append(desc)
        if accepted:
            results.add(name, STATUS_FAIL,
                         "exited 0 on invalid input: {}".format("; ".join(accepted)))
            return
        results.add(name, STATUS_PASS,
                     "all {} malformed invocations rejected with non-zero exit".format(len(cases)))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def check_pcd_to_map_conversion(results, timeout):
    name = "pcd_to_map converts synthetic ASCII + binary PCD to valid .pgm/.yaml"
    import shutil
    import tempfile
    tmp = tempfile.mkdtemp(prefix="g1_pcdmap_")
    try:
        pts = _synthetic_cloud()
        variants = [("ascii", os.path.join(tmp, "cloud_ascii.pcd"), False),
                    ("binary", os.path.join(tmp, "cloud_binary.pcd"), True)]
        for _label, pcd_path, is_bin in variants:
            _write_pcd(pcd_path, pts, binary=is_bin)

        for label, pcd_path, _is_bin in variants:
            out_dir = os.path.join(tmp, label)
            os.mkdir(out_dir)
            cmd = ["rosrun", "g1_slam", PCD_TO_MAP_BIN,
                   "--input", pcd_path, "--name", "unit", "--output-dir", out_dir,
                   "--resolution", "0.1", "--z-min", "0.0", "--z-max", "1.0"]
            code, output = run(cmd, max(timeout, 20))
            if code != 0:
                last_line = output.strip().splitlines()[-1] if output.strip() else ""
                results.add(name, STATUS_FAIL,
                             "{} PCD conversion failed (code={}): {}".format(label, code, last_line))
                return

            pgm = os.path.join(out_dir, "unit.pgm")
            ymap = os.path.join(out_dir, "unit.yaml")
            if not (os.path.isfile(pgm) and os.path.isfile(ymap)):
                results.add(name, STATUS_FAIL, "{}: expected outputs not created".format(label))
                return
            with open(pgm, "rb") as f:
                if f.read(2) != b"P5":
                    results.add(name, STATUS_FAIL,
                                 "{}: {} is not a binary (P5) PGM".format(label, pgm))
                    return
            try:
                import yaml
                with open(ymap) as f:
                    meta = yaml.safe_load(f)
            except Exception as exc:
                results.add(name, STATUS_FAIL, "{}: yaml unreadable: {}".format(label, exc))
                return
            for key in ("image", "resolution", "origin", "negate",
                        "occupied_thresh", "free_thresh"):
                if key not in meta:
                    results.add(name, STATUS_FAIL, "{}: yaml missing '{}'".format(label, key))
                    return
            if meta["image"] != "unit.pgm":
                results.add(name, STATUS_FAIL, "{}: yaml image={!r}".format(label, meta["image"]))
                return
            if abs(float(meta["resolution"]) - 0.1) > 1e-9:
                results.add(name, STATUS_FAIL,
                             "{}: yaml resolution {} != 0.1".format(label, meta["resolution"]))
                return
            if not isinstance(meta["origin"], list) or len(meta["origin"]) != 3:
                results.add(name, STATUS_FAIL, "{}: yaml origin malformed".format(label))
                return

            # A second identical run must refuse to overwrite.
            code2, _ = run(cmd, max(timeout, 20))
            if code2 == 0:
                results.add(name, STATUS_FAIL,
                             "{}: re-run overwrote an existing map".format(label))
                return

        results.add(name, STATUS_PASS,
                     "ASCII and binary PCD both produce a P5 .pgm + valid .yaml; "
                     "re-run refuses to overwrite")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def check_map_server_package(results):
    name = "map_server package resolution"
    try:
        import rospkg
        path = rospkg.RosPack().get_path("map_server")
    except Exception as exc:
        results.add(name, STATUS_FAIL,
                     "package not resolvable (install ros-noetic-map-server): {}".format(exc))
        return
    results.add(name, STATUS_PASS, "resolved at {}".format(path))


def check_map_server_launch(results, pkg_path, timeout):
    name = "map_server.launch XML, required 'map' arg, node resolution"
    if pkg_path is None:
        results.add(name, STATUS_FAIL, "cannot check, package path unknown")
        return

    launch_path = os.path.join(pkg_path, "launch", MAP_SERVER_LAUNCH)
    try:
        import xml.etree.ElementTree as ET
        root = ET.parse(launch_path).getroot()
    except Exception as exc:
        results.add(name, STATUS_FAIL, "XML parse error in {}: {}".format(launch_path, exc))
        return

    problems = []
    launch_args = {a.get("name"): a for a in root.findall("arg")}
    if "map" not in launch_args:
        problems.append("no <arg name='map'>")
    elif launch_args["map"].get("default") is not None or launch_args["map"].get("value") is not None:
        problems.append("<arg name='map'> has a default/value (must be caller-required)")
    if "frame_id" not in launch_args or launch_args["frame_id"].get("default") != DEFAULT_MAP_FRAME:
        problems.append("<arg name='frame_id'> default is not {!r}".format(DEFAULT_MAP_FRAME))

    nodes = root.findall("node")
    ms_nodes = [n for n in nodes if n.get("pkg") == "map_server" and n.get("type") == "map_server"]
    if not ms_nodes:
        problems.append("no <node pkg='map_server' type='map_server'>")
    elif "$(arg map)" not in (ms_nodes[0].get("args") or ""):
        problems.append("map_server node does not pass $(arg map) as args")
    if list(root.iter("include")):
        problems.append("has <include> - must not pull in FAST-LIO2 / the Livox driver")

    if problems:
        results.add(name, STATUS_FAIL, "; ".join(problems))
        return

    code_missing, _ = run(["roslaunch", "--nodes", PACKAGE_NAME, MAP_SERVER_LAUNCH], timeout)
    if code_missing == 0:
        results.add(name, STATUS_FAIL, "roslaunch accepted a missing 'map' arg")
        return

    code, output = run(["roslaunch", "--nodes", PACKAGE_NAME, MAP_SERVER_LAUNCH,
                        "map:=/tmp/g1_healthcheck_map.yaml"], timeout)
    if code != 0:
        last_line = output.strip().splitlines()[-1] if output.strip() else ""
        results.add(name, STATUS_FAIL,
                     "roslaunch --nodes ... map:=<path> failed (code={}): {}".format(code, last_line))
        return

    resolved = output.split()
    if MAP_SERVER_NODE not in resolved:
        results.add(name, STATUS_FAIL,
                     "{} not in roslaunch --nodes output".format(MAP_SERVER_NODE))
        return
    stray = [n for n in (FASTLIO_NODE_NAME, LIVOX_NODE_NAME) if n in resolved]
    if stray:
        results.add(name, STATUS_FAIL, "launch also starts {}".format(", ".join(stray)))
        return

    results.add(name, STATUS_PASS,
                 "XML valid, 'map' required, resolves only {} (no FAST-LIO2 / Livox)".format(
                     MAP_SERVER_NODE))


def check_map_server_frame_param(results, pkg_path, timeout):
    name = "map_server serves the map in {} by default (overridable)".format(DEFAULT_MAP_FRAME)
    if pkg_path is None:
        results.add(name, STATUS_FAIL, "cannot check, package path unknown")
        return

    def dump(extra_args):
        code, output = run(["roslaunch", "--dump-params", PACKAGE_NAME, MAP_SERVER_LAUNCH,
                            "map:=/tmp/g1_healthcheck_map.yaml"] + extra_args, timeout)
        if code != 0:
            return None
        try:
            import yaml
            return yaml.safe_load(output) or {}
        except Exception:
            return None

    default_params = dump([])
    if default_params is None:
        results.add(name, STATUS_FAIL, "roslaunch --dump-params failed")
        return
    frame = default_params.get("/map_server/frame_id")
    if frame != DEFAULT_MAP_FRAME:
        results.add(name, STATUS_FAIL,
                     "/map_server/frame_id defaulted to {!r} (expected {!r})".format(
                         frame, DEFAULT_MAP_FRAME))
        return

    override_params = dump(["frame_id:=map"])
    if override_params is None or override_params.get("/map_server/frame_id") != "map":
        results.add(name, STATUS_FAIL, "frame_id:=<frame> override not honoured")
        return

    results.add(name, STATUS_PASS,
                 "/map_server/frame_id defaults to {} and follows frame_id:=<frame>".format(
                     DEFAULT_MAP_FRAME))


def _sha256(path):
    import hashlib
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _import_tf_chain(pkg_path):
    """Import g1_slam.tf_chain, falling back to <pkg>/src for an unbuilt tree."""
    try:
        from g1_slam import tf_chain
        return tf_chain, None
    except ImportError as exc:
        first = exc
    if pkg_path:
        src_dir = os.path.join(pkg_path, "src")
        if src_dir not in sys.path:
            sys.path.insert(0, src_dir)
        try:
            from g1_slam import tf_chain
            return tf_chain, None
        except ImportError as exc:
            return None, exc
    return None, first


def check_vendored_urdf(results, pkg_path):
    """The vendored URDF and its upstream license must be byte-exact (SHA-256)."""
    name = "vendored URDF + license integrity (SHA-256)"
    if pkg_path is None:
        results.add(name, STATUS_FAIL, "cannot check, package path unknown")
        return

    problems = []
    for rel, expected in ((URDF_REL, URDF_SHA256), (URDF_LICENSE_REL, URDF_LICENSE_SHA256)):
        path = os.path.join(pkg_path, rel)
        if not os.path.isfile(path):
            problems.append("{} is missing".format(rel))
            continue
        try:
            actual = _sha256(path)
        except OSError as exc:
            problems.append("{} unreadable: {}".format(rel, exc))
            continue
        if actual != expected:
            problems.append("{} sha256 {} != recorded {} - the vendored copy was "
                            "modified; re-copy from upstream or update SOURCE.md".format(
                                rel, actual[:16], expected[:16]))

    if problems:
        results.add(name, STATUS_FAIL, "; ".join(problems))
        return

    results.add(name, STATUS_PASS,
                "{} and {} match the SHA-256 digests recorded in {}".format(
                    URDF_REL, URDF_LICENSE_REL, URDF_SOURCE_REL))


def check_urdf_source_md(results, pkg_path):
    """SOURCE.md must record the upstream URL, commit and both digests."""
    name = "urdf/SOURCE.md records upstream provenance"
    if pkg_path is None:
        results.add(name, STATUS_FAIL, "cannot check, package path unknown")
        return

    path = os.path.join(pkg_path, URDF_SOURCE_REL)
    try:
        with open(path) as handle:
            text = handle.read()
    except OSError as exc:
        results.add(name, STATUS_FAIL, "cannot read {}: {}".format(path, exc))
        return

    required = {
        "upstream URL": URDF_UPSTREAM_URL,
        "upstream commit": URDF_UPSTREAM_COMMIT,
        "URDF sha256": URDF_SHA256,
        "license sha256": URDF_LICENSE_SHA256,
    }
    missing = sorted(label for label, value in required.items() if value not in text)
    if missing:
        results.add(name, STATUS_FAIL,
                    "{} does not record: {}".format(URDF_SOURCE_REL, ", ".join(missing)))
        return

    # The vendored copy carries no meshes, so it must not claim visualization.
    if "NOT visualization" not in text and "not claimed" not in text:
        results.add(name, STATUS_FAIL,
                    "{} must state that the vendored URDF is kinematics/TF only "
                    "(the mesh references are relative and no meshes are vendored)".format(
                        URDF_SOURCE_REL))
        return

    results.add(name, STATUS_PASS,
                "records {} @ {} plus both SHA-256 digests, scoped kinematics/TF only".format(
                    URDF_UPSTREAM_URL, URDF_UPSTREAM_COMMIT[:12]))


def check_tf_chain_module(results, pkg_path):
    """g1_slam.tf_chain must import, agree with the URDF, and keep its direction."""
    name = "g1_slam.tf_chain transform direction and URDF agreement"
    tf_chain, exc = _import_tf_chain(pkg_path)
    if tf_chain is None:
        results.add(name, STATUS_FAIL,
                    "cannot import g1_slam.tf_chain ({}); catkin_python_setup() must "
                    "install the package from src/".format(exc))
        return

    problems = []

    forward = tf_chain.translation_of(tf_chain.body_to_mid360())
    if max(abs(a - b) for a, b in zip(forward, EXPECTED_EXTRINSIC_T)) > 1e-12:
        problems.append("{} -> {} translation {} != fastlio extrinsic_T {}".format(
            FASTLIO_BODY_FRAME, LIDAR_FRAME, forward, EXPECTED_EXTRINSIC_T))

    rot = tf_chain.rotation_of(tf_chain.body_to_mid360())
    flat = [rot[i][j] for i in range(3) for j in range(3)]
    if max(abs(a - b) for a, b in zip(flat, EXPECTED_EXTRINSIC_R)) > 1e-12:
        problems.append("{} -> {} rotation is not identity".format(
            FASTLIO_BODY_FRAME, LIDAR_FRAME))

    backward = tf_chain.translation_of(tf_chain.mid360_to_body())
    if max(abs(f + b) for f, b in zip(forward, backward)) > 1e-12:
        problems.append("{} -> {} {} is not the inverse of {} -> {} {}".format(
            LIDAR_FRAME, FASTLIO_BODY_FRAME, backward,
            FASTLIO_BODY_FRAME, LIDAR_FRAME, forward))

    if pkg_path is not None:
        urdf_path = os.path.join(pkg_path, URDF_REL)
        if os.path.isfile(urdf_path):
            try:
                problems.extend(tf_chain.verify_against_urdf(urdf_path))
            except Exception as exc:
                problems.append("cannot verify against {}: {}".format(urdf_path, exc))

    if problems:
        results.add(name, STATUS_FAIL, "; ".join(problems))
        return

    results.add(name, STATUS_PASS,
                "{} -> {} = {} (identity rotation), inverse checks out, and the "
                "vendored URDF still matches tf_chain's joint constants".format(
                    FASTLIO_BODY_FRAME, LIDAR_FRAME, list(forward)))


def check_g1_tf_config(results, pkg_path, require_hardware):
    """g1_tf.yaml must name the fixed frames, hold no paths, and not invent base_link."""
    name = "g1_tf.yaml validity and frame wiring"
    if pkg_path is None:
        results.add(name, STATUS_FAIL, "cannot check, package path unknown")
        return

    cfg_path = os.path.join(pkg_path, TF_CONFIG_REL)
    try:
        import yaml
        with open(cfg_path) as handle:
            cfg = yaml.safe_load(handle)
    except Exception as exc:
        results.add(name, STATUS_FAIL, "{}: {}".format(cfg_path, exc))
        return

    if not isinstance(cfg, dict):
        results.add(name, STATUS_FAIL, "{}: root is not a mapping".format(cfg_path))
        return

    expected_frames = {
        "world_frame": FASTLIO_MAP_FRAME,
        "robot_root_frame": ROBOT_ROOT_FRAME,
        "imu_frame": FASTLIO_BODY_FRAME,
        "lidar_frame": LIDAR_FRAME,
    }
    problems = []
    for key, want in sorted(expected_frames.items()):
        if cfg.get(key) != want:
            problems.append("{}={!r} (must be {!r})".format(key, cfg.get(key), want))

    for key in ("odom_topic", "joint_states_topic", "waist_yaw_joint_name"):
        value = cfg.get(key)
        if not isinstance(value, str) or not value.strip():
            problems.append("{} must be a non-empty string, got {!r}".format(key, value))

    if cfg.get("waist_yaw_joint_name") not in (None, WAIST_YAW_JOINT):
        problems.append("waist_yaw_joint_name={!r} (expected {!r})".format(
            cfg.get("waist_yaw_joint_name"), WAIST_YAW_JOINT))

    try:
        timeout = float(cfg.get("joint_states_timeout"))
        if timeout <= 0.0:
            problems.append("joint_states_timeout must be > 0, got {}".format(timeout))
    except (TypeError, ValueError):
        problems.append("joint_states_timeout is not a number: {!r}".format(
            cfg.get("joint_states_timeout")))

    # roslaunch expands $(find ...) in launch XML only, never inside plain YAML
    # loaded via rosparam. A literal substitution here would reach the node raw.
    leaked = sorted(k for k, v in cfg.items() if isinstance(v, str) and "$(" in v)
    if leaked:
        problems.append("unexpanded ROS substitution in YAML key(s) {}; pass resolved "
                        "paths from the launch file instead".format(", ".join(leaked)))

    if problems:
        results.add(name, STATUS_FAIL, "; ".join(problems))
        return

    alias = cfg.get("base_link_alias", "")
    alias = "" if alias is None else str(alias)
    if alias.strip():
        results.add(name, STATUS_FAIL,
                    "base_link_alias={!r}: the {} -> {} mapping is an unresolved "
                    "integration decision pending the navigation team; publishing a "
                    "guessed transform is not supported".format(
                        alias, ROBOT_ROOT_FRAME, UNRESOLVED_BASE_FRAME))
        return

    results.add(name, hardware_status(require_hardware),
                "{} -> {} wiring valid, no paths in YAML; {} -> {} deliberately "
                "unresolved (base_link_alias empty) pending the navigation team".format(
                    FASTLIO_MAP_FRAME, ROBOT_ROOT_FRAME,
                    ROBOT_ROOT_FRAME, UNRESOLVED_BASE_FRAME))


def check_g1_tf_publisher_script(results, pkg_path, timeout):
    name = "g1_tf_publisher.py offline --check"
    if pkg_path is None:
        results.add(name, STATUS_FAIL, "cannot check, package path unknown")
        return

    script_path = os.path.join(pkg_path, TF_PUBLISHER_REL)
    cfg_path = os.path.join(pkg_path, TF_CONFIG_REL)
    urdf_path = os.path.join(pkg_path, URDF_REL)
    code, output = run([sys.executable, script_path, "--check",
                        "--config", cfg_path, "--urdf", urdf_path], timeout)
    last_line = output.strip().splitlines()[-1] if output.strip() else ""

    if code == 0:
        results.add(name, STATUS_PASS, last_line or "configuration valid")
    elif code == 2:
        results.add(name, STATUS_WAIT, last_line or "pending an external interface")
    else:
        results.add(name, STATUS_FAIL,
                    "g1_tf_publisher.py --check failed (code={}): {}".format(code, last_line))


def check_robot_state_publisher_package(results, require_hardware):
    """robot_state_publisher turns the URDF + /joint_states into the robot's TF."""
    name = "robot_state_publisher package resolution"
    try:
        import rospkg
        path = rospkg.RosPack().get_path(ROBOT_STATE_PUBLISHER_PKG)
    except Exception as exc:
        # An unprovisioned environment, not a defect in g1_slam: WAIT by default,
        # FAIL under --require-hardware so an on-robot bring-up cannot skip it.
        results.add(name, hardware_status(require_hardware),
                    "{} is not installed ({}). g1_tf.launch cannot start the robot's "
                    "TF tree without it. Install with: apt-get install -y {}".format(
                        ROBOT_STATE_PUBLISHER_PKG, exc, ROBOT_STATE_PUBLISHER_APT))
        return None
    results.add(name, STATUS_PASS, "resolved at {}".format(path))
    return path


def check_g1_tf_launch(results, pkg_path, timeout, have_rsp):
    name = "g1_tf.launch XML and ROS package/node resolution"
    if pkg_path is None:
        results.add(name, STATUS_FAIL, "cannot check, package path unknown")
        return

    launch_path = os.path.join(pkg_path, "launch", "g1_tf.launch")
    try:
        import xml.etree.ElementTree as ET
        tree = ET.parse(launch_path)
    except Exception as exc:
        results.add(name, STATUS_FAIL, "XML parse error in {}: {}".format(launch_path, exc))
        return

    # The URDF path must come from a launch arg and be handed to the node as a
    # resolved parameter - never written into the YAML.
    root = tree.getroot()
    textfile_params = [p.get("textfile") for p in root.findall("param")
                       if p.get("name") == "robot_description"]
    if not textfile_params or not any(t and "$(arg urdf_file)" in t for t in textfile_params):
        results.add(name, STATUS_FAIL,
                    "g1_tf.launch must load robot_description with "
                    'textfile="$(arg urdf_file)"; found {}'.format(textfile_params))
        return

    # With robot_state_publisher absent, exercise the rest of the launch by
    # disabling that node rather than reporting a false failure.
    args = ["roslaunch", "--nodes", PACKAGE_NAME, "g1_tf.launch"]
    suffix = ""
    if not have_rsp:
        args.append("robot_state_publisher:=false")
        suffix = (" (checked with robot_state_publisher:=false because {} is not "
                  "installed)".format(ROBOT_STATE_PUBLISHER_PKG))

    code, output = run(args, timeout)
    if code != 0:
        last_line = output.strip().splitlines()[-1] if output.strip() else ""
        results.add(name, STATUS_FAIL,
                    "roslaunch --nodes {} g1_tf.launch failed (code={}): {}".format(
                        PACKAGE_NAME, code, last_line))
        return

    expected = [TF_NODE_NAME] + ([ROBOT_STATE_PUBLISHER_NODE] if have_rsp else [])
    missing = [n for n in expected if n not in output]
    if missing:
        results.add(name, STATUS_FAIL,
                    "expected node(s) {} not found in roslaunch --nodes output".format(
                        ", ".join(missing)))
        return

    status = STATUS_PASS if have_rsp else STATUS_WAIT
    results.add(name, status,
                "XML valid, robot_description from $(arg urdf_file), node(s) {} resolve{}".format(
                    ", ".join(expected), suffix))


def check_slam_bringup_wiring(results, pkg_path, timeout, have_rsp):
    name = "slam_bringup.launch composes driver + FAST-LIO2 + robot TF"
    if pkg_path is None:
        results.add(name, STATUS_FAIL, "cannot check, package path unknown")
        return

    launch_path = os.path.join(pkg_path, "launch", "slam_bringup.launch")
    try:
        import xml.etree.ElementTree as ET
        ET.parse(launch_path)
    except Exception as exc:
        results.add(name, STATUS_FAIL, "XML parse error in {}: {}".format(launch_path, exc))
        return

    args = ["roslaunch", "--nodes", PACKAGE_NAME, "slam_bringup.launch"]
    suffix = ""
    if not have_rsp:
        args.append("robot_state_publisher:=false")
        suffix = (" (checked with robot_state_publisher:=false because {} is not "
                  "installed)".format(ROBOT_STATE_PUBLISHER_PKG))

    code, output = run(args, timeout)
    if code != 0:
        last_line = output.strip().splitlines()[-1] if output.strip() else ""
        results.add(name, STATUS_FAIL,
                    "roslaunch --nodes {} slam_bringup.launch failed (code={}): {}".format(
                        PACKAGE_NAME, code, last_line))
        return

    expected = [LIVOX_NODE_NAME, FASTLIO_NODE_NAME, TF_NODE_NAME]
    if have_rsp:
        expected.append(ROBOT_STATE_PUBLISHER_NODE)
    missing = [n for n in expected if n not in output]
    if missing:
        results.add(name, STATUS_FAIL,
                    "slam_bringup.launch does not resolve node(s): {}".format(", ".join(missing)))
        return

    status = STATUS_PASS if have_rsp else STATUS_WAIT
    results.add(name, status, "resolves {}{}".format(", ".join(expected), suffix))


def _parse_fastlio_tf_edges(text):
    """(parent, child) pairs broadcast from FAST-LIO2's laserMapping.cpp.

    Matches tf::StampedTransform(<transform>, <stamp>, "<parent>", "<child>")
    which is the (frame_id, child_frame_id) argument order.
    """
    pattern = re.compile(
        r'StampedTransform\s*\([^,]+,[^,]+,\s*"([A-Za-z0-9_/]+)"\s*,\s*"([A-Za-z0-9_/]+)"\s*\)')
    return [(m.group(1), m.group(2)) for m in pattern.finditer(text)]


def check_fastlio_frame_contract(results, fastlio_pkg_path):
    name = "FAST-LIO2 TF broadcaster matches the TF framework's assumptions"
    if fastlio_pkg_path is None:
        results.add(name, STATUS_FAIL, "fast_lio package path unknown")
        return

    src_path = os.path.join(fastlio_pkg_path, "src", "laserMapping.cpp")
    try:
        with open(src_path) as handle:
            text = handle.read()
    except Exception as exc:
        results.add(name, STATUS_WAIT, "cannot read {}: {}".format(src_path, exc))
        return

    edges = _parse_fastlio_tf_edges(text)
    if not edges:
        results.add(name, STATUS_FAIL,
                    "no tf::StampedTransform broadcast found in {}; cannot confirm "
                    "FAST-LIO2 still publishes {} -> {}".format(
                        src_path, FASTLIO_MAP_FRAME, FASTLIO_BODY_FRAME))
        return

    if sorted(edges) != sorted(FASTLIO_TF_EDGES):
        results.add(name, STATUS_FAIL,
                    "FAST-LIO2 now broadcasts {} (expected exactly {}); the "
                    "{} -> {} chain in g1_tf_publisher may be wrong".format(
                        edges, FASTLIO_TF_EDGES, FASTLIO_MAP_FRAME, ROBOT_ROOT_FRAME))
        return

    results.add(name, STATUS_PASS,
                "FAST-LIO2 broadcasts {} -> {} (dynamic); g1_tf_publisher adds "
                "{} -> {} as a second child of {}".format(
                    FASTLIO_MAP_FRAME, FASTLIO_BODY_FRAME,
                    FASTLIO_MAP_FRAME, ROBOT_ROOT_FRAME, FASTLIO_MAP_FRAME))


def check_tf_single_parent(results, pkg_path, fastlio_pkg_path):
    """The whole tree - FAST-LIO2 + our node + the URDF - keeps one parent per frame."""
    name = "TF tree has a single parent per frame (FAST-LIO2 + g1_tf_publisher + URDF)"
    if pkg_path is None:
        results.add(name, STATUS_FAIL, "cannot check, package path unknown")
        return

    # FAST-LIO's edges: from source if readable, else the expected constant.
    fastlio_edges = list(FASTLIO_TF_EDGES)
    if fastlio_pkg_path is not None:
        src_path = os.path.join(fastlio_pkg_path, "src", "laserMapping.cpp")
        try:
            with open(src_path) as handle:
                parsed = _parse_fastlio_tf_edges(handle.read())
            if parsed:
                fastlio_edges = parsed
        except OSError:
            pass

    tf_chain, exc = _import_tf_chain(pkg_path)
    if tf_chain is None:
        results.add(name, STATUS_FAIL, "cannot import g1_slam.tf_chain: {}".format(exc))
        return

    urdf_path = os.path.join(pkg_path, URDF_REL)
    try:
        urdf_edges = [(parent, child) for parent, child, _n, _t
                      in tf_chain.joint_edges(urdf_path)]
    except Exception as exc:
        results.add(name, STATUS_FAIL, "cannot read joints from {}: {}".format(urdf_path, exc))
        return

    published = [(EXPECTED_TF_PARENT, EXPECTED_TF_CHILD)]
    all_edges = list(fastlio_edges) + published + urdf_edges

    parents_of = {}
    conflicts = []
    for parent, child in all_edges:
        if child in parents_of and parents_of[child] != parent:
            conflicts.append("{} is parented by both {} and {}".format(
                child, parents_of[child], parent))
        parents_of[child] = parent
        if parent == child:
            conflicts.append("{} is its own parent".format(child))

    # The Mid-360 extrinsic must stay a math constant. Broadcasting it would make
    # mid360_link a child of both torso_link (URDF) and body (FAST-LIO2's frame).
    if (FASTLIO_BODY_FRAME, LIDAR_FRAME) in all_edges:
        conflicts.append("{} -> {} must not be broadcast: {} is already a URDF child "
                         "of {}".format(FASTLIO_BODY_FRAME, LIDAR_FRAME,
                                        LIDAR_FRAME, TORSO_FRAME))

    # Nothing may invent base_link while the mapping is unresolved.
    frames = set(parents_of) | {p for p, _c in all_edges}
    if UNRESOLVED_BASE_FRAME in frames:
        conflicts.append("{} appears in the TF tree but the {} -> {} mapping is still "
                         "unresolved with the navigation team".format(
                             UNRESOLVED_BASE_FRAME, ROBOT_ROOT_FRAME, UNRESOLVED_BASE_FRAME))

    roots = sorted(frames - set(parents_of))
    if roots != [FASTLIO_MAP_FRAME]:
        conflicts.append("expected {} to be the only root frame, got {}".format(
            FASTLIO_MAP_FRAME, roots))

    if conflicts:
        results.add(name, STATUS_FAIL, "; ".join(sorted(set(conflicts))))
        return

    results.add(name, STATUS_PASS,
                "single parent per frame across {} frame(s): {} -> {{{}, {}}}, then the "
                "URDF chain {} -> {} -> {}; {} absent as required".format(
                    len(frames), FASTLIO_MAP_FRAME, FASTLIO_BODY_FRAME, ROBOT_ROOT_FRAME,
                    ROBOT_ROOT_FRAME, TORSO_FRAME, LIDAR_FRAME, UNRESOLVED_BASE_FRAME))


def check_joint_states(results, master, timeout, require_hardware):
    """/joint_states is external: absent means WAIT, never FAIL, never fabricated."""
    name = "external {} carries {}".format(JOINT_STATES_TOPIC, WAIST_YAW_JOINT)
    if master is None:
        results.add(name, hardware_status(require_hardware),
                    "no ROS master, so {} cannot be inspected. It is an external "
                    "interface owned by the robot-control / integration team; g1_slam "
                    "consumes it and never synthesizes it.".format(JOINT_STATES_TOPIC))
        return

    try:
        published = [t for t, _ty in master.getPublishedTopics("")[2]]
    except Exception as exc:
        results.add(name, hardware_status(require_hardware),
                    "cannot list topics from the master: {}".format(exc))
        return

    if JOINT_STATES_TOPIC not in published:
        results.add(name, hardware_status(require_hardware),
                    "{} is not published. {} -> {} will not be broadcast until the "
                    "robot-control / integration team provides it; g1_tf_publisher "
                    "stays up and publishes nothing rather than freezing {} at 0 "
                    "(which costs ~1 degree of heading per degree of waist rotation)".format(
                        JOINT_STATES_TOPIC, FASTLIO_MAP_FRAME, ROBOT_ROOT_FRAME,
                        WAIST_YAW_JOINT))
        return

    try:
        import rospy
        from sensor_msgs.msg import JointState
        global _node_initialized
        if not _node_initialized:
            rospy.init_node("g1_slam_health_check", anonymous=True, disable_signals=True)
            _node_initialized = True
        msg = rospy.wait_for_message(JOINT_STATES_TOPIC, JointState, timeout=timeout)
    except Exception as exc:
        results.add(name, hardware_status(require_hardware),
                    "{} is advertised but no message arrived within {}s: {}".format(
                        JOINT_STATES_TOPIC, timeout, exc))
        return

    if WAIST_YAW_JOINT not in list(msg.name):
        results.add(name, hardware_status(require_hardware),
                    "{} carries {} joint(s) but not {!r}; no {} -> {} can be "
                    "computed".format(JOINT_STATES_TOPIC, len(msg.name), WAIST_YAW_JOINT,
                                      FASTLIO_MAP_FRAME, ROBOT_ROOT_FRAME))
        return

    results.add(name, STATUS_PASS,
                "{} publishes {!r} among {} joint(s)".format(
                    JOINT_STATES_TOPIC, WAIST_YAW_JOINT, len(msg.name)))


def check_host_subnet(results, require_hardware):
    net = ipaddress.ip_interface(EXPECTED_HOST_IFACE).network

    code, output = run(["ip", "-o", "-4", "addr", "show"], 5)
    if code != 0:
        results.add("host has {}".format(EXPECTED_HOST_IFACE), hardware_status(require_hardware),
                     "could not read interface addresses: {}".format(output.strip()))
        return

    found = None
    for line in output.splitlines():
        for token in line.split():
            if "/" not in token:
                continue
            try:
                addr = ipaddress.ip_interface(token)
            except ValueError:
                continue
            if addr.ip in net:
                found = str(addr.ip)
                break
        if found:
            break

    if found:
        results.add("host has {}".format(EXPECTED_HOST_IFACE), STATUS_PASS,
                     "found {} on host".format(found))
    else:
        results.add("host has {}".format(EXPECTED_HOST_IFACE), hardware_status(require_hardware),
                     "no interface address in {} found on host".format(net))


def check_ping(results, timeout, require_hardware):
    wait_s = max(1, int(timeout))
    code, _ = run(["ping", "-c", "1", "-W", str(wait_s), EXPECTED_LIDAR_IP], timeout + 2)
    name = "ping {}".format(EXPECTED_LIDAR_IP)
    if code == 0:
        results.add(name, STATUS_PASS, "reply received")
    else:
        results.add(name, hardware_status(require_hardware),
                     "no reply within {}s".format(wait_s))


def check_ros_master(results, timeout, require_hardware):
    import socket
    old_timeout = socket.getdefaulttimeout()
    try:
        import rosgraph
        socket.setdefaulttimeout(timeout)
        master = rosgraph.Master("/slam_health_check")
        master.getPid()
    except Exception as exc:
        results.add("ROS master availability", hardware_status(require_hardware),
                     "master not reachable: {}".format(exc))
        return None
    finally:
        socket.setdefaulttimeout(old_timeout)

    results.add("ROS master availability", STATUS_PASS, "master responded")
    return master


def check_node(results, master, node_name, require_hardware):
    name = "{} node".format(node_name)
    if master is None:
        results.add(name, hardware_status(require_hardware), "ROS master unavailable")
        return
    try:
        master.lookupNode(node_name)
    except Exception as exc:
        results.add(name, hardware_status(require_hardware),
                     "node not registered: {}".format(exc))
        return
    results.add(name, STATUS_PASS, "node registered with master")


def _ensure_node_initialized():
    global _node_initialized
    if not _node_initialized:
        import rospy
        rospy.init_node("slam_health_check", anonymous=True,
                         disable_signals=True, log_level=rospy.ERROR)
        _node_initialized = True


def check_topic(results, master, topic, timeout, require_hardware):
    name = "{} publisher and message receipt".format(topic)

    if master is None:
        results.add(name, hardware_status(require_hardware), "ROS master unavailable")
        return

    try:
        publishers, _subs, _svcs = master.getSystemState()
        has_pub = any(t == topic and nodes for t, nodes in publishers)
    except Exception as exc:
        results.add(name, hardware_status(require_hardware),
                     "could not query system state: {}".format(exc))
        return

    if not has_pub:
        results.add(name, hardware_status(require_hardware),
                     "no publisher registered on {}".format(topic))
        return

    try:
        _ensure_node_initialized()
        import rospy
        rospy.wait_for_message(topic, rospy.AnyMsg, timeout=timeout)
        results.add(name, STATUS_PASS,
                     "publisher present and message received within {}s".format(timeout))
    except Exception as exc:
        results.add(name, hardware_status(require_hardware),
                     "publisher present but no message received within {}s: {}".format(timeout, exc))


def main():
    parser = argparse.ArgumentParser(description="G1 SLAM automated health check")
    parser.add_argument("--component",
                         choices=["mid360", "fastlio", "tf", "mapsave", "pcdmap", "mapserver"],
                         default="mid360", help="component to check")
    parser.add_argument("--timeout", type=float, default=5.0,
                         help="seconds to wait for network/topic responses (default: 5)")
    parser.add_argument("--json", action="store_true", help="emit machine-readable JSON")
    parser.add_argument("--require-hardware", action="store_true",
                         help="treat missing hardware/network/ROS state as FAIL instead of WAIT")
    args = parser.parse_args()

    results = Results(args.json)

    # Software checks: always FAIL on error, regardless of --require-hardware.
    pkg_path = check_package_files(results)

    if args.component == "mid360":
        check_livox_driver_package(results)
        check_config_json(results, pkg_path)
        check_sensors_launch(results, pkg_path, args.timeout)

        # Mid-360 runtime checks: WAIT by default, FAIL with --require-hardware.
        check_host_subnet(results, args.require_hardware)
        check_ping(results, args.timeout, args.require_hardware)
        master = check_ros_master(results, args.timeout, args.require_hardware)
        check_node(results, master, LIVOX_NODE_NAME, args.require_hardware)
        check_topic(results, master, LIDAR_TOPIC, args.timeout, args.require_hardware)
        check_topic(results, master, IMU_TOPIC, args.timeout, args.require_hardware)

    elif args.component == "fastlio":
        check_fastlio_package(results)
        check_fastlio_yaml(results, pkg_path)
        check_fastlio_launch(results, pkg_path, args.timeout)

        # FAST-LIO2 runtime checks: WAIT by default, FAIL with --require-hardware.
        master = check_ros_master(results, args.timeout, args.require_hardware)
        check_node(results, master, FASTLIO_NODE_NAME, args.require_hardware)
        check_topic(results, master, ODOM_TOPIC, args.timeout, args.require_hardware)
        check_topic(results, master, CLOUD_TOPIC, args.timeout, args.require_hardware)
        check_topic(results, master, PATH_TOPIC, args.timeout, args.require_hardware)

    elif args.component == "tf":
        # Offline: vendored-URDF integrity, chain math, config, launch resolution.
        fastlio_pkg_path = check_fastlio_package(results)
        check_vendored_urdf(results, pkg_path)
        check_urdf_source_md(results, pkg_path)
        check_tf_chain_module(results, pkg_path)
        check_g1_tf_config(results, pkg_path, args.require_hardware)
        check_g1_tf_publisher_script(results, pkg_path, args.timeout)
        have_rsp = check_robot_state_publisher_package(results, args.require_hardware) is not None
        check_g1_tf_launch(results, pkg_path, args.timeout, have_rsp)
        check_slam_bringup_wiring(results, pkg_path, args.timeout, have_rsp)
        check_fastlio_frame_contract(results, fastlio_pkg_path)
        check_tf_single_parent(results, pkg_path, fastlio_pkg_path)

        # Runtime: /joint_states is external. WAIT by default, FAIL only with
        # --require-hardware, so an offline/CI run never reports a false defect.
        master = check_ros_master(results, args.timeout, args.require_hardware)
        check_joint_states(results, master, args.timeout, args.require_hardware)

    elif args.component == "mapsave":
        check_maps_dir(results, pkg_path)
        check_save_map_script(results, pkg_path, args.timeout)
        fastlio_pkg_path = check_fastlio_package(results)

        # Depends on a completed mapping run: WAIT by default, FAIL with --require-hardware.
        check_map_source(results, fastlio_pkg_path, args.require_hardware)

    elif args.component == "pcdmap":
        # Entirely offline: build + CLI + synthetic round-trip, no ROS master needed.
        check_maps_dir(results, pkg_path)
        check_maps_gitignore(results, pkg_path)
        check_pcd_to_map_build(results, args.timeout)
        check_pcd_to_map_cli_errors(results, args.timeout)
        check_pcd_to_map_conversion(results, args.timeout)

    elif args.component == "mapserver":
        # Entirely offline: package + launch XML + roslaunch resolution, no master.
        check_map_server_package(results)
        check_map_server_launch(results, pkg_path, args.timeout)
        check_map_server_frame_param(results, pkg_path, args.timeout)

    overall = results.overall()
    if args.json:
        print(json.dumps({"component": args.component, "overall": overall,
                           "checks": results.records}, indent=2))
    else:
        print("[{}] overall - {} check(s), component={}".format(
            overall, len(results.records), args.component))

    return EXIT_CODES[overall]


if __name__ == "__main__":
    sys.exit(main())
