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
EXPECTED_HOST_IP = "192.168.123.100"
EXPECTED_HOST_IFACE = "192.168.123.100/24"
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
TF_NODE_NAME = "/g1_static_tf"
TF_CONFIG_REL = os.path.join("config", "g1_tf.yaml")
ROBOT_BASE_FRAME = "base_link"          # G1 base frame used across this repo
FASTLIO_MAP_FRAME = "camera_init"       # FAST-LIO2 odometry origin
FASTLIO_BODY_FRAME = "body"             # FAST-LIO2 tracked (Mid-360 IMU) frame

# (parent, child) TF edges FAST-LIO2's laserMapping node broadcasts itself.
# Verified against src/laserMapping.cpp by check_fastlio_frame_contract().
FASTLIO_TF_EDGES = [(FASTLIO_MAP_FRAME, FASTLIO_BODY_FRAME)]

# The static TF attaches the robot base UNDER FAST-LIO's tree:
#   camera_init --(FAST-LIO, dynamic)--> body --(g1_static_tf, static)--> base_link
# The parent MUST be exactly `body`. camera_init is the fixed world/map frame
# and is not an allowed static-TF parent: base_link is rigid to the sensor
# (body), not to the world.
EXPECTED_TF_PARENT = FASTLIO_BODY_FRAME
EXPECTED_TF_CHILD = ROBOT_BASE_FRAME

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
    "scripts/g1_static_tf.py",
    "src/pcd_to_map.cpp",
]

# Offline PCD -> 2D map converter (component "pcdmap").
PCD_TO_MAP_BIN = "pcd_to_map"
MAPS_GITIGNORE_PATTERNS = ("*.pcd", "*.pgm", "*.yaml")

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


def check_g1_tf_config(results, pkg_path, require_hardware):
    name = "g1_tf.yaml validity and transform state"
    if pkg_path is None:
        results.add(name, STATUS_FAIL, "cannot check, package path unknown")
        return

    cfg_path = os.path.join(pkg_path, TF_CONFIG_REL)
    try:
        import yaml
        with open(cfg_path) as f:
            cfg = yaml.safe_load(f)
    except Exception as exc:
        results.add(name, STATUS_FAIL, "{}: {}".format(cfg_path, exc))
        return

    try:
        enabled = cfg["enabled"]
        parent = cfg["parent_frame"]
        child = cfg["child_frame"]
        tvals = [float(cfg["translation"][k]) for k in ("x", "y", "z")]
        rvals = [float(cfg["rotation_rpy"][k]) for k in ("roll", "pitch", "yaw")]
    except (KeyError, IndexError, TypeError, ValueError) as exc:
        results.add(name, STATUS_FAIL,
                     "unexpected/invalid structure in {}: {}".format(cfg_path, exc))
        return

    problems = []
    if not isinstance(enabled, bool):
        problems.append("'enabled' is not a bool: {!r}".format(enabled))
    if parent != EXPECTED_TF_PARENT:
        problems.append("parent_frame={!r} (must be {!r}; camera_init is the fixed "
                        "world frame and is not an allowed static-TF parent)".format(
                            parent, EXPECTED_TF_PARENT))
    if child != EXPECTED_TF_CHILD:
        problems.append("child_frame={!r} (must be {!r})".format(child, EXPECTED_TF_CHILD))
    if problems:
        results.add(name, STATUS_FAIL, "; ".join(problems))
        return

    placeholder = all(v == 0.0 for v in tvals + rvals)
    if enabled and placeholder:
        results.add(name, STATUS_FAIL,
                     "static TF enabled but translation+rotation are all-zero "
                     "placeholders; identity is not a measured transform")
        return
    if placeholder:
        results.add(name, hardware_status(require_hardware),
                     "static transform {} -> {} not yet measured (all zeros) and "
                     "disabled; expected shipped state - measure base_link in the "
                     "Mid-360 IMU frame, fill {}, then set enabled: true".format(
                         parent, child, cfg_path))
        return

    results.add(name, STATUS_PASS,
                 "{} -> {} xyz={} rpy={} enabled={}".format(parent, child, tvals, rvals, enabled))


def check_g1_static_tf_script(results, pkg_path, timeout):
    name = "g1_static_tf.py offline --check"
    if pkg_path is None:
        results.add(name, STATUS_FAIL, "cannot check, package path unknown")
        return

    script_path = os.path.join(pkg_path, "scripts", "g1_static_tf.py")
    cfg_path = os.path.join(pkg_path, TF_CONFIG_REL)
    code, output = run([sys.executable, script_path, "--check", "--config", cfg_path], timeout)
    last_line = output.strip().splitlines()[-1] if output.strip() else ""

    if code == 0:
        results.add(name, STATUS_PASS, last_line or "configuration valid")
    elif code == 2:
        results.add(name, STATUS_WAIT, last_line or "mounting transform pending measurement")
    else:
        results.add(name, STATUS_FAIL,
                     "g1_static_tf.py --check failed (code={}): {}".format(code, last_line))


def check_g1_tf_launch(results, pkg_path, timeout):
    name = "g1_tf.launch XML and ROS package/node resolution"
    if pkg_path is None:
        results.add(name, STATUS_FAIL, "cannot check, package path unknown")
        return

    launch_path = os.path.join(pkg_path, "launch", "g1_tf.launch")
    try:
        import xml.etree.ElementTree as ET
        ET.parse(launch_path)
    except Exception as exc:
        results.add(name, STATUS_FAIL, "XML parse error in {}: {}".format(launch_path, exc))
        return

    code, output = run(["roslaunch", "--nodes", PACKAGE_NAME, "g1_tf.launch"], timeout)
    if code != 0:
        last_line = output.strip().splitlines()[-1] if output.strip() else ""
        results.add(name, STATUS_FAIL,
                     "roslaunch --nodes {} g1_tf.launch failed (code={}): {}".format(
                         PACKAGE_NAME, code, last_line))
        return
    if TF_NODE_NAME not in output:
        results.add(name, STATUS_FAIL,
                     "expected node {} not found in roslaunch --nodes output".format(TF_NODE_NAME))
        return

    results.add(name, STATUS_PASS, "XML valid, node {} resolves".format(TF_NODE_NAME))


def check_slam_bringup_wiring(results, pkg_path, timeout):
    name = "slam_bringup.launch composes driver + FAST-LIO2 + static TF"
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

    code, output = run(["roslaunch", "--nodes", PACKAGE_NAME, "slam_bringup.launch"], timeout)
    if code != 0:
        last_line = output.strip().splitlines()[-1] if output.strip() else ""
        results.add(name, STATUS_FAIL,
                     "roslaunch --nodes {} slam_bringup.launch failed (code={}): {}".format(
                         PACKAGE_NAME, code, last_line))
        return

    missing = [n for n in (LIVOX_NODE_NAME, FASTLIO_NODE_NAME, TF_NODE_NAME) if n not in output]
    if missing:
        results.add(name, STATUS_FAIL,
                     "slam_bringup.launch does not resolve node(s): {}".format(", ".join(missing)))
        return

    results.add(name, STATUS_PASS,
                 "resolves {}, {}, {}".format(LIVOX_NODE_NAME, FASTLIO_NODE_NAME, TF_NODE_NAME))


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
        with open(src_path) as f:
            text = f.read()
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
                     "FAST-LIO2 now broadcasts {} (expected exactly {}); the static TF "
                     "wiring in g1_tf.yaml may create a parent conflict or a gap".format(
                         edges, FASTLIO_TF_EDGES))
        return

    results.add(name, STATUS_PASS,
                 "FAST-LIO2 broadcasts {} -> {} (dynamic); static TF hangs {} under {}".format(
                     FASTLIO_MAP_FRAME, FASTLIO_BODY_FRAME, EXPECTED_TF_CHILD, EXPECTED_TF_PARENT))


def check_static_tf_single_parent(results, pkg_path, fastlio_pkg_path):
    """Guard: the whole TF tree must keep exactly one parent per frame."""
    name = "TF tree has a single parent per frame (no camera_init/body re-parenting)"
    if pkg_path is None:
        results.add(name, STATUS_FAIL, "cannot check, package path unknown")
        return

    # FAST-LIO's edges: from source if readable, else the expected constant.
    fastlio_edges = list(FASTLIO_TF_EDGES)
    if fastlio_pkg_path is not None:
        src_path = os.path.join(fastlio_pkg_path, "src", "laserMapping.cpp")
        try:
            with open(src_path) as f:
                parsed = _parse_fastlio_tf_edges(f.read())
            if parsed:
                fastlio_edges = parsed
        except OSError:
            pass

    cfg_path = os.path.join(pkg_path, TF_CONFIG_REL)
    try:
        import yaml
        with open(cfg_path) as f:
            cfg = yaml.safe_load(f)
        parent = cfg["parent_frame"]
        child = cfg["child_frame"]
    except Exception as exc:
        results.add(name, STATUS_FAIL, "cannot read frames from {}: {}".format(cfg_path, exc))
        return

    # Build the child -> parent map for every static + dynamic edge and flag
    # any child that is claimed twice.
    all_edges = list(fastlio_edges) + [(parent, child)]
    parents_of = {}
    conflicts = []
    for p, c in all_edges:
        if c in parents_of and parents_of[c] != p:
            conflicts.append("{} is parented by both {} and {}".format(c, parents_of[c], p))
        parents_of[c] = p
        if p == c:
            conflicts.append("{} is its own parent".format(c))

    owned = {c for _p, c in fastlio_edges}
    if child in owned:
        conflicts.append("static TF child {!r} is already broadcast by FAST-LIO2 "
                         "(it would get a second parent)".format(child))

    if conflicts:
        results.add(name, STATUS_FAIL, "; ".join(sorted(set(conflicts))))
        return

    edge_strs = ["{} -> {}".format(p, c) for p, c in all_edges]
    results.add(name, STATUS_PASS,
                 "single parent per frame; edges: {}".format(", ".join(edge_strs)))


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
                         choices=["mid360", "fastlio", "tf", "mapsave", "pcdmap"],
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
        # Entirely offline: file/XML/frame-contract checks, no ROS master needed.
        fastlio_pkg_path = check_fastlio_package(results)
        check_g1_tf_config(results, pkg_path, args.require_hardware)
        check_g1_static_tf_script(results, pkg_path, args.timeout)
        check_g1_tf_launch(results, pkg_path, args.timeout)
        check_slam_bringup_wiring(results, pkg_path, args.timeout)
        check_fastlio_frame_contract(results, fastlio_pkg_path)
        check_static_tf_single_parent(results, pkg_path, fastlio_pkg_path)

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
