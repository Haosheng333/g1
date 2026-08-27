#!/usr/bin/env python3
import argparse
import ipaddress
import json
import os
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

REQUIRED_FILES = [
    "package.xml",
    "CMakeLists.txt",
    "config/mid360_config.json",
    "config/fastlio.yaml",
    "launch/sensors.launch",
    "launch/fastlio_mapping.launch",
    "scripts/slam_health_check.py",
    "scripts/save_map.py",
]

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
    parser.add_argument("--component", choices=["mid360", "fastlio", "mapsave"], default="mid360",
                         help="component to check")
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

    elif args.component == "mapsave":
        check_maps_dir(results, pkg_path)
        check_save_map_script(results, pkg_path, args.timeout)
        fastlio_pkg_path = check_fastlio_package(results)

        # Depends on a completed mapping run: WAIT by default, FAIL with --require-hardware.
        check_map_source(results, fastlio_pkg_path, args.require_hardware)

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
