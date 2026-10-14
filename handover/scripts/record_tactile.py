#!/usr/bin/env python3
"""Record tactile frames (and optionally detector internals) to CSV.

READ-ONLY. This tool subscribes, times and writes a file. It advertises no
service, publishes no command, opens no hand and moves no arm. It does not arm
the monitor node either: run it alongside the monitor, or on its own.

Two modes, because calibration is a two-step process:

  --raw        (default) record only the frames. Needs ONLY the tactile adapter
               configured, so this is what you use BEFORE you have thresholds.
  --detector   also run a detector over the same frames and record baseline,
               moving average, per-channel difference, candidate state and
               events. Needs the calibration values, so use it to VALIDATE
               thresholds you derived from a --raw recording.

    rosrun handover record_tactile.py --out /tmp/hold.csv --seconds 60
    rosrun handover record_tactile.py --out /tmp/pull.csv --seconds 20 --detector

Columns: wall_sec, frame_stamp, source_valid, then one column per declared
channel, and with --detector: baseline_*, avg_*, diff_* per enabled channel
plus state, candidate, qualifying, event.
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

import rospy

from handover import tactile_adapter as ta
from handover import tactile_frame as tf
from handover.tactile_detector import TactileDetectorConfig, TactileReleaseDetector


def _load_from_params(namespace: str):
    """Read the same structures the monitor node reads, from the parameter server."""
    get = lambda key, default: rospy.get_param("%s/%s" % (namespace.rstrip("/"), key), default)
    tactile = dict(get("tactile", {}) or {})
    mappings = []
    for entry in (tactile.get("channels") or []):
        mappings.append(ta.ChannelMapping(
            name=str(entry.get("name", "")), finger=str(entry.get("finger", "")),
            kind=str(entry.get("kind", tf.OPAQUE)), field=str(entry.get("field", "")),
            unit=str(entry.get("unit", "unknown")),
            period=float(entry.get("period", 0.0) or 0.0)))
    adapter = ta.TactileAdapterConfig(
        enabled=bool(tactile.get("enabled", False)),
        topic=str(tactile.get("topic", "")),
        message_type=str(tactile.get("message_type", "")),
        stamp_field=str(tactile.get("stamp_field", "")),
        valid_field=str(tactile.get("valid_field", "")),
        expected_rate_hz=float(tactile.get("expected_rate_hz", 0.0) or 0.0),
        mappings=tuple(mappings))

    calib = dict(get("calibration", {}) or {})
    number = lambda v: None if v is None or v == "" else float(v)
    config = TactileDetectorConfig(
        channels=adapter.specs(),
        enabled_channels=tuple(calib.get("enabled_channels", []) or []),
        release_threshold={str(k): float(v) for k, v in
                           (calib.get("release_threshold") or {}).items() if v not in (None, "")},
        stable_tolerance={str(k): float(v) for k, v in
                          (calib.get("stable_tolerance") or {}).items() if v not in (None, "")},
        direction={str(k): str(v) for k, v in (calib.get("direction") or {}).items() if v},
        debounce_sec=number(calib.get("debounce_sec")),
        baseline_frames=int(get("baseline_frames", 10)),
        moving_average_frames=int(get("moving_average_frames", 10)),
        post_arm_suppress_sec=float(get("post_arm_suppress_sec", 1.0)),
        sample_timeout_sec=float(get("sample_timeout_sec", 0.5)),
        monitor_timeout_sec=float(get("monitor_timeout_sec", 0.0)),
        baseline_timeout_sec=float(get("baseline_timeout_sec", 10.0)),
        min_channels=int(calib.get("min_channels", 1)),
        drift_gain=float(calib.get("drift_gain", 0.05)))
    return adapter, config


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", required=True, help="CSV file to write")
    p.add_argument("--seconds", type=float, default=60.0, help="recording length")
    p.add_argument("--namespace", default="/auto_release_monitor_right",
                   help="node namespace whose tactile/calibration params to read")
    p.add_argument("--detector", action="store_true",
                   help="also run the detector and record its internals (needs calibration)")
    p.add_argument("--note", default="", help="free-text note stored in the header comment")
    args = p.parse_args()

    rospy.init_node("record_tactile", anonymous=True)
    adapter, config = _load_from_params(args.namespace)

    problems = adapter.validate()
    if problems:
        rospy.logfatal("NOT_READY: cannot record without a configured tactile adapter:")
        for item in problems:
            rospy.logfatal("  - %s", item)
        return 2
    if args.detector:
        problems = config.validate()
        if problems:
            rospy.logfatal("NOT_READY: --detector needs calibration values:")
            for item in problems:
                rospy.logfatal("  - %s", item)
            rospy.logfatal("Record with --raw first, derive thresholds, then re-run.")
            return 2

    try:
        message_class = ta.resolve_message_class(adapter.message_type)
    except (ImportError, ValueError) as exc:
        rospy.logfatal("cannot load tactile message type %r: %s", adapter.message_type, exc)
        return 2

    channels = [m.name for m in adapter.mappings]
    active = config.active_channels() if args.detector else []
    header = ["wall_sec", "frame_stamp", "source_valid"] + channels
    if args.detector:
        header += (["baseline_%s" % c for c in active]
                   + ["avg_%s" % c for c in active]
                   + ["diff_%s" % c for c in active]
                   + ["state", "candidate", "fault", "qualifying", "event"])

    detector = TactileReleaseDetector(config) if args.detector else None
    if detector is not None:
        outcome = detector.arm(rospy.Time.now().to_sec())
        rospy.loginfo("detector arm: %s", outcome.reason)

    path = os.path.abspath(args.out)
    rows = [0]
    handle = open(path, "w", newline="")
    writer = csv.writer(handle)
    handle.write("# handover tactile recording; note=%s; topic=%s; type=%s; detector=%s\n"
                 % (args.note, adapter.topic, adapter.message_type, bool(args.detector)))
    writer.writerow(header)

    def on_msg(msg):
        now = rospy.Time.now().to_sec()
        try:
            frame = ta.extract_frame(msg, adapter, now)
        except (KeyError, TypeError, ValueError) as exc:
            rospy.logwarn_throttle(5.0, "unreadable frame: %s" % (exc,))
            writer.writerow([now, "", "0"] + ["" for _ in channels]
                            + ([""] * (3 * len(active) + 5) if args.detector else []))
            rows[0] += 1
            return
        row = [now, frame.stamp, int(bool(frame.source_valid))]
        row += [frame.values.get(c, "") for c in channels]
        if detector is not None:
            event = detector.update(now, frame)
            baseline, avg = detector.baseline, detector.moving_average()
            row += [baseline.get(c, "") for c in active]
            row += [avg.get(c, "") for c in active]
            row += [event.diffs.get(c, "") for c in active]
            row += [event.state, int(event.candidate), int(event.fault),
                    "|".join(event.qualifying), "confirmed" if event.trigger else ""]
            if event.trigger:
                rospy.loginfo("CONFIRMED pull: %s", event.reason)
        writer.writerow(row)
        rows[0] += 1

    rospy.Subscriber(adapter.topic, message_class, on_msg, queue_size=50)
    rospy.loginfo("recording %s for %.1fs -> %s", adapter.topic, args.seconds, path)
    deadline = rospy.Time.now() + rospy.Duration(args.seconds)
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        rate.sleep()
    handle.close()

    expected = adapter.expected_rate_hz * args.seconds if adapter.expected_rate_hz else 0
    rospy.loginfo("wrote %d row(s) to %s", rows[0], path)
    if expected and rows[0] < 0.5 * expected:
        rospy.logwarn("expected roughly %d rows at %.1f Hz; the sensor may be dropping frames",
                      int(expected), adapter.expected_rate_hz)
    if rows[0] == 0:
        rospy.logerr("no frames arrived on %s: nothing was recorded", adapter.topic)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
