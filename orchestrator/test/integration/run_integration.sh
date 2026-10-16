#!/usr/bin/env bash
# Reproducible, isolated ROS integration test of the grasp-and-deliver pipeline.
#
#   orchestrator/test/integration/run_integration.sh
#
# ISOLATION, by construction:
#   * its own roscore on TEST_PORT (default 11399), never the default 11311;
#   * ROS_HOSTNAME/ROS_IP pinned to 127.0.0.1, so nothing binds the robot-facing
#     interface and no robot node can join this master;
#   * no SDK is imported anywhere: not unitree_sdk2py, bc_stark_sdk, ultralytics
#     or torch. Hardware-facing services are doubles_node.py;
#   * test-only parameters are uploaded to THIS master only. No file under
#     handover/config or orchestrator/config is modified.
#
# The orchestrator runs with enable_motion:=true on purpose: that is the only
# way its real RosBackend is constructed and real service calls are made. The
# services it reaches are the doubles on this isolated master.
set -u

PORT="${TEST_PORT:-11399}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGDIR="${TEST_LOGDIR:-/tmp/pipeline_itest}"
mkdir -p "$LOGDIR"

export ROS_MASTER_URI="http://127.0.0.1:${PORT}"
export ROS_HOSTNAME=127.0.0.1
export ROS_IP=127.0.0.1
export TEST_EVIDENCE_OUT="${TEST_EVIDENCE_OUT:-$LOGDIR/evidence.json}"

PIDS=()
cleanup() {
  for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null; done
  sleep 1
  for p in "${PIDS[@]:-}"; do kill -9 "$p" 2>/dev/null; done
}
trap cleanup EXIT

echo "== isolated master on :$PORT  (logs in $LOGDIR) =="
roscore -p "$PORT" > "$LOGDIR/roscore.log" 2>&1 &
PIDS+=($!)
for _ in $(seq 1 40); do
  rosparam list >/dev/null 2>&1 && break
  sleep 0.25
done
rosparam list >/dev/null 2>&1 || { echo "FATAL: roscore did not come up"; exit 1; }

echo "== uploading TEST-ONLY monitor params =="
rosparam load "$HERE/monitor_params.test.yaml" /auto_release_monitor

echo "== starting hardware-facing DOUBLES =="
# Invoked directly, not via rosrun: this is a test file and is deliberately
# NOT installed as a package executable.
python3 "$HERE/doubles_node.py" __name:=pipeline_doubles \
  _scenario:=happy _side:="${TEST_SIDE:-right}" > "$LOGDIR/doubles.log" 2>&1 &
PIDS+=($!)

echo "== starting the REAL auto_release_monitor =="
rosrun handover auto_release_monitor.py __name:=auto_release_monitor \
  > "$LOGDIR/monitor.log" 2>&1 &
PIDS+=($!)

echo "== starting the REAL arm_planning node (for /arm/convert_observation) =="
# The planning services are remapped ASIDE so the doubles serve them; only the
# frame conversion is served by production code here. The real node starts
# NOT_READY for planning (no torch) but handle_convert_observation is
# deliberately not gated on planner readiness.
# TEST-ONLY: a short lookup timeout so a missing transform fails fast.
# The cache keeps the production default: tf2 does not prune on read, so
# "stop publishing" never empties the buffer -- staleness is tested with an
# out-of-range stamp instead (see test_a_stale_stamp_is_refused).
rosparam set /arm_planning_real/tf_timeout_s 0.3
rosrun arm_planning arm_planning_node.py __name:=arm_planning_real \
  /arm/plan_to_pose:=/real_planner/plan_to_pose \
  /arm/plan_grasp_from_point:=/real_planner/plan_grasp_from_point \
  > "$LOGDIR/arm_planning.log" 2>&1 &
PIDS+=($!)
sleep 2

echo "== starting the REAL orchestrator (enable_motion:=true, doubles only) =="
rosparam load "$(rospack find orchestrator)/config/orchestrator.yaml" /orchestrator
rosparam set /orchestrator/enable_motion true
# TEST-ONLY timeouts. Production orchestrator.yaml keeps navigate: 1800 s
# (nav_client's own budget); a 30-minute wait is untestable, so the test
# master gets short ones. This narrows a wait; it never relaxes a check.
rosparam set /orchestrator/timeouts/navigate 4.0
rosparam set /orchestrator/timeouts/localization 5.0
rosparam set /orchestrator/timeouts/await_pull_sec 6.0
rosparam set /orchestrator/timeouts/service_call 5.0
rosparam set /orchestrator/side "${TEST_SIDE:-right}"
rosrun orchestrator orchestrator_node.py __name:=orchestrator \
  > "$LOGDIR/orchestrator.log" 2>&1 &
PIDS+=($!)

sleep 4
echo "== nodes on this master =="
rosnode list | sed 's/^/   /'

echo "== running the integration test =="
python3 "$HERE/test_ros_pipeline.py"
RC=$?

echo "== node logs: $LOGDIR =="
exit $RC
