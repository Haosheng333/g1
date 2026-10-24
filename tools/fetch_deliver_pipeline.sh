#!/usr/bin/env bash
# Start, check, dry-run and stop the fetch-and-deliver pipeline, one roslaunch
# per component, each with its own log and PID file.
#
#   tools/fetch_deliver_pipeline.sh start      # roscore + every component (dry run)
#   tools/fetch_deliver_pipeline.sh check      # the readiness checklist (tools/fetch_deliver_check.py)
#   tools/fetch_deliver_pipeline.sh dryrun     # rosservice call run_fetch_deliver (dry run)
#   tools/fetch_deliver_pipeline.sh status     # which components are running
#   tools/fetch_deliver_pipeline.sh stop       # stop everything it started
#   tools/fetch_deliver_pipeline.sh restart orchestrator   # e.g. after ENABLE_MOTION=true
#
# Run it INSIDE the container (docker/g1.sh enter), with the robot cable up.
# Settings are environment variables, all optional:
#   MAP_NAME=lab_map_DIP  SIDE=left  LABEL=bottle
#   JOINT_MODE=relay  JOINT_INPUT_TOPIC=/arm/joint_states  NETWORK_INTERFACE=
#   ENABLE_MOTION=false        the orchestrator's motion flag (false = dry run)
#   CAMERA_ON_PC2=true         false = RealSense driver on this host
#   VOICE=false  VOICE_DEVICE=4  VOICE_DRY_RUN=true  VOICE_CONFIRM=confirm
#   SKIP="voice"               space-separated components not to start
#
# Components, in start order (the names `restart`/`SKIP` use):
#   roscore navigation camera perception arm_grasp hands handover orchestrator voice
# `camera` and `hands` are the PC2 processes (start_remote_camera.sh,
# start_remote_hands.sh); they run here as background SSH sessions.
#
# Logs: ~/.ros/fetch_deliver/<component>.log   PIDs: ~/.ros/fetch_deliver/<component>.pid
# Stopped components may linger as <defunct> entries in ps: the container's PID 1
# (`sleep infinity`) never reaps them. They hold no resources.
set -u

MAP_NAME="${MAP_NAME:-lab_map_DIP}"
SIDE="${SIDE:-left}"
LABEL="${LABEL:-bottle}"
JOINT_MODE="${JOINT_MODE:-relay}"
JOINT_INPUT_TOPIC="${JOINT_INPUT_TOPIC:-/arm/joint_states}"
NETWORK_INTERFACE="${NETWORK_INTERFACE:-${G1_NETWORK_INTERFACE:-}}"
ENABLE_MOTION="${ENABLE_MOTION:-false}"
CAMERA_ON_PC2="${CAMERA_ON_PC2:-true}"
VOICE="${VOICE:-false}"
VOICE_DEVICE="${VOICE_DEVICE:-4}"
VOICE_DRY_RUN="${VOICE_DRY_RUN:-true}"
VOICE_CONFIRM="${VOICE_CONFIRM:-confirm}"
SKIP="${SKIP:-}"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIP="$(cd "$HERE/.." && pwd)"
RUN_DIR="${FETCH_DELIVER_RUN_DIR:-$HOME/.ros/fetch_deliver}"
mkdir -p "$RUN_DIR"

ALL="roscore navigation camera perception arm_grasp hands handover orchestrator voice"

# shellcheck disable=SC1090
source "$DIP/g1_ws/devel/setup.bash" --extend 2>/dev/null || {
  echo "ERROR: cannot source $DIP/g1_ws/devel/setup.bash; build the workspace first" >&2; exit 1; }

skipped() { [[ " $SKIP " == *" $1 "* ]]; }

# A process is alive if it exists and is not a zombie. The container's PID 1 is
# `sleep infinity`, which never reaps, so every detached component becomes a
# harmless <defunct> entry after it exits; kill -0 alone would call that alive.
alive() {
  local pid="$1" state
  [[ -r "/proc/$pid/stat" ]] || return 1
  state="$(awk '{print $3}' "/proc/$pid/stat" 2>/dev/null)"
  [[ "$state" != "Z" && "$state" != "X" ]]
}

running() {
  local pidfile="$RUN_DIR/$1.pid"
  [[ -f "$pidfile" ]] && alive "$(cat "$pidfile")"
}

# launch <name> <command...>: detached, its own log, PID recorded.
launch() {
  local name="$1"; shift
  if running "$name"; then echo "  $name: already running (pid $(cat "$RUN_DIR/$name.pid"))"; return; fi
  setsid nohup "$@" > "$RUN_DIR/$name.log" 2>&1 < /dev/null &
  echo $! > "$RUN_DIR/$name.pid"
  echo "  $name: started (pid $!)  log: $RUN_DIR/$name.log"
}

wait_for() {   # wait_for <seconds> <description> <command...>
  local secs="$1" what="$2"; shift 2
  local t=0
  while (( t < secs )); do
    if "$@" >/dev/null 2>&1; then return 0; fi
    sleep 1; t=$((t+1))
  done
  echo "  WARNING: $what not up after ${secs}s (see the log)"; return 1
}

component_cmd() {
  case "$1" in
    roscore)      echo roscore ;;
    navigation)   echo roslaunch system_bringup navigation.launch map_name:="$MAP_NAME" \
                    joint_mode:="$JOINT_MODE" joint_input_topic:="$JOINT_INPUT_TOPIC" \
                    network_interface:="$NETWORK_INTERFACE" motion_enabled:=false ;;
    camera)       if [[ "$CAMERA_ON_PC2" == "true" ]]; then echo rosrun perception start_remote_camera.sh; else echo ""; fi ;;
    perception)   if [[ "$CAMERA_ON_PC2" == "true" ]]; then echo roslaunch perception perception.launch local_camera:=false;
                  else echo roslaunch perception perception.launch local_camera:=true; fi ;;
    arm_grasp)    echo roslaunch arm_grasp arm_node_abs.launch rviz:=false publish_tf_stubs:=false grasp_bottle_arm:="$SIDE" ;;
    hands)        echo rosrun hand_control start_remote_hands.sh ;;
    handover)     echo roslaunch handover handover.launch arm:="$SIDE" label:="$LABEL" ;;
    orchestrator) echo roslaunch orchestrator orchestrator.launch side:="$SIDE" map_name:="$MAP_NAME" enable_motion:="$ENABLE_MOTION" ;;
    voice)        echo roslaunch g1_voice_control voice_control.launch input_device:="$VOICE_DEVICE" \
                    dry_run_backend:="$VOICE_DRY_RUN" confirmation_mode:="$VOICE_CONFIRM" ;;
    *) echo "unknown component $1" >&2; return 1 ;;
  esac
}

start_one() {
  local name="$1"
  if skipped "$name"; then echo "  $name: skipped (SKIP)"; return; fi
  if [[ "$name" == "voice" && "$VOICE" != "true" ]]; then echo "  voice: off (VOICE=true to start)"; return; fi
  local cmd; cmd="$(component_cmd "$name")" || return 1
  [[ -z "$cmd" ]] && { echo "  $name: nothing to start"; return; }
  # shellcheck disable=SC2086
  launch "$name" $cmd
  case "$name" in
    roscore)      wait_for 15 "roscore" rostopic list ;;
    navigation)   wait_for 20 "navigation_node" rosservice info /navigation/navigate ;;
    perception)   wait_for 30 "perception_node" rosservice info /perception/get_object_position ;;
    arm_grasp)    wait_for 90 "g1_arm_simple" rosservice info /g1_arm_simple/plan_grasp_from_point ;;
    handover)     wait_for 15 "handover_node" rosservice info /handover/start ;;
    orchestrator) wait_for 15 "orchestrator" rosservice info /orchestrator/run_fetch_deliver ;;
    camera)       wait_for 40 "camera topics from PC2" rostopic info /camera/color/image_raw ;;
    hands)        wait_for 40 "/g1_hand_$SIDE services from PC2" rosservice info "/g1_hand_$SIDE/grasp_5f" ;;
  esac
}

cmd_start() {
  echo "== fetch_deliver pipeline: start  (map=$MAP_NAME side=$SIDE enable_motion=$ENABLE_MOTION camera_on_pc2=$CAMERA_ON_PC2)"
  echo "   logs in $RUN_DIR"
  if [[ "$ENABLE_MOTION" == "true" ]]; then
    echo "   !! ENABLE_MOTION=true: the orchestrator will MOVE the robot on the next run_fetch_deliver call"
  fi
  for c in $ALL; do start_one "$c"; done
  echo "== started. Next: $0 check"
}

# Each component was started with setsid, so its PID is also its process
# group: SIGINT the group (roslaunch then stops its nodes), wait, SIGKILL
# whatever is left. Never pkill by name: a pattern such as "roscore" also
# matches the shell that is running this script.
stop_group() {
  local name="$1" pid="$2"
  kill -INT -- -"$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null
  local t=0
  while (( t < 15 )) && alive "$pid"; do sleep 1; t=$((t+1)); done
  if alive "$pid"; then
    kill -KILL -- -"$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null
    echo "  $name: killed after ${t}s (pid $pid)"
  else
    echo "  $name: stopped (pid $pid)"
  fi
}

cmd_stop() {
  echo "== fetch_deliver pipeline: stop"
  local list; list="$(echo $ALL | tr ' ' '\n' | tac | tr '\n' ' ')"
  for c in $list; do
    local pidfile="$RUN_DIR/$c.pid"
    [[ -f "$pidfile" ]] || continue
    local pid; pid="$(cat "$pidfile")"
    if alive "$pid"; then stop_group "$c" "$pid"; else echo "  $c: was not running"; fi
    rm -f "$pidfile"
  done
  echo "  done. PC2 processes were stopped through their SSH sessions; verify with: rosservice list | grep -E 'g1_hand_|camera'"
}

cmd_status() {
  for c in $ALL; do
    if running "$c"; then echo "  $c: running (pid $(cat "$RUN_DIR/$c.pid"))"; else echo "  $c: not running"; fi
  done
}

cmd_restart() {
  local name="${1:?component name}"
  local pidfile="$RUN_DIR/$name.pid"
  if [[ -f "$pidfile" ]]; then
    local pid; pid="$(cat "$pidfile")"
    alive "$pid" && stop_group "$name" "$pid"
    rm -f "$pidfile"
  fi
  start_one "$name"
}

cmd_dryrun() {
  echo "== dry run: run_fetch_deliver target=table_01 destination=chair_01 label=$LABEL"
  rosservice call /orchestrator/run_fetch_deliver "{target: table_01, destination: chair_01, label: $LABEL, dry_run: true}"
}

case "${1:-}" in
  start)   cmd_start ;;
  stop)    cmd_stop ;;
  status)  cmd_status ;;
  restart) cmd_restart "${2:-}" ;;
  check)   exec python3 "$HERE/fetch_deliver_check.py" --side "$SIDE" --map-name "$MAP_NAME" --label "$LABEL" "${@:2}" ;;
  dryrun)  cmd_dryrun ;;
  *) sed -n 2,30p "$0"; exit 2 ;;
esac
