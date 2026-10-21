#!/usr/bin/env bash
# Stop what perception.launch started: perception_node, perception_viewer, the
# image_view window and, if perception.launch started it (local_camera:=true),
# the local RealSense driver. ROS's Python nodes show up as "python3", so
# `killall roslaunch` misses them; this matches on the command line and
# excludes this script's own shell.
#
# NOT stopped by default, because other terminals may own them:
#   * roscore / rosmaster                -> --with-master
#   * the PC2 camera session (start_remote_camera.sh, an ssh/sshpass process)
#                                        -> --with-remote-camera
#   * other packages' roslaunch (arm_grasp, navigation, ...)
#   --all = both flags.
set -u
PATTERN='roslaunch perception|perception_node\.py|perception_viewer\.py|image_view|image_saver|realsense2_camera_node|nodelet manager __name:=realsense2_camera_manager'
EXCLUDE='sshpass|ssh |start_remote_camera'
for a in "$@"; do
  case "$a" in
    --with-master)        PATTERN="$PATTERN|rosmaster|roscore|roslaunch.*--core" ;;
    --with-remote-camera) EXCLUDE='^$' ;;
    --all)                PATTERN="$PATTERN|rosmaster|roscore|roslaunch.*--core"; EXCLUDE='^$' ;;
    *) echo "usage: stop_perception.sh [--with-master] [--with-remote-camera] [--all]" >&2; exit 2 ;;
  esac
done
list() { pgrep -af "$PATTERN" | grep -Ev "$EXCLUDE" | awk '{print $1}' | grep -vx "$$" | grep -vx "$PPID" || true; }
pids=$(list)
if [ -n "$pids" ]; then
  kill -INT $pids 2>/dev/null; sleep 2
  pids=$(list); [ -n "$pids" ] && kill -9 $pids 2>/dev/null
fi
sleep 1
# Zombies (state Z) are already dead; they linger only because their parent
# shell is gone and the container's init does not reap them. Ignore them.
left=$(list | xargs -r ps -o pid=,stat= -p 2>/dev/null | awk '$2 !~ /Z/' | wc -l)
echo "perception stopped; live leftover processes: $left"
