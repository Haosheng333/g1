#!/usr/bin/env bash
# Stop everything perception.launch started (and a stray roscore/rosmaster).
# ROS's Python nodes show up as "python3", so `killall roslaunch` misses them;
# this matches on the command line and excludes this script's own shell.
set -u
PATTERN='roslaunch|rosmaster|roscore|perception_node\.py|perception_viewer\.py|image_view|image_saver|realsense2_camera|nodelet'
pids=$(pgrep -f "$PATTERN" | grep -vx "$$" | grep -vx "$PPID" || true)
if [ -n "$pids" ]; then
  kill -INT $pids 2>/dev/null; sleep 2
  pids=$(pgrep -f "$PATTERN" | grep -vx "$$" | grep -vx "$PPID" || true)
  [ -n "$pids" ] && kill -9 $pids 2>/dev/null
fi
sleep 1
# Zombies (state Z) are already dead; they linger only because their parent
# shell is gone and the container's init does not reap them. Ignore them.
left=$(pgrep -f "$PATTERN" | grep -vx "$$" | grep -vx "$PPID" | xargs -r ps -o pid=,stat= -p 2>/dev/null | awk '$2 !~ /Z/' | wc -l)
echo "perception stopped; live leftover processes: $left"
