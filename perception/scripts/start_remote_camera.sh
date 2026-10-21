#!/usr/bin/env bash
# Start the RealSense (D455) ROS driver on G1 PC2 over SSH, publishing to the
# ROS master on this workstation:
#   /camera/color/image_raw                 1280x720 @ 15 fps
#   /camera/aligned_depth_to_color/image_raw 16UC1 mm, aligned to colour
#   /camera/color/camera_info
#
# Adapted from g1_intellect's g1_perception/scripts/start_remote_camera.sh.
# Differences: runs realsense2_camera's rs_camera.launch on PC2 directly (no
# dependency on PC2's g1_intellect checkout), and the stream settings are the
# ones perception.launch local_camera:=true uses, so the two paths are
# interchangeable.
#
# Prerequisites
#   * roscore running on this workstation, started in a shell where the robot
#     cable was already up (ROS_IP / ROS_MASTER_URI then point at the robot
#     interface, see g1_docker/g1_env.sh).
#   * the camera plugged into PC2's USB 3 port.
#   * sshpass on this machine (the container has it).
#   * PC2's clock synced to the workstation (g1_intellect
#     scripts/setup_remote_timesync.sh, once per boot) or frame ages drift.
#
# Idempotent: any RealSense already running on PC2 is stopped first so the new
# launch owns the USB device and stamps frames with the current PC2 clock.
#
# Usage
#   rosrun perception start_remote_camera.sh          # Ctrl+C stops the camera
#   G1_PC2_IP=... G1_PC2_USER=... G1_PC2_PASS=... rosrun perception start_remote_camera.sh
set -euo pipefail

REMOTE_USER="${G1_PC2_USER:-unitree}"
REMOTE_IP="${G1_PC2_IP:-192.168.123.164}"
REMOTE_PASS="${G1_PC2_PASS:-123}"
REMOTE_ROS_SETUP="${G1_PC2_ROS_SETUP:-/opt/ros/noetic/setup.bash}"

# Stream settings: keep in sync with launch/perception.launch (local_camera).
CAM_COLOR_W=1280
CAM_COLOR_H=720
CAM_DEPTH_W=848
CAM_DEPTH_H=480
CAM_FPS=15
CAM_RESET=true          # USB reset at start; clears the stream-start stall after an unclean stop

NIF="${G1_NETWORK_INTERFACE:-}"
if [[ -z "$NIF" ]]; then
  echo "ERROR: G1_NETWORK_INTERFACE is not set (the robot-facing interface, e.g. enp196s0)." >&2
  exit 1
fi
MASTER_IP="$(ip -4 -o addr show dev "$NIF" 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -n1)"
if [[ -z "$MASTER_IP" ]]; then
  echo "ERROR: no IPv4 address on '$NIF'. Is the robot cable connected?" >&2
  exit 1
fi
if ! command -v sshpass >/dev/null 2>&1; then
  echo "ERROR: sshpass not found (apt-get install -y sshpass)." >&2
  exit 1
fi
if ! ping -c1 -W1 "$REMOTE_IP" >/dev/null 2>&1; then
  echo "ERROR: PC2 ($REMOTE_IP) does not answer ping." >&2
  exit 1
fi

SSH=(sshpass -p "$REMOTE_PASS" ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null "${REMOTE_USER}@${REMOTE_IP}")

echo "Stopping any RealSense already running on PC2..."
"${SSH[@]}" 'pkill -f realsense2_camera; pkill -f rs_camera; pkill -f realsense_d435i; true' >/dev/null 2>&1 || true
sleep 2

echo "Starting RealSense on PC2 ${REMOTE_USER}@${REMOTE_IP}"
echo "  ROS master: http://${MASTER_IP}:11311  (via ${NIF})"
echo "  Stream: color ${CAM_COLOR_W}x${CAM_COLOR_H} / depth ${CAM_DEPTH_W}x${CAM_DEPTH_H} @ ${CAM_FPS} fps, align_depth, initial_reset=${CAM_RESET}"
echo "  Check here with: rostopic hz /camera/color/image_raw /camera/aligned_depth_to_color/image_raw"

# Local variables expand here (double quotes); the remote shell sees the result.
sshpass -p "$REMOTE_PASS" ssh -tt \
  -o StrictHostKeyChecking=no \
  -o UserKnownHostsFile=/dev/null \
  "${REMOTE_USER}@${REMOTE_IP}" "bash -lc '
    source ${REMOTE_ROS_SETUP}
    export ROS_MASTER_URI=http://${MASTER_IP}:11311
    export ROS_IP=${REMOTE_IP}
    roslaunch realsense2_camera rs_camera.launch \
      align_depth:=true \
      color_width:=${CAM_COLOR_W} color_height:=${CAM_COLOR_H} \
      depth_width:=${CAM_DEPTH_W} depth_height:=${CAM_DEPTH_H} \
      color_fps:=${CAM_FPS} depth_fps:=${CAM_FPS} \
      initial_reset:=${CAM_RESET}
  '"
