#!/usr/bin/env bash
# Start the BrainCo Revo2 hand nodes on G1 PC2 over SSH, registered with the ROS
# master on this workstation. They provide the services arm_grasp calls:
#   /g1_hand_{left,right}/pre_grasp   /grasp_5f   /release   (+ berry/point variants)
#
# Adapted from g1_intellect's g1_hands/scripts/start_remote_hands.sh. The hand
# nodes themselves are g1_intellect's `g1_hands` package running from PC2's
# checkout (both hands hang off PC2's USB buses); DIP only starts them.
# Starting the nodes does not move the fingers.
#
# Prerequisites
#   * roscore running on this workstation, started in a shell where the robot
#     cable was already up (ROS_IP / ROS_MASTER_URI on the robot interface).
#   * sshpass on this machine (the container has it).
#
# Idempotent: hand nodes already running on PC2 are stopped first so the new
# launch owns the serial buses (a stale node keeps its Modbus port open and
# the new one fails auto-detection).
#
# Usage
#   rosrun hand_control start_remote_hands.sh                         # Ctrl+C stops the hands
#   rosrun hand_control start_remote_hands.sh bringup_left_hand:=false
#   G1_HAND_RIGHT_PORT=/dev/ttyUSB2 rosrun hand_control start_remote_hands.sh
# Extra arguments are appended to the remote roslaunch verbatim and override
# the per-side defaults below (roslaunch lets the last occurrence win).
set -euo pipefail

REMOTE_USER="${G1_PC2_USER:-unitree}"
REMOTE_IP="${G1_PC2_IP:-192.168.123.164}"
REMOTE_PASS="${G1_PC2_PASS:-123}"
# Workspace on PC2 holding g1_hands. $HOME stays unexpanded on purpose: the
# remote shell resolves it.
REMOTE_WS="${G1_PC2_WS:-\$HOME/Workspace/g1_intellect/catkin_ws}"

# Per-side serial bus config, passed explicitly so this script is the single
# source of truth even if PC2's launch file defaults are stale.
HAND_RIGHT_PORT="${G1_HAND_RIGHT_PORT:-/dev/ttyUSB1}"
HAND_RIGHT_SLAVE_ID="${G1_HAND_RIGHT_SLAVE_ID:-127}"
HAND_LEFT_PORT="${G1_HAND_LEFT_PORT:-/dev/ttyUSB0}"
HAND_LEFT_SLAVE_ID="${G1_HAND_LEFT_SLAVE_ID:-126}"
BRINGUP_RIGHT_HAND="${G1_BRINGUP_RIGHT_HAND:-true}"
BRINGUP_LEFT_HAND="${G1_BRINGUP_LEFT_HAND:-true}"

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

echo "PC2 serial devices: $("${SSH[@]}" 'ls /dev/ttyUSB* 2>/dev/null | tr "\n" " "' 2>/dev/null || echo '(could not list)')"
echo "Stopping any existing hand nodes on PC2..."
# The bracket keeps pkill from matching the shell carrying the pattern.
"${SSH[@]}" 'pkill -f "[r]evo2_tactile_node"; pkill -f "[r]evo2_.*_hand_node"; true' >/dev/null 2>&1 || true
sleep 2

echo "Starting Revo2 hands on PC2 ${REMOTE_USER}@${REMOTE_IP}"
echo "  ROS master: http://${MASTER_IP}:11311  (via ${NIF})"
echo "  Right: ${BRINGUP_RIGHT_HAND} on ${HAND_RIGHT_PORT} (slave ${HAND_RIGHT_SLAVE_ID})"
echo "  Left:  ${BRINGUP_LEFT_HAND} on ${HAND_LEFT_PORT} (slave ${HAND_LEFT_SLAVE_ID})"
echo "  Check here with: rosservice list | grep g1_hand_"

# Local variables expand here (double quotes); $HOME in REMOTE_WS expands on PC2.
sshpass -p "$REMOTE_PASS" ssh -tt \
  -o StrictHostKeyChecking=no \
  -o UserKnownHostsFile=/dev/null \
  "${REMOTE_USER}@${REMOTE_IP}" "bash -lc '
    source /opt/ros/noetic/setup.bash
    source ${REMOTE_WS}/devel/setup.bash
    export ROS_MASTER_URI=http://${MASTER_IP}:11311
    export ROS_IP=${REMOTE_IP}
    roslaunch g1_hands revo2_dual_hand_node.launch \
      bringup_right_hand:=${BRINGUP_RIGHT_HAND} \
      bringup_left_hand:=${BRINGUP_LEFT_HAND} \
      right_port:=${HAND_RIGHT_PORT} right_slave_id:=${HAND_RIGHT_SLAVE_ID} \
      left_port:=${HAND_LEFT_PORT} left_slave_id:=${HAND_LEFT_SLAVE_ID} \
      $*
  '"
