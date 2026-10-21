#!/usr/bin/env python3
"""CLI for the DIP handover node (needs handover.launch running).
(Named handover_cli.py: a scripts/handover.py would shadow the `handover`
Python package for handover_node.py.)

  rosrun handover handover_cli.py start      # arm holding the can -> handover pose, monitor armed
  rosrun handover handover_cli.py release    # open the hand now, settle, home the arm (manual trigger)
  rosrun handover handover_cli.py abort      # disarm the monitor, arm stays put
  rosrun handover handover_cli.py state      # print the latched state
"""
import sys

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest

CMDS = {"start": "/handover/start", "release": "/handover/run_auto_release", "abort": "/handover/abort"}


def main(argv):
    cmd = argv[1] if len(argv) > 1 else ""
    if cmd not in CMDS and cmd != "state":
        print(__doc__)
        return 2
    rospy.init_node("handover_cli", anonymous=True)
    if cmd == "state":
        msg = rospy.wait_for_message("/handover/state", String, timeout=5.0)
        print(msg.data)
        return 0
    rospy.wait_for_service(CMDS[cmd], timeout=10.0)
    resp = rospy.ServiceProxy(CMDS[cmd], Trigger)(TriggerRequest())
    print("%s: %s - %s" % (cmd, "OK" if resp.success else "FAILED", resp.message))
    return 0 if resp.success else 1


if __name__ == "__main__":
    sys.exit(main(rospy.myargv(sys.argv)))
