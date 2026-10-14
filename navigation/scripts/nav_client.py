"""Client helper that implements the orchestrator's run_navigate EXACTLY as in doc 4.3,
including the saw_active guard (a terminal state only counts after an ACTIVE one was seen).

    from nav_client import run_navigate
    result = run_navigate("table", 0.6)      # -> "SUCCESS" | "FAILURE" | "TIMEOUT"
"""
import threading

import rospy
from std_msgs.msg import String

ACTIVE = ("PLANNING", "FOLLOWING", "RECOVERING")
TERMINAL = ("SUCCESS", "FAILURE")


class StatusWatcher:
    def __init__(self):
        self.latest = None
        self.lock = threading.Lock()
        self.sub = rospy.Subscriber("/navigation/status", String, self._cb, queue_size=10)

    def _cb(self, msg):
        with self.lock:
            self.latest = msg.data

    def wait_terminal(self, timeout=1800.0):
        saw_active = False
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self.lock:
                s = self.latest
            if s in ACTIVE:
                saw_active = True
            elif s in TERMINAL and saw_active:
                return s
            rate.sleep()
        return "TIMEOUT"


def run_navigate(target, approach, timeout=1800.0):
    from g1_navigation.srv import NavigateToObject

    watcher = StatusWatcher()
    rospy.wait_for_service("/navigation/navigate", timeout=10.0)
    call = rospy.ServiceProxy("/navigation/navigate", NavigateToObject)
    resp = call(class_name=target, approach_distance=float(approach))
    if not resp.success:
        rospy.logerr("navigate('%s') rejected: %s", target, resp.message)
        return "FAILURE"
    return watcher.wait_terminal(timeout)
