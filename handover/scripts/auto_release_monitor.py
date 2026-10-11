#!/usr/bin/env python3
import threading
import time
import rospy
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger, TriggerResponse
from handover.srv import ArmMonitor, ArmMonitorResponse
from handover.detector import Detector

class Monitor:
    def __init__(self):
        self.lock = threading.RLock()
        self.side = None
        self.detector = Detector(**rospy.get_param('~detector', {}))
        self.status = rospy.Publisher('/auto_release_monitor/status', String, queue_size=1, latch=True)
        self.subs = [rospy.Subscriber('/hand_'+s+'/tactile', Float64MultiArray, self.sample, callback_args=s, queue_size=1) for s in ('left','right')]
        rospy.Service('/auto_release_monitor/arm', ArmMonitor, self.arm)
        rospy.Service('/auto_release_monitor/disarm', Trigger, self.disarm)
        rospy.Timer(rospy.Duration(0.05), self.tick)

    def arm(self, req):
        with self.lock:
            if req.arm not in ('left','right'):
                return ArmMonitorResponse(False, 'Invalid arm')
            self.side = req.arm
            self.detector.arm(time.monotonic())
        return ArmMonitorResponse(True, 'BASELINING')

    def disarm(self, req):
        with self.lock:
            self.detector.disarm()
            self.side = None
        return TriggerResponse(True, 'IDLE')

    def tick(self, event):
        with self.lock:
            state = self.detector.check_stale(time.monotonic())
        self.status.publish(state)

    def sample(self, msg, side):
        with self.lock:
            trigger = side == self.side and self.detector.feed(msg.data, time.monotonic())
        if trigger:
            threading.Thread(target=self.notify, daemon=True).start()

    def notify(self):
        try:
            rospy.wait_for_service('/orchestrator/run_auto_release', timeout=2)
            result = rospy.ServiceProxy('/orchestrator/run_auto_release', Trigger)()
            if not result.success:
                rospy.logerr('Release rejected: %s', result.message)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr('Release notification failed; no automatic retry: %s', exc)

if __name__ == '__main__':
    rospy.init_node('auto_release_monitor')
    Monitor()
    rospy.spin()
