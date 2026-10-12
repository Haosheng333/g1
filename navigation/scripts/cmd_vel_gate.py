#!/usr/bin/env python3
"""cmd_vel_gate - /navigation/cmd_vel --(when motion enabled)--> /locomotion/cmd_vel  (doc 4.3).

* Disabled (default): nothing is forwarded; one zero Twist is sent on the
  enabled -> disabled transition so the walking controller stops.
* Enabled: the newest command is forwarded at ~20 Hz, clamped to safe limits.
  If no command arrived for `timeout` seconds a zero Twist is sent (watchdog).
Enable / disable:  rosservice call /navigation/enable_motion "data: true"
"""
import threading

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse


def _clamp(v, lim):
    return max(-lim, min(lim, v))


class CmdVelGate:
    def __init__(self):
        rospy.init_node("cmd_vel_gate")
        p = rospy.get_param
        self.enabled = bool(p("~enabled", False))
        self.timeout = float(p("~timeout", 0.5))
        self.rate_hz = float(p("~rate", 20.0))
        self.max_vx = float(p("~max_vx", 0.5))
        self.max_vy = float(p("~max_vy", 0.3))
        self.max_wz = float(p("~max_wz", 1.0))
        self.lock = threading.Lock()
        self.last, self.last_t = None, 0.0
        self.was_enabled = False

        self.pub = rospy.Publisher(p("~out_topic", "/locomotion/cmd_vel"), Twist, queue_size=10)
        self.state_pub = rospy.Publisher("/navigation/motion_enabled", Bool, queue_size=1, latch=True)
        rospy.Subscriber(p("~in_topic", "/navigation/cmd_vel"), Twist, self.cmd_cb, queue_size=10)
        rospy.Service("/navigation/enable_motion", SetBool, self.enable_cb)
        self.state_pub.publish(Bool(data=self.enabled))
        rospy.loginfo("[cmd_vel_gate] motion %s", "ENABLED" if self.enabled else "disabled")

    def cmd_cb(self, msg):
        with self.lock:
            self.last, self.last_t = msg, rospy.Time.now().to_sec()

    def enable_cb(self, req):
        with self.lock:
            self.enabled = bool(req.data)
        self.state_pub.publish(Bool(data=self.enabled))
        rospy.logwarn("[cmd_vel_gate] motion %s", "ENABLED" if self.enabled else "disabled")
        return SetBoolResponse(success=True, message="motion enabled" if self.enabled else "motion disabled")

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            with self.lock:
                enabled, last, last_t = self.enabled, self.last, self.last_t
            if enabled:
                out = Twist()
                if last is not None and rospy.Time.now().to_sec() - last_t <= self.timeout:
                    out.linear.x = _clamp(last.linear.x, self.max_vx)
                    out.linear.y = _clamp(last.linear.y, self.max_vy)
                    out.angular.z = _clamp(last.angular.z, self.max_wz)
                self.pub.publish(out)
            elif self.was_enabled:
                self.pub.publish(Twist())
            self.was_enabled = enabled
            rate.sleep()
        if self.was_enabled:
            self.pub.publish(Twist())


if __name__ == "__main__":
    try:
        CmdVelGate().run()
    except rospy.ROSInterruptException:
        pass
