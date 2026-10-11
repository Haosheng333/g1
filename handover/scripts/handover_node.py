#!/usr/bin/env python3
import math
import threading
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from arm_control.srv import MoveArmJoints
from handover.srv import ArmMonitor, PlanHandover, SetHeldObject, SetHeldObjectResponse

class Handover:
    def __init__(self):
        self.lock = threading.Lock()
        self.arm = self.label = None
        self.state = 'EMPTY'
        self.pub = rospy.Publisher('/handover/status', String, queue_size=1, latch=True)
        self.monitor_sub = rospy.Subscriber('/auto_release_monitor/status', String, self.monitor_status)
        rospy.Service('/handover/set_held_object', SetHeldObject, self.set_held)
        rospy.Service('/orchestrator/run_handover', Trigger, self.start)
        rospy.Service('/orchestrator/run_auto_release', Trigger, self.release)
        self.set_state('EMPTY')

    def set_state(self, state):
        self.state = state
        self.pub.publish(state)

    def monitor_status(self, msg):
        if msg.data == 'FAULT' and self.state == 'WAITING':
            self.set_state('FAULT')
            rospy.logerr('Tactile stream fault. Holding object; operator intervention required.')

    def rpc(self, name, kind, *args):
        rospy.wait_for_service(name, timeout=rospy.get_param('~service_wait', 5.0))
        result = rospy.ServiceProxy(name, kind)(*args)
        if not result.success:
            raise RuntimeError(name+': '+result.message)
        return result

    def set_held(self, req):
        if not self.lock.acquire(False):
            return SetHeldObjectResponse(False, 'Busy')
        try:
            if self.state not in ('EMPTY', 'HOLDING') or req.arm not in ('left','right') or not req.label:
                return SetHeldObjectResponse(False, 'Invalid request or state')
            self.arm, self.label = req.arm, req.label
            self.set_state('HOLDING')
            return SetHeldObjectResponse(True, 'Recorded; caller must verify actual grasp')
        finally:
            self.lock.release()

    def validate(self, path):
        if not path or len(path)%5 or not all(math.isfinite(v) for v in path):
            raise RuntimeError('Invalid planned path')
        return list(path)

    def move(self, path):
        # Retain every planned waypoint; subdivide each segment to <=5 degrees.
        out = list(path[:5])
        for k in range(5, len(path), 5):
            a, b = path[k-5:k], path[k:k+5]
            steps = max(1, math.ceil(max(abs(y-x) for x,y in zip(a,b))/math.radians(5)))
            for j in range(1, steps+1):
                out.extend(x+(y-x)*j/steps for x,y in zip(a,b))
        self.rpc('/arm/move_arm_joints', MoveArmJoints, self.arm, out, 10.0)

    def start(self, req):
        if not self.lock.acquire(False):
            return TriggerResponse(False, 'Busy')
        try:
            if self.state != 'HOLDING' or not self.label:
                return TriggerResponse(False, 'Requires verified held object')
            target = rospy.get_param('~target')  # No unvalidated hardware default.
            if len(target) != 3 or not all(math.isfinite(v) for v in target):
                raise RuntimeError('Invalid target')
            self.set_state('PLANNING')
            plan = self.rpc('/arm/plan_handover', PlanHandover, self.arm, *target)
            self.paths = [self.validate(getattr(plan, key)) for key in
                          ('staging_path_flat','reach_path_flat','retract_path_flat','home_path_flat')]
            for a,b in zip(self.paths, self.paths[1:]):
                if max(abs(x-y) for x,y in zip(a[-5:],b[:5])) > 0.001:
                    raise RuntimeError('Planner returned disconnected path legs')
            self.set_state('EXTENDING')
            self.move(self.paths[0]); self.move(self.paths[1])
            rospy.sleep(1.0)
            self.set_state('WAITING')
            self.rpc('/auto_release_monitor/arm', ArmMonitor, self.arm, self.label)
            return TriggerResponse(True, 'Waiting for recipient; completion is /handover/status DONE')
        except Exception as exc:
            self.set_state('FAULT')
            return TriggerResponse(False, str(exc))
        finally:
            self.lock.release()

    def release(self, req):
        if not self.lock.acquire(False):
            return TriggerResponse(False, 'Busy')
        try:
            if self.state != 'WAITING':
                return TriggerResponse(False, 'Not waiting for recipient')
            self.set_state('RELEASING')
            self.rpc('/auto_release_monitor/disarm', Trigger)
            self.rpc('/hand_'+self.arm+'/release', Trigger)
            self.label = None
            rospy.sleep(1.5)
            self.set_state('RETRACTING')
            self.move(self.paths[2]); self.move(self.paths[3])
            self.arm = None
            self.set_state('DONE')
            # Allow subsequent verified grasps without losing DONE notification.
            self.state = 'EMPTY'
            return TriggerResponse(True, 'Released and stowed')
        except Exception as exc:
            self.set_state('FAULT')
            return TriggerResponse(False, str(exc))
        finally:
            self.lock.release()

if __name__ == '__main__':
    rospy.init_node('handover_node')
    Handover()
    rospy.spin()
