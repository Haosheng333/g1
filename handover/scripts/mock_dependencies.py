#!/usr/bin/env python3
import time
import rospy
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger, TriggerResponse
from arm_control.srv import MoveArmJoints, MoveArmJointsResponse
from handover.srv import PlanHandover, PlanHandoverResponse

def plan(req):
    a=[0.0]*5; b=[0.1]*5
    return PlanHandoverResponse(True,'MOCK ONLY',a,a+b,b+a,a)

if __name__ == '__main__':
    rospy.init_node('handover_mock_dependencies')
    rospy.Service('/arm/plan_handover', PlanHandover, plan)
    rospy.Service('/arm/move_arm_joints', MoveArmJoints, lambda r: MoveArmJointsResponse(True,'MOCK: no movement'))
    for side in ('left','right'):
        rospy.Service('/hand_'+side+'/release', Trigger, lambda r: TriggerResponse(True,'MOCK: fingers opened'))
    pub=rospy.Publisher('/hand_right/tactile', Float64MultiArray, queue_size=1)
    rate=rospy.Rate(30)
    while not rospy.is_shutdown():
        value=120.0 if rospy.get_param('~pull',False) else 100.0
        pub.publish(Float64MultiArray(data=[value]*10))
        rate.sleep()
