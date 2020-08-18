#!/usr/bin/env python

import rospy
import smach

from std_srvs.srv import Trigger

class Pregrasp(smach.State):
    """
    Move to pregrasp pose, close to the target object, through kinematic planning
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])

    def execute(self, userdata):
        try:
            rospy.wait_for_service('/kinova_manipulation/pregrasp')
            pregrasp = rospy.ServiceProxy('/kinova_manipulation/pregrasp', Trigger)
            result = pregrasp()
        except rospy.ServiceException, e:
            print("Pregrasp service failed: %s" % e)
            return 'failure'
        if result.success:
            return 'success'
        else:
            return 'failure'

class VisualServo(smach.State):
    """
    Approach the target object through visual servoing
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])

    def execute(self, userdata):
        try:
            rospy.wait_for_service('/kinova_manipulation/visual_servo')
            vs = rospy.ServiceProxy('/kinova_manipulation/visual_servo', Trigger)
            result = vs()
        except rospy.ServiceException, e:
            print("Visual servoing service failed: %s" % e)
            return 'failure'
        if result.success:
            return 'success'
        else:
            return 'failure'