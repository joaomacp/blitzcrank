#!/usr/bin/env python

import os
import math
import rospy
import smach
import tf

from std_srvs.srv import Trigger

mbot = mbot_class.mbotRobot

class MoveHeadObject(smach.State):
    """
    Move MBot head angle to point to object (a little to the right of it, so that wrist markers are visible)
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        listener = tf.TransformListener()
        try:
            (trans, rot) = listener.lookupTransform('target_marker', 'base_link', rospy.Time(0))
            heading_rad = math.atan2(trans.y, trans.x)
            mbot().hri.rotate_head_value(180 - math.degrees(heading_rad) + 20, 20, True):
         except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
            rospy.logerr("Failed to lookup target_marker->base_link tf: %s" % e)

        return 'success'

class CloseGripper(smach.State):
    """
    Close Kinova gripper
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        # TODO implement using kinova driver, or actions... This is a hacky way to do it
        os.system('rosrun kinova_demo fingers_action_client.py j2s6s300 kinova percent 100 100 100')
        rospy.sleep(4)
        return 'success'

class OpenGripper(smach.State):
    """
    Open Kinova gripper
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        # TODO implement using kinova driver, or actions... This is a hacky way to do it
        os.system('rosrun kinova_demo fingers_action_client.py j2s6s300 kinova percent 0 0 0')
        rospy.sleep(4)
        return 'success'

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
            rospy.logerr("Pregrasp service failed: %s" % e)
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
            rospy.logerr("Visual servoing service failed: %s" % e)
            return 'failure'
        if result.success:
            return 'success'
        else:
            return 'failure'