#!/usr/bin/env python

import os
import math
import rospy
import smach
import tf

from std_srvs.srv import Trigger

# mbot robot class
from mbot_robot_class_ros import mbot as mbot_class
mbot = mbot_class.mbotRobot

class MoveHeadObject(smach.State):
    """
    Move MBot head angle to point to object (a little to the right of it, so that wrist markers are visible)
    """

    def __init__(self, tf_listener, target_frame):
        smach.State.__init__(self, outcomes=['success'])
        self.tf_listener = tf_listener
        self.target_frame = target_frame

    def execute(self, userdata):
        rospy.sleep(2)
        try:
            (trans, _) = self.tf_listener.lookupTransform('base_link', self.target_frame, rospy.Time(0))
            rospy.loginfo('trans: %f %f' % (trans[0], trans[1]))
            heading_rad = (math.pi/2) - math.atan2(trans[1], trans[0])
            rospy.loginfo('heading rad: %f' % heading_rad)
            rospy.loginfo('rotating head: %f' % (math.degrees(heading_rad) + 15))
            mbot().hri.rotate_head_value(math.degrees(heading_rad) + 15, 15, True)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
            rospy.logerr("Failed to lookup %s->base_link tf: %s" % (self.target_frame, e))

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
