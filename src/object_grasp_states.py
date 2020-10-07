#!/usr/bin/env python

import os
import math
import rospy
import smach
import tf

from std_srvs.srv import Trigger, Empty
from blitzcrank.srv import AddCollisionObjects

# mbot robot class
from mbot_robot_class_ros import mbot as mbot_class
mbot = mbot_class.mbotRobot

# Kinova manipulation
import moveit_commander
__commander = None
__arm = None

class MoveHeadObject(smach.State):
    """
    Move MBot head angle to point to object (a little to the right of it, so that wrist markers are visible)
    """

    def __init__(self, tf_listener, target_frame):
        smach.State.__init__(self, outcomes=['success'])
        self.tf_listener = tf_listener
        self.target_frame = target_frame

    def execute(self, userdata):
        try:
            (trans, _) = self.tf_listener.lookupTransform('base_link', self.target_frame, rospy.Time(0))
            heading_rad = (math.pi/2) - math.atan2(trans[1], trans[0])
            mbot().hri.rotate_head_value(math.degrees(heading_rad) + 15, 15, True)
            rospy.sleep(1) # Let camera image stabilize, so that object pose is correct
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
            rospy.logerr("Failed to lookup %s->base_link tf: %s" % (self.target_frame, e))

        return 'success'

class ClearOctomap(smach.State):
    """
    Clear the planning scene's octomap
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.wait_for_service('/clear_octomap')
        clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
        clear_octomap()
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
        return 'success'

class AddCollisionObjectsState(smach.State):
    """
    Add ground and side planes, add target object cylinder
    """

    def __init__(self, add_target_cylinder=True):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.add_target_cylinder = add_target_cylinder

    def execute(self, userdata):
        try:
            rospy.wait_for_service('/kinova_manipulation/add_collision_objects')
            add_objects = rospy.ServiceProxy('/kinova_manipulation/add_collision_objects', AddCollisionObjects)
            result = add_objects(self.add_target_cylinder)
        except rospy.ServiceException, e:
            rospy.logerr("add_collision_objects service failed: %s" % e)
            return 'failure'
        if result.success:
            return 'success'
        else:
            return 'failure'

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

# TODO - this belongs in manipulation_states
class MoveEefRelative(smach.State):
    """
    Move the Kinova end-effector, relative to where it currently is (in base_link frame:
    x is forward-backward, y is right-left, z is up-down)
    """

    def __init__(self, x=None, y=None, z=None):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.x = x
        self.y = y
        self.z = z

    def execute(self, userdata):
        if move_eef_relative(self.x, self.y, self.z, wait=True):
            return 'success'
        else:
            return 'failure'

# TODO - this belongs in manipulation_states
class SetArmResting(smach.State):
    """
    Place the kinova arm in its resting pose
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])

    def execute(self, userdata):
        if go_to_pose('mbot_resting', wait=True):
            return 'success'
        else:
            return 'failure'

# TODO - this belongs in manipulation_states
class SetArmWalking(smach.State):
    """
    Place the kinova arm in its walking pose
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])

    def execute(self, userdata):
        if go_to_pose('mbot_walking', wait=True):
            return 'success'
        else:
            return 'failure'

# Kinova manipulation - auxiliary functions - TODO place in new mbot_class component: kinova_manipulation
def initialize_moveit_commander():
    global __commander
    global __arm

    if __commander is None:
        # Initialize moveit commander
        try:
            from sys import argv
            moveit_commander.roscpp_initialize(argv)
            __commander = moveit_commander.RobotCommander()
        except RuntimeError as e:
            rospy.logerr('robot_description not found. Did you bringup?')
            return False
        except Exception as e:
            rospy.logerr('could not initialize moveit_commander, unknown error: {}'.format(e))
            return False
        
        # Initialize MoveIt group 'arm'
        try:
            group = 'arm'
            __arm = __commander.get_group(group)
        except moveit_commander.MoveItCommanderException as e:
            rospy.logerr("group '{}' does not exist".format(group))
            return False
        except RuntimeError as e:
            rospy.logwarn("moveit not available")
            return False

    return True

def go_to_pose(pose, wait=False):
    global __arm

    if not initialize_moveit_commander():
        return False

    try:
        __arm.set_named_target(pose)
        __arm.go(wait=wait)
        return True
    except moveit_commander.MoveItCommanderException as e:
        # moveit already prints an error
        #rospy.logerr("pose '{}' doe_s not exist".format(pose))
        return False
    except Exception as e:
        rospy.logerr("unknown error {}".format(e))
        return False

def move_eef_relative(x, y, z, wait=False):
    global __arm

    if not initialize_moveit_commander():
        return False
    
    try:
        pose = __arm.get_current_pose()
        rospy.loginfo('current pose: %f %f %f' % (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))

        if x is not None:
            pose.pose.position.x += x
        if y is not None:
            pose.pose.position.y += y
        if z is not None:
            pose.pose.position.z += z

        __arm.set_pose_target(pose)
        __arm.go(wait=wait)
    
        return True
    except moveit_commander.MoveItCommanderException as e:
        # moveit already prints an error
        #rospy.logerr("pose '{}' doe_s not exist".format(pose))
        return False
    except Exception as e:
        rospy.logerr("unknown error {}".format(e))
        return False
