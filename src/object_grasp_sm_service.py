#!/usr/bin/env python

import random
import threading
import rospy
import tf
import smach
import smach_ros
from smach import State,StateMachine
from std_srvs.srv import Trigger, TriggerResponse

import mbot_states.perception_states as percep_states
import mbot_states.hri_states as hri_states
import mbot_states.kinova_manipulation_states as manip_states

# mbot robot class
from mbot_robot_class_ros import mbot as mbot_class
mbot = mbot_class.mbotRobot

class ObjectGraspSM:
    def __init__(self):
        self.listener = tf.TransformListener()

        if not rospy.has_param('/pregrasp_service/target_frame'):
            rospy.signal_shutdown('"/pregrasp_service/target_frame" param not found')
        self.target_frame = rospy.get_param('/pregrasp_service/target_frame')

        self.running_sm = False

        self.smach_service = rospy.Service('/object_grasp_sm',
                                        Trigger,
                                        self.run_sm)

    def run_sm(self, request):
        if self.running_sm:
            response = TriggerResponse()
            response.success = False
            response.message = "Error: Object grasp state machine is already running"
            return response

        self.running_sm = True
        sm = smach.StateMachine(outcomes=['OVERALL_SUCCESS', 'OVERALL_FAILURE'])

        with sm:

            # For now, target object must be visible by the centered head.
            # TODO if target object not present, move head left - try to add collision objects, move head right - try again

            # Move head to center
            sm.add('HEAD_FRONT', hri_states.MoveHeadNew(90, 15, True),
                transitions={'success': 'TILT_CAMERA_DOWN',
                                'failure': 'TILT_CAMERA_DOWN'})

            # Tilt camera down to see table and object
            sm.add('TILT_CAMERA_DOWN', percep_states.TiltHeadCam(-30),
                transitions={'success': 'CLEAR_OCTOMAP',
                                'failure': 'CLEAR_OCTOMAP'})
            # Clear octomap voxels
            sm.add('CLEAR_OCTOMAP', manip_states.ClearOctomap(),
                    transitions={'success': 'ADD_COLLISION_OBJECTS'})

            # Add MoveIt collision objects
            sm.add('ADD_COLLISION_OBJECTS', manip_states.AddCollisionObjectsState(),
                    transitions={'success': 'HEAD_LEFT',
                                'failure': 'OVERALL_FAILURE'})

            # Move head side to side to build octomap
            sm.add('HEAD_LEFT', hri_states.MoveHeadNew(40, 40, True),
                transitions={'success': 'HEAD_RIGHT',
                                'failure': 'HEAD_RIGHT'})

            sm.add('HEAD_RIGHT', hri_states.MoveHeadNew(140, 15, True),
                transitions={'success': 'HEAD_OBJECT',
                                'failure': 'HEAD_OBJECT'})   

            sm.add('HEAD_OBJECT', manip_states.MoveHeadObject(self.target_frame),
                transitions={'success': 'PREGRASP',
                             'failure': 'OVERALL_FAILURE'})

            sm.add('PREGRASP', manip_states.Pregrasp(),
                transitions={'success': 'VISUAL_SERVO',
                                'failure': 'OVERALL_FAILURE'})

            sm.add('VISUAL_SERVO', manip_states.VisualServo(),
                transitions={'success': 'CLOSE_GRIPPER',
                                'failure': 'OVERALL_FAILURE'})

            sm.add('CLOSE_GRIPPER', manip_states.CloseGripper(),
                    transitions={'success': 'LIFT_ARM'})

            sm.add('LIFT_ARM', manip_states.MoveEefRelative(z=0.05),
                    transitions={'success': 'MOVE_ARM_TO_GOAL',
                                'failure': 'OVERALL_FAILURE'})

            # Randomizing goal position: either 8cm to the left or to the right, randomly
            y_delta = 0.03 if random.random() < 0.5 else -0.03
            sm.add('MOVE_ARM_TO_GOAL', manip_states.MoveEefRelative(y=y_delta),
                    transitions={'success': 'LOWER_ARM',
                                'failure': 'OVERALL_FAILURE'})

            sm.add('LOWER_ARM', manip_states.MoveEefRelative(x=0, y=0, z=-0.05),
                    transitions={'success': 'OPEN_GRIPPER',
                                'failure': 'OVERALL_FAILURE'})


            sm.add('OPEN_GRIPPER', manip_states.OpenGripper(),
                    transitions={'success': 'REST_ARM'})

            sm.add('REST_ARM', manip_states.SetArmResting(),
                    transitions={'success': 'OVERALL_SUCCESS',
                                'failure': 'OVERALL_FAILURE'})

        # Smach viewer
        sis = smach_ros.IntrospectionServer('object_grasp_sm_viewer', sm, '/OBJECT_GRASP_SM')
        sis.start()

        # Start smach
        outcome = sm.execute()
        self.running_sm = False

        response = TriggerResponse()
        response.success = (outcome == 'OVERALL_SUCCESS')
        return response

if __name__ == '__main__':
    rospy.init_node('object_grasp_sm', anonymous=False)
    mbot(enabled_components=['perception', 'hri', 'kinova_manipulation'])
    object_grasp_sm = ObjectGraspSM()
    rospy.spin()