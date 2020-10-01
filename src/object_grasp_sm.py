#!/usr/bin/env python

import random
import threading
import rospy
import tf
import smach
import smach_ros
from smach import State,StateMachine

import mbot_states.perception_states as percep_states
import mbot_states.hri_states as hri_states
import object_grasp_states as grasping_states

# mbot robot class
from mbot_robot_class_ros import mbot as mbot_class
mbot = mbot_class.mbotRobot

def object_grasp_sm():
    global listener
    global target_frame

    sm = smach.StateMachine(outcomes=['OVERALL_SUCCESS', 'OVERALL_FAILURE'])

    with sm:

        # Head angle should be centered before starting this SMach.
        # Alternative (TODO): - add a "clear octomap" state - implement using PlanningSceneMonitor
        #                     - turn head center - clear octomap - continue
        # Also: for now, target object must be visible by the centered head.
        # TODO if target object not present, move head left - try to add collision objects, move head right - try again

        # Clear octomap voxels
        sm.add('CLEAR_OCTOMAP', grasping_states.ClearOctomap(),
                transitions={'success': 'TILT_CAMERA_DOWN'})

        # Tilt camera down to see table and object
        sm.add('TILT_CAMERA_DOWN', percep_states.TiltHeadCam(-30),
               transitions={'success': 'ADD_COLLISION_OBJECTS',
                            'failure': 'ADD_COLLISION_OBJECTS'})

        # Add MoveIt collision objects
        sm.add('ADD_COLLISION_OBJECTS', grasping_states.AddCollisionObjects(),
                transitions={'success': 'HEAD_LEFT',
                            'failure': 'HEAD_LEFT'})

        # Move head side to side to build octomap
        sm.add('HEAD_LEFT', hri_states.MoveHeadNew(40, 40, True),
               transitions={'success': 'HEAD_RIGHT',
                            'failure': 'HEAD_RIGHT'})

        sm.add('HEAD_RIGHT', hri_states.MoveHeadNew(140, 15, True),
               transitions={'success': 'HEAD_FRONT',
                            'failure': 'HEAD_FRONT'})      

        sm.add('HEAD_FRONT', hri_states.MoveHeadNew(90, 15, True),
               transitions={'success': 'HEAD_OBJECT',
                            'failure': 'HEAD_OBJECT'})

        sm.add('HEAD_OBJECT', grasping_states.MoveHeadObject(listener, target_frame),
               transitions={'success': 'PREGRASP'})

        sm.add('PREGRASP', grasping_states.Pregrasp(),
               transitions={'success': 'VISUAL_SERVO',
                            'failure': 'OVERALL_FAILURE'})

        sm.add('VISUAL_SERVO', grasping_states.VisualServo(),
               transitions={'success': 'CLOSE_GRIPPER',
                            'failure': 'OVERALL_FAILURE'})

        sm.add('CLOSE_GRIPPER', grasping_states.CloseGripper(),
                transitions={'success': 'LIFT_ARM'})

        sm.add('LIFT_ARM', grasping_states.MoveEefRelative(z=0.05),
                transitions={'success': 'MOVE_ARM_TO_GOAL',
                             'failure': 'OVERALL_FAILURE'})

        # Randomizing goal position: either 8cm to the left or to the right, randomly
        y_delta = 0.03 if random.random() < 0.5 else -0.03
        sm.add('MOVE_ARM_TO_GOAL', grasping_states.MoveEefRelative(y=y_delta),
                transitions={'success': 'LOWER_ARM',
                             'failure': 'OVERALL_FAILURE'})

        sm.add('LOWER_ARM', grasping_states.MoveEefRelative(x=0, y=0, z=-0.05),
                transitions={'success': 'OPEN_GRIPPER',
                             'failure': 'OVERALL_FAILURE'})


        sm.add('OPEN_GRIPPER', grasping_states.OpenGripper(),
                transitions={'success': 'REST_ARM'})

        sm.add('REST_ARM', grasping_states.RestArm(),
                transitions={'success': 'OVERALL_SUCCESS',
                             'failure': 'OVERALL_FAILURE'})

    # Smach viewer
    sis = smach_ros.IntrospectionServer('object_grasp_sm_viewer', sm, '/OBJECT_GRASP_SM')
    sis.start()

    # Create a thread to execute the smach container
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    # Wait for ctrl-c
    rospy.spin()

    rospy.logwarn('ctrl + c detected!!! preempting smach execution')

    # Request the container to preempt
    sm.request_preempt()

    # Block until everything is preempted
    # (you could do something more complicated to get the execution outcome if you want it)
    # TODO: if this smach is as part of another smach (e.g. for a competition task)
    #       then we want the execution outcome (e.g. if grasping fails, move robot and try again)
    smach_thread.join()

if __name__ == '__main__':
    rospy.init_node('object_grasp_sm', anonymous=False)

    if not rospy.has_param('target_frame'):
        rospy.signal_shutdown('"target_frame" param not found')
    target_frame = rospy.get_param('target_frame')

    listener = tf.TransformListener()
    mbot(enabled_components=['perception', 'hri'])
    rospy.sleep(1.0)
    object_grasp_sm()
    