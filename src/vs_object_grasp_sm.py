#!/usr/bin/env python

import threading
import rospy
import smach
import smach_ros
from smach import State,StateMachine

import mbot_states.perception_states as percep_states
import mbot_states.hri_states as hri_states
import vs_object_grasp_states as vs_states

# mbot robot class
from mbot_robot_class_ros import mbot as mbot_class
mbot = mbot_class.mbotRobot

def vs_object_grasp_sm():
    sm = smach.StateMachine(outcomes=['OVERALL_SUCCESS', 'OVERALL_FAILURE'])

    with sm:

        # TODO (if needed) add a "clear octomap" state - implement using PlanningSceneMonitor

        # Tilt camera down to see table and object
        sm.add('TILT_CAMERA_DOWN', percep_states.TiltHeadCam(-30),
               transitions={'success': 'HEAD_LEFT',
                            'failure': 'HEAD_LEFT'})

        # Move head side to side to build octomap
        sm.add('HEAD_LEFT', hri_states.MoveHeadNew(40, 40, True),
               transitions={'success': 'HEAD_RIGHT',
                            'failure': 'HEAD_RIGHT'})

        sm.add('HEAD_RIGHT', hri_states.MoveHeadNew(140, 20, True),
               transitions={'success': 'HEAD_FRONT',
                            'failure': 'HEAD_FRONT'})      

        # End up with head looking a bit to the right, so that the wrist marker is more visible
        # TODO Use target object's position to make the head look straight to it
        sm.add('HEAD_FRONT', hri_states.MoveHeadNew(100, 20, True),
               transitions={'success': 'PREGRASP',
                            'failure': 'PREGRASP'})

        sm.add('PREGRASP', vs_states.Pregrasp(),
               transitions={'success': 'VISUAL_SERVO',
                            'failure': 'OVERALL_FAILURE'})

        sm.add('VISUAL_SERVO', vs_states.VisualServo(),
               transitions={'success': 'CLOSE_GRIPPER',
                            'failure': 'OVERALL_FAILURE'})

        sm.add('CLOSE_GRIPPER', vs_states.CloseGripper(),
                transitions={'success': 'OPEN_GRIPPER'})

        sm.add('OPEN_GRIPPER', vs_states.OpenGripper(),
                transitions={'success': 'OVERALL_SUCCESS'})

    # Smach viewer
    sis = smach_ros.IntrospectionServer('vs_object_grasp_sm_viewer', sm, '/VS_OBJECT_GRASP_SM')
    sis.start()

    # Create a thread to execute the smach container
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    # Wait for ctrl-c
    rospy.spin()

    rospy.logwarn("ctrl + c detected!!! preempting smach execution")

    # Request the container to preempt
    sm.request_preempt()

    # Block until everything is preempted
    # (you could do something more complicated to get the execution outcome if you want it)
    # TODO: if this smach is as part of another smach (e.g. for a competition task)
    #       then we want the execution outcome (e.g. if grasping fails, move robot and try again)
    smach_thread.join()

if __name__ == '__main__':
    rospy.init_node('vs_object_grasp_sm', anonymous=False)
    mbot(enabled_components=['perception', 'hri'])
    print('before sleep')
    rospy.sleep(1.0)
    vs_object_grasp_sm()
