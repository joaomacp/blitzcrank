#!/usr/bin/env python

import rospy
import smach
import smach_ros
from smach import State,StateMachine

import vs_object_grasp_states as vs_states

def vs_object_grasp_sm():
    sm = smach.StateMachine(outcomes=['OVERALL_SUCCESS', 'OVERALL_FAILURE'])

    with sm:

        sm.add('PREGRASP', vs_states.Pregrasp(),
               transitions={'success': 'VISUAL_SERVO',
                            'failure': 'OVERALL_FAILURE'})

        sm.add('VISUAL_SERVO', vs_states.VisualServo(),
               transitions={'success': 'OVERALL_SUCCESS',
                            'failure': 'OVERALL_FAILURE'})

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.loginfo('Final outcome: %s' % outcome)

def main():
    rospy.init_node('vs_object_grasp_sm', anonymous=False)
    rospy.sleep(0.5)
    vs_object_grasp_sm()