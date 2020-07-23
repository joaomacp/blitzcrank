#! /usr/bin/env python
import rospy
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene

rospy.init_node("allow_object_collision", anonymous=True)

# This adds the 'target_object' cylinder (or other shape) to the AllowedCollisionMatrix, so that the arm can collide with it
# Made in python because it's more convenient, add to "add_collision_object.cpp" for better cleanliness
# Not necessary if we implement the 'pick' call -> cleanest option.

pubPlanningScene = rospy.Publisher('/move_group/monitored_planning_scene', PlanningScene)
rospy.wait_for_service('/get_planning_scene', 10.0)
get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
request = PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
response = get_planning_scene(request)

acm = response.scene.allowed_collision_matrix
if not 'target_object' in acm.default_entry_names:
    # add to allowed collision matrix
    acm.default_entry_names += ['target_object']
    acm.default_entry_values += [True]

    planning_scene_diff = PlanningScene(
            is_diff=True,
            allowed_collision_matrix=acm)

    pubPlanningScene.publish(planning_scene_diff)
    rospy.sleep(1.0)
