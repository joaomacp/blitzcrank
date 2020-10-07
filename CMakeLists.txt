cmake_minimum_required(VERSION 2.8.3)
project(blitzcrank)
add_definitions(-std=c++11)

find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    rospy
    std_msgs
    std_srvs
    geometry_msgs
    darknet_ros_py
    kinova_msgs
    mbot_perception_msgs
    moveit_msgs
    message_generation
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_commander
    pluginlib
    cmake_modules
    geometric_shapes
    tf2
    tf2_ros
    smach
)

catkin_python_setup()

add_service_files(
  FILES
  AddCollisionObjects.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    message_generation
    tf2
    tf2_ros
    std_msgs
    std_srvs
    geometry_msgs
    kinova_msgs
    mbot_perception_msgs
    moveit_msgs
    moveit_core
    moveit_ros_planning_interface
    moveit_ros_perception
    moveit_commander
    interactive_markers
    smach
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

add_executable(pregrasp_service src/pregrasp_service.cpp)
add_executable(joint_control src/joint_control.cpp)
add_executable(transform_republish src/transform_republish.cpp)
add_executable(object_localizer_republish src/object_localizer_republish.cpp)
add_executable(visual_servoing src/visual_servoing.cpp)

target_link_libraries(pregrasp_service ${catkin_LIBRARIES})
add_dependencies(pregrasp_service ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(joint_control ${catkin_LIBRARIES})
target_link_libraries(transform_republish ${catkin_LIBRARIES})
target_link_libraries(object_localizer_republish ${catkin_LIBRARIES})
target_link_libraries(visual_servoing ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS pregrasp_service DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS joint_control DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS transform_republish DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS object_localizer_republish DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS visual_servoing DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
