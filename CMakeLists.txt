cmake_minimum_required(VERSION 2.8.3)
project(blitzcrank)
add_definitions(-std=c++11)

find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    message_generation
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    cmake_modules
    geometric_shapes
    tf2
    tf2_ros
)

add_service_files(
  FILES
  SetCameraOffset.srv
  GetTargetAlignment.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_generation
    tf2
    tf2_ros
    std_msgs
    geometry_msgs
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

add_executable(vertical_planning_scene src/vertical_planning_scene.cpp)

add_executable(vertical_approach_target src/vertical_approach_target.cpp)

add_executable(horizontal_approach_target src/horizontal_approach_target.cpp)

add_executable(joint_trajectory_control src/joint_trajectory_control.cpp)
add_executable(visual_servoing src/visual_servoing.cpp)
add_executable(test_tf_broadcaster src/test_tf_broadcaster.cpp)

add_executable(gazebo_interface src/gazebo_interface.cpp)
add_dependencies(gazebo_interface ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(vertical_planning_scene ${catkin_LIBRARIES})

target_link_libraries(vertical_approach_target ${catkin_LIBRARIES})

target_link_libraries(joint_trajectory_control ${catkin_LIBRARIES})
target_link_libraries(visual_servoing ${catkin_LIBRARIES})

target_link_libraries(horizontal_approach_target ${catkin_LIBRARIES})
add_dependencies(horizontal_approach_target ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(gazebo_interface ${catkin_LIBRARIES})
target_link_libraries(test_tf_broadcaster ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS vertical_planning_scene DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS vertical_approach_target DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS horizontal_approach_target DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS joint_trajectory_control DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS visual_servoing DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS gazebo_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS test_tf_broadcaster DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})