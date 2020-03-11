cmake_minimum_required(VERSION 2.8.3)
project(blitzcrank)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    cmake_modules
    geometric_shapes
    tf2
    tf2_ros
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
)
find_package(Eigen REQUIRED)

###########
## Build ##
###########

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(vertical_planning_scene src/vertical_planning_scene.cpp)
target_link_libraries(vertical_planning_scene ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS vertical_planning_scene DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(vertical_approach_marker src/vertical_approach_marker.cpp)
target_link_libraries(vertical_approach_marker ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS vertical_approach_marker DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})