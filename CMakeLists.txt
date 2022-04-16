project( carplanner_tools )
cmake_minimum_required(VERSION 2.8)
enable_testing()

# Add to module path, so we can find our cmake modules
list( APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules )

#find_package( carplanner REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    std_srvs
    tf
    geometry_msgs
    nav_msgs
    message_generation
    visualization_msgs
    dynamic_reconfigure
    actionlib_msgs
    drive_control
    carplanner_msgs
)

add_definitions(${EIGEN_DEFINITIONS})

# generate_dynamic_reconfigure_options(
#   cfg/MochaPlannerConfig.cfg
#   # cfg/MochaControllerConfig.cfg
#   # cfg/MochaConfig.cfg
# )

# add_message_files(
#   DIRECTORY
#   msg
#   FILES
#   ApplyVelocitiesData.msg
#   MotionSample.msg
#   Command.msg
#   #  CommandList.msg
#   VehicleState.msg
#   PathArray.msg
#   OdometryArray.msg
#   #  ControlPlan.msg
#   MotionPlan.msg
#   TriangleMeshStamped.msg
# )

# add_service_files(
#   DIRECTORY
#     srv
#   FILES
#     SetDriveMode.srv
#     SetSimMode.srv
#     ResetMesh.srv
#     Replay.srv
#     EnableTerrainPlanning.srv
#     EnableContinuousPlanning.srv
# )

# add_action_files(
#   DIRECTORY 
#     action
#   FILES 
#   #ApplyAllVelocities.action
#     BatchApplyVelocities.action
#     ApplyVelocities.action
#     SetState.action
#     GetState.action
#     UpdateState.action 
# #    GetGravityCompensation.action 
# #    GetFrictionCompensation.action 
#     # GetControlDelay.action 
#     # GetInertiaTensor.action
#     # SetNoDelay.action
#     Raycast.action
#     MTTest.action
# )

# generate_messages(
#   DEPENDENCIES
#   roscpp
#   std_msgs
#   std_srvs
#   geometry_msgs
#   nav_msgs
#   actionlib_msgs
#   carplanner_msgs
# )

catkin_package(
  INCLUDE_DIRS include
  # CATKIN_DEPENDS message_runtime actionlib_msgs nav_msgs
  #LIBRARIES 
  )

include_directories(
	#${CarPlanner_INCLUDE_DIRS}
	#${carplanner_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  include
	)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

add_executable( command2cmdvel
  src/command2cmdvel.cpp
)
target_link_libraries( command2cmdvel
  ${catkin_LIBRARIES}
)

add_executable( vehiclestate2pose
  src/vehiclestate2pose.cpp
)
target_link_libraries( vehiclestate2pose
  ${catkin_LIBRARIES}
)

add_executable( motionsample2path
  src/motionsample2path.cpp
)
target_link_libraries( motionsample2path
  ${catkin_LIBRARIES}
)

add_executable( patharrayparser
  src/patharrayparser.cpp
)
target_link_libraries( patharrayparser
  ${catkin_LIBRARIES}
)

add_executable( waypoint_generator
 	src/waypoint_generator.cpp
)
target_link_libraries( waypoint_generator
  ${catkin_LIBRARIES}
)

add_executable( goal_generator
  src/goal_generator.cpp
)
target_link_libraries( goal_generator
  ${catkin_LIBRARIES}
)

add_executable( tf2odom
  src/tf2odom.cpp
)
target_link_libraries( tf2odom
  ${catkin_LIBRARIES}
)

add_executable( pose2setstate
  src/pose2setstate.cpp
)
target_link_libraries( pose2setstate
  ${catkin_LIBRARIES}
)

add_executable( pose2odom
  src/pose2odom.cpp
)
target_link_libraries( pose2odom
  ${catkin_LIBRARIES}
)

add_executable( odom_reframer
  src/odom_reframer.cpp
)
target_link_libraries( odom_reframer
  ${catkin_LIBRARIES}
)

add_executable( controlcommand2drivecommand
  src/controlcommand2drivecommand.cpp
)
target_link_libraries( controlcommand2drivecommand
  ${catkin_LIBRARIES}
)
