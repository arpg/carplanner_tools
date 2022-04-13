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
    tf
    geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

add_executable( ros_localizer_tester
  src/ros_localizer_tester.cpp
)
target_link_libraries( ros_localizer_tester
  ${catkin_LIBRARIES}
)
