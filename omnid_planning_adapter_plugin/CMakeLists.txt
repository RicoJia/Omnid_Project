cmake_minimum_required(VERSION 3.0.2)
project(omnid_planning_adapter_plugin)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  pluginlib
  roscpp
)
find_package(omnid_core)
find_package(cmakeme)
cmakeme_defaults(Debug)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES omnid_planning_adapter_plugin
  CATKIN_DEPENDS
        moveit_core
        moveit_ros_planning
        pluginlib
        roscpp
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
        include
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/omnid_planning_adapter_plugin.cpp
 )

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        ${catkin_LIBRARIES}
        omnid_core::omnid_core
       )


# cmakeme_install(${PROJECT_NAME} NAMESPACE ${PROJECT_NAME}  DEPENDS omnid_core)

#############
## Install ##
#############

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
