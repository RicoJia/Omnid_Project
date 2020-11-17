cmake_minimum_required(VERSION 3.0.2)
project(omnid_move_group_interface)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
        moveit_ros_planning
        moveit_core
        moveit_ros_planning_interface
        moveit_visual_tools
        visualization_msgs
        tf2_armadillo
        )

find_package (Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS roscpp
  DEPENDS system_lib Eigen3
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/omnid_group_ik.cpp
 )
 target_link_libraries(
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
 )

 add_executable(${PROJECT_NAME}_node src/omnid_move_group_interface_node.cpp)

 target_link_libraries(${PROJECT_NAME}_node
         ${catkin_LIBRARIES}
         ${Boost_LIBRARIES}
 )

 install(TARGETS ${PROJECT_NAME}_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
 )

# Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
 )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_omnid_move_group_interface.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
