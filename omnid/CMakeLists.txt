cmake_minimum_required(VERSION 2.8.3)
project(omnid)

 add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
        dynamic_reconfigure
)
generate_dynamic_reconfigure_options(
        cfg/omnid.cfg
)
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES omnid
  CATKIN_DEPENDS geometry_msgs rospy
  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
 install(PROGRAMS
   scripts/omnid_simulator
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
 install(FILES
   launch/omnid.launch
   # myfile2
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )

