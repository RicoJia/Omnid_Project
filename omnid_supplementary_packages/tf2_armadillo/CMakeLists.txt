cmake_minimum_required(VERSION 3.0.2)
project(tf2_armadillo)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2 geometry_msgs
)
find_package(Armadillo REQUIRED)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
        ${ARMADILLO_INCLUDE_DIRS}
)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp tf2 geometry_msgs
        DEPENDS systemlib
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#        FILES_MATCHING PATTERN "*.h"
        )

# Add gtest based cpp test target and link libraries
 if(CATKIN_ENABLE_TESTING)
     catkin_add_gtest(${PROJECT_NAME}-test test/test_tf2_armadillo.cpp)
     target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} ${GTEST_LIBRARIES} ${ARMADILLO_LIBRARIES})
 endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
