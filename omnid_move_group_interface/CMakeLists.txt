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
find_package(Armadillo REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib Eigen3
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
        ${BLAS_INCLUDE_DIRS}
        ${LAPACK_INCLUDE_DIRS}
)

## Declare a C++ library
## Armadillo Runtime Library
#set(ARMA_LIBS ${ARMA_LIBS} ${MKL_LIBRARIES})
set(ARMA_LIBS ${ARMA_LIBS} ${BLAS_LIBRARIES})
set(ARMA_LIBS ${ARMA_LIBS} ${LAPACK_LIBRARIES})

 if(ARMA_USE_ATLAS STREQUAL true)
  set(ARMA_LIBS ${ARMA_LIBS} ${CBLAS_LIBRARIES})
  set(ARMA_LIBS ${ARMA_LIBS} ${CLAPACK_LIBRARIES})
 endif()

 add_executable(${PROJECT_NAME}_node src/omnid_move_group_interface_node.cpp)

 target_link_libraries(${PROJECT_NAME}_node
         ${catkin_LIBRARIES}
         ${Boost_LIBRARIES}
         ${ARMA_LIBS}
 )

 install(TARGETS ${PROJECT_NAME}_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp"
 )
