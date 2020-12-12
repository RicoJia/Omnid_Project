cmake_minimum_required(VERSION 3.0.2)
project(omnid_move_group_interface)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
        moveit_ros_planning
        moveit_core
        moveit_ros_planning_interface
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
 add_library(armadillo INTERFACE IMPORTED)
 set_property(TARGET armadillo PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${ARMADILLO_INCLUDE_DIRS})

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
