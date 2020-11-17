/// \file Robot group level IK for calculating the pose of each robot and the end effector. The main setup is a group of omnid robots trying to carry an object
/// We assume the object frame is placed at the centroid of the omnid robots bases.

#ifndef OMNID_PROJECT_OMNID_GROUP_IK_HPP
#define OMNID_PROJECT_OMNID_GROUP_IK_HPP

#include <Eigen/Dense>

/// \brief High Level IK that update each robot's end effector pose in group_reference_frame for a given object platfor pose in the world frame
/// \return true if a valid pose is found for the robot. False if not.

bool getRobotPose(){}

#endif //OMNID_PROJECT_OMNID_GROUP_IK_HPP
