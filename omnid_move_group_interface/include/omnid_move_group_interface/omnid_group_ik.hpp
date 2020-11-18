/// \file
/// \brief Robot group level IK for calculating the pose of each robot and the end effector. The main setup is a group of omnid robots trying to carry an object
/// We assume the object frame is placed at the centroid of the omnid robots bases.

#ifndef OMNID_PROJECT_OMNID_GROUP_IK_HPP
#define OMNID_PROJECT_OMNID_GROUP_IK_HPP

#include <armadillo>

typedef arma::Mat<double> mat;

namespace omnid_group_ik{

    /// \brief High Level IK that update each robot's end effector pose in group_reference_frame for a given object platfor pose in a reference frame
    /// \param in_pose - object platform pose in the world frame
    /// \param ref_to_robot - transform from the reference frame to the robot frame.
    /// \param out_pose - robot end effector pose (in the reference frame) to be updated.
    /// \return true if a valid pose is found for the robot. False if not.

    bool getRobotEefPose(const mat::fixed<4,4>& in_pose, const mat::fixed<4,4>& ref_to_world, const mat::fixed<4,4>& ref_to_robot, mat::fixed<4,4>& out_pose ){

        out_pose = (ref_to_world * in_pose);
        out_pose = out_pose * ref_to_robot;;
        //TODO add collision check

        return true;
    }
}

#endif //OMNID_PROJECT_OMNID_GROUP_IK_HPP
