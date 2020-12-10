/// \file
/// \brief Robot group level IK for calculating the pose of each robot and the end effector. The main setup is a group of omnid robots trying to carry an object
/// We assume the object frame is placed at the centroid of the omnid robots bases.

#ifndef OMNID_PROJECT_OMNID_GROUP_IK_HPP
#define OMNID_PROJECT_OMNID_GROUP_IK_HPP
#define ARMA_DONT_USE_WRAPPER

#include <armadillo>

typedef arma::Mat<double> mat;

namespace omnid_group_ik{

    /// \brief High Level IK that update each robot's body frame end effector pose for a given object platfor pose in a reference frame
    /// \param in_pose - object platform pose in the world frame
    /// \param robot_to_world - transform from the robot body frame to the world frame.
    /// \param ref_to_robot = transform from the reference frame to robot
    /// \param out_pose - robot end effector pose (in the reference frame) to be updated.
    //TODO: take out the inverse
    void getRobotEefPose(const mat::fixed<4, 4> &in_pose, const mat::fixed<4, 4> &robot_to_world,
                         const mat::fixed<4, 4> &ref_to_world, const mat::fixed<4, 4> &robot_to_world_inv, mat::fixed<4, 4> &out_pose) {
        // T_eff_ref = T_robot_world * T_in * T_ref_world * T_world_robot
        out_pose = (robot_to_world * in_pose);
        out_pose = out_pose * ref_to_world;
//        mat world_to_robot = arma::inv(robot_to_world);
        out_pose = out_pose * robot_to_world_inv;
    }
/*
    void getRobotEefPose(const mat& in_pose, const mat &robot_to_world,
                         const mat& ref_to_world, mat &out_pose) {
        // T_eff_ref = T_robot_world * T_in * T_ref_world * T_world_robot
        out_pose = (robot_to_world * in_pose);
        out_pose = out_pose * ref_to_world;
        mat world_to_robot = arma::inv(robot_to_world);
        out_pose = out_pose * world_to_robot;
    }
*/


}

#endif //OMNID_PROJECT_OMNID_GROUP_IK_HPP
