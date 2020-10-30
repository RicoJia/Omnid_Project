/// file
/// \brief: The kinematics plugin for the Omnid Moveit_config package
/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2020, Northwestern University
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Northwestern University nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Rico Ruotong Jia */

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman */

#ifndef OMNID_PROJECT_OMNID_KINEMATICS_PLUGIN_H
#define OMNID_PROJECT_OMNID_KINEMATICS_PLUGIN_H

// ROS
#include <ros/ros.h>
#include <random_numbers/random_numbers.h>
// System
#include <boost/shared_ptr.hpp>
// ROS msgs
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/KinematicSolverInfo.h>
#include <moveit_msgs/MoveItErrorCodes.h>
// KDL
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <moveit/kdl_kinematics_plugin/joint_mimic.hpp>
// MoveIt!
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace omnid_kinematics{
    class OmnidKinematicsPlugin : public kinematics::KinematicsBase{
    public:
        /// \brief Default constructor
        OmnidKinematicsPlugin();


        /**
         * @brief Given a desired pose of the end-effector, compute the joint angles to reach it
         * @param ik_pose the desired pose of the link
         * @param ik_seed_state an initial guess solution for the inverse kinematics
         * @param solution the solution vector
         * @param error_code an error code that encodes the reason for failure or success
         * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
         * @return True if a valid solution was found, false otherwise
         */
        virtual bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                                   const std::vector<double> &ik_seed_state,
                                   std::vector<double> &solution,
                                   moveit_msgs::MoveItErrorCodes &error_code,
                                   const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

        /**
         * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
         * This particular method is intended for "searching" for a solutions by stepping through the redundancy
         * (or other numerical routines).
         * @param ik_pose the desired pose of the link
         * @param ik_seed_state an initial guess solution for the inverse kinematics
         * @param timeout The amount of time (in seconds) available to the solver
         * @param solution the solution vector
         * @param error_code an error code that encodes the reason for failure or success
         * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
         * @return True if a valid solution was found, false otherwise
         */
        virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                      const std::vector<double> &ik_seed_state,
                                      double timeout,
                                      std::vector<double> &solution,
                                      moveit_msgs::MoveItErrorCodes &error_code,
                                      const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;
        /**
         * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
         * This particular method is intended for "searching" for a solutions by stepping through the redundancy
         * (or other numerical routines).
         * @param ik_pose the desired pose of the link
         * @param ik_seed_state an initial guess solution for the inverse kinematics
         * @param timeout The amount of time (in seconds) available to the solver
         * @param consistency_limits the distance that any joint in the solution can be from the corresponding joints in the current seed state
         * @param solution the solution vector
         * @param error_code an error code that encodes the reason for failure or success
         * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
         * @return True if a valid solution was found, false otherwise
         */
        virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                      const std::vector<double> &ik_seed_state,
                                      double timeout,
                                      const std::vector<double> &consistency_limits,
                                      std::vector<double> &solution,
                                      moveit_msgs::MoveItErrorCodes &error_code,
                                      const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;
        /**
         * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
         * This particular method is intended for "searching" for a solutions by stepping through the redundancy
         * (or other numerical routines).
         * @param ik_pose the desired pose of the link
         * @param ik_seed_state an initial guess solution for the inverse kinematics
         * @param timeout The amount of time (in seconds) available to the solver
         * @param solution the solution vector
         * @param solution_callback A callback solution for the IK solution
         * @param error_code an error code that encodes the reason for failure or success
         * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
         * @return True if a valid solution was found, false otherwise
         */
        virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                      const std::vector<double> &ik_seed_state,
                                      double timeout,
                                      std::vector<double> &solution,
                                      const IKCallbackFn &solution_callback,
                                      moveit_msgs::MoveItErrorCodes &error_code,
                                      const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

        /**
         * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
         * This particular method is intended for "searching" for a solutions by stepping through the redundancy
         * (or other numerical routines).
         * @param ik_pose the desired pose of the link
         * @param ik_seed_state an initial guess solution for the inverse kinematics
         * @param timeout The amount of time (in seconds) available to the solver
         * @param consistency_limits the distance that any joint in the solution can be from the corresponding joints in the current seed state
         * @param solution the solution vector
         * @param solution_callback A callback solution for the IK solution
         * @param error_code an error code that encodes the reason for failure or success
         * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
         * @return True if a valid solution was found, false otherwise
         */
        virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                      const std::vector<double> &ik_seed_state,
                                      double timeout,
                                      const std::vector<double> &consistency_limits,
                                      std::vector<double> &solution,
                                      const IKCallbackFn &solution_callback,
                                      moveit_msgs::MoveItErrorCodes &error_code,
                                      const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;
        /**
         * @brief  Initialization function for the kinematics, for use with kinematic chain IK solvers
         * @param robot_description This parameter can be used as an identifier for the robot kinematics it is computed for;
         * For example, the name of the ROS parameter that contains the robot description;
         * @param group_name The group for which this solver is being configured
         * @param base_frame The base frame in which all input poses are expected.
         * This may (or may not) be the root frame of the chain that the solver operates on
         * @param tip_frame The tip of the chain
         * @param search_discretization The discretization of the search when the solver steps through the redundancy
         * @return True if initialization was successful, false otherwise
         */
        virtual bool initialize(const std::string &robot_description,
                                const std::string &group_name,
                                const std::string &base_name,
                                const std::string &tip_name,
                                double search_discretization);

        /**
         * @brief Given a set of joint angles and a set of links, compute their pose
         * @param link_names A set of links for which FK needs to be computed
         * @param joint_angles The state for which FK is being computed
         * @param poses The resultant set of poses (in the frame returned by getBaseFrame())
         * @return True if a valid solution was found, false otherwise
         */
        virtual bool getPositionFK(const std::vector<std::string> &link_names,
                                   const std::vector<double> &joint_angles,
                                   std::vector<geometry_msgs::Pose> &poses) const;

        /**
        * @brief Return all the joint names in the order they are used internally
        */
        const std::vector<std::string>& getJointNames() const;

        /**
        * @brief Return all the link names in the order they are represented internally
        */
        const std::vector<std::string>& getLinkNames() const;

    protected:
        bool active_; /** Internal variable that indicates whether solvers are configured and ready */
        unsigned int dimension_; /** Dimension of the group */

        robot_model::RobotModelPtr robot_model_;

        moveit_msgs::KinematicSolverInfo fk_chain_info_; /** Store information for the forward kinematics solver */
        moveit_msgs::KinematicSolverInfo ik_chain_info_; /** Stores information of all joints */
        std::vector<std::string> actuated_joint_names_;  /** Stores names of the actuated joints, such as theta_1*/
        std::vector<std::string> end_effector_joint_names_;     /**Stores names of the end effectors' prismatic joints */
        std::vector<const robot_model::JointModel*> mimic_joint_arr_;    /** Stores all mimic joints in an array*/

        std::vector<double> joint_min_, joint_max_;
        robot_state::RobotStatePtr state_, state_2_;

        /**
        * @brief Return the index of a joint given its name
        */
        int getJointIndex(const std::string &name) const;


    };
}

#endif //OMNID_PROJECT_OMNID_KINEMATICS_PLUGIN_H
