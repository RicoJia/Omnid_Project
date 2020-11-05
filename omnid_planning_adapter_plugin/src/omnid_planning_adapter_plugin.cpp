/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Ioan Sucan, Rico Ruotong Jia*/
//TODO declare dependence on omnid_core
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <class_loader/class_loader.hpp>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include "omnid_core/delta_robot.h"


namespace omnid_planning_adapter_plugin
{
    class OmnidPlanningRequestAdapter : public planning_request_adapter::PlanningRequestAdapter
    {
    public:
        std::string getDescription() const override
        {
            return "Planning adapter for post-processing planning results for the Omnid Delta Arm Project";
        }

        bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                          const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
                          std::vector<std::size_t>& /*added_path_index*/) const override
        {
            bool success = planner(planning_scene, req, res);

            //get robot trajectory msg - its sequence is the same as the waypoints
            moveit_msgs::RobotTrajectory trajectory_msg;
            res.trajectory_->getRobotTrajectoryMsg(trajectory_msg);

            //initialize indices
            if (!names_init_){
                build_indices_arr(xyz_names_, xyz_indices_, trajectory_msg);
                build_indices_arr(theta_names_, theta_indices_, trajectory_msg);
                names_init_ = true;
            }
            for(auto& point: trajectory_msg.joint_trajectory.points){
                std::vector<double>& positions = point.positions;
                struct type_angular_position joints;
                struct type_linear_position xyz_pose;
                joints.theta1 = positions.at(theta_indices_.at(0));
                joints.theta2 = positions.at(theta_indices_.at(1));
                joints.theta3 = positions.at(theta_indices_.at(2));
                delta_robot_forward_kinematics(&DELTA_ROBOT, &joints, &xyz_pose);
                positions.at(xyz_indices_.at(0)) = xyz_pose.x;
                positions.at(xyz_indices_.at(1)) = xyz_pose.y;
                positions.at(xyz_indices_.at(2)) = xyz_pose.z;
                //set each individual waypoints.
                res.trajectory_ -> setRobotTrajectoryMsg (planning_scene -> getCurrentState(), trajectory_msg);

            }
            //TODO
            for(auto name:theta_indices_){
                ROS_INFO_STREAM("xyz index: "<<name);
            }
            return success;
        }

        void initialize(const ros::NodeHandle& /*nh*/) override
        {
            names_init_ = false;
            xyz_indices_.resize(3);
            // TODO read in params with names
            std::string theta_name = "theta";

            //build name lookups.  theta_indices_ and xyz_indices are in the same order
            theta_names_.resize(3);
            theta_indices_.resize(3);
            for (unsigned int i = 0; i < 3; ++i){
                theta_names_.at(i) = theta_name + "_" + std::to_string(i+1);
            }
            //TODO REad in params
            xyz_names_ = {"x", "y", "z"};
        }

    private:
        mutable bool names_init_;
        std::vector <std::string> theta_names_;
        mutable std::vector<unsigned int> theta_indices_; //assume the the joints are named in the pattern xxx_1, xxx_2, xxx_3.
        std::vector <std::string> xyz_names_;
        mutable std::vector<unsigned int> xyz_indices_;

        /// \brief: pass in the joint model group, and the names and index arrays of either the xyz or the regular joints
        void build_indices_arr(const std::vector<std::string>& names, std::vector<unsigned int>& indices, const moveit_msgs::RobotTrajectory& trajectory_msg) const
        {
            unsigned int current_id = 0;
            for(const auto& name:trajectory_msg.joint_trajectory.joint_names){
                auto name_itr = std::find(names.begin(), names.end(), name);
                if (name_itr != names.end()){
                    indices.at(name_itr - names.begin()) = current_id;
                }
                ++current_id;
            }
        };
    };
}

CLASS_LOADER_REGISTER_CLASS(omnid_planning_adapter_plugin::OmnidPlanningRequestAdapter, planning_request_adapter::PlanningRequestAdapter);


