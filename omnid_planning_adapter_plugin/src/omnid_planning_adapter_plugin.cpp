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
/* Author: Rico Ruotong Jia*/

/// \file
/// \brief This plugin first calculates each robot's end effector position, given the object platform position. Then, it fixes each end effector's pose, given their joint values.
/// Name initialization.
///     1. names of non-end-effector planning groups of each robot in SRDF
///     2. names of the frames: object platform base, world, each robot's "floating world" frame (see URDF of the robot)


#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <class_loader/class_loader.hpp>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include "omnid_core/delta_robot.h"
#include "omnid_planning_adapter_plugin/omnid_planning_adapter_plugin.h"

using std::vector;
using std::string; 

namespace omnid_planning_adapter_plugin
{
    class OmnidPlanningRequestAdapter : public planning_request_adapter::PlanningRequestAdapter
    {
    public:
        std::string getDescription() const override
        {
            return "Planning adapter for pre-processing and post-processing planning results for the Omnid Delta Arm Project";
        }

        bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                          const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
                          std::vector<std::size_t>& /*added_path_index*/) const override
        {
            // Thought: we modify each point along the trajectory, robot by robot.
            // Workflow:
            //  1. Recognize object platform's roll, pitch, yaw.
            //  2. Plan object platform trajectory
            //  3. Calculate (Note this is NOT PLANNING)pose of each robot eef based on object platform trajectory, using high level IK
            //  4. Call low level IK, get all joint angles


            bool success = planner(planning_scene, req, res);

            moveit_msgs::RobotTrajectory traj;
            res.trajectory_->getRobotTrajectoryMsg(traj);
            if(!object_platform_.nameInitialized())
                object_platform_.initializeJointLookup(traj.joint_trajectory.joint_names);

            for(auto& traj_point: traj.joint_trajectory.points) {
                //get the pose of the object platform
                tf2::Transform platform_pose;
                object_platform_.calcPlatformPoseTf(traj_point, platform_pose);

                for (unsigned int i = 0; i < delta_robots_.size(); ++i){
                    // high-level planning for the robot
                    tf2::Transform robot_to_eef;
                    const auto& robot = delta_robots_.at(i);
                    object_platform_.getNextEefPose(platform_pose, robot.getRobotToWorldTf(),
                                                    robot.getWorldToRobotTf(), robot_to_eef);

                    if (!robot.nameInitialized()){
                        robot.initializeJointLookup(traj.joint_trajectory.joint_names, planning_scene);
                    }

                    if(!robot.calcLowLevelIk(planning_scene, robot_to_eef, traj_point)){
                        ROS_WARN_STREAM("[omnid_planning_adapter_plugin]: Planning failed at robot " << i);
                        return false;
                    }
                }

                res.trajectory_ -> setRobotTrajectoryMsg (planning_scene -> getCurrentState(), traj);
            }
            return success;
        }

        void initialize(const ros::NodeHandle& /*nh_*/) override
        {
            nh_ = ros::NodeHandle ("omnid_moveit");
            vector<string> robot_planning_group_names; // each robot (not including the object platform arm) planning group's name in SRDF
            vector<string> robot_names;    //the first part of joint names in URDF.
            string object_platform_name;
            string group_reference_frame_name;
            string world_frame_name;
            string floating_world_frame_name; // the name of the first "floating world" name of each robot. please include '/' if there is any. see each robot's URDF

            if (!readRosParamStringVec(robot_planning_group_names, "robot_planning_group_names"))
                robot_planning_group_names = {"end_effector_arm_1", "end_effector_arm_2", "end_effector_arm_3"};

            if(!readRosParamStringVec(robot_names, "robot_names"))
                robot_names = {"robot_1", "robot_2", "robot_3"};
            object_platform_name = nh_.param("object_platform_name", string("object_platform"));
            group_reference_frame_name = nh_.param("group_reference_frame_name", string("object_platform_base"));
            world_frame_name = nh_.param("world_frame_name", string("world"));
            floating_world_frame_name = nh_.param("floating_world_frame_name", string("/floating_world_0"));

            // Initialize the robots
            object_platform_ = Object_Platform_Adapter(object_platform_name, group_reference_frame_name,
                                                       world_frame_name);
            delta_robots_.reserve(robot_names.size());
            for (unsigned int i = 0; i < robot_names.size(); ++i){
                //This is for base frame name = robot frames robot_1/floating_world_0. Change this if necessary.
                delta_robots_.emplace_back(robot_planning_group_names.at(i), robot_names.at(i), robot_names.at(i) + floating_world_frame_name, world_frame_name);
            }
        }

    private:
        ros::NodeHandle nh_;

        vector<Delta_Robot_Adapter> delta_robots_;
        Object_Platform_Adapter object_platform_;

        bool readRosParamStringVec(vector<string>& vec, string param_name){
            if (nh_.hasParam(param_name)){
                XmlRpc::XmlRpcValue temp;
                nh_.getParam(param_name, temp);
                for (unsigned int i = 0; i < temp.size(); ++i ){
                    vec.push_back(static_cast<string>(temp[i]));
                }
                return true;
            }
            else return false;
        }

    };
}

CLASS_LOADER_REGISTER_CLASS(omnid_planning_adapter_plugin::OmnidPlanningRequestAdapter, planning_request_adapter::PlanningRequestAdapter);

