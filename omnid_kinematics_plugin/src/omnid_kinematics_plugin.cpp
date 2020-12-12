#include <class_loader/class_loader.h>
// URDF, SRDF
#include <urdf_model/model.h>
#include <srdfdom/model.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include "omnid_core/delta_robot.h"
#include <cmath>
#include "omnid_kinematics_plugin/omnid_kinematics_plugin.h"
//#include "../include/omnid_kinematics_plugin/omnid_kinematics_plugin.h"

CLASS_LOADER_REGISTER_CLASS(omnid_kinematics::OmnidKinematicsPlugin, kinematics::KinematicsBase)

namespace omnid_kinematics{

    OmnidKinematicsPlugin::OmnidKinematicsPlugin():active_(false){}

    /// \brief Initialize joint names and limits. Planning scene has an array for the end-effector-joint -names (as long as they're revolute, needs to be checked).
    // Here, joint name checking looks for hard-coded keywords theta, beta.
    bool OmnidKinematicsPlugin::initialize(const std::string &robot_description,
                                        const std::string& group_name,
                                        const std::string& base_frame,
                                        const std::string& tip_frame,
                                        double search_discretization){

        // Set the parameters for the solver, for use with kinematic chain IK solvers
        setValues(robot_description, group_name, base_frame, tip_frame, search_discretization);
        //Load SRDF
        rdf_loader::RDFLoader rdf_loader(robot_description_);
        const srdf::ModelSharedPtr &srdf = rdf_loader.getSRDF();
        const urdf::ModelInterfaceSharedPtr &urdf_model = rdf_loader.getURDF();
        // load urdf, srdf models
        if (!urdf_model || !srdf)
        {
            ROS_ERROR_NAMED("omnid_kinematics_plugin","URDF and SRDF must be loaded.");
            return false;
        }
        robot_model_.reset(new robot_model::RobotModel(urdf_model, srdf));
        robot_model::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group_name);
        if (!joint_model_group)
            return false;

        // We skip the single DOF joint checking, since we only care about the after spring joints
        // store joint names, joint limits, and the tip info into the IK & FK Chains
        ros::NodeHandle nh;
        std::string theta_name, gamma_name, beta_name;
        nh.getParam("theta_name", theta_name);
        nh.getParam("gamma_name", gamma_name);
        nh.getParam("beta_name", beta_name);
        dimension_ = joint_model_group->getActiveJointModels().size() + joint_model_group->getMimicJointModels().size();
        for (std::size_t i=0; i < joint_model_group->getJointModels().size(); ++i)
        {
            const robot_model::JointModel* jm = joint_model_group->getJointModels()[i];
            if(jm->getType() == moveit::core::JointModel::REVOLUTE || jm->getType() == moveit::core::JointModel::PRISMATIC)
            {
                ik_chain_info_.joint_names.push_back(joint_model_group->getJointModelNames()[i]);
                const std::vector<moveit_msgs::JointLimits> &jvec = jm->getVariableBoundsMsg();
                ik_chain_info_.limits.insert(ik_chain_info_.limits.end(), jvec.begin(), jvec.end());
                if(jm->getType() == moveit::core::JointModel::REVOLUTE)
                {
                    //first check whether it belongs to the set of active joints in the group
                    if (jm->getMimic() == NULL && jm->getVariableCount() > 0) {
                        std::string name = jm->getName();
                        if(name.find(theta_name) != std::string::npos){
                            actuated_joint_names_.push_back(name);
//                            actuated_joint_names_.push_back(joint_model_group->getJointModelNames()[i]);
                        }
                        else if(name.find(gamma_name) != std::string::npos){
                            gamma_joint_names_.push_back(name);
                        }
                        else if(name.find(beta_name) != std::string::npos){
                            beta_joint_names_.push_back(name);
                        }
                    }
                    else if (jm->getMimic() && joint_model_group->hasJointModel(jm->getMimic()->getName())){
                        mimic_joint_arr_.push_back(jm);
                    }
                }
                else
                {
                    end_effector_joint_names_.push_back(joint_model_group->getJointModelNames()[i]);
                }
            }
        }

        fk_chain_info_.joint_names = ik_chain_info_.joint_names;
        fk_chain_info_.limits = ik_chain_info_.limits;

        if(!joint_model_group->hasLinkModel(getTipFrame()))
        {
            ROS_ERROR_NAMED("omnid_kinematics_plugin","Could not find tip name in joint group '%s'", group_name.c_str());
            return false;
        }
        ik_chain_info_.link_names.push_back(getTipFrame());
        fk_chain_info_.link_names = joint_model_group->getLinkModelNames();

        joint_min_.resize(ik_chain_info_.limits.size());
        joint_max_.resize(ik_chain_info_.limits.size());

        for(unsigned int i=0; i < ik_chain_info_.limits.size(); i++)
        {
            joint_min_.at(i) = ik_chain_info_.limits[i].min_position;
            joint_max_.at(i) = ik_chain_info_.limits[i].max_position;
        }

        // Setup the joint state groups that we need
        state_.reset(new robot_state::RobotState(robot_model_));
        state_2_.reset(new robot_state::RobotState(robot_model_));
        active_ = true;
        ROS_INFO_NAMED("omnid_kinematics_plugin","omnid_kinematics_plugin initialized");

        return true;
    }

    bool OmnidKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           std::vector<double> &solution,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           const kinematics::KinematicsQueryOptions &options) const
    {
        const IKCallbackFn solution_callback = 0;
        std::vector<double> consistency_limits;

        return searchPositionIK(ik_pose,
                                ik_seed_state,
                                default_timeout_,
                                consistency_limits,
                                solution,
                                solution_callback,
                                error_code,
                                options);
    }

    bool OmnidKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              std::vector<double> &solution,
                                              moveit_msgs::MoveItErrorCodes &error_code,
                                              const kinematics::KinematicsQueryOptions &options) const
    {

        const IKCallbackFn solution_callback = 0;
        std::vector<double> consistency_limits;
        return searchPositionIK(ik_pose,
                                ik_seed_state,
                                timeout,
                                consistency_limits,
                                solution,
                                solution_callback,
                                error_code,
                                options);
    }


    bool OmnidKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              const std::vector<double> &consistency_limits,
                                              std::vector<double> &solution,
                                              moveit_msgs::MoveItErrorCodes &error_code,
                                              const kinematics::KinematicsQueryOptions &options) const
    {
        const IKCallbackFn solution_callback = 0;
        return searchPositionIK(ik_pose,
                                ik_seed_state,
                                timeout,
                                consistency_limits,
                                solution,
                                solution_callback,
                                error_code,
                                options);
    }

    bool OmnidKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              std::vector<double> &solution,
                                              const IKCallbackFn &solution_callback,
                                              moveit_msgs::MoveItErrorCodes &error_code,
                                              const kinematics::KinematicsQueryOptions &options) const
    {
        std::vector<double> consistency_limits;
        return searchPositionIK(ik_pose,
                                ik_seed_state,
                                timeout,
                                consistency_limits,
                                solution,
                                solution_callback,
                                error_code,
                                options);
    }

    bool OmnidKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              const std::vector<double> &consistency_limits,
                                              std::vector<double> &solution,
                                              const IKCallbackFn &solution_callback,
                                              moveit_msgs::MoveItErrorCodes &error_code,
                                              const kinematics::KinematicsQueryOptions &options) const
    {
        // Actual working function
        // starting timing for time_out, then check for active, dimension, consistency limits

        if(!active_) {
            ROS_ERROR_NAMED("omnid_kinematics_plugin","kinematics not active");
            error_code.val = error_code.NO_IK_SOLUTION;
            return false;
        }

        if(ik_seed_state.size() != dimension_) {
            ROS_ERROR_STREAM_NAMED("omnid_kinematics_plugin","Seed state must have size " << dimension_ << " instead of size " << ik_seed_state.size());
            error_code.val = error_code.NO_IK_SOLUTION;
            return false;
        }

        if(!consistency_limits.empty() && consistency_limits.size() != dimension_) {
            ROS_ERROR_STREAM_NAMED("omnid_kinematics_plugin","Consistency limits be empty or must have size " << dimension_ << " instead of size " << consistency_limits.size());
            error_code.val = error_code.NO_IK_SOLUTION;
            return false;
        }

        //Fill in the x, y, z prismatic joints
        std::vector<double> pose_vec{ik_pose.position.x, ik_pose.position.y, ik_pose.position.z};

        solution = std::vector<double>(dimension_, 0);

        for (unsigned int i = 0; i < end_effector_joint_names_.size(); ++i) {
            const std::string joint_name = end_effector_joint_names_.at(i);
            solution.at(getJointIndex(joint_name)) = pose_vec.at(i);
        }

        //Solving IK for after-spring angles
        const struct type_linear_position pos = {.x = ik_pose.position.x, .y = ik_pose.position.y, .z = ik_pose.position.z};
        struct type_angular_position thetas;
        delta_robot_inverse_kinematics(&DELTA_ROBOT, &pos, &thetas);
        std::vector<double> thetas_vec{thetas.theta1, thetas.theta2, thetas.theta3};

        // Solution elements are in the same order as theta1, 2, 3
        for (unsigned int i = 0; i < actuated_joint_names_.size(); ++i) {
            const std::string joint_name = actuated_joint_names_.at(i);
            if (std::isnan(thetas_vec.at(i))) {
//                ROS_WARN_STREAM(joint_name<<" hitting singularity. cannot plan. ");
                error_code.val = error_code.NO_IK_SOLUTION;
                return false;
            }
            int joint_index = getJointIndex(joint_name);
            // apply joint limits here
            if(joint_min_.at(joint_index) > thetas_vec.at(i) || joint_max_.at(joint_index) < thetas_vec.at(i)){
//                ROS_WARN_STREAM(joint_name << " is hitting joint limits");
                error_code.val = error_code.NO_IK_SOLUTION;
                return false;
            }

            solution.at(joint_index) = thetas_vec.at(i);
        }
        // Solving for knee angles
        struct delta_robot_knee_angles knees;
        delta_robot_knees(&DELTA_ROBOT, &pos, &thetas, &knees);
        std::vector<double> gammas_vec{knees.gamma[0], knees.gamma[1], knees.gamma[2]};
        std::vector<double> betas_vec{knees.beta[0], knees.beta[1], knees.beta[2]};
        for (unsigned int i = 0; i < beta_joint_names_.size(); ++i) {
            const std::string beta_joint_name = beta_joint_names_.at(i);
            const std::string gamma_joint_name = gamma_joint_names_.at(i);
            solution.at(getJointIndex(beta_joint_name)) = betas_vec.at(i);
            solution.at(getJointIndex(gamma_joint_name)) = gammas_vec.at(i);
        }

        // see if this solution passes the callback function test
        if(!solution_callback.empty())
            solution_callback(ik_pose, solution, error_code);
        else
            error_code.val = error_code.SUCCESS;
        if (error_code.val == error_code.SUCCESS)
            return true;
        else{
            ROS_DEBUG_NAMED("kdl","An IK that satisifes the constraints and is collision free could not be found");
            error_code.val = error_code.NO_IK_SOLUTION;
            return false;
        }

    }

    bool OmnidKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                           const std::vector<double> &joint_angles,
                                           std::vector<geometry_msgs::Pose> &poses) const {
        if (!active_) {
            ROS_ERROR_NAMED("omnid_kinematics_plugin", "kinematics not active");
            return false;
        }
        poses.resize(link_names.size());
        if (joint_angles.size() != dimension_) {
            ROS_ERROR_NAMED("omnid_kinematics_plugin", "Joint angles vector must have size: %d", dimension_);
            return false;
        }

        // calculating thetas
        std::vector<double> thetas_vec(actuated_joint_names_.size(), 0);
        for (unsigned int i = 0; i < actuated_joint_names_.size(); ++i) {
            const std::string joint_name = actuated_joint_names_.at(i);
            int joint_index = getJointIndex(joint_name);
            // apply joint limits here
            if(joint_min_.at(joint_index) > joint_angles.at(joint_index) || joint_max_.at(joint_index) < joint_angles.at(joint_index)){
                ROS_WARN_STREAM(joint_name << " in Omnid Forward-Kinematics Request is out of its joint limit");
                return false;
            }
        }
        struct type_angular_position joints;
        struct type_linear_position xyz_pose;
        joints.theta1 = thetas_vec.at(0);
        joints.theta2 = thetas_vec.at(1);
        joints.theta3 = thetas_vec.at(2);
        delta_robot_forward_kinematics(&DELTA_ROBOT, &joints, &xyz_pose);

        geometry_msgs::Pose end_effector_pose;
        end_effector_pose.position.x = xyz_pose.x;
        end_effector_pose.position.y = xyz_pose.y;
        end_effector_pose.position.z = xyz_pose.z;
        poses.push_back(end_effector_pose);
        return true;
    }

    const std::vector<std::string>& OmnidKinematicsPlugin::getJointNames() const
    {
        return ik_chain_info_.joint_names;
    }

    const std::vector<std::string>& OmnidKinematicsPlugin::getLinkNames() const
    {
        return ik_chain_info_.link_names;
    }

    int OmnidKinematicsPlugin::getJointIndex(const std::string &name) const
    {
        for (unsigned int i=0; i < ik_chain_info_.joint_names.size(); i++) {
            if (ik_chain_info_.joint_names[i] == name)
                return i;
        }
        return -1;
    }

}
