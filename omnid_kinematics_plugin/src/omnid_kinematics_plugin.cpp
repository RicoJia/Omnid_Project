#include <class_loader/class_loader.h>
// URDF, SRDF
#include <urdf_model/model.h>
#include <srdfdom/model.h>
#include <moveit/rdf_loader/rdf_loader.h>

//TODO
//#include <omnid_kinematics_plugin/omnid_kinematics_plugin.h>
#include <../include/omnid_kinematics_plugin/omnid_kinematics_plugin.h>

CLASS_LOADER_REGISTER_CLASS(omnid_kinematics::OmnidKinematicsPlugin, kinematics::KinematicsBase)

namespace omnid_kinematics{

    OmnidKinematicsPlugin::OmnidKinematicsPlugin():active_(false){}


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

        // Here we skip chain checking because we work with a parallel manipulator. All we need is the three after-spring joint names
        // We also skips single DOF joint checking, since we only care about the after spring joints

        // store joint names, joint limits, and the tip info into the IK & FK Chains
        dimension_ = joint_model_group->getActiveJointModels().size() + joint_model_group->getMimicJointModels().size();
        for (std::size_t i=0; i < joint_model_group->getJointModels().size(); ++i)
        {
            robot_model::JointModel* jm = joint_model_group->getJointModels()[i];
            if(jm->getType() == moveit::core::JointModel::REVOLUTE || jm->getType() == moveit::core::JointModel::PRISMATIC)
            {
                ik_chain_info_.joint_names.push_back(joint_model_group->getJointModelNames()[i]);
                const std::vector<moveit_msgs::JointLimits> &jvec = jm->getVariableBoundsMsg();
                ik_chain_info_.limits.insert(ik_chain_info_.limits.end(), jvec.begin(), jvec.end());

                if(jm->getType() == moveit::core::JointModel::REVOLUTE)
                {
                    //first check whether it belongs to the set of active joints in the group
                    if (jm->getMimic() == NULL && jm->getVariableCount() > 0) {
                        actuated_joint_names_.push_back(joint_model_group->getJointModelNames()[i]);
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
            joint_min_.push_back(ik_chain_info_.limits[i].min_position);
            joint_max_.push_back(ik_chain_info_.limits[i].max_position);
        }

        //TODO joint_model_group -> getMimicJointModels(), return a vector of JointModels
        //TODO: oint_model_group -> getJointModelNames, for names


        // Check for mimic joints
//        bool has_mimic_joints = joint_model_group->getMimicJointModels().size() > 0;
//        std::vector<unsigned int> redundant_joints_map_index;
//        std::vector<kdl_kinematics_plugin::JointMimic> mimic_joints;
//        unsigned int joint_counter = 0;
//        for (std::size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
//        {
//            const robot_model::JointModel *jm = robot_model_->getJointModel(kdl_chain_.segments[i].getJoint().getName());
//
//            //first check whether it belongs to the set of active joints in the group
//            if (jm->getMimic() == NULL && jm->getVariableCount() > 0)
//            {
//                kdl_kinematics_plugin::JointMimic mimic_joint;
//                mimic_joint.reset(joint_counter);
//                mimic_joint.joint_name = kdl_chain_.segments[i].getJoint().getName();
//                mimic_joint.active = true;
//                mimic_joints.push_back(mimic_joint);
//                ++joint_counter;
//                continue;
//            }
//            if (joint_model_group->hasJointModel(jm->getName()))
//            {
//                if (jm->getMimic() && joint_model_group->hasJointModel(jm->getMimic()->getName()))
//                {
//                    kdl_kinematics_plugin::JointMimic mimic_joint;
//                    mimic_joint.reset(joint_counter);
//                    mimic_joint.joint_name = kdl_chain_.segments[i].getJoint().getName();
//                    mimic_joint.offset = jm->getMimicOffset();
//                    mimic_joint.multiplier = jm->getMimicFactor();
//                    mimic_joints.push_back(mimic_joint);
//                    continue;
//                }
//            }
//        }
//        for (std::size_t i = 0; i < mimic_joints.size(); ++i)
//        {
//            if(!mimic_joints[i].active)
//            {
//                const robot_model::JointModel* joint_model = joint_model_group->getJointModel(mimic_joints[i].joint_name)->getMimic();
//                for(std::size_t j=0; j < mimic_joints.size(); ++j)
//                {
//                    if(mimic_joints[j].joint_name == joint_model->getName())
//                    {
//                        mimic_joints[i].map_index = mimic_joints[j].map_index;
//                    }
//                }
//            }
//        }
//        mimic_joints_ = mimic_joints;

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

/////////

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
    {// Actual working function
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

        solution.resize(dimension_);

        // TODO
        double cheat_angle = 1.57;
        // We don't even need a timeout for this!
        for (auto& s: solution){
            s = cheat_angle;
        }
        error_code.val = error_code.SUCCESS;
        ROS_INFO_NAMED("omnid_kinematics_plugin","HEHEHE IK");
        for (auto& i : ik_chain_info_.joint_names){
            ROS_INFO_STREAM("omnid_kinematics_plugin joint name: " <<i);
        }
        return true;

    }

    bool OmnidKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                           const std::vector<double> &joint_angles,
                                           std::vector<geometry_msgs::Pose> &poses) const {
        ros::WallTime n1 = ros::WallTime::now();
        if (!active_) {
            ROS_ERROR_NAMED("omnid_kinematics_plugin", "kinematics not active");
            return false;
        }
        poses.resize(link_names.size());
        if (joint_angles.size() != dimension_) {
            ROS_ERROR_NAMED("omnid_kinematics_plugin", "Joint angles vector must have size: %d", dimension_);
            return false;
        }

        //TODO
        geometry_msgs::Pose cheat_pose;
        cheat_pose.position.x = 0.5;
        cheat_pose.position.y = 0.5;
        cheat_pose.position.z = 0.5;
        poses.push_back(cheat_pose);
        ROS_INFO_NAMED("omnid_kinematics_plugin","HEHEHE FK");
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
