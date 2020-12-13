//
// Created by ricojia on 12/2/20.
//

#ifndef OMNID_RICO_OMNID_PLANNING_ADAPTER_PLUGIN_H
#define OMNID_RICO_OMNID_PLANNING_ADAPTER_PLUGIN_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>
#include <moveit/kinematics_base/kinematics_base.h>

//TF transforms
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


using std::string;
using std::vector;

namespace omnid_planning_adapter_plugin{
    void printTfTransform(const tf2::Transform& tf){
        auto origin = tf.getOrigin();
        ROS_INFO_STREAM("x: "<< origin.getX()<<" y: "<< origin.getY()<<" z: "<< origin.getZ());
        auto quat = tf.getRotation();
        auto angle = quat.getAngle();
        ROS_INFO_STREAM("angle: "<<angle);
    }

    // Base class that's only to be used by derived class
    class Adapter{

    public:
        /// \brief return a constant reference of Transform Robot -> World.
        const tf2::Transform& getRobotToWorldTf() const
        {
            return robot_to_world_tf_;
        }

        bool nameInitialized() const
        {
            return name_init_;
        }

    protected:
        Adapter() = default;
        Adapter(const std::string &robot_name, const std::string& body_frame_name, const std::string& world_frame_name) :
                robot_name_(robot_name),
                body_frame_name_(body_frame_name),
                world_frame_name_(world_frame_name),
                name_init_(false)
        {
            getTf2Transform(body_frame_name, world_frame_name, robot_to_world_tf_);
            world_to_robot_tf_ = robot_to_world_tf_.inverse();
        }
        tf2::Transform robot_to_world_tf_; //T_robot_world
        tf2::Transform world_to_robot_tf_;
        std::string robot_name_;
        std::string body_frame_name_;
        std::string world_frame_name_;
        mutable bool name_init_;

        /// \brief get the head and tail iterators of a range of elements in a string vector that contains a string.
        /// the range is [head, tail). If no range is found, then the end of string is returned.
        void getRangeInStringVec(const std::vector <std::string> &str_vec, const std::string &to_search,
                                 std::vector<std::string>::const_iterator &head,
                                 std::vector<std::string>::const_iterator &tail) const{
            // increment head and tail together, until to_search is found. Then, increment tail until the end of the range.
            for(head = str_vec.cbegin(), tail = head; head != str_vec.cend(); ++head, ++tail){
                if ((*head).find(to_search) != string::npos){
                    for (; tail!=str_vec.cend() && (*tail).find(to_search) != string::npos; ++tail);
                    break;
                }
            }
        }

        /// \brief return the const_iterator of the first element in a string vector that contains to_search
        vector<string>::const_iterator findString(const std::string& to_search, const vector<string>::const_iterator head, const vector<string>::const_iterator tail) const
        {
            auto itr = head;
            for(; itr != tail && (*itr).find(to_search) == string::npos; ++itr );
            return itr;
        }

    private:
        /// \brief listen to the transform Tsource -> target and store it in result.
        /// \param target_frame: frame we express source frame in.
        /// \param source_frame: frame we want to express in target_frame.
        /// \param result: where the result transform is stored
        void getTf2Transform(const std::string& target_frame, const std::string& source_frame, tf2::Transform& result)
        {
            tf2_ros::Buffer tf_buffer;
            tf2_ros::TransformListener tf_listener (tf_buffer);
            try{
                auto reference_to_robot_tf_ = tf_buffer.lookupTransform(target_frame, source_frame, ros::Time(0),
                                                                        ros::Duration(3.0));
                tf2::fromMsg(reference_to_robot_tf_.transform, result);
            }
            catch(std::exception &e){
                ROS_ERROR_STREAM("omnid_planning_adapter_plugin: cannot initialize TF. " << e.what());
                ros::shutdown();
            };
        }
    };

    class Object_Platform_Adapter : public Adapter{
    public:
        Object_Platform_Adapter() = default;
        Object_Platform_Adapter(const string& robot_name, const string& body_frame_name, const string& world_frame_name) :
            Adapter(robot_name, body_frame_name, world_frame_name)
        {
            ros::NodeHandle nh;
            double object_clearance, object_thickness, h_platform;
            nh.getParam("object_clearance", object_clearance);
            nh.getParam("object_thickness", object_thickness);
            nh.getParam("h_platform", h_platform);
            object_z_correction_ = object_clearance + (object_thickness + h_platform)/2.0;

        }

        /// \brief initialize a look up table that maps joint names to their indices in joint_names.
        bool initializeJointLookup(const std::vector <std::string> &joint_names) const {
            // first, it finds the positions of head and tail. Then, it will build a map of joint_name to index.
            vector<string>::const_iterator head, tail;
            getRangeInStringVec(joint_names, robot_name_, head, tail);
            name_init_ = true;
            vector <string> to_search_keywords{"x", "y", "z", "roll", "pitch", "yaw"};
            for (unsigned int i = 0; i < to_search_keywords.size(); ++i) {
                string keyword = to_search_keywords.at(i);
                auto itr = findString(keyword, head, tail);
                joint_lookup_[keyword] = itr - joint_names.cbegin();
            }
        }

        /// \brief calculate the world frame pose of the object platform from a trajectory point and store it.
        /// \param traj_point - a point that consists of all joint values
        /// \param tf - next pose of the object platform (to be filled in)
        void calcPlatformPoseTf(const trajectory_msgs::JointTrajectoryPoint& traj_point, tf2::Transform& tf)const
        {
            //read  x, y, z, roll, pitch, yaw as the body frame pose, then convert it to tf2::Transform
            double x = traj_point.positions[joint_lookup_["x"]];
            double y = traj_point.positions[joint_lookup_["y"]];
            double z = traj_point.positions[joint_lookup_["z"]] - object_z_correction_; //this come from the overall clearance between the object and each robot platform
            double roll = traj_point.positions[joint_lookup_["roll"]];
            double pitch = traj_point.positions[joint_lookup_["pitch"]];
            double yaw = traj_point.positions[joint_lookup_["yaw"]];
            tf.setOrigin({x, y, z});
            tf2::Quaternion quat;
            quat.setEulerZYX(yaw, pitch, roll);
            tf.setRotation(quat);
            //get world frame pose
            tf = world_to_robot_tf_ * tf;
        }

        /// \brief calculate the next body-frame eef pose of a robot. The result is stored in robot_to_eef_tf.
        void getNextEefPose(const tf2::Transform &world_to_obj_tf, const tf2::Transform & delta_robot_to_world_tf,
                            const tf2::Transform &world_to_delta_robot_tf, tf2::Transform &robot_to_eef_tf) const
        {
            robot_to_eef_tf = delta_robot_to_world_tf * world_to_obj_tf  * robot_to_world_tf_ * world_to_delta_robot_tf;
        }

    private:
        mutable std::unordered_map<std::string, unsigned int> joint_lookup_;
        double object_z_correction_;
    };

    class Delta_Robot_Adapter : public Adapter{
    public:
        Delta_Robot_Adapter() = default;
        Delta_Robot_Adapter(const string& planning_group_name, const string& robot_name, const string& body_frame_name, const string& world_frame_name) :
            Adapter(robot_name, body_frame_name, world_frame_name),
            planning_group_name_(planning_group_name)
        {}

        /// \brief return a constant reference of Transform world -> robot.
        const tf2::Transform& getWorldToRobotTf() const
        {
            return world_to_robot_tf_;
        }

        bool initializeJointLookup(std::vector<std::string> joint_names, const planning_scene::PlanningSceneConstPtr &planning_scene) const
        {
            // we are going to build a lookup from the IK indices to the the trajectory message (joint_names).
            // find the range in the joint_names vector
            vector<string>::const_iterator head, tail;
            getRangeInStringVec(joint_names, robot_name_, head, tail);
            // grab the IK solver sequence from the joint_model_group->getJointModels(), then find the index of the corresponding names. 
            const robot_model::JointModelGroup* joint_model_group = planning_scene->getRobotModel()->getJointModelGroup(
                    planning_group_name_);
            auto ik_joint_names = joint_model_group->getJointModelNames();
            ik_solution_dimension_ = joint_model_group->getActiveJointModels().size() + joint_model_group->getMimicJointModels().size();
            unsigned int ik_solution_index = 0;
            for (std::size_t i=0; i < joint_model_group->getJointModels().size(); ++i) {
                const robot_model::JointModel *jm = joint_model_group->getJointModels()[i];
                if (jm->getType() == moveit::core::JointModel::REVOLUTE ||
                    jm->getType() == moveit::core::JointModel::PRISMATIC)
                {
                        //find the corresponding index of the joint name.
                    auto itr = findString(ik_joint_names[i], head, tail);
                    if(itr != tail){
                        unsigned int traj_msg_index = itr - joint_names.cbegin();
                        traj_to_ik_index_lookup_[traj_msg_index] = ik_solution_index;
                    }
                    ++ik_solution_index;
                }
            }
                name_init_ = true;
        }

        bool
        calcLowLevelIk(const planning_scene::PlanningSceneConstPtr &planning_scene, const tf2::Transform &robot_to_eef,
                       trajectory_msgs::JointTrajectoryPoint &traj_point) const
       {
            // Workflow:
            //  1. convert the robot eef to pose, generate some seed
            //  2. get the solver instance and feed the robot with the new pose, seed.
            //  3. get success and the new IK poses. If not successful, we are going to return false. TrajPoint is untouched.
            //  4. if successful, traverse through all traj point, using the head and tail iterators.
            //  5. find each entry in the IK solution, then fill the entry into traj_point.
            //  6. return true.

            geometry_msgs::Pose pose = tf2::toMsg(robot_to_eef, pose);
            vector<double> seed(ik_solution_dimension_);
            vector<double> ik_sol; ik_sol.reserve(ik_solution_dimension_);
            moveit_msgs::MoveItErrorCodes error;
            kinematics::KinematicsQueryOptions options;
            auto solver_ptr = planning_scene -> getRobotModel() -> getJointModelGroup(planning_group_name_) -> getSolverInstance();
            if(solver_ptr){
                bool success = solver_ptr->searchPositionIK(pose, seed, 0.01, ik_sol, error, options);
                if(!success) {
                    return false;}
            }

           // Traversing through all joints we're interested in and fill out traj_point
            for (auto beg = traj_to_ik_index_lookup_.cbegin(); beg != traj_to_ik_index_lookup_.cend(); ++beg)
            {
                unsigned int traj_i = beg -> first, ik_i = beg -> second;
                traj_point.positions[traj_i] = ik_sol.at(ik_i);
            }
            return true;
        }

    private:
        // map for joint name in trajectory_msg -> IK solution index
        mutable std::unordered_map<unsigned int, unsigned int> traj_to_ik_index_lookup_;
        mutable unsigned int ik_solution_dimension_;
        std::string planning_group_name_;
    };

}

#endif //OMNID_RICO_OMNID_PLANNING_ADAPTER_PLUGIN_H
