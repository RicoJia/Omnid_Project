/// \file
/// \brief The move_group interface for controlling a group of delta arms. Launch this node alongside moveit_config.
/// \author Rico Ruotong Jia (ruotongjia2020@u.northwestern.edu)
/// SUBSCRIBE:
///     /move_group/goal - Listening for goal planning requsts from Rviz or an external user.(part of ROS action)
///     /move_group/display_planned_path - Display planned path on Rviz
///     attach objects?
/// Tutorials:
///     moveit_commander example https://www.theconstructsim.com/ros-qa-138-how-to-set-a-sequence-of-goals-in-moveit-for-a-manipulator/

//Our own library
#include "omnid_move_group_interface/omnid_group_ik.hpp"

//Moveit!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>
#include <moveit/kinematics_base/kinematics_base.h>

//TF transforms
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2/transform_datatypes.h"
#include "tf2_armadillo/tf2_armadillo.h"

//C++ helpers
#include <memory>
#include <algorithm>

namespace rvt = rviz_visual_tools;
using std::make_unique;
using std::for_each;
using std::vector;
using geometry_msgs::Pose;

typedef arma::Mat<double > mat;

//TASK: try to drag the marker, see if it follows u.
//Milestones:
// 2. , iterate thru all robot: get inverse of reference->robot tf, tf -> eigen, Pose -> eigen
// 3. group_planner: get world -> reference tf
// 4. pose -> eigen, transformStamped -> eigen
// 5. add eigen, so you can do multiplication.
// 6. eigen -> pose.
//TODO: fix the publisher of the object platform

namespace omnid_group_planning{

    //TODO
    using std::cout;
    using std::endl;
    void printGeoMsg(const std::string name, const geometry_msgs::TransformStamped& t){
        auto trans = t.transform;
        cout<<name<<" Translation: "<<trans.translation.x<<" "<<trans.translation.y<<" "<<trans.translation.z<<endl;
    }
    void printGeoMsg(const std::string name, const geometry_msgs::Pose& pose){
        auto pos = pose.position;
        cout<<name<<" position: "<<pos.x<<" "<<pos.y<<" "<<pos.z<<endl;
    }

    /// \brief listen to tf2::Transform and return it. An exception will be returned if no tf is heard
    /// \param target_frame: frame we express source frame in.
    /// \param source_frame: frame we want to express in target_frame.
    geometry_msgs::TransformStamped listenToTf2Transform(std::string target_frame, std::string source_frame){
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener (tf_buffer);
        try{
            //Tsource -> target
            auto reference_to_robot_tf_ = tf_buffer.lookupTransform(target_frame, source_frame, ros::Time(0),
                                                               ros::Duration(3.0));
            return reference_to_robot_tf_;
        }
        catch(std::exception &e){
            ROS_ERROR_STREAM("omnid_move_group_interface: cannot initialize TF. " << e.what());
            exit(1);    //TODO Not sure?
        };
    }


    class Single_Omnid_Planner{
    public:
        /// \brief - constructor of an arm planning module
        /// \param planning_group_name - name of the arm's planning group, which is a subgroup of a larger planning group in SRDF
        /// \param nh - node handle
        /// \param body_frame_name - name of the frame that we want to express our end_effector pose in.
        /// \param reference_frame_name - name of the reference frame of the robot group. Usually this is the centroid of the planning robot group.
        Single_Omnid_Planner(std::string planning_group_name, ros::NodeHandle nh, std::string body_frame_name, std::string world_frame_name);

        /// \brief publish the pose next pose for visualization in the robot's body frame.
        void publish();

        /// \brief Update the next pose for visualization
        /// \param pose - body_frame or world frame pose. The pose will be in world frame after this step.
        /// \param  isWorldPose - if the pose is already in world frame, it will be directly stored.
        /// Else, the pose will be transformed into the world frame.
        void updateNextPose(const geometry_msgs::Pose& pose, bool isWorldPose = false);

        /// \brief return TF from the group reference frame to the robot base frame.
        /// \return Robot to World TF
        geometry_msgs::TransformStamped getRobotToWorldTF();

        /// \brief checks if a pose (in body frame) can be achieved by calling IK
        /// \param pose - potential next pose in body frame
        /// \return true if there is a valid combination of actuated joint values for achieving this pose. Else, false.
        bool isValidPose(const geometry_msgs::Pose& pose);

    private:
        std::string eef_name_;  //tip of the link that's attached to the end_effector, for example "robot_3/dummy_platform_link"
        ros::Publisher goal_state_pub_;
        moveit::planning_interface::MoveGroupInterface move_group_;
        geometry_msgs::PoseStamped eef_pose_stamped;
        geometry_msgs::TransformStamped robot_to_world_tf_; //T_robot_world
        geometry_msgs::TransformStamped world_to_robot_tf_;
    };

    Single_Omnid_Planner::Single_Omnid_Planner(std::string planning_group_name, ros::NodeHandle nh, std::string body_frame_name, std::string world_frame_name):
            move_group_(planning_group_name)
    {
        eef_name_ =  move_group_.getEndEffectorLink();
        eef_pose_stamped.header.frame_id = world_frame_name;
        eef_pose_stamped.pose.orientation.w = 1;
        goal_state_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/rviz/moveit/move_marker/goal_" + eef_name_, 1);
        robot_to_world_tf_ = listenToTf2Transform(body_frame_name, world_frame_name);
        world_to_robot_tf_ = listenToTf2Transform(world_frame_name, body_frame_name);
    }

    void Single_Omnid_Planner::publish()
    {
        goal_state_pub_.publish(eef_pose_stamped);
    }

    void Single_Omnid_Planner::updateNextPose(const geometry_msgs::Pose &pose, bool isWorldPose)
    {
        if (!isWorldPose){
            tf2::doTransform(pose, eef_pose_stamped.pose, world_to_robot_tf_);  //in world frame
        }
        else{
            eef_pose_stamped.pose = pose;
        }
    }

    geometry_msgs::TransformStamped Single_Omnid_Planner::getRobotToWorldTF()
    {
        return robot_to_world_tf_;
    }

    bool Single_Omnid_Planner::isValidPose(const geometry_msgs::Pose &pose) {
        auto joint_model_group = move_group_.getRobotModel()->getJointModelGroup(move_group_.getName());
        double dimension_ =
                joint_model_group->getActiveJointModels().size() + joint_model_group->getMimicJointModels().size();

        std::vector<double> seed(dimension_);
        std::vector<double> ik_sol;
        ik_sol.reserve(dimension_);
        moveit_msgs::MoveItErrorCodes error;
        kinematics::KinematicsQueryOptions options;
        auto solver_ptr = joint_model_group -> getSolverInstance();
        if (solver_ptr){
            bool success = solver_ptr->searchPositionIK(pose, seed, 0.01, ik_sol, error, options);
            if(!success) ROS_WARN_STREAM("We're getting outside of the workspace");
            return success;
        }
        else{
            ROS_ERROR_STREAM("IK solver for individual robots is not available");
            return false;
        }
    }


    /// \brief Interface for updating a group of omnid robots.
    class Omnid_Group_Planner{
    public:
        Omnid_Group_Planner(ros::NodeHandle nh);
    protected:
        unsigned int leg_num_;  //number of legs, we are going to number our legs starting from 1.
        std::vector<std::unique_ptr<Single_Omnid_Planner>> arms_;
        std::unique_ptr<Single_Omnid_Planner> object_platform_;
        ros::Subscriber eef_sub_;

        std::string world_frame_name_;
        // Here each robot body frame name is body_frame_name_prefix_+ID+body_frame_name_suffix_, such as robot_1/floating_world_0
        std::string body_frame_name_prefix_;
        std::string body_frame_name_suffix_;
        // Frame where the object is placed.
        std::string group_reference_frame_name_;
        // Here each robot's planning group name is planning_group_prefix + ID
        std::string planning_group_prefix_;
        std::string object_platform_planning_group_name_;
        std::string eef_update_topic_name_;
        std::string eef_update_frame_name_;

        /// \brief listen to interactive marker position updates in world frame, then we update each marker's next world pose.
        void subCB(const visualization_msgs::InteractiveMarkerUpdate::ConstPtr& msg);   //callback function for object_platform pose update
    private:
        /// \brief initialize frame names
        void initParams();

        /// \brief High Level IK that update each robot's end effector pose (in its own base_link frame) for a given object platfor pose in the world frame
        /// \param object_platform_world_pose - object platform pose in the world frame.
        /// \param next_robot_poses - vector to store updated robot poses for the next time step.
        /// \param robot_ptrs - vector that stores each delta robots.
        /// \return true if a valid pose is found for each object platform, correspondingly next_robot_poses will be updated. False if not, and next_robot_poses will be empty
        bool highLevelIK(const geometry_msgs::Pose& object_platform_world_pose, std::vector<Pose>& next_robot_poses, const std::vector<std::unique_ptr<Single_Omnid_Planner>>& robot_ptrs);
    };

    Omnid_Group_Planner::Omnid_Group_Planner(ros::NodeHandle nh):
        object_platform_(nullptr)
    {
        initParams();
        //Initialize arms.
        arms_.reserve(leg_num_);
        for (unsigned int i = 0; i < leg_num_; ++i){
            unsigned int robot_id = i + 1;
            const std::string planning_group_name = planning_group_prefix_ + std::to_string(robot_id);
            const std::string body_frame_name = body_frame_name_prefix_ + std::to_string(robot_id) + body_frame_name_suffix_;
            arms_.push_back( make_unique<Single_Omnid_Planner>(planning_group_name, nh, body_frame_name, world_frame_name_) );
        }
        object_platform_ = make_unique<Single_Omnid_Planner>(object_platform_planning_group_name_, nh, group_reference_frame_name_, world_frame_name_);
        eef_sub_ = nh.subscribe<visualization_msgs::InteractiveMarkerUpdate>(eef_update_topic_name_, 1,boost::bind(&omnid_group_planning::Omnid_Group_Planner::subCB, this,_1));
    };

    void Omnid_Group_Planner::initParams()
    {
        //TODO - Make these yaml params
        leg_num_ = 3;
        world_frame_name_ = "world";
        body_frame_name_prefix_ = "robot_";
        body_frame_name_suffix_ = "/floating_world_0";
        group_reference_frame_name_ = "object_platform_base";
        planning_group_prefix_ = "end_effector_arm_";
        object_platform_planning_group_name_ = "object_platform_arm";
        eef_update_topic_name_ = "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update";
        eef_update_frame_name_ = "EE:goal_object_platform_yaw_link";
    }

    void Omnid_Group_Planner::subCB(const visualization_msgs::InteractiveMarkerUpdate::ConstPtr &msg)
    {
        //Calculate the high level IK, then publish each arm's position if a valid set of joint positions can be found
        for(const auto& marker_pose: msg->poses){
            if (marker_pose.name == eef_update_frame_name_) {
                vector<Pose> next_robot_poses(leg_num_);
                if(highLevelIK(marker_pose.pose, next_robot_poses, arms_)){
                    object_platform_->updateNextPose(marker_pose.pose, true);     //we update the marker pose if it's good
                    for (unsigned id = 0; id < leg_num_; ++id) {
                        auto& arm = arms_.at(id);
                        arm->updateNextPose(next_robot_poses.at(id));
                        arm->publish();
                    }
                }
                else{
                    object_platform_->publish();
                }
            }
        }
    }


    bool Omnid_Group_Planner::highLevelIK(const geometry_msgs::Pose &object_platform_world_pose,
                                          std::vector<Pose>& next_robot_poses,
                                          const std::vector<std::unique_ptr<Single_Omnid_Planner>>& robot_ptrs)
    {
        //TODO: to add a fromMsg in tf2_armadillo for this
        //T_wo'
        tf2::Transform world_to_obj_tf;
        tf2::fromMsg(object_platform_world_pose, world_to_obj_tf);
        mat::fixed<4,4> world_to_obj_arma;
        tf2::fromMsg(world_to_obj_tf, world_to_obj_arma);
        //T_ow
        mat::fixed<4,4> ref_to_world_arma;
        tf2::fromMsg(object_platform_->getRobotToWorldTF().transform, ref_to_world_arma);

        for(unsigned int id = 0; id < leg_num_; ++id){

            //T_robot_world
            auto robot_to_world = arms_.at(id) -> getRobotToWorldTF();
            mat::fixed<4,4> robot_to_world_arma;
            tf2::fromMsg(robot_to_world.transform, robot_to_world_arma);

            //TODO: take this out after figuring out how to do inv in armadillo
            mat::fixed<4,4> robot_to_world_inv_arma;
            tf2::Transform robot_to_world_tf;
            tf2::fromMsg(robot_to_world.transform, robot_to_world_tf);
            tf2::fromMsg(robot_to_world_tf.inverse(), robot_to_world_inv_arma);

            mat::fixed<4,4> robot_to_eef_arma;
            // get next robot eef pose in the reference frame
            omnid_group_ik::getRobotEefPose(world_to_obj_arma, robot_to_world_arma, ref_to_world_arma, robot_to_world_inv_arma, robot_to_eef_arma);

            //TODO add to tf2_armadillo: toMsg(const mat::fixed<4, 4> T, geometry_msgs::Pose&)
            //check the validity of the pose
            Pose robot_to_eef_pose;
            tf2::toMsg(robot_to_eef_arma, robot_to_eef_pose);

            const auto& robot_ptr = robot_ptrs.at(id);

            if (!robot_ptr -> isValidPose(robot_to_eef_pose)){

                next_robot_poses.clear();
                ROS_WARN_STREAM("omnid_move_group_interface: cannot find valid robot end effector poses given the object platform pose");
                return false;
            }

            // update next_robot_pose in the robot's body frame
            next_robot_poses.at(id) = robot_to_eef_pose;
        }
        return true;
    }

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "omnid_move_group_interface");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    omnid_group_planning::Omnid_Group_Planner ogp(nh);
    ros::waitForShutdown();

    ros::shutdown();
    return 0;
}
