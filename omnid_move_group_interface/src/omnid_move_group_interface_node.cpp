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
    void printGeoTrans(const std::string name, const geometry_msgs::TransformStamped& t){
        auto trans = t.transform;
        cout<<name<<" Translation: "<<trans.translation.x<<" "<<trans.translation.y<<" "<<trans.translation.z<<endl;
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
        Single_Omnid_Planner(std::string planning_group_name, ros::NodeHandle nh, std::string body_frame_name, std::string group_reference_frame_name_, std::string world_frame_name);

        /// \brief publish the pose next pose for visualization in the robot's body frame.
        void publish();

        /// \brief Update the next pose for visualization in the refence frame of the planning group
        void updateNextPose(const geometry_msgs::Pose& pose);

        /// \brief return TF from the group reference frame to the robot base frame.
        geometry_msgs::TransformStamped getReftoRobotTF();

    protected:
        std::string eef_name_;  //tip of the link that's attached to the end_effector, for example "robot_3/dummy_platform_link"
        ros::Publisher goal_state_pub_;
        moveit::planning_interface::MoveGroupInterface move_group_;
        geometry_msgs::PoseStamped eef_pose_stamped;
        geometry_msgs::TransformStamped reference_to_robot_tf_;
    };

    Single_Omnid_Planner::Single_Omnid_Planner(std::string planning_group_name, ros::NodeHandle nh, std::string body_frame_name, std::string group_reference_frame_name_, std::string world_frame_name):
            move_group_(planning_group_name)
    {
        eef_name_ =  move_group_.getEndEffectorLink();
        eef_pose_stamped.header.frame_id = world_frame_name;
        goal_state_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/rviz/moveit/move_marker/goal_" + eef_name_, 1);
        reference_to_robot_tf_ = listenToTf2Transform(group_reference_frame_name_, body_frame_name);
    }

    void Single_Omnid_Planner::publish()
    {
        goal_state_pub_.publish(eef_pose_stamped);
    }

    void Single_Omnid_Planner::updateNextPose(const geometry_msgs::Pose &pose)
    {
        eef_pose_stamped.pose = pose;
    }

    geometry_msgs::TransformStamped Single_Omnid_Planner::getReftoRobotTF()
    {
        return reference_to_robot_tf_;
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
        geometry_msgs::TransformStamped ref_to_world_tf_;
        tf2::Transform world_to_ref_tf_;

        /// \brief listen to interactive marker position updates in world frame, then we update each marker's next world pose.
        void subCB(const visualization_msgs::InteractiveMarkerUpdate::ConstPtr& msg);   //callback function for object_platform pose update
    private:
        /// \brief initialize frame names
        void initParams();

        /// \brief High Level IK that update each robot's end effector pose in group_reference_frame for a given object platfor pose in the world frame
        /// \param object_platform_world_pose - object platform pose in the world frame.
        /// \param next_robot_poses - vector to store updated robot poses for the next time step.
        /// \return true if a valid pose is found for each object platform, correspondingly next_robot_poses will be updated. False if not, and next_robot_poses will be empty
        bool highLevelIK(const geometry_msgs::Pose& object_platform_world_pose, std::vector<geometry_msgs::Pose>& next_robot_poses);
    };

    Omnid_Group_Planner::Omnid_Group_Planner(ros::NodeHandle nh):
        object_platform_(nullptr)
    {
        initParams();
        ref_to_world_tf_ = listenToTf2Transform(group_reference_frame_name_, world_frame_name_);    //To->w
        auto world_to_ref_tf = listenToTf2Transform(world_frame_name_, group_reference_frame_name_);
        tf2::fromMsg(world_to_ref_tf.transform, world_to_ref_tf_);
        //TODO
        printGeoTrans("reference -> world", ref_to_world_tf_);

        //Initialize arms.
        arms_.reserve(leg_num_);
        for (unsigned int i = 0; i < leg_num_; ++i){
            unsigned int robot_id = i + 1;
            const std::string planning_group_name = planning_group_prefix_ + std::to_string(robot_id);
            const std::string body_frame_name = body_frame_name_prefix_ + std::to_string(robot_id) + body_frame_name_suffix_;
            arms_.push_back( make_unique<Single_Omnid_Planner>(planning_group_name, nh, body_frame_name, group_reference_frame_name_, world_frame_name_) );
        }
        object_platform_ = make_unique<Single_Omnid_Planner>(object_platform_planning_group_name_, nh, group_reference_frame_name_, group_reference_frame_name_, world_frame_name_);    //we need the object_platform to have its own base frame as the world refence frame.
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
        for(const auto& marker_pose: msg->poses){
            if (marker_pose.name == eef_update_frame_name_) {
                // update next robot poses
                vector<Pose> next_robot_poses(leg_num_);
                if(highLevelIK(marker_pose.pose, next_robot_poses)){
                    for (unsigned id = 0; id < leg_num_; ++id) {
                        auto& arm = arms_.at(id);
                        arm->updateNextPose(next_robot_poses.at(id));
                        arm->publish();
                    }
                }
            }
        }
    }

    bool Omnid_Group_Planner::highLevelIK(const geometry_msgs::Pose &object_platform_world_pose,
                                          std::vector <Pose>& next_robot_poses)
    {
        tf2::Transform world_to_obj_tf;
        tf2::fromMsg(object_platform_world_pose, world_to_obj_tf);
        mat::fixed<4,4> world_to_obj_arma;
        tf2::fromMsg(world_to_obj_tf, world_to_obj_arma);

        for(unsigned int id = 0; id < leg_num_; ++id){
            auto ref_to_robot_tf = arms_.at(id) -> getReftoRobotTF();
            mat::fixed<4,4> ref_to_robot_arma, ref_to_world_arma, ref_to_robot_eef_arma;
            tf2::fromMsg(ref_to_robot_tf.transform, ref_to_robot_arma);
            tf2::fromMsg(ref_to_world_tf_.transform, ref_to_world_arma);
            // get next robot eef pose in the reference frame
            bool success = omnid_group_ik::getRobotEefPose(world_to_obj_arma, ref_to_world_arma, ref_to_robot_arma,
                                           ref_to_robot_eef_arma);
            if (!success){
                next_robot_poses.clear();
                ROS_WARN_STREAM("omnid_move_group_interface: cannot find valid robot end effector poses given the object platform pose");
                return false;
            }
            //update next_robot_poses: first in reference_frame, then publish in world frame.
            tf2::Transform ref_to_robot_eef, world_to_robot_eef;
            tf2::toMsg(ref_to_robot_eef_arma, ref_to_robot_eef);
            world_to_robot_eef = world_to_ref_tf_ * ref_to_robot_eef; 
            tf2::toMsg(world_to_robot_eef, next_robot_poses.at(id));
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

//    static const std::string PLANNING_GROUP_ARM_1 = "end_effector_arm_1";
//    static const std::string EEF_1 = "end_effector_1";
//    moveit::planning_interface::MoveGroupInterface move_group_arm_1(PLANNING_GROUP_ARM_1);
//    moveit::planning_interface::MoveGroupInterface move_group_eef_1(EEF_1);
//    // We will use the :planning_interface:`PlanningSceneInterface class to add and remove collision objects in our "virtual world" scene
//    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//    // Raw pointers are frequently used to refer to the planning group for improved performance.
//    const moveit::core::JointModelGroup* joint_model_group_arm_1 = move_group_arm_1.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM_1);
//    const moveit::core::JointModelGroup* joint_model_group_eef_1 = move_group_eef_1.getCurrentState()->getJointModelGroup(EEF_1);
//
//
//        // add a publisher that publishes random goal states
//    ros::Publisher goal_state_pub = nh.advertise<moveit_msgs::RobotState>("/rviz/moveit/update_custom_goal_state", 1);
//    ros::Publisher goal_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/rviz/moveit/move_marker/goal_robot_3/dummy_platform_link", 1);
//
////    ros::Subscriber eef_sub = nh.subscribe<visualization_msgs::InteractiveMarkerUpdate>("/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update", 1, subCB);
//    moveit::core::RobotStatePtr robot_state_;
//    moveit_msgs::RobotState robot_state_msg;
//    geometry_msgs::PoseStamped robot_pose_msg;
//
//    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
//    // get robot state
//    robot_state_ = move_group_arm_1.getCurrentState();
//    // Allow time to publish messages
//    ros::Rate r(10);
//
//    while (true)
//    {
//        robot_pose_msg.header.frame_id = "robot_3/floating_world_0";
//        robot_pose_msg.pose.orientation.w = 1;
//        robot_pose_msg.pose.position.z = 0.3;
//        goal_pose_pub.publish(robot_pose_msg);
////        robot_state_->setToRandomPositions(joint_model_group_arm_1);
////        robot_state_msg.joint_state = getJointStateMsg(robot_state_, joint_model_group_arm_1);
////        goal_state_pub.publish(robot_state_msg);
//
//        ros::spinOnce();
//        r.sleep();
//    }



    ros::shutdown();
    return 0;
}
