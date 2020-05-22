//
// Created by zeid on 2/27/20.
//

#ifndef SRC_ROBOT_CONTROLLER_H
#define SRC_ROBOT_CONTROLLER_H


#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>
#include <stdarg.h>
#include <tf/transform_listener.h>
#include <map>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <tf2_ros/buffer.h>
#include <vector>
#include <iostream>
#include <string>
#include <initializer_list>

#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>

class RobotController {
public:
    RobotController(std::string arm_id);
    ~RobotController();
    bool Planner();
    void Execute();
    void GoToTarget(std::initializer_list<geometry_msgs::Pose> list);
    void GoToTarget1(const geometry_msgs::Pose& pose);
    // void SendRobotHome();
    void SendRobotTo(std::map<std::string, double>);
    void SendRobotTo(std::string joint_name, double joint_value);
    bool DropPart(geometry_msgs::Pose pose);
    void GripperToggle(const bool& state);
    void GripperCallback(const osrf_gear::VacuumGripperState::ConstPtr& grip);
    void GripperStateCheck(geometry_msgs::Pose pose);
    bool PickPart(const geometry_msgs::Pose& part_pose, bool pick_once=false);
    void RobotGoHome();
    double getRotationCompensate(const geometry_msgs::Pose&, 
        const geometry_msgs::Pose&);
    bool DropPart2(geometry_msgs::Pose);
    bool PickPart2(const geometry_msgs::Pose&, const geometry_msgs::Pose&);
    std::map<std::string, double> home_joint_pose_1;
    std::map<std::string, double> home_joint_pose_2;
    std::map<std::string, double> conveyer_pose;
    std::map<std::string, double> rail_pick_trans_pose;
    geometry_msgs::Pose belt_pickup_pose;
    bool Busy;

    std::map<std::string, double> check_qc_pose;
    geometry_msgs::Pose throw_away_pose;

    std::map<std::string, double> RailLeft; // Flipping: Default side Arm 1 Position
    std::map<std::string, double> RailRight; // Flipping: Default side Arm 2 Position


    // a 2 by 2 matrix with specific (x,y,z) world coordinates for placing multiple parts from the belt
    std::vector<geometry_msgs::Pose> tray_store_poses;
    bool gripper_state_;

//    bool checkRobotPose(std::map<std::string, double>);
//    bool PickPart_advance(const geometry_msgs::Pose&, bool pick_once=false);

    std::string id;
private:
    // subscriber of other arm's linear actuactor pose
    ros::Subscriber line_act_pose_sub;
    ros::Publisher line_act_pose_pub;
    

    ros::NodeHandle robot_controller_nh_;
    moveit::planning_interface::MoveGroupInterface::Options robot_controller_options;

    ros::ServiceClient gripper_client_;
    ros::NodeHandle gripper_nh_;
    ros::Subscriber gripper_subscriber_;

    tf::TransformListener robot_tf_listener_;
    tf::StampedTransform robot_tf_transform_;
    tf::TransformListener agv_tf_listener_;
    tf::StampedTransform agv_tf_transform_;

    // tf2_ros::Buffer robot_tf_buffer_;
    // tf2_ros::Buffer agv_tf_buffer_;

    geometry_msgs::Pose target_pose_;

    moveit::planning_interface::MoveGroupInterface robot_move_group_;
    moveit::planning_interface::MoveGroupInterface::Plan robot_planner_;

    osrf_gear::VacuumGripperControl gripper_service_;
    osrf_gear::VacuumGripperState gripper_status_;

    std::string object;
    bool plan_success_;

    std::map<std::string, double> home_joint_pose_0;
    // std::vector<double> home_joint_pose_;
    // std::vector<double> end_position_;
    std::map<std::string, double> end_position_;

    geometry_msgs::Pose home_cart_pose_;
    geometry_msgs::Quaternion fixed_orientation_;
    geometry_msgs::Pose agv_position_;

    double offset_;
    double roll_def_,pitch_def_,yaw_def_;
    tf::Quaternion q;
    int counter_;
    bool  drop_flag_;

    double y_comp;
};
#endif //SRC_ROBOT_CONTROLLER_H
