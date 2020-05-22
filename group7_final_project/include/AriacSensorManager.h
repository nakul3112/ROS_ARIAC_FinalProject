//
// Created by zeid on 2/27/20.
//

#pragma once

#include <unordered_map>
#include <string>
#include <cmath>
#include <set>
#include <queue>
#include <map>
#include <array>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <algorithm>

#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <tf2_ros/buffer.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <osrf_gear/Proximity.h>
#include <osrf_gear/Order.h>
#include "RobotController.h"

#include "ros/subscriber.h"
#include "ros/node_handle.h"
#include "ros/topic_manager.h"

#include "ros/publisher.h"
#include "ros/publication.h"

class AriacSensorManager {
private:
    ros::NodeHandle sensor_nh_;

    // Subscriber
    ros::Subscriber lc_belt_sub; // Subscribe to the '/ariac/lc_belt' topic
    ros::Subscriber bb_1_sub; // Subscribe to the '/ariac/break_beam_1_change' topic
    ros::Subscriber bb_arm1_sub; // Subscribe to the '/ariac/break_beam_1_change' topic
    ros::Subscriber bb_arm2_sub; // Subscribe to the '/ariac/break_beam_1_change' topic
    ros::Subscriber bb_coming_sub; // Subscribe to the '/ariac/break_beam_coming' topic

    ros::Subscriber orders_sub; 
    ros::Subscriber lc_gear_sub; // Subscribe to the '/ariac/lc_gear' topic
    ros::Subscriber qc_1_sub; // Subscribe to the '/ariac/quality_control_sensor_2' topic
    ros::Subscriber lc_bin_1_sub; // Subscribe to the '/ariac/lc_gear' topic
    ros::Subscriber lc_agv_1_sub;
    ros::Subscriber lc_agv_2_sub;

    ros::Subscriber lc_bin_2_sub; // Subscribe to the '/ariac/lc_gear' topic
    ros::Subscriber lc_bin_3_sub; // Subscribe to the '/ariac/lc_gear' topic
    ros::Subscriber lc_bin_4_sub; // Subscribe to the '/ariac/lc_gear' topic
    ros::Subscriber lc_bin_5_sub; // Subscribe to the '/ariac/lc_gear' topic
    ros::Subscriber lc_bin_6_sub; // Subscribe to the '/ariac/lc_gear' topic

    ros::Subscriber  sensor_balckout_sub;

    bool qc_1_redFlag;
    ros::Subscriber qc_2_sub; // Subscribe to the '/ariac/quality_control_sensor_2' topic
    bool qc_2_redFlag;

    // Order/Product/Pose containers
    std::vector<osrf_gear::Order> received_orders_;
    std::vector<osrf_gear::Order> updated_received_orders_;
    unsigned int order_number;
    std::unordered_map<std::string, geometry_msgs::Pose> belt_part_map; // map for checked part from the belt
    std::unordered_map<std::string, geometry_msgs::Pose> gear_bin_map; // map for checked part in the bin
//    std::queue<std::pair<std::string, std::string>> incoming_partQ; // the queue store (part_type, part_frame_name) from the belt
    std::queue<std::pair<std::string, int>> incoming_partQ; // the queue store (part_type, part_frame_name) from the belt

    std::pair<std::string, std::string> popped_incoming_part;
    std::unordered_map<std::string, unsigned int> belt_part_counter; // map which calculate # of part_type from belt
    // std::multiset<std::string> desired_parts; // mulitset for desired parts in current order
    std::multimap<std::string, geometry_msgs::Pose> desired_parts_info; // mulitset for desired parts in current order
    std::unordered_map<std::string, size_t> task; // task to be done for each oreder
    // std::vector<geometry_msgs::Pose> gear_bin_vector;
    std::vector<std::pair<std::string, geometry_msgs::Pose>> gear_bin_vector;
    bool order_receiving_flag;
    std::set<std::string> parts_to_pickup_belt;

    // Robot related
    RobotController arm1;
    RobotController arm2;
    std::map<std::string, double> arm2_check_qc_pose;
    std::map<std::string, double> arm2_transition_pose;
    // std::map<std::string, double> go_transition_pose;
    // std::map<std::string, double> back_transition_pose;
    std::map<std::string, double> arm1_bin_pose;
    std::map<std::string, double> arm1_check_qc_pose;

    bool arm1_busy;
    bool arm2_busy;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    osrf_gear::LogicalCameraImage current_parts_1_;
    osrf_gear::LogicalCameraImage current_parts_2_;
    osrf_gear::LogicalCameraImage current_parts_3_;
    osrf_gear::LogicalCameraImage current_parts_4_;
    osrf_gear::LogicalCameraImage current_parts_5_;
    osrf_gear::LogicalCameraImage current_parts_6_;
    osrf_gear::LogicalCameraImage current_parts_agv1_;
    osrf_gear::LogicalCameraImage current_parts_agv2_;
    osrf_gear::LogicalCameraImage sensorBlackOutImage;
    osrf_gear::LogicalCameraImage quality_image1;
    osrf_gear::LogicalCameraImage quality_image2;

    bool  cam_1_, cam_2_,cam_3_, cam_4_, cam_5_,cam_6_;
    bool Flag_updateKit;
    int order_counter;
    int camera1_frame_counter_, camera2_frame_counter_, camera3_frame_counter_;
    int camera4_frame_counter_, camera5_frame_counter_, camera6_frame_counter_;
    std::map<std::string, std::vector<std::string>> product_frame_list_;

    int NumPartsToRemove, NumPartsToModify, NumPartsToAdd;

    tf::TransformListener camera_tf_listener_;
    tf::StampedTransform camera_tf_transform_;

    std::pair<std::string,geometry_msgs::Pose> product_type_pose_;
//    std::vector<geometry_msgs::Pose> empty_vector;
    std::map<std::string, std::vector<geometry_msgs::Pose>> order_update_product_type_pose_;
    std::map<std::string, std::vector<geometry_msgs::Pose>> order_update_copy;

    std::map<std::string, std::vector<geometry_msgs::Pose>> built_kit_product_type_pose_;
    std::map<std::string, std::vector<geometry_msgs::Pose>> parts_to_remove_product_type_pose_;
//    std::map<std::string, std::vector<geometry_msgs::Pose>> parts_to_place_in_other_tray;
    std::map<std::string, std::vector<std::pair<geometry_msgs::Pose , geometry_msgs::Pose> > > parts_to_place_in_other_tray;

//    std::map<std::string, std::vector<geometry_msgs::Pose>> parts_from_belt;
    std::array<std::map<std::string, std::vector<geometry_msgs::Pose>>, 2> parts_from_belt;
    std::array<std::map<std::string, std::vector<geometry_msgs::Pose>>, 2> parts_from_belt_copy;
    std::map<std::string, std::vector<geometry_msgs::Pose>> parts_from_belt_temp;
    std::map<std::string, std::vector<geometry_msgs::Pose>> parts_from_bin_reachable;
    std::map<std::string, std::vector<geometry_msgs::Pose>> parts_from_bin_unreachable;
//    std::unordered_map<std::string, unsigned int> belt_part_counter; // map which calculate # of part_type from belt

    std::map<std::string, std::vector<std::pair<geometry_msgs::Pose , geometry_msgs::Pose> > > parts_back_from_tray;
    std::vector<int> update_store_idxs;

    std::map<std::string, std::vector<geometry_msgs::Pose>> current_tray1_elements;
    std::map<std::string, std::vector<geometry_msgs::Pose>> current_tray2_elements;


    std::map<std::string, std::vector<geometry_msgs::Pose>> current_belt_parts_on_tray1;
    std::map<std::string, std::vector<geometry_msgs::Pose>> current_belt_parts_on_tray2;
    std::pair<std::string, geometry_msgs::Pose> wrong_poses_map;
    std::pair<std::string, geometry_msgs::Pose> wrong_poses_pair_belt;
    std::pair<std::string, geometry_msgs::Pose> wrong_poses_pair_correct;
    std::pair<std::string, geometry_msgs::Pose> wrong_poses_pair_other_tray;

//    std::vector<geometry_msgs::Pose> partsDroppedOnTray1;// Parts dropped on kit
//    std::vector<geometry_msgs::Pose> partsDroppedOnTray2;// Parts dropped on kit

    RobotController* this_arm;
    RobotController* that_arm;

    int agv_id_global;
    int test_counter;
    bool belt_temp_flag;
    bool correct_drop_later;
    bool belt_drop_later;
    bool recorrect_later;
    bool correct_other_tray_later;

    // std::array<bool, 4> tray_1_store_usage;
    // std::array<bool, 4> tray_2_store_usage;
    std::array<std::array<std::string, 4>, 2> tray_store_usage;
    std::map<std::string, std::vector<geometry_msgs::Pose>> OccupiedSpacesOnTray;

    int belt_tray_id;
    int belt_parts_need_num;
    bool sensor_black_out;
    bool sensor_black_out_global;

    std::vector<bool> obejct_size;
    ros::Time current_time;
    ros::Time camera_time_now;
    ros::Time camera_time_prev;
    int value;

    ros::Duration wait_for_coming;
    ros::Duration wait_time_threshold;
    bool arm_grab_belt;
    bool arm_in_middle;
    bool black_out_occured;


public:
    AriacSensorManager();
    ~AriacSensorManager();
    void order_callback(const osrf_gear::Order::ConstPtr &);
    void bb_1_callback(const osrf_gear::Proximity::ConstPtr &);
    void bb_arm1_callback(const osrf_gear::Proximity::ConstPtr &);
    void bb_arm2_callback(const osrf_gear::Proximity::ConstPtr &);
    void bb_coming_callback(const osrf_gear::Proximity::ConstPtr &);

    void lc_belt_callback(const osrf_gear::LogicalCameraImage::ConstPtr &);    
    void qc_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
    void qc_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr &);

    void lc_bin_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    void lc_bin_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    void lc_bin_3_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    void lc_bin_4_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    void lc_bin_5_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    void lc_bin_6_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);

    void lc_agv_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    void lc_agv_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);

    void SensorBlackOutcallback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);

    void BuildProductFrames(int camera_id);
    geometry_msgs::Pose GetPartPose(const std::string& src_frame,
                                                        const std::string& target_frame);
//    bool PickAndPlace(const std::pair<std::string,geometry_msgs::Pose> product_type_pose, int agv_id, RobotController& arm);
    std::string GetProductFrame(std::string product_type);
    void ExecuteOrder();
    void UpdateKit(int agv_id);
    void PickAndThrow(geometry_msgs::Pose part_pose, std::string product_type, RobotController& arm);
    void CorrectPose(geometry_msgs::Pose current_pose, geometry_msgs::Pose updated_pose,
            std::string product_type, int agv_id, RobotController& arm, bool flipCheckReq);
    bool QualityCheck(int agv_id);
    void buildUpdatedKitMap();
    void WhatToRemove();
    void WhatToModify();
    void WhatToAdd();

    void removeParts(int agv_id, RobotController& arm);
    void modifyPose(int agv_id);
    void addParts(int agv_id);

    void ReExecute(int agv_id);
    void SubmitAGV(int num);

    geometry_msgs::Pose kitToWorld(geometry_msgs::Pose part_pose, int agv_id);
//    bool PickAndPlace(const std::pair<std::string, geometry_msgs::Pose>, int agv_id, RobotController& arm);

    bool FlipPart(RobotController* owner, RobotController* helper, geometry_msgs::Pose railDropPickPose);
    bool PickAndPlace(const std::pair<std::string, geometry_msgs::Pose> product_type_pose,
                                          int agv_id, RobotController& arm);
    void SegregateParts(const std::pair<std::string, geometry_msgs::Pose> type_pose_, int agv_id);
//    void PickAndPlaceFromBelt(geometry_msgs::Pose updated_pose,
//            std::string product_type, int agv_id, int incoming_part_counter, RobotController& arm);
    void  PutPartsIntoOtherTray(geometry_msgs::Pose pick_pose, geometry_msgs::Pose drop_pose, std::string product_type,
            int flag, RobotController& arm_x, RobotController& arm_y, bool flipCheckReq);
    void  RemoveBeltParts();
    void  AddBeltParts();
    std::map<std::string, std::vector<geometry_msgs::Pose>> getCurrentParts(int tray_id);
    std::pair<bool ,geometry_msgs::Pose> VerifyDropPose(geometry_msgs::Pose drop_pose,
            std::string part_type, std::map<std::string, std::vector<geometry_msgs::Pose>>& previous_tray_elements, int tray_id);
    std::pair<bool ,geometry_msgs::Pose> VerifyDropPose(std::pair<std::string, geometry_msgs::Pose> wrong_poses_map,
            std::map<std::string, std::vector<geometry_msgs::Pose>>& previous_tray_elements, int tray_id);

    bool comparePose(geometry_msgs::Pose Pose1, geometry_msgs::Pose Pose2);
    //    void grab_bin1(const osrf_gear::LogicalCameraImage::ConstPtr&);
//    void lc_agv_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr &);
//    void grab_gear();

    bool init_, everything_ready;

    void PickAndPlaceFromBelt_test(std::string, int, int, RobotController&, bool sensor_black_out_=false);
    void SegregateBeltParts(const std::pair<std::string, geometry_msgs::Pose>, int);
    void BuildKitBeltParts(bool same_tray, geometry_msgs::Pose pick_pose,
                                               geometry_msgs::Pose drop_pose, std::string product_type,
                                               RobotController& arm_x, RobotController& arm_y, int);

    geometry_msgs::Pose TransformPoses(geometry_msgs::Pose part_pose, std::string src_coord,
                                                           const std::string& trg_coord);

    void CheckQualityRemoveNAdd(int tray_id);

//    std::string getPartTypeFromQC(const osrf_gear::LogicalCameraImage&, std::string, geometry_msgs::Pose);

};

