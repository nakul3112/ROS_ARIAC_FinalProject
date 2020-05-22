#include "AriacSensorManager.h"
#include <osrf_gear/AGVControl.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

using namespace std;

AriacSensorManager::AriacSensorManager() :
        task {},
        arm1 {"arm1"},
        arm2 {"arm2"},
        tfListener(tfBuffer),
        parts_from_belt {},
        parts_from_belt_temp {},
        parts_from_bin_reachable {},
        parts_from_bin_unreachable {}
{
    orders_sub = sensor_nh_.subscribe("/ariac/orders", 10,
                                      & AriacSensorManager::order_callback, this);

    lc_bin_1_sub = sensor_nh_.subscribe("/ariac/lc_bin_1", 10,
                                        & AriacSensorManager::lc_bin_1_callback, this);

    lc_bin_2_sub = sensor_nh_.subscribe("/ariac/lc_bin_2", 10,
                                        &AriacSensorManager::lc_bin_2_callback, this);
    lc_bin_3_sub = sensor_nh_.subscribe("/ariac/lc_bin_3", 10,
                                        &AriacSensorManager::lc_bin_3_callback, this);

    lc_bin_4_sub = sensor_nh_.subscribe("/ariac/lc_bin_4", 10,
                                        &AriacSensorManager::lc_bin_4_callback, this);

    lc_bin_5_sub = sensor_nh_.subscribe("/ariac/lc_bin_5", 10,
                                        &AriacSensorManager::lc_bin_5_callback, this);

    lc_bin_6_sub = sensor_nh_.subscribe("/ariac/lc_bin_6", 10,
                                        &AriacSensorManager::lc_bin_6_callback, this);

    sensor_balckout_sub = sensor_nh_.subscribe("/ariac/lc_bin_6", 10,
                                           &AriacSensorManager::SensorBlackOutcallback, this);

    lc_agv_1_sub = sensor_nh_.subscribe("/ariac/lc_agv_1", 10,
                                        &AriacSensorManager::lc_agv_1_callback, this);

    lc_agv_2_sub = sensor_nh_.subscribe("/ariac/lc_agv_2", 10,
                                        &AriacSensorManager::lc_agv_2_callback, this);

    qc_1_sub = sensor_nh_.subscribe("/ariac/quality_control_sensor_1", 10,
                                    & AriacSensorManager::qc_1_callback, this);

    qc_2_sub = sensor_nh_.subscribe("/ariac/quality_control_sensor_2", 10,
                                    & AriacSensorManager::qc_2_callback, this);

    bb_1_sub = sensor_nh_.subscribe("/ariac/break_beam_1_change", 10,
                                    & AriacSensorManager::bb_1_callback, this);
    
    bb_coming_sub = sensor_nh_.subscribe("/ariac/break_beam_coming", 10,
                                    & AriacSensorManager::bb_coming_callback, this);
    
    init_ = false;
    cam_1_ = false;
    cam_2_ = false;
    cam_3_ = false;
    cam_4_ = false;
    cam_5_ = false;
    cam_6_ = false;
    Flag_updateKit = false;
    belt_temp_flag = false;
    sensor_black_out_global = false;
    correct_drop_later = false;
    belt_drop_later = false;
    recorrect_later = false;
    correct_other_tray_later = false;

    test_counter = 0;

    NumPartsToRemove = 0;
    NumPartsToModify = 0;
    NumPartsToAdd = 0;

    camera1_frame_counter_ = 1;
    camera2_frame_counter_ = 1;
    camera3_frame_counter_ = 1;
    camera4_frame_counter_ = 1;
    camera5_frame_counter_ = 1;
    camera6_frame_counter_ = 1;

    order_counter = 0;

    order_number = 0;
    int counter = 0;

    qc_1_redFlag = false;
    qc_2_redFlag = false;
    order_receiving_flag = false;
    arm1_busy = false;
    arm2_busy = false;

    everything_ready = false;

    array<string, 4> tray_1_store_usage = {};
    array<string, 4> tray_2_store_usage = {};
    tray_store_usage = {tray_1_store_usage, tray_2_store_usage};
    belt_parts_need_num = 0;

    sensor_black_out = false;
    obejct_size = {};

    arm_grab_belt = true;
    arm_in_middle = false;
    wait_time_threshold = ros::Duration(60.0);

//    above_rail_center.position.x = 0.3;
//    above_rail_center.position.y = 0;
//    above_rail_center.position.z = 0.9 + 0.075; // check if 0.075 makes it faster and won't crash on the rail
}

AriacSensorManager::~AriacSensorManager() {}

void AriacSensorManager::order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
    ros::AsyncSpinner spinner(0);
    spinner.start();

    ROS_INFO_STREAM("[AriacSensorManager][order_callback]:Received order:\n" << *order_msg);
    received_orders_.push_back(*order_msg);
    // setDesiredParts();
    order_receiving_flag = true;
    order_counter += 1;

    if (order_counter >1) {
        Flag_updateKit = true;
        ROS_INFO_STREAM("[AriacSensorManager][order_callback]:Flag Update Set to true:\n" << Flag_updateKit);
        order_counter = 0;
    }else{
        ExecuteOrder();
    }
}

void AriacSensorManager::buildUpdatedKitMap(){
    ROS_INFO_STREAM("[buildUpdatedKitMap]:updated desired parts");
    auto updated_order = received_orders_.back();
    auto order_id = updated_order.order_id;
    auto shipments = updated_order.shipments;
    for (const auto &shipment: shipments){
        auto shipment_type = shipment.shipment_type;
        auto products = shipment.products;
        ROS_INFO_STREAM("Order ID: " << order_id);
        ROS_INFO_STREAM("Shipment Type: " << shipment_type);
        for (const auto &product: products){
            order_update_product_type_pose_[product.type].emplace_back(product.pose);
        }
    }

}

void AriacSensorManager::lc_bin_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
//    ros::AsyncSpinner spinner(0);
//    spinner.start();

    if (init_|| cam_1_ == true) return;
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 1: '" << image_msg->models.size() << "' objects.");
    if (image_msg->models.size() == 0)
        ROS_ERROR_STREAM("Logical Camera 1 does not see anything");

    current_parts_1_ = *image_msg;
    this->BuildProductFrames(1);
}

void AriacSensorManager::lc_bin_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
//    ros::AsyncSpinner spinner(0);
//    spinner.start();

    if (init_ || cam_2_ == true) return;

    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 2: '" << image_msg->models.size() << "' objects.");
    if (image_msg->models.size() == 0)
        ROS_ERROR_STREAM("Logical Camera 2 does not see anything");

    current_parts_2_ = *image_msg;
    this->BuildProductFrames(2);
}

void AriacSensorManager::lc_bin_3_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
//    ros::AsyncSpinner spinner(0);
//    spinner.start();

    if (init_ || cam_3_ == true) return;
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 3: '" << image_msg->models.size() << "' objects.");
    if (image_msg->models.size() == 0)
        ROS_ERROR_STREAM("Logical Camera 3 does not see anything");

    current_parts_3_ = *image_msg;
    this->BuildProductFrames(3);
}

void AriacSensorManager::lc_bin_4_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
//    ros::AsyncSpinner spinner(0);
//    spinner.start();

    if (init_ || cam_4_ == true) return;
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 4: '" << image_msg->models.size() << "' objects.");

    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 4: message " << *image_msg << "' objects.");

    if (image_msg->models.size() == 0)
        ROS_ERROR_STREAM("Logical Camera 4 does not see anything");



    current_parts_4_ = *image_msg;
    this->BuildProductFrames(4);
}

void AriacSensorManager::lc_bin_5_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
//    ros::AsyncSpinner spinner(0);
//    spinner.start();

    if (init_ || cam_5_ == true) return;
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 5: '" << image_msg->models.size() << "' objects.");
    if (image_msg->models.size() == 0)
        ROS_ERROR_STREAM("Logical Camera 5 does not see anything");

    current_parts_5_ = *image_msg;
    this->BuildProductFrames(5);
}

void AriacSensorManager::lc_bin_6_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
//    ros::AsyncSpinner spinner(0);
//    spinner.start();

    if (init_ || cam_6_ == true) return;
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 6: '" << image_msg->models.size() << "' objects.");
    if (image_msg->models.size() == 0)
        ROS_ERROR_STREAM("Logical Camera 6 does not see anything");

    current_parts_6_ = *image_msg;
    this->BuildProductFrames(6);
}

void AriacSensorManager::lc_belt_callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){

    ros::AsyncSpinner spinner(0);
    spinner.start();

    if (image_msg->models.size() == 0) return;

    auto imageMessage = *image_msg ;

//    ros::AsyncSpinner spinner(0);
//    spinner.start();

    ros::Duration timeout(0.2);

    std::string part_type = imageMessage.models[0].type;
    ROS_INFO_STREAM("[lc_belt_callback]: incoming part type = " << part_type);
    // find if this type of part has passed by before
    if (belt_part_counter.find(part_type) == belt_part_counter.end()){
        belt_part_counter.insert(make_pair(part_type, 0));
    }
    else{
        belt_part_counter[part_type]++;
    }

    std::pair<std::string, int> part_pair;
    part_pair = {part_type, belt_part_counter[part_type]};
    incoming_partQ.push(part_pair);

    lc_belt_sub.shutdown();
}

void AriacSensorManager::lc_agv_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){

    if (image_msg->models.size() == 0)
        return ;
    current_parts_agv1_ = *image_msg;

}

void AriacSensorManager::lc_agv_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){

    if (image_msg->models.size() == 0)
        return ;

    current_parts_agv2_ = *image_msg;

}

void AriacSensorManager::bb_1_callback(const osrf_gear::Proximity::ConstPtr & msg) {
    ros::AsyncSpinner spinner(0);
    spinner.start();
    if (msg->object_detected){
        lc_belt_sub = sensor_nh_.subscribe("/ariac/lc_belt", 10,
                                           & AriacSensorManager::lc_belt_callback, this);
    }
}

//void AriacSensorManager::bb_arm2_callback(const osrf_gear::Proximity::ConstPtr & msg) {
//    ros::AsyncSpinner spinner(0);
//    spinner.start();
//
//    if (msg->object_detected && !arm2.Busy) {
//
//        auto incoming_part = incoming_partQ.front();
//        incoming_partQ.pop();
//
//        auto incoming_part_type = incoming_part.first;
//        auto incoming_part_counter = incoming_part.second;
//
//        ROS_INFO_STREAM("[bb_arm2_callback]: Break beam triggered");
//
//        // if the incoming_part_type exists in the map then do PickAndPlaceFromBelt
//        if (parts_from_belt_temp.find(incoming_part_type) != parts_from_belt_temp.end()) {
//            auto vect = parts_from_belt_temp[incoming_part_type];
//            geometry_msgs::Pose updated_pose = vect.back();
//            PickAndPlaceFromBelt(updated_pose, incoming_part_type, agv_id_global, incoming_part_counter, arm2);
//        }
//    }
//}

//void AriacSensorManager::bb_arm1_callback(const osrf_gear::Proximity::ConstPtr & msg) {
//    ros::AsyncSpinner spinner(0);
//    spinner.start();
//
//    if (msg->object_detected && !arm1.Busy) {
//
//        auto incoming_part = incoming_partQ.front();
//        incoming_partQ.pop();
//
//        auto incoming_part_type = incoming_part.first;
//        auto incoming_part_counter = incoming_part.second;
//
//        ROS_INFO_STREAM("[bb_arm1_callback]: Break beam triggered");
//
//        // if the incoming_part_type exists in the map then do PickAndPlaceFromBelt
//        if (parts_from_belt_temp.find(incoming_part_type) != parts_from_belt_temp.end()){
//            auto vect = parts_from_belt_temp[incoming_part_type];
//            geometry_msgs::Pose updated_pose = vect.back();
//            PickAndPlaceFromBelt(updated_pose, incoming_part_type, agv_id_global, incoming_part_counter, arm1);
//        }
//    }
//}

//void AriacSensorManager::PickAndPlaceFromBelt(geometry_msgs::Pose updated_pose,
//                                     std::string product_type, int agv_id, int incoming_part_counter, RobotController& arm){
//
//    arm.Busy = true;
//    int tray_id;
//    if (agv_id == 1){
//        tray_id = 2;
//    }else{
//        tray_id = 1;
//    }
//
//    arm.SendRobotTo(arm.conveyer_pose);
//
//    ROS_INFO_STREAM("[AriacSensorManager]:[PickAndPlaceFromBelt]:Inside PickAndPlaceFromBelt ...");
//
//    geometry_msgs::Pose part_pose;
//
//    part_pose = arm.belt_pickup_pose;
//
//    if (product_type == "pulley_part")
//        part_pose.position.z += 0.062;
//
//    if (product_type == "disk_part")
//        part_pose.position.z += 0.01;
//
//    //--task the robot to pick up this part
//    bool failed_pick = arm.PickPart(part_pose);
////    ros::Duration(0.5).sleep();
//
//    arm.SendRobotTo(arm.home_joint_pose_1);
//
//    // Function to get the drop pose in world coordinates
//    geometry_msgs::Pose drop_pose = kitToWorld(updated_pose, tray_id);
//    ROS_INFO_STREAM("[AriacSensorManager]:[PickAndPlaceFromBelt]: Drop Pose : " << drop_pose);
//
//    drop_pose.position.z += 0.05;
//    auto result = arm.DropPart(drop_pose);
//    arm.SendRobotTo(arm.home_joint_pose_1);
//
//    // --- Check if the drop was intentional or not ----- //
//    // Once the part is placed, Check if the part is place in the correct drop pose, If not, it's a intentional drop
//    std::pair<bool ,geometry_msgs::Pose> status;
//    if(tray_id == 1){
//        status = VerifyDropPose(updated_pose, product_type, current_belt_parts_on_tray1, tray_id);
//    }else {
//        status = VerifyDropPose(updated_pose, product_type, current_belt_parts_on_tray2, tray_id);
//    }
//
//    // We Need to correct the pose
//    if (!status.first){
//        // If the pose is not correct do the CorrectPose();
//        auto wrong_drop_pose = status.second;
//        CorrectPose(wrong_drop_pose, updated_pose, product_type, tray_id, arm);
//    }else{
//        // Do Nothing and Continue
//    }
//
//    // Here we can do the quality check, return type would be a bool, whether good or bad
//    bool quality = QualityCheck(tray_id);
//
//    ROS_INFO_STREAM("[AriacSensorManager]:[PickAndPlaceFromBelt]: Quality is : " << quality);
//
//    // if part is faluty:
//    if (quality){
//        ROS_INFO_STREAM("[AriacSensorManager]:[PickAndPlaceFromBelt]: In side Quality If : " << quality);
//
//        //------pick the part and throw it away using the drop_pose-------
//        PickAndThrow(drop_pose, product_type, arm);
////        ros::Duration(0.5).sleep();
//        //------ Do PickAndPlace again------------------------------------
//        arm.SendRobotTo(arm.home_joint_pose_1);
//
//    }else{
//        arm.SendRobotTo(arm.home_joint_pose_1);
//
//        // Remove part from the map --------------------
//        if (parts_from_belt_temp[product_type].size() > 1){
//            parts_from_belt_temp[product_type].pop_back();
//        }else if(parts_from_belt_temp[product_type].size() == 1){
//            parts_from_belt_temp.erase(product_type);
//            belt_temp_flag = true;
//        }
//        //---------------------------------------------
//
//        std::string product_frame;
//        //--build the frame for added new product to the tray
//        product_frame = "lc_agv_" +  std::to_string(tray_id) + "_" + product_type + "_" +
//                        std::to_string(incoming_part_counter) + "_frame";
//
//        ROS_INFO_STREAM("[AriacSensorManager]:[PickAndPlaceFromBelt]: Frame name named is : " << product_frame);
//        product_frame_list_[product_type].emplace_back(product_frame);
//
//        if (tray_id == 1){
////                partsDroppedOnTray1.push_back(product_type_pose.second);
//            current_belt_parts_on_tray1[product_type].emplace_back(updated_pose);
//        }
//        // -- Adding parts that has been successfully placed on the tray.
//        if (tray_id == 2){
////                partsDroppedOnTray2.push_back(product_type_pose.second);
//            current_belt_parts_on_tray2[product_type].emplace_back(updated_pose);
//        }
//
//    }
//
//    arm.Busy = false;
//    arm.SendRobotTo(arm.conveyer_pose);
//
//}

void AriacSensorManager::SensorBlackOutcallback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){

    ros::AsyncSpinner spinner(0);
    spinner.start();

    camera_time_now = ros::Time::now();

}

void AriacSensorManager::bb_coming_callback(const osrf_gear::Proximity::ConstPtr & msg)
{
    ros::AsyncSpinner spinner(0);
    spinner.start();

    if(camera_time_prev == camera_time_now){
        value++;
    }else{
        //        ROS_WARN_STREAM("[bb_coming_callback] : Not sensor blackOut" );
        camera_time_prev = camera_time_now;
        value = 0;
    }

    if (value >15){
        ROS_ERROR_STREAM_THROTTLE(10, "[bb_coming_callback] : It is a blackOut. Value is:" << value);
        sensor_black_out_global = true;
        black_out_occured = true;
    }else{
//        ROS_ERROR_STREAM("[bb_coming_callback] : Not a blackOut. Value is: " << value);
        sensor_black_out_global = false;
    }

    if (msg->object_detected)
    {
        ROS_WARN_STREAM_THROTTLE(10, "parts_from_belt_temp.empty() = " << parts_from_belt_temp.empty());
        if (!parts_from_belt_temp.empty())
            arm_grab_belt = true;
        else 
            arm_grab_belt = false;
        current_time = ros::Time::now();
    }
    else 
    {
        wait_for_coming = ros::Time::now() - current_time;
        if (wait_for_coming > wait_time_threshold)
            arm_grab_belt = false;
    }
}

void AriacSensorManager::bb_arm1_callback(const osrf_gear::Proximity::ConstPtr & msg)
{
    ros::AsyncSpinner spinner(0);
    spinner.start();

    if (msg->object_detected)
    {
        auto incoming_part = incoming_partQ.front();
        auto incoming_part_type = incoming_part.first;
        auto incoming_part_counter = incoming_part.second;
        ROS_WARN_STREAM("[bb_arm1_callback]: Break beam triggered");
        ROS_INFO_STREAM("[bb_arm1_callback]: parts type from belt: " << incoming_part_type);
        ROS_INFO_STREAM("[bb_arm1_callback]: parts counter from belt: " << incoming_part_counter);
        // ROS_INFO_STREAM("[bb_arm1_callback]: incoming_partQ.size() = " << incoming_partQ.size());
        if (incoming_partQ.empty())
        {
            ROS_WARN_STREAM("[bb_arm1_callback]: incoming_partQ is empty but break beam triggered ---> Sensor Black Out");
            sensor_black_out = true;
            arm_grab_belt = false;
            return;
        }
        sensor_black_out = false;
        incoming_partQ.pop();

        // if
        // 1) it is the prouct we need (erase the belt map temp in QC)
        // 2) there are still space in the closer traybool is_on_pose = arm1.checkRobotPose(arm1.conveyer_pose);
        // bool is_at_pose = arm1.checkRobotPose(arm1.conveyer_pose);
        ROS_WARN_STREAM("[bb_arm1_callback]: arm_grab_belt = " << arm_grab_belt);
        ROS_WARN_STREAM("[bb_arm1_callback]: arm1_busy = " << arm1_busy);
        ROS_WARN_STREAM("[bb_arm1_callback]: arm_in_middle = " << arm_in_middle);

        if (arm_grab_belt && !arm1_busy && !arm_in_middle)
        {
            int tray_id = 0;
            if (agv_id_global == 1)
                tray_id = 2;
            else
                tray_id = 1;

            ROS_INFO_STREAM("[bb_arm1_callback]: condition 1: " << (parts_from_belt_temp.find(incoming_part_type) != parts_from_belt_temp.end()));
            // ROS_INFO_STREAM("[bb_arm1_callback]: condition 2: " <<  !(*(tray_store_usage[tray_id - 1].end() - 1)));
            ROS_INFO_STREAM("[bb_arm1_callback]: condition 2: " <<  (tray_store_usage[tray_id - 1].end() - 1)->empty());

            // if ((parts_from_belt_temp.find(incoming_part_type) != parts_from_belt_temp.end())
            //     && !(*(tray_store_usage[tray_id - 1].end() - 1)))
            if ((parts_from_belt_temp.find(incoming_part_type) != parts_from_belt_temp.end())
                && (tray_store_usage[tray_id - 1].end() - 1)->empty())
            {
                PickAndPlaceFromBelt_test(incoming_part_type, agv_id_global, incoming_part_counter, arm1);
            }
        }
        else
            ROS_WARN_STREAM("[bb_arm1_callback]: Robot is at somewher else... ");
    }
}

void AriacSensorManager::bb_arm2_callback(const osrf_gear::Proximity::ConstPtr & msg)
{
    ros::AsyncSpinner spinner(0);
    spinner.start();

    if (msg->object_detected)
    {
        auto incoming_part = incoming_partQ.front();
        auto incoming_part_type = incoming_part.first;
        auto incoming_part_counter = incoming_part.second;
        ROS_WARN_STREAM("[bb_arm2_callback]: Break beam triggered");
        ROS_INFO_STREAM("[bb_arm2_callback]: parts type from belt: " << incoming_part_type);
        ROS_INFO_STREAM("[bb_arm2_callback]: parts counter from belt: " << incoming_part_counter);
        if (incoming_partQ.empty())
        {
            ROS_WARN_STREAM("[bb_arm2_callback]: incoming_partQ is empty but break beam triggered ---> Sensor Black Out");
            sensor_black_out = true;
            arm_grab_belt = false;
            return;
        }
        ROS_WARN_STREAM("[bb_arm2_callback]: Passed blackout testing ");

        sensor_black_out = false;
        incoming_partQ.pop();

        // bool is_at_pose = arm2.checkRobotPose(arm2.conveyer_pose);
        // ROS_WARN_STREAM("[bb_arm2_callback]: Check arm2 conveyer_pose: " << is_at_pose);
        // if (is_at_pose)
        ROS_WARN_STREAM("[bb_arm2_callback]: arm_grab_belt = " << arm_grab_belt);
        ROS_WARN_STREAM("[bb_arm2_callback]: arm1_busy = " << arm2_busy);
        ROS_WARN_STREAM("[bb_arm2_callback]: arm_in_middle = " << arm_in_middle);

        if (arm_grab_belt && !arm2_busy && !arm_in_middle)
        {
            int tray_id = 0;
            if (agv_id_global == 1)
                tray_id = 2;
            else
                tray_id = 1;

            ROS_INFO_STREAM("[bb_arm2_callback]: condition 1: " << (parts_from_belt_temp.find(incoming_part_type) != parts_from_belt_temp.end()));
            // ROS_INFO_STREAM("[bb_arm2_callback]: condition 2: " <<  !(*(tray_store_usage[tray_id - 1].end() - 1)));
            ROS_INFO_STREAM("[bb_arm1_callback]: condition 2: " <<  (*(tray_store_usage[tray_id - 1].end() - 1)).empty());

            // if ((parts_from_belt_temp.find(incoming_part_type) != parts_from_belt_temp.end())
            //     && !(*(tray_store_usage[tray_id - 1].end() - 1)))
            if ((parts_from_belt_temp.find(incoming_part_type) != parts_from_belt_temp.end())
                && (*(tray_store_usage[tray_id - 1].end() - 1)).empty())
            {
                PickAndPlaceFromBelt_test(incoming_part_type, agv_id_global, incoming_part_counter, arm2);
            }
        }
        else
            ROS_WARN_STREAM("[bb_arm2_callback]: Robot is at somewher else... ");
    }
}

//void AriacSensorManager::PickAndPlaceFromBelt(geometry_msgs::Pose updated_pose,
//                                              std::string product_type, int agv_id, int incoming_part_counter, RobotController& arm)
//{
//    arm.Busy = true;
//    int tray_id;
//    if (agv_id == 1){
//        tray_id = 2;
//    }else{
//        tray_id = 1;
//    }
//
//    arm.SendRobotTo(arm.conveyer_pose);
//
//    ROS_INFO_STREAM("[AriacSensorManager]:[PickAndPlaceFromBelt]:Inside PickAndPlaceFromBelt ...");
//
//    geometry_msgs::Pose part_pose;
//
//    part_pose = arm.belt_pickup_pose;
//
//    if (product_type == "pulley_part")
//        part_pose.position.z += 0.062;
//
//    if (product_type == "disk_part")
//        part_pose.position.z += 0.01;
//
//    //--task the robot to pick up this part
//    bool failed_pick = arm.PickPart(part_pose);
////    ros::Duration(0.5).sleep();
//
//    arm.SendRobotTo(arm.home_joint_pose_1);
//
//    // Function to get the drop pose in world coordinates
////    geometry_msgs::Pose drop_pose = kitToWorld(updated_pose, tray_id);
////    ROS_INFO_STREAM("[AriacSensorManager]:[PickAndPlaceFromBelt]: Drop Pose : " << drop_pose);
//
//    geometry_msgs::Pose drop_pose_kit = arm.tray_store_poses[2];
//    geometry_msgs::Pose drop_pose = kitToWorld(drop_pose_kit, tray_id);
//// tray_store_usage[tray_id - 1][i] = true;
//
//    drop_pose.orientation.x = part_pose.orientation.x;
//    drop_pose.orientation.y = part_pose.orientation.y;
//    drop_pose.orientation.z = part_pose.orientation.z;
//    drop_pose.orientation.w = part_pose.orientation.w;
//
//    drop_pose.position.z += 0.05;
//    auto result = arm.DropPart(drop_pose);
//    arm.SendRobotTo(arm.home_joint_pose_1);
//
//    // --- Check if the drop was intentional or not ----- //
//    // Once the part is placed, Check if the part is place in the correct drop pose, If not, it's a intentional drop
//    std::pair<bool ,geometry_msgs::Pose> status;
//    if(tray_id == 1){
//        status = VerifyDropPose(drop_pose_kit, product_type, current_belt_parts_on_tray1, tray_id);
//    }else {
//        status = VerifyDropPose(drop_pose_kit, product_type, current_belt_parts_on_tray2, tray_id);
//    }
//
//    // We Need to correct the pose
//    if (!status.first){
//        // If the pose is not correct do the CorrectPose();
//        auto wrong_drop_pose = status.second;
//        CorrectPose(wrong_drop_pose, drop_pose_kit, product_type, tray_id, arm);
//    }else{
//        // Do Nothing and Continue
//    }
//
//    // Here we can do the quality check, return type would be a bool, whether good or bad
//    bool quality = QualityCheck(tray_id);
//
//    ROS_INFO_STREAM("[AriacSensorManager]:[PickAndPlaceFromBelt]: Quality is : " << quality);
//
//    // if part is faluty:
//    if (quality){
//        ROS_INFO_STREAM("[AriacSensorManager]:[PickAndPlaceFromBelt]: In side Quality If : " << quality);
//
//        //------pick the part and throw it away using the drop_pose-------
//        PickAndThrow(drop_pose, product_type, arm);
////        ros::Duration(0.5).sleep();
//        //------ Do PickAndPlace again------------------------------------
//        arm.SendRobotTo(arm.home_joint_pose_1);
//
//    }else{
//        arm.SendRobotTo(arm.home_joint_pose_1);
//
//        // Remove part from the map --------------------
//        if (parts_from_belt_temp[product_type].size() > 1){
//            parts_from_belt_temp[product_type].pop_back();
//        }else if(parts_from_belt_temp[product_type].size() == 1){
//            parts_from_belt_temp.erase(product_type);
//            belt_temp_flag = true;
//        }
//        //---------------------------------------------
//
//        std::string product_frame;
//        //--build the frame for added new product to the tray
//        product_frame = "lc_agv_" +  std::to_string(tray_id) + "_" + product_type + "_" +
//                        std::to_string(incoming_part_counter) + "_frame";
//
//        ROS_INFO_STREAM("[AriacSensorManager]:[PickAndPlaceFromBelt]: Frame name named is : " << product_frame);
//        product_frame_list_[product_type].emplace_back(product_frame);
//
//        if (tray_id == 1){
////                partsDroppedOnTray1.push_back(product_type_pose.second);
//            current_belt_parts_on_tray1[product_type].emplace_back(drop_pose_kit);
//        }
//        // -- Adding parts that has been successfully placed on the tray.
//        if (tray_id == 2){
////                partsDroppedOnTray2.push_back(product_type_pose.second);
//            current_belt_parts_on_tray2[product_type].emplace_back(drop_pose_kit);
//        }
//
//    }
//
//    arm.Busy = false;
//    arm.SendRobotTo(arm.conveyer_pose);
//
//}

void AriacSensorManager::PickAndPlaceFromBelt_test(std::string product_type,
        int agv_id, int incoming_part_counter, RobotController& arm, bool sensor_black_out_)
{
    int tray_id;
    if (agv_id == 1) {
        arm2_busy = true;
        tray_id = 2;
        belt_tray_id = 2;
    } else {
        arm1_busy = true;
        tray_id = 1;
        belt_tray_id = 1;
    }

     arm.SendRobotTo(arm.conveyer_pose);

    geometry_msgs::Pose part_pose;

    part_pose = arm.belt_pickup_pose;

    if (product_type == "pulley_part")
        part_pose.position.z += 0.062;

    if (product_type == "disk_part")
        part_pose.position.z += 0.01;

    //--task the robot to pick up this part
    bool if_pick = arm.PickPart(part_pose, true);
    if (!if_pick) {
        ROS_WARN_STREAM("[PickAndPlaceFromBelt_test]: fail to pick up from belt, return stand by pose ...");
        arm.SendRobotTo(arm.conveyer_pose);
        arm2_busy = false;
        arm1_busy = false;
        return;
    }
    arm.SendRobotTo(arm.home_joint_pose_1);

    // Function to get the drop pose in world coordinates
    // geometry_msgs::Pose drop_pose = kitToWorld(updated_pose, tray_id);
    geometry_msgs::Pose drop_pose;
    bool belt_successful_drop = false;

    for (int i = 0; i < tray_store_usage[tray_id - 1].size(); ++i) {
        // if (!tray_store_usage[tray_id - 1][i])
        // {
        if (tray_store_usage[tray_id - 1][i].empty())
        {

            geometry_msgs::Pose drop_pose_kit = arm.tray_store_poses[i];
            drop_pose = kitToWorld(drop_pose_kit, tray_id);
            // tray_store_usage[tray_id - 1][i] = true;

            drop_pose.orientation.x = part_pose.orientation.x;
            drop_pose.orientation.y = part_pose.orientation.y;
            drop_pose.orientation.z = part_pose.orientation.z;
            drop_pose.orientation.w = part_pose.orientation.w;

            drop_pose.position.z += 0.4;
            arm.GoToTarget1(drop_pose);


            if (!arm.gripper_state_){
                belt_successful_drop = false;
            }else{
                belt_successful_drop = true;
            }

            arm.GripperToggle(false);

            if (belt_successful_drop){
                ROS_WARN_STREAM("[PickAndPlaceFromBelt_test]: It's a successful drop");
            }else{
                ROS_WARN_STREAM("[PickAndPlaceFromBelt_test]: It's not a successful drop");
            }

            std::string true_type_belt;
            geometry_msgs::Pose true_pose_belt;


            if(!belt_successful_drop || belt_drop_later){

                if(!sensor_black_out_global){
                    // Once the part is placed, Check if the part is place in the correct drop pose, If not, it's a intentional drop
                    std::pair<bool ,geometry_msgs::Pose> drop_status;

                    if (!belt_drop_later){
                        true_type_belt = product_type;
                        true_pose_belt = drop_pose_kit;
                    }else{
                        true_type_belt = wrong_poses_pair_belt.first;
                        true_pose_belt = wrong_poses_pair_belt.second;
                        ROS_INFO_STREAM("[AriacSensorManager]:[PickAndPlace]: Sensor Blackout finished "
                                        "using the wrong_poses_pair_belt : ");
                        //  wrong_poses_pair_belt.erase(true_type_belt);
                        wrong_poses_pair_belt = {};
                    }
                    if(belt_tray_id == 1){
                        drop_status = VerifyDropPose(true_pose_belt, true_type_belt,
                                current_belt_parts_on_tray1, belt_tray_id);
                    }else {
//                    drop_status = VerifyDropPose(product_type_pose.second, product_type, current_tray2_elements, agv_id);
                        drop_status = VerifyDropPose(true_pose_belt, true_type_belt,
                                current_belt_parts_on_tray2, belt_tray_id);
                    }
                    // We Need to correct the pose
                    if (!drop_status.first){
                        // If the pose is not correct do the CorrectPose();
                        auto wrong_drop_pose_belt = drop_status.second;
                        CorrectPose(wrong_drop_pose_belt, true_pose_belt, true_type_belt, belt_tray_id, arm, false);
                    }else{
                        // Do Nothing and Continue
                    }
                    belt_drop_later = false;
                }else{
                    // add the parts to a list whose poses needs to be fixed wrong_poses_pair_belt
                    // wrong_poses_pair_belt has all the parts which are not in their correct poses
                    //std::pair<std::string, geometry_msgs::Pose>
                    ROS_INFO_STREAM("[AriacSensorManager]:[PickAndPlace]: Sensor Blackout taking "
                                    "Place Will save it to wrong_poses_pair_belt : ");
                    wrong_poses_pair_belt.first = product_type;
                    wrong_poses_pair_belt.second = drop_pose_kit;
                    belt_drop_later = true;
                }
            }

            arm.SendRobotTo(arm.conveyer_pose);

            arm2_busy = false;
            arm1_busy = false;

            if (sensor_black_out_) {
                ROS_WARN_STREAM("[PickAndPlaceFromBelt_test]: Sesor black out. No part name be assigned.");
            }

            bool faulty = QualityCheck(tray_id);
            if (!faulty && !sensor_black_out_)
            {
                if (parts_from_belt_temp[product_type].size() > 1) {
                    parts_from_belt_temp[product_type].pop_back();
                }
                else {
                    if (parts_from_belt_temp[product_type].size() == 1)
                        parts_from_belt_temp.erase(product_type);
                }

                std::string product_frame;
                //--build the frame for added new product to the tray
//                product_frame = "lc_agv_" + std::to_string(tray_id) + "_" + product_type + "_" +
//                                std::to_string(incoming_part_counter) + "_frame";

                ROS_INFO_STREAM("[PickAndPlaceFromBelt_test]: Part frame name is: " << product_frame);
//                product_frame_list_[product_type].emplace_back(product_frame);
                tray_store_usage[tray_id - 1][i] = product_type;
            }
            OccupiedSpacesOnTray[product_type].emplace_back(drop_pose_kit);
            if (tray_id == 1){
                current_belt_parts_on_tray1[product_type].emplace_back(drop_pose_kit);
            }
            // -- Adding parts that has been successfully placed on the tray.
            if (tray_id == 2){
                current_belt_parts_on_tray2[product_type].emplace_back(drop_pose_kit);
            }
            break;
        }
    }
    for (int i = 0; i < tray_store_usage[tray_id - 1].size(); ++i) {
        ROS_INFO_STREAM("[PickAndPlaceFromBelt_test]: tray_store_usage[" << tray_id - 1 << "][" << i << "] = " << tray_store_usage[tray_id - 1][i]);
    }
}

void AriacSensorManager::BuildProductFrames(int camera_id){

//    ros::AsyncSpinner spinner(0);
//    spinner.start();

    if (camera_id == 1) {
        for (auto& msg : current_parts_1_.models) {
            //--build the frame for each product
            std::string product_frame = "lc_bin_1_" + msg.type + "_" +
                                        std::to_string(camera1_frame_counter_) + "_frame";

            product_frame_list_[msg.type].emplace_back(product_frame);
            camera1_frame_counter_++;
        }
        cam_1_ = true;
    }
    else if (camera_id == 2) {
        for (auto& msg : current_parts_2_.models) {
            //--build the frame for each product
            std::string product_frame = "lc_bin_2_" + msg.type + "_" +
                                        std::to_string(camera2_frame_counter_) + "_frame";

            product_frame_list_[msg.type].emplace_back(product_frame);
            camera2_frame_counter_++;
        }
        cam_2_ = true;
    }
    else if (camera_id == 3) {
        for (auto& msg : current_parts_3_.models) {
            //--build the frame for each product
            std::string product_frame = "lc_bin_3_" + msg.type + "_" +
                                        std::to_string(camera3_frame_counter_) + "_frame";

            product_frame_list_[msg.type].emplace_back(product_frame);
            camera3_frame_counter_++;
        }
        cam_3_ = true;
    }
    else if (camera_id == 4) {
        for (auto& msg : current_parts_4_.models) {
            //--build the frame for each product
            std::string product_frame = "lc_bin_4_" + msg.type + "_" +
                                        std::to_string(camera4_frame_counter_) + "_frame";

            product_frame_list_[msg.type].emplace_back(product_frame);
            camera4_frame_counter_++;
        }
        cam_4_ = true;
    }
    else if (camera_id == 5) {
        for (auto& msg : current_parts_5_.models) {
            //--build the frame for each product
            std::string product_frame = "lc_bin_5_" + msg.type + "_" +
                                        std::to_string(camera5_frame_counter_) + "_frame";

            product_frame_list_[msg.type].emplace_back(product_frame);
            camera5_frame_counter_++;
        }
        cam_5_ = true;
    }
    else if (camera_id == 6) {
        for (auto& msg : current_parts_6_.models) {
            //--build the frame for each product
            std::string product_frame = "lc_bin_6_" + msg.type + "_" +
                                        std::to_string(camera6_frame_counter_) + "_frame";

            product_frame_list_[msg.type].emplace_back(product_frame);
            camera6_frame_counter_++;
        }
        cam_6_ = true;
    }

    if (cam_1_ && cam_2_ && cam_3_ && cam_4_ && cam_5_ && cam_6_) {
        init_ = true;

    }
}

geometry_msgs::Pose AriacSensorManager::GetPartPose(const std::string& src_frame,
                                                    const std::string& target_frame) {
    geometry_msgs::Pose part_pose;

    ROS_INFO_STREAM("Getting part pose...");

    if (init_) {
        camera_tf_listener_.waitForTransform(src_frame, target_frame, ros::Time(0),
                                             ros::Duration(3));
        camera_tf_listener_.lookupTransform(src_frame, target_frame, ros::Time(0),
                                            camera_tf_transform_);

        part_pose.position.x = camera_tf_transform_.getOrigin().x();
        part_pose.position.y = camera_tf_transform_.getOrigin().y();
        part_pose.position.z = camera_tf_transform_.getOrigin().z();

    } else {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        this->BuildProductFrames(1);
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        this->BuildProductFrames(2);
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        this->BuildProductFrames(3);
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        this->BuildProductFrames(4);
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        this->BuildProductFrames(5);
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        this->BuildProductFrames(6);

        part_pose = this->GetPartPose(src_frame, target_frame);
    }

    return part_pose;
}
//void AriacSensorManager::setDesiredParts(){
//    ROS_INFO_STREAM("[setDesiredParts]:Setting desired parts");
//    auto current_order = received_orders_[order_number];
//    auto order_id = current_order.order_id;
//    auto shipments = current_order.shipments;
//    for (const auto &shipment: shipments){
//        auto shipment_type = shipment.shipment_type;
//        auto products = shipment.products;
//        ROS_INFO_STREAM("Order ID: " << order_id);
//        ROS_INFO_STREAM("Shipment Type: " << shipment_type);
//        for (const auto &product: products){
//            desired_parts_info.insert({product.type, product.pose});
//            parts_to_pickup_belt.insert(product.type);
//            ++(task[product.type]); // Adding product name in task variable
//        }
//    }
//
//    ROS_INFO_STREAM("[setDesiredParts]:The current desired_parts are:");
//    for (const auto & part : desired_parts_info){
//        std::cout << part.first << std::endl;
//        std::cout << "Pose:\n";
//        ROS_INFO_STREAM(part.second);
//    }
//
//    ROS_INFO_STREAM("[ASM]:[setDesiredParts]:The Parts_to_pickup_belt set is:");
//    for (auto it = parts_to_pickup_belt.begin(); it != parts_to_pickup_belt.end(); ++it)
//        std::cout << *it << "\n";
//
//    if (!order_id.empty())
//        ++order_number;
//
//    everything_ready = true;
//
//    if (order_number <=1){
//        ExecuteOrder();
//    }
//
//}

std::string AriacSensorManager::GetProductFrame(std::string product_type) {
    //--Grab the last one from the list then remove it
    if (!product_frame_list_.empty()) {
        std::string frame = product_frame_list_[product_type].back();
        ROS_INFO_STREAM("Frame >>>> " << frame);

        if (product_frame_list_[product_type].size() > 1){
            product_frame_list_[product_type].pop_back();
        }else if(product_frame_list_[product_type].size() == 1){
            product_frame_list_.erase(product_type);
        }

        return frame;
    } else {
        ROS_ERROR_STREAM("No product frame found for " << product_type);
        ros::shutdown();
    }
}

geometry_msgs::Pose AriacSensorManager::kitToWorld(geometry_msgs::Pose part_pose, int agv_id){

    std::string tray_ID = "kit_tray_" + std::to_string(agv_id);

    geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;
    StampedPose_in.header.frame_id = tray_ID;
    StampedPose_in.pose = part_pose;


    StampedPose_out = tfBuffer.transform(StampedPose_in, "world");
    geometry_msgs::Pose part_pose_kit = StampedPose_out.pose;

    return part_pose_kit;
}

geometry_msgs::Pose AriacSensorManager::TransformPoses(geometry_msgs::Pose part_pose, std::string src_coord,
        std::string trg_coord){

//    std::string src = src_coord;    // if KitTray1 we give as "kit_tray_1" vice versa
//    std::string trg = trg_coord;   // if world we give as  "world"

    geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;
    StampedPose_in.header.frame_id = src_coord;
    StampedPose_in.pose = part_pose;

    StampedPose_out = tfBuffer.transform(StampedPose_in, trg_coord);
    geometry_msgs::Pose tranformed_pose = StampedPose_out.pose;

    return tranformed_pose;
}

bool AriacSensorManager::QualityCheck(int agv_id){

//    ROS_INFO_STREAM("[AriacSensorManager]:[QualityCheck]: In side QualityCheck : ");

    bool quality_flag = false;
    if (agv_id == 1){
        quality_flag =  qc_1_redFlag;
    }
    else if (agv_id == 2){
        quality_flag =  qc_2_redFlag;
    }
    return quality_flag;
}

void AriacSensorManager::PickAndThrow(geometry_msgs::Pose part_pose, std::string product_type,
        RobotController& arm){

    // takes the part_pose in world coordinates
    ROS_INFO_STREAM("[AriacSensorManager]:[PickAndThrow]:Inside PickAndThrow ...");

    if (product_type == "pulley_part")
        part_pose.position.z += 0.062;
    //--task the robot to pick up this part
    bool failed_pick = arm.PickPart(part_pose);
    ROS_WARN_STREAM("Picking up state " << failed_pick);
    ros::Duration(0.5).sleep();

    while (!failed_pick) {
//        auto part_pose = GetPartPose("/world", product_frame);
        failed_pick = arm.PickPart(part_pose);
    }

    arm.DropPart(arm.throw_away_pose);
//    ros::Duration(1).sleep();
}

void AriacSensorManager::CorrectPose(geometry_msgs::Pose current_pose, geometry_msgs::Pose updated_pose,
        std::string product_type, int agv_id, RobotController& arm, bool flipCheckReq = false){


    ROS_INFO_STREAM("[AriacSensorManager]:[CorrectPose]:Inside CorrectPose ...");

    if(agv_id == 1) {
        this_arm = &arm;
        that_arm = &arm2;
//        Other_tray_ID = 2;
//        This_tray_ID = 1;
        // that_arm->SendRobotTo("linear_arm_actuator_joint", -1.18);
        // that_arm->SendRobotTo(that_arm->home_joint_pose_1);
    }
    else if(agv_id == 2){
        this_arm = &arm2;
        that_arm = &arm;
//        Other_tray_ID = 1;
//        This_tray_ID = 2;
        // that_arm->SendRobotTo("linear_arm_actuator_joint", 1.18);
        // that_arm->SendRobotTo(that_arm->home_joint_pose_1);
    }


    geometry_msgs::Pose part_pose = kitToWorld(current_pose, agv_id);

    if (product_type == "pulley_part")
        part_pose.position.z += 0.062;
    //--task the robot to pick up this part
    bool failed_pick = arm.PickPart(part_pose);
//        ROS_WARN_STREAM("Picking up state " << failed_pick);
//    ros::Duration(0.5).sleep();

    while (!failed_pick) {
//        auto part_pose = GetPartPose("/world", product_frame);
        failed_pick = arm.PickPart(part_pose);
    }
//    ros::Duration(0.5).sleep();
    arm.SendRobotTo(arm.home_joint_pose_1);
//    ros::Duration(1).sleep();

    if (flipCheckReq){
        // do the flipping here
        // ================================== Flipping Section =============================== //
        bool isFlippingRequired{false};

        tf2::Quaternion quat_D;
        tf2::fromMsg(updated_pose.orientation, quat_D);
        double d_R, d_P, d_Y;
        if (product_type == "pulley_part"){

            tf2::Matrix3x3(quat_D).getRPY(d_R, d_P, d_Y);
            ROS_INFO_STREAM("[ASM][PutPartsIntoOtherTray] - RPY: " << d_R << "; "<< d_P << "; " << d_Y);

            ROS_INFO_STREAM("[AriacSensorManager]:[PutPartsIntoOtherTray]:Part successfully picked.");

            // Flip the part if needed
            if((abs(d_R) < 3.16 && abs(d_R) > 3.10) ^ (abs(d_P) < 3.16 && abs(d_P) > 3.10)) { ///
                ROS_WARN_STREAM("[ASM]:[PutPartsIntoOtherTray]: Flipping Required.");
                isFlippingRequired = true;
            }
            else{
                ROS_WARN_STREAM("[ASM]:[PutPartsIntoOtherTray]: Flipping not Required");
                isFlippingRequired = false;
            }


            if (isFlippingRequired) {

                geometry_msgs::Pose above_rail_center;
                above_rail_center.position.x = 0.3;
                above_rail_center.position.y = 0;
                above_rail_center.position.z = 0.9 + 0.075; // check if 0.075 makes it faster and won't crash on the rail
                if (product_type == "pulley_part")
                    above_rail_center.position.z += 0.062; // check if dropping pulley_part on the rail correctly
                above_rail_center.orientation.x = updated_pose.orientation.x;
                above_rail_center.orientation.y = updated_pose.orientation.y;
                above_rail_center.orientation.z = updated_pose.orientation.z;
                above_rail_center.orientation.w = updated_pose.orientation.w;

                arm.SendRobotTo(arm.home_joint_pose_2);
                arm.SendRobotTo(arm.rail_pick_trans_pose);
                this->FlipPart(this_arm, that_arm, above_rail_center);
                arm.SendRobotTo(arm.home_joint_pose_2);
            }
        }
        // -------------------------------- Flip part completed -------------------------- //
    }


    //After picking and before dropping the part take a snapshot of what's on the tray,
    // give that as currrentTrayelements. This is to be done only if there is no sensor blackout

//    bool generate_success = false;
//    std::map<std::string, std::vector<geometry_msgs::Pose>> current_tray_elements;
//
//    if(!sensor_black_out_global){
//        current_tray_elements = getCurrentParts(agv_id);
//        generate_success = true;
//    }

    // Function to get the drop pose in world coordinates
    geometry_msgs::Pose drop_pose = kitToWorld(updated_pose, agv_id);
    ROS_INFO_STREAM("[AriacSensorManager]:[CorrectPose]: Drop Pose : " << drop_pose);

    drop_pose.position.z += 0.05;
    auto result = arm.DropPart(drop_pose);
//    ros::Duration(1.5).sleep();
    arm.SendRobotTo(arm.home_joint_pose_1);
//    ros::Duration(1.5).sleep();
    // Here check if the placed pose is CorrectPose or not

//    std::string true_type;
//    geometry_msgs::Pose true_pose;
//
//    if((!result || recorrect_later) & generate_success){
//
//        if(!sensor_black_out_global){
//            // Once the part is placed, Check if the part is place in the correct drop pose, If not, it's a intentional drop
//            std::pair<bool ,geometry_msgs::Pose> status;
//
//            if (!recorrect_later){
//                true_type = product_type;
//                true_pose = updated_pose;
//                                // KitTrayFrame
//            }else{
//                true_type = wrong_poses_pair_correct.first;
//                true_pose = wrong_poses_pair_correct.second;
//                ROS_INFO_STREAM("[AriacSensorManager]:[PickAndPlace]: Sensor Blackout finished "
//                                "using the wrong_poses_map : ");
//                //  wrong_poses_map.erase(true_type);
//                wrong_poses_pair_correct = {};
//            }
//            status = VerifyDropPose(true_pose, true_type, current_tray_elements, agv_id);
//            // We Need to correct the pose
//            if (!status.first){
//                // If the pose is not correct do the CorrectPose();
//                auto wrong_drop_pose = status.second;
//                CorrectPose(wrong_drop_pose, true_pose, true_type, agv_id, *this_arm);
//            }else{
//                // Do Nothing and Continue
//            }
//            recorrect_later = false;
//        }else{
//            // add the parts to a list whose poses needs to be fixed wrong_poses_map
//            // wrong_poses_map has all the parts which are not in their correct poses
//            //std::pair<std::string, geometry_msgs::Pose>
//            ROS_INFO_STREAM("[AriacSensorManager]:[PickAndPlace]: Sensor Blackout taking "
//                            "Place Will save it to wrong_poses_map : ");
//            wrong_poses_pair_correct.first = product_type;
//            wrong_poses_pair_correct.second = updated_pose;
//            recorrect_later = true;
//        }
//    }

}

std::map<std::string, std::vector<geometry_msgs::Pose>> AriacSensorManager::getCurrentParts(int tray_id){

    // This function get the current parts on the tray
    // and returns a map

    ROS_INFO_STREAM("[AriacSensorManager][getCurrentParts]: Inside getCurrentParts ");

    std::map<std::string, std::vector<geometry_msgs::Pose>> CurrentPartsSeen;

    std::string tray_ID = "kit_tray_" + std::to_string(tray_id);
    std::string camera_ID = "lc_agv_" + std::to_string(tray_id) + "_frame";

    osrf_gear::LogicalCameraImage current_parts_agv;

    if(tray_id == 1){
        current_parts_agv = current_parts_agv1_;
    }
    else{
        current_parts_agv = current_parts_agv2_;
    }

    for (auto& msg : current_parts_agv.models) {
        auto part_type = msg.type;
        auto part_pose = msg.pose;

        geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;
        StampedPose_in.header.frame_id = camera_ID;
        auto partPoseInLC = msg.pose;
        StampedPose_in.pose = partPoseInLC;

        StampedPose_out = tfBuffer.transform(StampedPose_in, tray_ID);
        geometry_msgs::Pose part_pose_kit = StampedPose_out.pose;

        ROS_INFO_STREAM("[AriacSensorManager][getCurrentParts]: part_type in Kit Frame: "<< part_type);
        ROS_INFO_STREAM("[AriacSensorManager][getCurrentParts]: part_pose_kit in Kit Frame: "<< part_pose_kit);

        CurrentPartsSeen[part_type].emplace_back(part_pose_kit);
    }

    return CurrentPartsSeen;

}

bool AriacSensorManager::comparePose(geometry_msgs::Pose Pose1, geometry_msgs::Pose Pose2){

    bool same;
    auto x_diff = Pose1.position.x - Pose2.position.x;
    auto y_diff = Pose1.position.y - Pose2.position.y;

    if ( abs(x_diff) <= 0.05 && abs(y_diff) <= 0.05){
        same = true;
    }else{
        same = false;
    }

    return same;
}

std::pair<bool ,geometry_msgs::Pose> AriacSensorManager::VerifyDropPose(geometry_msgs::Pose drop_pose,
        std::string part_type, std::map<std::string, std::vector<geometry_msgs::Pose>>& previous_tray_elements, int tray_id)
{
    // takes in the part_type and drop_pose where it should actually be dropped

    std::pair<bool ,geometry_msgs::Pose> status;

    ROS_INFO_STREAM("[AriacSensorManager][VerifyDropPose]: Inside VerifyDropPose of tray" << tray_id);

    auto previous_vect = previous_tray_elements[part_type];

    // we need to get poses of all parts that are being seen by the lc_agv
    // get the vector of the part_type
    auto current_map =  getCurrentParts(tray_id);
    auto current_vect = current_map[part_type];

    ROS_INFO_STREAM("[AriacSensorManager][VerifyDropPose]: Current Vector of tray" << tray_id);
    for (auto part_pose : current_vect){
        ROS_INFO_STREAM("[AriacSensorManager][VerifyDropPose]: part_pose in tray" << tray_id << " is:" << part_pose);
    }

    ROS_INFO_STREAM("[AriacSensorManager][VerifyDropPose]: previous_vect of tray" << tray_id);
    for (auto part_pose : previous_vect){
        ROS_INFO_STREAM("[AriacSensorManager][VerifyDropPose]: part_pose in tray" << tray_id << "is :" << part_pose);
    }

    geometry_msgs::Pose new_pose;
    // compare that vector with previous_vect
    bool same;

    if(previous_vect.size() ==0){
        new_pose = current_vect.back();
    }else{
        for (int i=0; i < current_vect.size(); i++){
            for(auto prev_pose : previous_vect){
                same = comparePose(current_vect[i], prev_pose);
                if(same){
                    current_vect.erase(current_vect.begin()+i);
                }
            }
        }
        new_pose = current_vect.back();
    }

    ROS_INFO_STREAM("[AriacSensorManager][VerifyDropPose]: new_pose in tray" << tray_id<< "is :" << new_pose);
    ROS_INFO_STREAM("[AriacSensorManager][VerifyDropPose]: drop_pose in tray" << tray_id<< "is :" << drop_pose);

    same = comparePose(new_pose, drop_pose);
    // checks if the dop pose is correct or not
    if (same){
        // if correct returns same drop pose
        // Part is in the correct drop pose
        status.first = true;
        status.second = drop_pose;
        ROS_INFO_STREAM("[AriacSensorManager][VerifyDropPose]: Status is true in tray" << tray_id);

    }else {
        // else returns the pose where thee part has been dropped
        // Part is in the wrong drop pose
        status.first = false;
        status.second = new_pose;
        ROS_INFO_STREAM("[AriacSensorManager][VerifyDropPose]: Status is false in tray" << tray_id);

    }

    return status;
}

std::pair<bool ,geometry_msgs::Pose> AriacSensorManager::VerifyDropPose(std::pair<std::string, geometry_msgs::Pose>
        wrong_poses_map, std::map<std::string, std::vector<geometry_msgs::Pose>>& previous_tray_elements, int tray_id)
{
    // takes in the part_type and wrong_poses_map where it should actually be dropped
    // wrong_poses_map has the actual drop poses of the parts which are in the wrong poses


    std::pair<bool ,geometry_msgs::Pose> status;

    ROS_INFO_STREAM("[AriacSensorManager][VerifyDropPose]: Inside VerifyDropPose of tray" << tray_id);

    auto part_type = wrong_poses_map.first;
    auto drop_pose = wrong_poses_map.second;

    auto previous_vect = previous_tray_elements[part_type];

    // we need to get poses of all parts that are being seen by the lc_agv
    // get the vector of the part_type
    auto current_map =  getCurrentParts(tray_id);
    auto current_vect = current_map[part_type];

    ROS_INFO_STREAM("[AriacSensorManager][VerifyDropPose]: Current Vector of tray" << tray_id);
    for (auto part_pose : current_vect){
        ROS_INFO_STREAM("[AriacSensorManager][VerifyDropPose]: part_pose in tray" << tray_id << " is:" << part_pose);
    }

    ROS_INFO_STREAM("[AriacSensorManager][VerifyDropPose]: previous_vect of tray" << tray_id);
    for (auto part_pose : previous_vect){
        ROS_INFO_STREAM("[AriacSensorManager][VerifyDropPose]: part_pose in tray" << tray_id << "is :" << part_pose);
    }

    geometry_msgs::Pose new_pose;
    // compare that vector with previous_vect
    bool same;

    if(previous_vect.size() ==0){
        new_pose = current_vect.back();
    }else{
        for (int i=0; i < current_vect.size(); i++){
            for(auto prev_pose : previous_vect){
                same = comparePose(current_vect[i], prev_pose);
                if(same){
                    current_vect.erase(current_vect.begin()+i);
                }
            }
        }
        new_pose = current_vect.back();
    }

    ROS_INFO_STREAM("[AriacSensorManager][VerifyDropPose]: new_pose in tray" << tray_id<< "is :" << new_pose);
    ROS_INFO_STREAM("[AriacSensorManager][VerifyDropPose]: drop_pose in tray" << tray_id<< "is :" << drop_pose);

    same = comparePose(new_pose, drop_pose);
    // checks if the dop pose is correct or not
    if (same){
        // if correct returns same drop pose
        // Part is in the correct drop pose
        status.first = true;
        status.second = drop_pose;
        ROS_INFO_STREAM("[AriacSensorManager][VerifyDropPose]: Status is true in tray" << tray_id);

    }else {
        // else returns the pose where thee part has been dropped
        // Part is in the wrong drop pose
        status.first = false;
        status.second = new_pose;
        ROS_INFO_STREAM("[AriacSensorManager][VerifyDropPose]: Status is false in tray" << tray_id);

    }

    return status;
}

bool AriacSensorManager::FlipPart(RobotController* owner, RobotController* helper, geometry_msgs::Pose railDropPickPose){
    ROS_INFO_STREAM("[ASM]:[FlipPart]: Function Called");
    bool result{false};
    auto railLeft_ = this_arm->RailLeft;
    auto railRight_ = this_arm->RailRight;
    railLeft_["linear_arm_actuator_joint"] = 0.0;
    railRight_["linear_arm_actuator_joint"] = -0.3;

    //======== If arm1 has the part in hand initially i.e. this_arm = 2; that_arm = 1; =====//
    if (owner->id == "arm1") {

        // Send arm1 to transition pose.
        owner->SendRobotTo(railLeft_);
        owner->SendRobotTo(owner->RailLeft);
        ROS_INFO_STREAM("[ASM]:[FlipPart]: Arm1 Transition Pose reached");

        // Send arm2 to transition pose.
        helper->SendRobotTo(railRight_);
        helper->SendRobotTo(helper->RailRight);
        ROS_INFO_STREAM("[ASM]:[FlipPart]: Arm2 Transition Pose reached");

        // Exchange the part.
        helper->GripperToggle(true);
        owner->GripperToggle(false);
        ros::Duration(0.5).sleep();

        // Arm1 moves away to give space for dropping operation
        owner->SendRobotTo(owner->rail_pick_trans_pose);

        // Arm2 drops the part
        railDropPickPose.position.z = 0.9 + 0.2;
        helper->SendRobotTo(helper->rail_pick_trans_pose);
        result = helper->DropPart(railDropPickPose);


        // Arm2 moves away.
        helper->SendRobotTo(helper->rail_pick_trans_pose);
//        helper->SendRobotTo("linear_arm_actuator_joint",1.00);
        helper->SendRobotTo(helper->home_joint_pose_2); // Towards the tray
        ROS_INFO_STREAM("[ASM][FlipPart]: Arm 2 moved away");

        // Arm1 picks up the part.
        owner->SendRobotTo(owner->rail_pick_trans_pose);
        railDropPickPose.position.z = 0.9 + 0.1;
        owner->PickPart(railDropPickPose);
        ros::Duration(0.5).sleep();
        ROS_INFO_STREAM("[ASM][FlipPart]: Arm1 picked Part");

        owner->SendRobotTo(owner->rail_pick_trans_pose);
    }

    //======== If arm2 has the part in hand initially i.e. this_arm = 2; that_arm = 1; =====//
    if (owner->id == "arm2") {

        // Send arm2 to transition pose.
        owner->SendRobotTo(railRight_);
        owner->SendRobotTo(owner->RailRight);
        ROS_INFO_STREAM("[ASM]:[FlipPart]: Arm1 Transition Pose reached");

        // Send arm1 to transition pose.
        helper->SendRobotTo(railLeft_);
        helper->SendRobotTo(helper->RailLeft);
        ROS_INFO_STREAM("[ASM]:[FlipPart]: Arm2 Transition Pose reached");

        // Exchange the part.
        helper->GripperToggle(true);
        owner->GripperToggle(false);
//        ros::Duration(0.5).sleep();

        // Arm2 moves away to give space for dropping operation
        owner->SendRobotTo(owner->rail_pick_trans_pose);

        // Arm1 drops the part.
        railDropPickPose.position.z = 0.9 + 0.2;
        helper->SendRobotTo(helper->rail_pick_trans_pose); // -- edited: 4/30
//        ROS_WARN_STREAM("[ASM]:[FlipPart]: Sending Arm1 to this pose: " << railDropPickPose);
        helper->GoToTarget1(railDropPickPose);
        helper->GripperToggle(false);
        ROS_INFO_STREAM("[ASM][FlipPart]: Arm1 dropped the part.");

        // Arm1 moves away.
        helper->SendRobotTo(railLeft_);
//        helper->SendRobotTo("linear_arm_actuator_joint",1.00);
        helper->SendRobotTo(helper->home_joint_pose_1); // Towards the tray
        ROS_INFO_STREAM("[ASM][FlipPart]: Arm 1 moved away");    //Perfect till here

        // Arm2 picks up the part.
        owner->SendRobotTo(owner->rail_pick_trans_pose);
        railDropPickPose.position.z = 0.9 + 0.1;
        owner->PickPart(railDropPickPose);
        ROS_INFO_STREAM("[ASM][FlipPart]: Arm2 picked Part");

        // Send Arm2 to home pose
        owner->SendRobotTo(owner->rail_pick_trans_pose);
    }
    return !result;
}

bool AriacSensorManager::PickAndPlace(const std::pair<std::string, geometry_msgs::Pose> product_type_pose,
                                      int agv_id, RobotController& arm)
{
    if(init_){
        std::vector<int> unreachable {};
        if(agv_id == 1) {
            this_arm = &arm;
            that_arm = &arm2;
            unreachable = {5,6,2};
        }
        else {
            this_arm = &arm;
            that_arm = &arm1;
            unreachable = {1,2,1};
        }

        std::string product_type = product_type_pose.first;
        ROS_WARN_STREAM("[AriacSensorManager]:[PickAndPlace]:Product type >>>> " << product_type);
        std::string product_frame = this->GetProductFrame(product_type);

        ROS_WARN_STREAM("Product frame >>>> " << product_frame);
        auto part_pose = GetPartPose("/world", product_frame);

        geometry_msgs::Pose above_rail_center;
        // place the desired part on the rail
        above_rail_center.position.x = 0.3; // edit: 0.3
        above_rail_center.position.y = 0;
        above_rail_center.position.z = 0.9 + 0.075; // check if 0.075 makes it faster and won't crash on the rail
        above_rail_center.orientation.x = part_pose.orientation.x;
        above_rail_center.orientation.y = part_pose.orientation.y;
        above_rail_center.orientation.z = part_pose.orientation.z;
        above_rail_center.orientation.w = part_pose.orientation.w;

        // if the product frame name contains substring indicating it is unreachable
        if ((product_frame.find("lc_bin_" + std::to_string(unreachable[0])) != -1) ||
            (product_frame.find("lc_bin_" + std::to_string(unreachable[1])) != -1) ||
            (product_frame.find("lc_agv_" + std::to_string(unreachable[2])) != -1))
        {
            ROS_INFO_STREAM("Current process part is in unreachable bin!");
            that_arm->SendRobotTo(that_arm->home_joint_pose_1);
            that_arm->SendRobotTo(that_arm->home_joint_pose_2);
            bool failed_pick = that_arm->PickPart(part_pose);
            // ros::Duration(0.5).sleep();

            while (!failed_pick) {
                failed_pick = that_arm->PickPart(part_pose);
            }

            that_arm->SendRobotTo(that_arm->home_joint_pose_2);

            if (product_type == "pulley_part")
                above_rail_center.position.z += 0.062; // check if dropping pulley_part on the rail correctly

            ROS_INFO_STREAM("[AriacSensorManager]:[PickAndPlace]: Drop Pose : " << above_rail_center);

            that_arm->SendRobotTo(that_arm->rail_pick_trans_pose);

            auto result = that_arm->DropPart(above_rail_center);

            that_arm->SendRobotTo(that_arm->home_joint_pose_2);
            // ros::Duration(0.5).sleep();
            that_arm->SendRobotTo(that_arm->home_joint_pose_1);
            part_pose = above_rail_center;
            part_pose.position.z -= 0.015;

            this_arm->SendRobotTo(this_arm->home_joint_pose_2);

            this_arm->SendRobotTo(this_arm->rail_pick_trans_pose); // go to the transtion pose for picking part on the rail
        }

        if (product_type == "pulley_part")
            part_pose.position.z += 0.062;

        bool failed_pick = this_arm->PickPart(part_pose);
        // ros::Duration(0.5).sleep();

        while (!failed_pick) {
            failed_pick = this_arm->PickPart(part_pose);
        }


        // ================================== Flipping Section =============================== //
        bool isFlippingRequired{false};

        tf2::Quaternion quat_D;
        tf2::fromMsg(product_type_pose.second.orientation, quat_D);
        double d_R, d_P, d_Y;
        if (product_type == "pulley_part"){

            tf2::Matrix3x3(quat_D).getRPY(d_R, d_P, d_Y);
            ROS_INFO_STREAM("[ASM][PickAndPlace] - RPY: " << d_R << "; "<< d_P << "; " << d_Y);

            ROS_INFO_STREAM("[AriacSensorManager]:[PickAndPlace]:Part successfully picked.");

            // Flip the part if needed
            if((abs(d_R) < 3.16 && abs(d_R) > 3.10) ^ (abs(d_P) < 3.16 && abs(d_P) > 3.10)) { ///
                ROS_WARN_STREAM("[ASM]:[PickAndPlace]: Flipping Required.");
                isFlippingRequired = true;
            }
            else{
                ROS_WARN_STREAM("[ASM]:[PickAndPlace]: Flipping not Required");
                isFlippingRequired = false;
            }

            if (isFlippingRequired){
                while (arm_grab_belt) {
                    ROS_INFO_STREAM_THROTTLE(10, "[PickAndPlace]: Inside Flipping Section. Waiting arm to pick up from the belt inside Flipping Part");
                }
                arm_in_middle = true;
                this_arm->SendRobotTo(this_arm->home_joint_pose_2);
                this_arm->SendRobotTo(this_arm->rail_pick_trans_pose);
//                ROS_WARN_STREAM("[ASM]:[PickAndPlace]: Position passed to Flip Part is: " << above_rail_center);
                this->FlipPart(this_arm, that_arm, above_rail_center);
                arm_in_middle = false;
                isFlippingRequired = false;
            }
        }
        // -------------------------------- Flip part completed -------------------------- //

        // ros::Duration(0.5).sleep();
        ROS_WARN_STREAM("[AriacSensorManager]:[PickAndPlace]:Checking " );
        this_arm->SendRobotTo(this_arm->home_joint_pose_2);
        this_arm->RobotGoHome();
        // ros::Duration(1).sleep();

        // Function to get the drop pose in world coordinates
        geometry_msgs::Pose drop_pose = kitToWorld(product_type_pose.second, agv_id);
//        geometry_msgs::Pose drop_pose_temp = drop_pose;
        ROS_INFO_STREAM("[AriacSensorManager]:[PickAndPlace]: Drop Pose : " << drop_pose);
        drop_pose.position.z += 0.05;

        // before doing droppart we need to check if the gripper status turns false.
        auto result = this_arm->DropPart(drop_pose);

        ROS_INFO_STREAM("[AriacSensorManager]:[PickAndPlace]: Drop Part result : " << result);


        drop_pose.position.z -= 0.05;
        // ros::Duration(0.5).sleep();
        this_arm->RobotGoHome();

        // --- Check if the drop was intentional or not ----- //

        //if the 'result' from DropPart() function is 0, then the drop was unsuccessful, it is an intentional drop
        // that was given in the .yaml file and we need to do pose verification,
        // if the  'result' from DropPart() function is 1, the dropping part is successful and we do not have to
        // verify the pose

        // if there is a sensor blackout and the result is 0 ...... just add to a list and correct later
        // if there is no sensor blackout and the result is 0 ...... correct the pose

        // if there is a sensor blackout and the result is 1 ...... Continue
        // if there is no sensor blackout and the result is 1 ...... Continue

        std::string true_type;
        geometry_msgs::Pose true_pose;

        if(!result || correct_drop_later){

            if(!sensor_black_out_global){
                // Once the part is placed, Check if the part is place in the correct drop pose, If not, it's a intentional drop
                std::pair<bool ,geometry_msgs::Pose> status;

                if (!correct_drop_later){
                    true_type = product_type;
                    true_pose = product_type_pose.second;
                }else{
                    true_type = wrong_poses_map.first;
                    true_pose = wrong_poses_map.second;
                    ROS_INFO_STREAM("[AriacSensorManager]:[PickAndPlace]: Sensor Blackout finished "
                                    "using the wrong_poses_map : ");
                    //  wrong_poses_map.erase(true_type);
                    wrong_poses_map = {};
                }
                if(agv_id == 1){
                    status = VerifyDropPose(true_pose, true_type, current_tray1_elements, agv_id);
                }else {
//                    status = VerifyDropPose(product_type_pose.second, product_type, current_tray2_elements, agv_id);
                    status = VerifyDropPose(true_pose, true_type, current_tray2_elements, agv_id);
                }
                // We Need to correct the pose
                if (!status.first){
                    // If the pose is not correct do the CorrectPose();
                    auto wrong_drop_pose = status.second;
                    CorrectPose(wrong_drop_pose, true_pose, true_type, agv_id, *this_arm);
                }else{
                    // Do Nothing and Continue
                }
                correct_drop_later = false;
            }else{
                // add the parts to a list whose poses needs to be fixed wrong_poses_map
                // wrong_poses_map has all the parts which are not in their correct poses
                //std::pair<std::string, geometry_msgs::Pose>
                ROS_INFO_STREAM("[AriacSensorManager]:[PickAndPlace]: Sensor Blackout taking "
                                "Place Will save it to wrong_poses_map : ");
                wrong_poses_map.first = product_type;
                wrong_poses_map.second = product_type_pose.second;
                correct_drop_later = true;
            }
        }

        // If there is something in this wrong_poses_map, If there is anything and there is not sensor_blackout,
        // I'm gonna correct the poses

        // Here we can do the quality check, return type would be a bool, whether good or bad
        bool quality = QualityCheck(agv_id);

        ROS_INFO_STREAM("[AriacSensorManager]:[PickAndPlace]: Quality is : " << quality);

        // if part is faluty:
        if (quality){
            ROS_INFO_STREAM("[AriacSensorManager]:[PickAndPlace]: In side Quality If : " << quality);

            //------pick the part and throw it away using the drop_pose-------
            PickAndThrow(drop_pose, product_type, *this_arm);
            // ros::Duration(1.5).sleep();
            //------ Do PickAndPlace again------------------------------------
            this_arm->RobotGoHome();
            PickAndPlace(product_type_pose, agv_id, *this_arm);
        }else{

            if (agv_id == 1){
//                partsDroppedOnTray1.push_back(product_type_pose.second);
                current_tray1_elements[product_type].emplace_back(product_type_pose.second);
            }
            // -- Adding parts that has been successfully placed on the tray.
            if (agv_id == 2){
//                partsDroppedOnTray2.push_back(product_type_pose.second);
                current_tray2_elements[product_type].emplace_back(product_type_pose.second);
            }
        }
    }
}

void AriacSensorManager::SegregateParts(const std::pair<std::string, geometry_msgs::Pose> type_pose_, int agv_id) 
{
    std::vector<int> unreachable;
    if (agv_id == 1) {
        unreachable = {5, 6};
    } else {
        unreachable = {1, 2};
    }

    ROS_INFO_STREAM("[AriacSensorManager][SegregateParts] : .first " << type_pose_.first);

    auto part_type = type_pose_.first;

    if (product_frame_list_.find(part_type) == product_frame_list_.end()) {
//        parts_from_belt[type_pose_.first].emplace_back(type_pose_.second);
//        ROS_INFO_STREAM("[AriacSensorManager][SegregateParts] : Part is on the belt");
        // add to the parts_from_belt
    } else {
        ROS_INFO_STREAM("[AriacSensorManager][SegregateParts] : Part found in the bin! Type is : " << type_pose_.first);

        auto frames_vect = product_frame_list_[type_pose_.first];
        auto product_frame = frames_vect.back();

        if ((product_frame.find("lc_bin_" + std::to_string(unreachable[0])) != -1) ||
            (product_frame.find("lc_bin_" + std::to_string(unreachable[1])) != -1)) {
            // if in the unreachable add to parts_from_bin_unreachable
            ROS_INFO_STREAM("[AriacSensorManager][SegregateParts] : Part is unreachable ");
            parts_from_bin_unreachable[type_pose_.first].emplace_back(type_pose_.second);
        } else {
            // else add to parts_from_bin_reachable
            parts_from_bin_reachable[type_pose_.first].emplace_back(type_pose_.second);
            ROS_INFO_STREAM("[AriacSensorManager][SegregateParts] : Part is reachable");
        }
    }
}

void AriacSensorManager::SegregateBeltParts(const std::pair<std::string, geometry_msgs::Pose> type_pose_, int agv_id)
{
    auto part_type = type_pose_.first;
    if (product_frame_list_.find(part_type) == product_frame_list_.end())
    {
        parts_from_belt_temp[part_type].emplace_back(type_pose_.second);
        parts_from_belt[agv_id - 1][part_type].emplace_back(type_pose_.second);
    }
}

void AriacSensorManager::ExecuteOrder() 
{
    ROS_WARN(">>>>>> Executing order...");
    bool pick_n_place_success{false};
    ros::spinOnce();
    ros::Duration(1.0).sleep();
    for (const auto &order:received_orders_){
        auto order_id = order.order_id;
        auto shipments = order.shipments;
        int tray_index_counter = 0;

        for (const auto &shipment: shipments) // loop trough shipments and
        {
            auto agv = shipment.agv_id.back();
            int agv_id = (shipment.agv_id == "any") ? 1 : agv - '0';
            // ROS_INFO_STREAM("AGV ID: " << agv_id);
            auto products = shipment.products;
            for (const auto &product: products)
            {
                product_type_pose_.first = product.type;
                product_type_pose_.second = product.pose;
                SegregateBeltParts(product_type_pose_, agv_id);
            }
        }

        if (agv_id_global == 1) {
            belt_tray_id = 2;
        } else {
            belt_tray_id = 1;
        }

        // parts_from_belt_temp = parts_from_belt; // for collecting enough number of parts from belt
        std::cout << "--------------------Belt Parts Needed in current order ----------------------" << std::endl;
        std::cout << "\nFrom Belt:\n";
        std::cout << ">> parts_from_belt\n";
        int needed_belt_parts_num = 0;
        for (size_t i = 0; i < parts_from_belt.size(); ++i)
        {
//                std::cout << "For AGV " << i + 1 << ":\n";
            for(auto const& pair : parts_from_belt[i])
            {
                auto parts_vec = parts_from_belt[i][pair.first];
//                    std::cout << "  " << pair.first << ": " << parts_vec.size() << std::endl;
                needed_belt_parts_num += parts_vec.size();
            }
        }
        std::cout << "\n>>parts_from_belt_temp\n";
        for(auto const& pair : parts_from_belt_temp)
        {
            auto parts_vec = parts_from_belt_temp[pair.first];
            std::cout << pair.first << ": " << parts_vec.size() << std::endl;
        }
        std::cout << "-----------------------------------------" << std::endl;

        for(const auto &shipment: shipments){

            auto shipment_type = shipment.shipment_type;
            ROS_INFO_STREAM("Shipment Type: " << shipment_type);
            auto agv = shipment.agv_id.back();
            int agv_id = (shipment.agv_id == "any") ? 1 : agv - '0';
            agv_id_global = agv_id;

            if (agv_id_global == 1){
                arm2.SendRobotTo(arm2.conveyer_pose);
                bb_arm2_sub = sensor_nh_.subscribe("/ariac/break_beam_2_change", 10,
                                                   & AriacSensorManager::bb_arm2_callback, this);
            }else{
                arm1.SendRobotTo(arm1.conveyer_pose);
                bb_arm1_sub = sensor_nh_.subscribe("/ariac/break_beam_3_change", 10,
                                                   & AriacSensorManager::bb_arm1_callback, this);
            }
            auto products = shipment.products;
            ROS_INFO_STREAM("Order ID: " << order_id);
            ROS_INFO_STREAM("Shipment Type: " << shipment_type);
            ROS_INFO_STREAM("AGV ID: " << agv_id);

            // here we can separate the products into different lists like parts_from_bin_reachable
            // and parts_from_belt and parts_from_bin_unreachable and parts_in_the_tray

            // search if the part in the bins or not
            for (const auto &product: products){
                product_type_pose_.first = product.type;
                product_type_pose_.second = product.pose;
                SegregateParts(product_type_pose_, agv_id);
            }

            std::cout << "[ExecuteOrder]: agv_id :" <<  agv_id << "belt_tray_id" << belt_tray_id << std::endl;

            if(agv_id == belt_tray_id)
            { // 2          // 2
                // get neede amount of belt part type from order->shipment->products
                for (const auto &product: products)
                {
                    ros::spinOnce();
                    if (parts_from_belt[agv_id - 1].find(product.type) != parts_from_belt[agv_id - 1].end())
                    {        //  2nd row                                                  // 2nd row
                        ROS_INFO_STREAM("[ExecuteOrder]: Already inside the IF parts from belt for loop");
                        product_type_pose_.first = product.type;

                        int temp_index;
                        geometry_msgs::Pose part_pose;
                        bool belt_part_exists = false;
                        while (!belt_part_exists){
                            // check if this part type exists in the tray_store_usage

//                            ROS_INFO_STREAM("[ExecuteOrder]: Inside the While Loop");

                            for (int i = 0; i < tray_store_usage[belt_tray_id - 1].size(); i++) {
                                //                       2nd row --> kitTray2
                                if (tray_store_usage[belt_tray_id - 1][i] == product_type_pose_.first) {
                                    belt_part_exists = true;
                                    temp_index = i;
                                    if (agv_id == 1) {
                                        part_pose = arm1.tray_store_poses[i];
                                        //  Kittray1
                                    } else if (agv_id == 2) {
                                        part_pose = arm2.tray_store_poses[i];
                                        // kittray2
                                    }

                                    break;
                                } else {
                                    belt_part_exists = false;
                                }
                            }
//                            ROS_INFO_STREAM("[ExecuteOrder]: Outside the For Loop");
                        }
                        ROS_INFO_STREAM("[ExecuteOrder]: Outside the While Loop");


                        for (auto pose : parts_from_belt[agv_id - 1][product_type_pose_.first]) {
                            //

                            while (arm_grab_belt) {
                                ROS_INFO_STREAM_THROTTLE(10, "[ExecuteOrder]: Picking from belt...");
                            }
                            arm_in_middle = true;
                            product_type_pose_.second = pose;
                            // 1
                            // Kittray2
                            if (agv_id == 1){
                                geometry_msgs::Pose pick_pose = part_pose;
                                // kittray1

                                // we are getting the pose in kitTray coords
                                ROS_INFO_STREAM("Already inside the agv_id_1");
                                // if same tray
                                if (agv_id == belt_tray_id) {
                                    // 1            //1         //1             //1
                                    BuildKitBeltParts(true, pick_pose, product_type_pose_.second,
                                                      product_type_pose_.first,
                                                      arm1, arm2, 1);
                                } else {
                                    //else in the opposite tray
                                    BuildKitBeltParts(false, pick_pose, product_type_pose_.second,
                                                      product_type_pose_.first,
                                                      arm1, arm2, 1);
                                }
                                // need to clear this item tray_store_usage[belt_tray_id-1][temp_index]
                                tray_store_usage[belt_tray_id-1][temp_index] = "";

                            } else if (agv_id == 2) {
                                geometry_msgs::Pose pick_pose = part_pose;
                                // kittray2

                                ROS_INFO_STREAM("Already inside the agv_id_2");

                                // if same tray

                                if (agv_id == belt_tray_id) {
                                    // 2        // 2            //2          //2
                                    BuildKitBeltParts(true, pick_pose, product_type_pose_.second,
                                                      product_type_pose_.first,
                                                      arm2, arm1, 0);
                                } else {
                                    // else in the opposite tray
                                    BuildKitBeltParts(false, pick_pose, product_type_pose_.second,
                                                      product_type_pose_.first,
                                                      arm1, arm2, 0);
                                    tray_store_usage[belt_tray_id-1][temp_index] = "";
                                }
                            }
                            arm_in_middle = false;
                            built_kit_product_type_pose_[product_type_pose_.first].emplace_back(
                                    product_type_pose_.second);
                            // pop out uneeded drop pose if vector size > 1, erase
                            // the key-vector pair in map if vector size = 1
                            if (parts_from_belt[agv_id - 1][product_type_pose_.first].size() > 1) {
                                parts_from_belt[agv_id - 1][product_type_pose_.first].pop_back();
                            } else if (parts_from_belt[agv_id - 1][product_type_pose_.first].size() == 1) {
                                parts_from_belt[agv_id - 1].erase(product_type_pose_.first);
                            }
                        }

                    }
                }

                for (auto const& pair : parts_from_bin_reachable)
                {
                    ros::spinOnce();
                    product_type_pose_.first = pair.first;
                    for (auto pose : parts_from_bin_reachable[pair.first])
                    {
                        product_type_pose_.second = pose;
                        if (agv_id == 1){
                            pick_n_place_success =  PickAndPlace(product_type_pose_, agv_id, arm1);
                        }else if(agv_id ==2){
                            pick_n_place_success =  PickAndPlace(product_type_pose_, agv_id, arm2);
                        }
                        // We put the parts which are good into  a map here
                        built_kit_product_type_pose_[product_type_pose_.first].emplace_back(product_type_pose_.second);
                    }
                }

                std::cout << "[AriacSensorManager][ExecuteOrder] : Arm finished picking up from conveyor belt " << std::endl;

                for (auto const& pair : parts_from_bin_unreachable)
                {
                    while (arm_grab_belt) {
                        ROS_INFO_STREAM_THROTTLE(10, "[ExecuteOrder]: Picking from belt...");
                    }
                    ros::spinOnce();
                    arm_in_middle = true;
                    product_type_pose_.first = pair.first;
                    //ROS_INFO_STREAM("Product type: " << product_type_pose_.first);

                    for (auto pose : parts_from_bin_unreachable[pair.first]){
                        product_type_pose_.second = pose;
                        ROS_INFO_STREAM("Product pose: " << product_type_pose_.second.position.x);
                        if (agv_id == 1){
                            pick_n_place_success = PickAndPlace(product_type_pose_, agv_id, arm1);
                        }else if(agv_id ==2){
                            pick_n_place_success = PickAndPlace(product_type_pose_, agv_id, arm2);
                        }
                        // We put the parts which are good into  a map here
                        built_kit_product_type_pose_[product_type_pose_.first].emplace_back(product_type_pose_.second);
                    }
                    arm_in_middle = false;
                }

            }
            else
            {
                // here we can loop separately through the parts_from_bin_reachable
                // and parts_from_belt and parts_from_bin_unreachable parts_in_the_tray
                for (auto const& pair : parts_from_bin_reachable)
                {
                    ros::spinOnce();
                    product_type_pose_.first = pair.first;
                    for (auto pose : parts_from_bin_reachable[pair.first]){
                        product_type_pose_.second = pose;
                        if (agv_id == 1){
                            pick_n_place_success =  PickAndPlace(product_type_pose_, agv_id, arm1);
                        }else if(agv_id ==2){
                            pick_n_place_success =  PickAndPlace(product_type_pose_, agv_id, arm2);
                        }
                       // We put the parts which are good into  a map here
                        built_kit_product_type_pose_[product_type_pose_.first].emplace_back(product_type_pose_.second);
                    }
                }

                std::cout << "[AriacSensorManager][ExecuteOrder] : Arm finished picking up from conveyor belt " <<std::endl;

                for (auto const& pair : parts_from_bin_unreachable)
                {
                    while (arm_grab_belt) {
                        ROS_INFO_STREAM_THROTTLE(10, "[ExecuteOrder]: Picking from belt...");
                    }
                    ros::spinOnce();
                    arm_in_middle = true;
                    product_type_pose_.first = pair.first;
                    //ROS_INFO_STREAM("Product type: " << product_type_pose_.first);

                    for (auto pose : parts_from_bin_unreachable[pair.first]){
                        product_type_pose_.second = pose;
                        ROS_INFO_STREAM("[ExecuteOrder]:Product pose: " << product_type_pose_.second.position.x);
                        if (agv_id == 1){
                            pick_n_place_success = PickAndPlace(product_type_pose_, agv_id, arm1);
                        }else if(agv_id ==2){
                            pick_n_place_success = PickAndPlace(product_type_pose_, agv_id, arm2);
                        }
                        // We put the parts which are good into  a map here
                        built_kit_product_type_pose_[product_type_pose_.first].emplace_back(product_type_pose_.second);
                    }
                    arm_in_middle = false;
                }
                // get neede amount of belt part type from order->shipment->products
                for (const auto &product: products)
                {
                    ros::spinOnce();
                    if (parts_from_belt[agv_id - 1].find(product.type) != parts_from_belt[agv_id - 1].end())
                    {           // 1st row                                      //1st row
                        ROS_INFO_STREAM("[ExecuteOrder]: Already inside ELSE the parts from belt for loop");
                        product_type_pose_.first = product.type;

                        int temp_index;
                        geometry_msgs::Pose part_pose;
                        bool belt_part_exists = false;

                        while (!belt_part_exists){

//                            ROS_INFO_STREAM("[ExecuteOrder]: Inside the while loop");
                            // check if this part type exists in the tray_store_usage
                                                                // 2
                            for (int i = 0; i < tray_store_usage[belt_tray_id - 1].size(); i++) {
                                //                  // 2nd row
                                if (tray_store_usage[belt_tray_id - 1][i] == product_type_pose_.first) {
                                    // 2nd row
                                    belt_part_exists = true;
                                    temp_index = i;
                                    if (agv_id == 1) {
                                        part_pose = arm1.tray_store_poses[i];
                                        //1
                                    } else if (agv_id == 2) {
                                        part_pose = arm2.tray_store_poses[i];
                                        //2
                                    }
                                    break;
                                } else {
                                    belt_part_exists = false;
                                }
                            }
//                            ROS_INFO_STREAM("[ExecuteOrder]: After the for loop");
                        }
                        ROS_INFO_STREAM("[ExecuteOrder]: outside the while loop");


                        for (auto pose : parts_from_belt[agv_id - 1][product_type_pose_.first])
                        {                   //1
                            while (arm_grab_belt) {
                                ROS_INFO_STREAM_THROTTLE(10, "[ExecuteOrder]: Picking from belt...");
                            }
                            arm_in_middle = true;
                            product_type_pose_.second = pose;
                                    //1
                            if (agv_id == 1)
                            {
                                geometry_msgs::Pose pick_pose = part_pose;
                                // we are getting the pose in kitTray coords

                                ROS_INFO_STREAM("Already inside the agv_id_1");
                                // if same tray
                                if (agv_id == belt_tray_id) {
                                    BuildKitBeltParts(true, pick_pose, product_type_pose_.second,
                                                      product_type_pose_.first,
                                                      arm1, arm2, 1);
                                } else {
                                    //else in the opposite tray
                                                            //
                                    BuildKitBeltParts(false, pick_pose, product_type_pose_.second,
                                                      product_type_pose_.first,
                                                      arm1, arm2, 1);
                                }
                                // need to clear this item tray_store_usage[belt_tray_id-1][temp_index]
                                tray_store_usage[belt_tray_id-1][temp_index] = "";

                            }
                            else if (agv_id == 2)
                            {
                                geometry_msgs::Pose pick_pose = part_pose;
                                ROS_INFO_STREAM("Already inside the agv_id_2");

                                // if same tray
                                if (agv_id == belt_tray_id) {
                                    BuildKitBeltParts(true, pick_pose, product_type_pose_.second,
                                                      product_type_pose_.first,
                                                      arm2, arm1, 0);
                                } else {
                                    // else in the opposite tray
                                    BuildKitBeltParts(false, pick_pose, product_type_pose_.second,
                                                      product_type_pose_.first,
                                                      arm1, arm2, 0);
                                    tray_store_usage[belt_tray_id-1][temp_index] = "";
                                }
                            }
                            arm_in_middle = false;
                            built_kit_product_type_pose_[product_type_pose_.first].emplace_back(
                                    product_type_pose_.second);

                            if (parts_from_belt[agv_id - 1][product_type_pose_.first].size() > 1) {
                                parts_from_belt[agv_id - 1][product_type_pose_.first].pop_back();
                            } else if (parts_from_belt[agv_id - 1][product_type_pose_.first].size() == 1) {
                                parts_from_belt[agv_id - 1].erase(product_type_pose_.first);
                            }
                        }
                    }
                }

//                if (wrong_poses_map){
//                     if there are uncorrecteddropped parts correct them.
//                }

            }

            ros::Duration(6.0).sleep();
            if (Flag_updateKit){
                buildUpdatedKitMap();
                UpdateKit(agv_id);
                Flag_updateKit = false;
            }
            if (agv_id_global == 1){
                bb_arm2_sub.shutdown();
            }else{
                bb_arm1_sub.shutdown();
            }
            SubmitAGV(agv_id);
            parts_from_bin_reachable.clear();
            parts_from_bin_unreachable.clear();
//            parts_from_belt.clear();

            ROS_INFO_STREAM("[AriacSensorManager][ExecuteOrder] : Submitting AGV " << agv_id);
            int finish=1;
            ROS_WARN_STREAM( "before going to start building the "
                             "next shipment black_out_occured is: " << black_out_occured);

        }
    }
}


void AriacSensorManager::BuildKitBeltParts(bool same_tray, geometry_msgs::Pose pick_pose,
                                           geometry_msgs::Pose drop_pose, std::string product_type,
                                           RobotController& arm_x, RobotController& arm_y, int flag){

    // pick_pose is in the KitTray coordinates
    // drop_pose is in the kit tray coordinates , if agv1 its in kittray1 coord viceversa

    // 2 , 2, flag=1

    if (same_tray){
        // use CorrectPose() to just modify the pose
        // CorrectPose() will take the current_pose,updated_pose in same KitTray Coordinates
        CorrectPose(pick_pose,drop_pose,product_type,agv_id_global, arm_x, true);
                    // 2        //2                                    //1

    }else{
        // Use PutPartsIntoOtherTray() to put into the opposite tray
        // PutPartsIntoOtherTray() takes the pose in Opposite KitTray Coords
        PutPartsIntoOtherTray(pick_pose,drop_pose,product_type, flag, arm_x, arm_y, true);
    }
}

void  AriacSensorManager::PutPartsIntoOtherTray(geometry_msgs::Pose pick_pose,
        geometry_msgs::Pose drop_pose, std::string product_type, int flag,
        RobotController& arm_x, RobotController& arm_y, bool flipCheckReq= false){
    // --- This function will place a conveyor belt part from one tray into the other tray
    // --- This function will need the pose from where to pickup and where to drop.
    // --- This function will use both the arms to pass the part by placing the part at the center of the rail
    // --- There should be no other part at the drop pose where it is being dropped in the tray
    // --- What should the drop pose be ?? This can be the same pick_up pose but in other tray's frame

    int Other_tray_ID;
    int This_tray_ID;
    int agv_id = agv_id_global;


    if(flag == 0) { // straight, basically to remove from their original locations
        this_arm = &arm_x;
        that_arm = &arm_y;
        Other_tray_ID = 2;
        This_tray_ID = 1;
        // that_arm->SendRobotTo("linear_arm_actuator_joint", -1.18);
        // that_arm->SendRobotTo(that_arm->home_joint_pose_1);
    }
    else if(flag == 1){ // reverse, basically to add back to their original locations
        this_arm = &arm_y;
        that_arm = &arm_x;
        Other_tray_ID = 1;
        This_tray_ID = 2;
        // that_arm->SendRobotTo("linear_arm_actuator_joint", 1.18);
        // that_arm->SendRobotTo(that_arm->home_joint_pose_1);
    }

    // Pick the part with the KitBuildingARM
    this_arm->SendRobotTo(this_arm->home_joint_pose_1);
    auto part_pose = kitToWorld(pick_pose, This_tray_ID);

    if (product_type == "pulley_part")
        part_pose.position.z += 0.062;

    bool failed_pick = this_arm->PickPart(part_pose);
//    ros::Duration(0.5).sleep();

    while (!failed_pick) {
        // auto part_pose = GetPartPose("/world", product_frame);
        failed_pick = this_arm->PickPart(part_pose);
    }

//    ros::Duration(0.5).sleep();
    // that_arm->RobotGoHome();
    this_arm->SendRobotTo(this_arm->home_joint_pose_1);
    this_arm->SendRobotTo(this_arm->home_joint_pose_2);

    // place the desired part on the rail
    geometry_msgs::Pose above_rail_center;
    above_rail_center.position.x = 0.3;
    // above_rail_center.position.y = -0.383;
    above_rail_center.position.y = 0;
    // above_rail_center.position.z = 0.9 + 0.1;
    above_rail_center.position.z = 0.9 + 0.075; // check if 0.075 makes it faster and won't crash on the rail
    if (product_type == "pulley_part")
        above_rail_center.position.z += 0.062; // check if dropping pulley_part on the rail correctly
    above_rail_center.orientation.x = part_pose.orientation.x;
    above_rail_center.orientation.y = part_pose.orientation.y;
    above_rail_center.orientation.z = part_pose.orientation.z;
    above_rail_center.orientation.w = part_pose.orientation.w;

    ROS_INFO_STREAM("[AriacSensorManager]:[PutPartsIntoOtherTray]: Drop Pose : " << above_rail_center);
    // that_arm->RobotGoHome();
    this_arm->SendRobotTo(this_arm->rail_pick_trans_pose);

    // drop the part with the KitBuildingARM
    auto result = this_arm->DropPart(above_rail_center);
    this_arm->SendRobotTo(this_arm->home_joint_pose_2);
    this_arm->SendRobotTo(this_arm->home_joint_pose_1);

    // Pick the part with the Other arm
    part_pose = above_rail_center;
    part_pose.position.z -= 0.015;

    that_arm->SendRobotTo(that_arm->home_joint_pose_1);
    that_arm->SendRobotTo(that_arm->home_joint_pose_2);
    that_arm->SendRobotTo(that_arm->rail_pick_trans_pose);

    failed_pick = that_arm->PickPart(part_pose);
//    ros::Duration(0.5).sleep();

    while (!failed_pick) {
        // auto part_pose = GetPartPose("/world", product_frame);
        failed_pick = that_arm->PickPart(part_pose);
    }

    if (flipCheckReq){
        // do the flipping here
        // ================================== Flipping Section =============================== //
        bool isFlippingRequired{false};

        tf2::Quaternion quat_D;
        tf2::fromMsg(drop_pose.orientation, quat_D);
        double d_R, d_P, d_Y;
        if (product_type == "pulley_part"){

            tf2::Matrix3x3(quat_D).getRPY(d_R, d_P, d_Y);
            ROS_INFO_STREAM("[ASM][PutPartsIntoOtherTray] - RPY: " << d_R << "; "<< d_P << "; " << d_Y);

            ROS_INFO_STREAM("[AriacSensorManager]:[PutPartsIntoOtherTray]:Part successfully picked.");

            // Flip the part if needed
            if((abs(d_R) < 3.16 && abs(d_R) > 3.10) ^ (abs(d_P) < 3.16 && abs(d_P) > 3.10)) { ///
                ROS_WARN_STREAM("[ASM]:[PutPartsIntoOtherTray]: Flipping Required.");
                isFlippingRequired = true;
            }
            else{
                ROS_WARN_STREAM("[ASM]:[PutPartsIntoOtherTray]: Flipping not Required");
                isFlippingRequired = false;
            }

            if (isFlippingRequired){
                that_arm->SendRobotTo(that_arm->rail_pick_trans_pose);
                this->FlipPart(that_arm, this_arm, above_rail_center);
            }
        }
        // -------------------------------- Flip part completed -------------------------- //
    }

    that_arm->SendRobotTo(that_arm->home_joint_pose_2);

    bool generate_success = false;
    std::map<std::string, std::vector<geometry_msgs::Pose>> current_tray_elements {};

    if(!sensor_black_out_global){
        current_tray_elements = getCurrentParts(Other_tray_ID);
        generate_success = true;
    }

    that_arm->SendRobotTo(that_arm->home_joint_pose_1);

    // Drop the part in the Other Tray
    auto new_drop_pose = kitToWorld(drop_pose, Other_tray_ID);

    result = that_arm->DropPart(new_drop_pose);

    that_arm->SendRobotTo(that_arm->home_joint_pose_1);

    std::string true_type;
    geometry_msgs::Pose true_pose;

    if((!result || correct_other_tray_later) & generate_success){

        if(!sensor_black_out_global){
            // Once the part is placed, Check if the part is place in the correct drop pose, If not, it's a intentional drop
            std::pair<bool ,geometry_msgs::Pose> status;

            if (!correct_other_tray_later){
                true_type = product_type;
                true_pose = drop_pose;
                // KitTrayFrame
            }else{
                true_type = wrong_poses_pair_other_tray.first;
                true_pose = wrong_poses_pair_other_tray.second;
                ROS_INFO_STREAM("[AriacSensorManager]:[PutPartsIntoOtherTray]: Sensor Blackout finished "
                                "using the wrong_poses_map : ");
                //  wrong_poses_map.erase(true_type);
                wrong_poses_pair_other_tray = {};
            }
            status = VerifyDropPose(true_pose, true_type, current_tray_elements, Other_tray_ID);
            // We Need to correct the pose
            if (!status.first){
                // If the pose is not correct do the CorrectPose();
                auto wrong_drop_pose = status.second;
                CorrectPose(wrong_drop_pose, true_pose, true_type, Other_tray_ID, *that_arm);
            }else{
                // Do Nothing and Continue
            }
            correct_other_tray_later = false;
        }else{
            // add the parts to a list whose poses needs to be fixed wrong_poses_map
            // wrong_poses_map has all the parts which are not in their correct poses
            //std::pair<std::string, geometry_msgs::Pose>
            ROS_INFO_STREAM("[AriacSensorManager]:[PutPartsIntoOtherTray]: Sensor Blackout taking "
                            "Place Will save it to wrong_poses_map : ");
            wrong_poses_pair_other_tray.first = product_type;
            wrong_poses_pair_other_tray.second = drop_pose;
            correct_other_tray_later = true;
        }
    }


}

void  AriacSensorManager::WhatToRemove(){

    vector<std::string> keys;

    // Looping through the maps
    for(auto const& item: built_kit_product_type_pose_){

        keys.push_back(item.first);
        auto part_type = item.first;

        std::cout << "[AriacSensorManager][WhatToRemove] : The part type is : " << part_type << "\n";

        // condition to check if that type of part_type exists in both the maps
        if (order_update_product_type_pose_.find(part_type) != order_update_product_type_pose_.end()){

            auto update = order_update_product_type_pose_[part_type];
            auto built = built_kit_product_type_pose_[part_type];

            if (built.size() > update.size()){
                int difference = built.size() - update.size();
                NumPartsToRemove = NumPartsToRemove + difference;  // Lets say
                std::cout << "[AriacSensorManager][WhatToRemove] : The difference is : " << difference << "\n";
                int i=0;
                bool same_flag = false;

                // Select a pose from built
                for (auto built_pose: built){
                    same_flag = false;

                    // search with every pose in update
                    for(auto update_pose: update){

                        // if the pose matches with one of the pose, dont remove
                        if (built_pose == update_pose){
                            same_flag = true;
                        }
                    }
                    // find out what to remove
                    // if the pose doesn't match with any pose in the update, we can safely remove
                    if (!same_flag && i< difference){
                        parts_to_remove_product_type_pose_[part_type].emplace_back(built_pose);

                        // Erase built_pose that will be thrown away. Using the std::remove to find the part
                        // from the vector to remove it for built_map
                        built_kit_product_type_pose_[part_type].erase(std::remove(built_kit_product_type_pose_[part_type].begin(),
                                                                                  built_kit_product_type_pose_[part_type].end(), built_pose),
                                                                      built_kit_product_type_pose_[part_type].end());

                        // increasing the counter here, because we only nee 'difference' number of parts to remove
                        i++;
                    }
                }
            }
        }else{
            // Enters here as the part type doesn't exist in the updated order
            // We have to remove all the parts with that part type
            std::cout << "[AriacSensorManager][WhatToRemove] : non existent parts type is : " << part_type << "\n";
            auto built_non_existent = built_kit_product_type_pose_[part_type];
            parts_to_remove_product_type_pose_[part_type] = built_non_existent;

            // Erase built_non_existent pose from the built map that will be thrown away.
            // Using the std::remove to find the part from the vector to remove it for built_map
            built_kit_product_type_pose_.erase(part_type);
            NumPartsToRemove += built_non_existent.size();
            // not found
        }
    }
}

//// Second, Check for number of parts to be changed positions
//void  AriacSensorManager::WhatToModify(){
//
//    order_update_copy = order_update_product_type_pose_;
//
//    // Looping through the maps
//    for (auto const& built_item: built_kit_product_type_pose_){
//
//        auto built_part_type = built_item.first;        //gear_part
//        std::cout << "[AriacSensorManager][WhatToModify] : The part type is : " << built_part_type << "\n";
//
//        for (auto built_pose : built_item.second){
//
//            for (auto const& order_item: order_update_copy){
//
//                auto order_part_type = order_item.first;
//
//                for (auto update_pose : order_item.second){
//
//                    if (built_pose == update_pose &&  built_part_type == order_part_type){
//
//                        std::cout << "[AriacSensorManager][WhatToModify] : Poses match for same type, Lucky!! : " << "\n";
////                        std::cout << "[AriacSensorManager][WhatToModify] : order type!! : " << order_part_type << "\n";
////                        std::cout << "[AriacSensorManager][WhatToModify] : Built type!! : " << built_part_type << "\n";
////                        std::cout << "[AriacSensorManager][WhatToModify] : order pose!! : " << update_pose << "\n";
////                        std::cout << "[AriacSensorManager][WhatToModify] : Built pose!! : " << built_pose << "\n";
//
//                        order_update_copy[order_part_type].erase(std::remove(order_update_copy[order_part_type].begin(),
//                                order_update_copy[order_part_type].end(), update_pose),
//                                        order_update_copy[order_part_type].end());
//
//                        built_kit_product_type_pose_[built_part_type].erase(std::remove(built_kit_product_type_pose_[built_part_type].begin(),
//                                built_kit_product_type_pose_[built_part_type].end(), built_pose),
//                                        built_kit_product_type_pose_[built_part_type].end());
//
//                    }else if(built_pose == update_pose &&  built_part_type != order_part_type){
//
//                        std::cout << "[AriacSensorManager][WhatToModify] : Poses match with different type, More Work!! : " << "\n";
////                        std::cout << "[AriacSensorManager][WhatToModify] : order type!! : " << order_part_type << "\n";
////                        std::cout << "[AriacSensorManager][WhatToModify] : Built type!! : " << built_part_type << "\n";
////                        std::cout << "[AriacSensorManager][WhatToModify] : order pose!! : " << update_pose << "\n";
////                        std::cout << "[AriacSensorManager][WhatToModify] : Built pose!! : " << built_pose << "\n";
//
//                        built_kit_product_type_pose_[built_part_type].erase(std::remove(built_kit_product_type_pose_[built_part_type].begin(),
//                                built_kit_product_type_pose_[built_part_type].end(), built_pose),
//                                        built_kit_product_type_pose_[built_part_type].end());
//
//                        // If the part was pickup from the conveyor belt add to parts_to_place_in_other_tray
//                        if(parts_from_belt[agv_id_global-1].find(built_part_type) != parts_from_belt[agv_id_global-1].end()){
//
//                            // find a vacant space on the tray and get the space
//                            // Give the space as a droppose for the parts_to_place_in_other_tray.
//
//                            parts_to_place_in_other_tray[built_part_type].emplace_back(built_pose);
//                            std::cout << "[AriacSensorManager][WhatToModify] : Update pose for conv belt part!! : " << update_pose << endl;
//
//                            // Here we need to compare the belt part vectors in both the maps.
//                            // Delete the ones that are not matching, retain the one that are matching.
//
//                            auto vect_built = built_kit_product_type_pose_[built_part_type];
//                            auto vect_update = order_update_copy[built_part_type];
//
//                            for (auto vect_up_part : vect_update){
//                                auto it = std::find(vect_built.begin(), vect_built.end(), vect_up_part);
//                                if(it == vect_built.end()){
//                                    // If not found
//                                    order_update_copy[built_part_type].erase(std::remove(order_update_copy[built_part_type].begin(),
//                                            order_update_copy[built_part_type].end(), vect_up_part),
//                                                    order_update_copy[built_part_type].end());
//
//                                    parts_back_from_tray[built_part_type].emplace_back(make_pair(vect_up_part, built_pose));
//                                    break;
//                                }
//                            }
//
////                            order_update_copy.erase(built_part_type);
////                            order_update_copy[built_part_type].erase(std::remove(order_update_copy[built_part_type].begin(),
////                                    order_update_copy[built_part_type].end(), update_pose),
////                                            order_update_copy[built_part_type].end());
//
//                        }else{
//                            // Else add to parts_to_remove_product_type_pose_
//                            parts_to_remove_product_type_pose_[built_part_type].emplace_back(built_pose);
//                        }
//
//                        NumPartsToRemove += 1;
//
//                    }
//                }
//            }
//        }
//    }
//
//    for (auto const& built_item: built_kit_product_type_pose_) {
//        auto built_part_type = built_item.first;
//        NumPartsToModify = NumPartsToModify + built_item.second.size();
//    }
//}

// Second, Check for number of parts to be changed positions
void  AriacSensorManager::WhatToModify(){
    order_update_copy = order_update_product_type_pose_;
    // Looping through the maps
    for (auto const& built_item: built_kit_product_type_pose_){
        auto built_part_type = built_item.first;        //gear_part
        std::cout << "[AriacSensorManager][WhatToModify] : The part type is : " << built_part_type << "\n";
        for (auto built_pose : built_item.second){
            for (auto const& order_item: order_update_copy){
                auto order_part_type = order_item.first;
                for (auto update_pose : order_item.second){
                    if (built_pose == update_pose &&  built_part_type == order_part_type){

                        std::cout << "[AriacSensorManager][WhatToModify] : Poses match for same type, Lucky!! : " << "\n";
//                        std::cout << "[AriacSensorManager][WhatToModify] : order type!! : " << order_part_type << "\n";
//                        std::cout << "[AriacSensorManager][WhatToModify] : Built type!! : " << built_part_type << "\n";
//                        std::cout << "[AriacSensorManager][WhatToModify] : order pose!! : " << update_pose << "\n";
//                        std::cout << "[AriacSensorManager][WhatToModify] : Built pose!! : " << built_pose << "\n";
                        order_update_copy[order_part_type].erase(std::remove(order_update_copy[order_part_type].begin(),
                                                                             order_update_copy[order_part_type].end(), update_pose),
                                                                 order_update_copy[order_part_type].end());

                        built_kit_product_type_pose_[built_part_type].erase
                                (std::remove(built_kit_product_type_pose_[built_part_type].begin(),
                                             built_kit_product_type_pose_[built_part_type].end(), built_pose),
                                 built_kit_product_type_pose_[built_part_type].end());

                    }else if(built_pose == update_pose &&  built_part_type != order_part_type){
                        std::cout << "[AriacSensorManager][WhatToModify] : Poses match with different type, More Work!! : " << "\n";
//                        std::cout << "[AriacSensorManager][WhatToModify] : order type!! : " << order_part_type << "\n";
//                        std::cout << "[AriacSensorManager][WhatToModify] : Built type!! : " << built_part_type << "\n";
//                        std::cout << "[AriacSensorManager][WhatToModify] : order pose!! : " << update_pose << "\n";
//                        std::cout << "[AriacSensorManager][WhatToModify] : Built pose!! : " << built_pose << "\n";

                        built_kit_product_type_pose_[built_part_type].erase
                                (std::remove(built_kit_product_type_pose_[built_part_type].begin(),
                                             built_kit_product_type_pose_[built_part_type].end(), built_pose),
                                 built_kit_product_type_pose_[built_part_type].end());

                        // If the part was pickup from the conveyor belt add to parts_to_place_in_other_tray
                        if(parts_from_belt[agv_id_global-1].find(built_part_type) != parts_from_belt[agv_id_global-1].end()){

                            // find a vacant space on the tray and get the space
                            bool space_available = false;
                            int temp_index;
                            geometry_msgs::Pose drop_pose_tray;
                            // check if this part type exists in the tray_store_usage
                            for (int i=0; i<tray_store_usage[agv_id_global-1].size();i++){
                                if(tray_store_usage[agv_id_global-1][i] == ""){
                                    space_available = true;
                                    temp_index = i;
                                    if(agv_id_global == 1){
                                        drop_pose_tray =  arm1.tray_store_poses[i];
                                    }else if(agv_id_global == 2){
                                        drop_pose_tray =  arm2.tray_store_poses[i];
                                    }
                                    break;
                                }
                                else{
                                    space_available = false;
                                }
                            }
                            // Give the space as a droppose for the parts_to_place_in_other_tray.
                            if(space_available){
                                parts_to_place_in_other_tray[built_part_type].emplace_back(make_pair(drop_pose_tray, built_pose));
                                update_store_idxs.push_back(temp_index);
                                std::cout << "[AriacSensorManager][WhatToModify] : Update pose for conv belt part!! : " << update_pose << endl;
                                // Here we need to compare the belt part vectors in both the maps.
                                // Delete the ones that are not matching, retain the one that are matching.

                                auto vect_built = built_kit_product_type_pose_[built_part_type];
                                auto vect_update = order_update_copy[built_part_type];

                                for (auto vect_up_part : vect_update){
                                    auto it = std::find(vect_built.begin(), vect_built.end(), vect_up_part);
                                    if(it == vect_built.end()){
                                        // If not found
                                        order_update_copy[built_part_type].erase(std::remove(order_update_copy[built_part_type].begin(),
                                                                                             order_update_copy[built_part_type].end(), vect_up_part),
                                                                                 order_update_copy[built_part_type].end());
                                        parts_back_from_tray[built_part_type].emplace_back(make_pair(vect_up_part, drop_pose_tray));
                                        break;
                                    }
                                }
                            }
                        }else{
                            // Else add to parts_to_remove_product_type_pose_
                            parts_to_remove_product_type_pose_[built_part_type].emplace_back(built_pose);
                            ROS_ERROR_STREAM("[ASM]:WHATTOMODIFY: >> " << built_part_type << "; " << built_pose);
                        }
                        NumPartsToRemove += 1;
                    }
                }
            }
        }
    }
    for (auto const& built_item: built_kit_product_type_pose_) {
        auto built_part_type = built_item.first;
        NumPartsToModify = NumPartsToModify + built_item.second.size();
    }
}

void AriacSensorManager::WhatToAdd(){

    std::cout << "[AriacSensorManager][WhatToAdd] : Inside the what to add : " << "\n";

    for(auto const& pair : order_update_copy){

        if(pair.second.size() > 0){
            NumPartsToAdd += pair.second.size();
        }
    }

    std::cout << "[AriacSensorManager][WhatToAdd] : Num of parts to add : " << "\n";

}


void  AriacSensorManager::RemoveBeltParts(){

    for(auto const& pair : parts_to_place_in_other_tray){
        std::cout << "[AriacSensorManager][RemoveBeltParts] : belt part type to remove from kit" << pair.first << std::endl;
        auto part_type = pair.first;
        auto drop_pick_pair = pair.second;

        for (auto const& pair : drop_pick_pair){
//            std::cout << "[AriacSensorManager][RemoveBeltParts] : belt part pose to remove from kit " << part_pose << std::endl;

            auto drop_pose = pair.first;
            auto pick_pose = pair.second;

            if(agv_id_global == 1){
                PutPartsIntoOtherTray(pick_pose,drop_pose,part_type, 0, arm1, arm2);
            }else{
                PutPartsIntoOtherTray(pick_pose,drop_pose,part_type, 0, arm2, arm1);
            }

        }
        // Use this update_store_idxs to update the tray_store_usage
        ROS_ERROR_STREAM("we shouldn't do update_store_idxs.back() ---> Need to Look at this");
        ROS_ERROR_STREAM("we shouldn't do update_store_idxs.back() ---> Need to Look at this");
        ROS_ERROR_STREAM("we shouldn't do update_store_idxs.back() ---> Need to Look at this");
        ROS_ERROR_STREAM("we shouldn't do update_store_idxs.back() ---> Need to Look at this");
        ROS_ERROR_STREAM("Did you edit this important thing????");

        auto idx = update_store_idxs.back();
        if(agv_id_global == 1){
            tray_store_usage[1][idx] = part_type;
        }else if (agv_id_global == 2){
            tray_store_usage[0][idx] = part_type;
        }
    }
}

void  AriacSensorManager::AddBeltParts(){

        for(auto const& pair : parts_back_from_tray){
        std::cout << "[AriacSensorManager][AddBeltParts] : part type to Add " << pair.first << std::endl;
        auto part_type = pair.first;
        auto drop_pick_pair = pair.second;

        for (auto const& pair : drop_pick_pair){

            auto drop_pose = pair.first;
            auto pick_pose = pair.second;

//            std::cout << "[AriacSensorManager][AddBeltParts] : part pose to remove " << part_pose << std::endl;

            if(agv_id_global == 1){
                PutPartsIntoOtherTray(pick_pose,drop_pose,part_type, 1, arm1, arm2);
            }else{
                PutPartsIntoOtherTray(pick_pose,drop_pose,part_type, 1, arm2, arm1);
            }

        }
        // we shouldn't do update_store_idxs.back() ---> Need to Look at this

        ROS_ERROR_STREAM("we shouldn't do update_store_idxs.back() ---> Need to Look at this");
        ROS_ERROR_STREAM("we shouldn't do update_store_idxs.back() ---> Need to Look at this");
        ROS_ERROR_STREAM("we shouldn't do update_store_idxs.back() ---> Need to Look at this");
        ROS_ERROR_STREAM("we shouldn't do update_store_idxs.back() ---> Need to Look at this");
        ROS_ERROR_STREAM("Did you edit this important thing????");

        // Use this update_store_idxs to update the tray_store_usage
        auto idx = update_store_idxs.back(); // we shouldn't do back
        if(agv_id_global == 1){
            tray_store_usage[1][idx] = "";
        }else if (agv_id_global == 2){
            tray_store_usage[0][idx] = "";
        }

        }
}

void AriacSensorManager::removeParts(int agv_id, RobotController& arm){

//    int agv_id = 2;
    for(auto const& pair : parts_to_remove_product_type_pose_){
        std::cout << "[AriacSensorManager][removeParts] : part type to remove " << pair.first << std::endl;
        auto part_type = pair.first;

        for (auto part_pose : pair.second){
            std::cout << "[AriacSensorManager][removeParts] : part pose to remove " << part_pose << std::endl;

            // if the part to remove is a conveyor belt part
            // Do PutBackIntoTheTray()

            geometry_msgs::Pose drop_pose = kitToWorld(part_pose, agv_id);

            arm.SendRobotTo(arm.home_joint_pose_1);
//            ros::Duration(0.5).sleep();

            PickAndThrow(drop_pose, part_type, arm);
            ros::Duration(0.5).sleep();

        }
    }
}

void AriacSensorManager::modifyPose(int agv_id){

    // we need to get the current pose and where to place
    // iterate through whatever is left on the Built_kit_map and search for that part_type in Order_update_map.
    // built_kit_map gives the current pose and order_update_kit gives the updated pose.

//    int agv_id = 2;

    for (auto const& built_item: built_kit_product_type_pose_){

        auto built_part_type = built_item.first;
        std::cout << "[AriacSensorManager][modifyPose] : The part type is : " << built_part_type << "\n";

        for (auto built_pose : built_item.second) {

            auto order_vect = order_update_copy[built_part_type];

            auto update_pose = order_vect.back();
            order_vect.pop_back();

            // use pickandplace, here built_pose is current pose, update_pose is the updated pose where the
            // parts needs to be placed
//            CorrectPose(built_pose, update_pose, built_part_type, agv_id);
            if (agv_id == 1){
                CorrectPose(built_pose, update_pose, built_part_type, agv_id, arm1);
            }else if(agv_id ==2){
                CorrectPose(built_pose, update_pose, built_part_type, agv_id, arm2);
            }


            order_update_copy[built_part_type].erase(std::remove(order_update_copy[built_part_type].begin(),
                    order_update_copy[built_part_type].end(), update_pose),
                            order_update_copy[built_part_type].end());

            built_kit_product_type_pose_[built_part_type].erase(std::remove(built_kit_product_type_pose_[built_part_type].begin(),
                    built_kit_product_type_pose_[built_part_type].end(), built_pose),
                    built_kit_product_type_pose_[built_part_type].end());
        }
    }
}

void AriacSensorManager::addParts(int agv_id){

//    int agv_id = 2;

    std::cout << "[AriacSensorManager][AddParts] : Inside the AddParts : "  << "\n";

    for(auto const& pair : order_update_copy){

        auto part_type = pair.first;

        if(pair.second.size() >0){

            for(auto part_pose : pair.second){

                std::pair<std::string,geometry_msgs::Pose> type_pose_pair;
                type_pose_pair.first = part_type;
                type_pose_pair.second = part_pose;

                if (agv_id == 1){
                    PickAndPlace(type_pose_pair, agv_id, arm1);
                }else if(agv_id ==2){
                    PickAndPlace(type_pose_pair, agv_id, arm2);
                }
            }
        }
    }
}

void AriacSensorManager::ReExecute(int agv_id){

//    int agv_id = 2;

    std::cout << "[AriacSensorManager][ReExecute] : Inside the ReExecute : "  << "\n";

    for(auto const& pair : order_update_product_type_pose_){

        auto part_type = pair.first;

        for(auto part_pose : pair.second){

            std::pair<std::string,geometry_msgs::Pose> type_pose_pair;
            type_pose_pair.first = part_type;
            type_pose_pair.second = part_pose;

            if (agv_id == 1){
                PickAndPlace(type_pose_pair, agv_id, arm1);
            }else if(agv_id ==2){
                PickAndPlace(type_pose_pair, agv_id, arm2);
            }


//            PickAndPlace(type_pose_pair, agv_id);

        }
    }
}

void AriacSensorManager::UpdateKit(int agv_id){

    // First built a map for new updated orders --> may be we should do this in the order callback.

    // First, Check for number of Parts to be removed.
    WhatToRemove();

    for(auto const& pair : parts_to_remove_product_type_pose_){
        std::cout << "[AriacSensorManager][UpdateKit] : Parts to remove:  part type is " << pair.first << std::endl;
        auto parts = parts_to_remove_product_type_pose_[pair.first];

        for (auto pa : parts){
            std::cout << "[AriacSensorManager][UpdateKit] : Parts to remove: part pose is " << pa << std::endl;
        }
    }

//    std::cout << "[AriacSensorManager][UpdateKit] : Total Number of Parts to remove " << NumPartsToRemove << std::endl;

    for(auto const& pair : built_kit_product_type_pose_){
        std::cout << "[AriacSensorManager][UpdateKit] : After removal part type in built_kit is " << pair.first << std::endl;
        auto parts = built_kit_product_type_pose_[pair.first];

        for (auto pa : parts){
            std::cout << "[AriacSensorManager][UpdateKit] : After removal part pose in built_kit is " << pa << std::endl;
        }
    }

    // Second, Check for number of parts to be changed positions
    WhatToModify();

    std::cout << "[AriacSensorManager][UpdateKit] : Total Number of Parts to Modify " << NumPartsToModify << std::endl;
    std::cout << "[AriacSensorManager][UpdateKit] : Total Number of Parts to remove after WhatToModify " << NumPartsToRemove << std::endl;

    for(auto const& pair : built_kit_product_type_pose_){
        std::cout << "[AriacSensorManager][UpdateKit] : After whatToModify part type in built_kit is " << pair.first << std::endl;
        auto parts = built_kit_product_type_pose_[pair.first];

        for (auto pa : parts){
            std::cout << "[AriacSensorManager][UpdateKit] : After whatToModify part pose in built_kit is " << pa << std::endl;
        }
    }

    // Third, Check for number of Parts to be Added.
    WhatToAdd();

//    for(auto const& pair : parts_to_place_in_other_tray){
//        std::cout << "[AriacSensorManager][UpdateKit] : After whatToModify part type in parts_to_place_in_other_tray is " << pair.first << std::endl;
//        auto parts = parts_to_place_in_other_tray[pair.first];
//
//        for (auto pa : parts){
//            std::cout << "[AriacSensorManager][UpdateKit] : After whatToModify part pose in parts_to_place_in_other_tray is " << pa << std::endl;
//        }
//    }


    int TotalNumberOfChanges = NumPartsToRemove + NumPartsToModify + NumPartsToAdd;
    int threshold = 1000;

    std::cout << "[AriacSensorManager][UpdateKit] : Total Number of Changes " << TotalNumberOfChanges << std::endl;

    // Find the total number of changes to be done and compare with a threshold. Based on this comparison
    // decide whether to proceed with the changes/ build a kit


    // If number of changes is less than threshold:
    if (TotalNumberOfChanges < threshold){

        // Call the function to removePart()
        if (agv_id == 1){
            removeParts(agv_id, arm1); // we will place the conveyor belt part on the belt
        }else if(agv_id ==2){
            removeParts(agv_id, arm2);
        }

        // Call the function to put the conveyor belt parts to other tray
        RemoveBeltParts();

        // Call the function to modifyPosition()
        modifyPose(agv_id);

        for(auto const& pair : order_update_copy){
            std::cout << "[AriacSensorManager][UpdateKit] : After whatToModify part type in order_update is " << pair.first << std::endl;
            auto parts = order_update_copy[pair.first];

            for (auto pa : parts){
                std::cout << "[AriacSensorManager][UpdateKit] : After whatToModify part type in order_update is " << pa << std::endl;
            }
        }

        for(auto const& pair : parts_back_from_tray){

            auto part_type = pair.first;
            auto droppick_pair = pair.second;

            std::cout << "[AriacSensorManager][UpdateKit] : After whatToModify part type in parts_back_from_tray is " << pair.first << std::endl;

            for (auto const& pa: droppick_pair){
                std::cout << "[AriacSensorManager][UpdateKit] : After whatToModify drop_pose in parts_back_from_tray " << pa.first << std::endl;
                std::cout << "[AriacSensorManager][UpdateKit] : After whatToModify pick_pose in parts_back_from_tray " << pa.second << std::endl;
            }
        }

        // Call AddBeltParts() to put the parts back into the Build Kit Tray
        AddBeltParts();
        // Call the function to addParts()
        addParts(agv_id);  // here we add new parts

    }else{

        SubmitAGV(agv_id);
        ros::Duration(15.0).sleep();
        ReExecute(agv_id);
    }
    // Else if number of changes is greater that threshold:
    // Call the function to ExecuteOrder() for order_0_update_0
}

void AriacSensorManager::SubmitAGV(int num) {
    std::string s = std::to_string(num);
    ros::ServiceClient start_client =
            sensor_nh_.serviceClient<osrf_gear::AGVControl>("/ariac/agv"+s);
    if (!start_client.exists()) {
        ROS_INFO("Waiting for the client to be ready...");
        start_client.waitForExistence();
        ROS_INFO("Service started.");
    }

    osrf_gear::AGVControl srv;
    // srv.request.kit_type = "order_0_kit_0";
    start_client.call(srv);

    if (!srv.response.success) {
        ROS_ERROR_STREAM("Service failed!");
    } else
        ROS_INFO("Service succeeded.");
}

void AriacSensorManager::qc_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){

    quality_image1 = *image_msg;
    if (image_msg->models.size() == 0){
        qc_2_redFlag = false;
    }
    else {
        ROS_WARN_STREAM_ONCE("[qc_2_callback] detected faulty part");
        qc_2_redFlag = true;
    }
}

void AriacSensorManager::qc_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){

    quality_image2 = *image_msg;
    if (image_msg->models.size() == 0)
        qc_1_redFlag = false;
    else{
        ROS_WARN_STREAM("[qc_1_callback]: faulty part detected");
        qc_1_redFlag = true;
    }
}

void AriacSensorManager::CheckQuality(int tray_id){

    osrf_gear::LogicalCameraImage quality_image;
    if(tray_id ==1){
        quality_image = quality_image1;
    }else{
        quality_image = quality_image2;
    }

    auto model =  quality_image.models;
    auto part_pose = model[0].pose;

    auto qc_frame = "quality_control_sensor_" + std::to_string(tray_id) + "_frame";

    // get the part pose of faulty product
    auto part_pose_Q_to_W = TransformPoses(part_pose, qc_frame, "world");

    // get the part type of part_pose_Q_to_W
    // string part_type = getPartTypeFromQC(current_parts_agv2_, camera_ID, part_pose_kit);

    // Now get the LogicalCamera msg
    // Loop through LogicalCamera msg and compare the part_pose_Q_to_W and part_type  --> finding if it exists
    // if exists --> call PickAndThrow() and save that thrown away part to a map
    // parts_to_be_added --> this map needs to be iterate before submitting the AGV
    // and the PickAndPlace() needs to be called to add the parts_to_be_added
}

//
//void AriacSensorManager::qc_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
//    if (image_msg->models.size() == 0){
//        qc_2_redFlag = false;
//    }
//    else
//    {
//        ROS_WARN_STREAM_ONCE("[qc_2_callback] detected faulty part");
//        qc_2_redFlag = true;
//        std::string camera_ID = "lc_agv_" + std::to_string(2) + "_frame";
//        std::string qc_frame = "quality_control_sensor_" + std::to_string(2) + "_frame";
//        for (auto& model : image_msg->models)
//        {
//            auto part_pose = model.pose;
//            geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;
//            StampedPose_in.header.frame_id = qc_frame;
//            StampedPose_in.pose = model.pose;
//            StampedPose_out = tfBuffer.transform(StampedPose_in, "world");
//            geometry_msgs::Pose part_pose_kit = StampedPose_out.pose;
//
//            string part_type = getPartTypeFromQC(current_parts_agv2_, camera_ID, part_pose_kit);
//
//            if (!part_type.empty()) {
//                if (current_faulty_parts_on_tray2[part_type].empty())
//                    current_faulty_parts_on_tray2[part_type].emplace_back(part_pose_kit);
//                else
//                {
//                    for (auto pose: current_faulty_parts_on_tray2[part_type]) {
//                        ROS_WARN_STREAM("pose" << pose);
//                        ROS_WARN_STREAM("part_pose_kit" << part_pose_kit);
//                        if (!comparePose(pose, part_pose_kit)) {
//                            current_faulty_parts_on_tray2[part_type].emplace_back(part_pose_kit);
//                        }
//                    }
//                    ROS_ERROR_STREAM("[qc_2_callback] current_faulty_parts_on_tray2[part_type].size() = "
//                                             << current_faulty_parts_on_tray2[part_type].size());
//                }
//            }
//        }
//    }
//}

//void AriacSensorManager::qc_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
//    if (image_msg->models.size() == 0)
//        qc_1_redFlag = false;
//    else
//    {
//        ROS_WARN_STREAM("[qc_1_callback]: faulty part detected");
//        qc_1_redFlag = true;
//        std::string camera_ID = "lc_agv_" + std::to_string(1) + "_frame";
//        std::string qc_frame = "quality_control_sensor_" + std::to_string(1) + "_frame";
//        for (auto& model : image_msg->models)
//        {
//            auto part_pose = model.pose;
//            geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;
//            StampedPose_in.header.frame_id = qc_frame;
//            StampedPose_in.pose = model.pose;
//            StampedPose_out = tfBuffer.transform(StampedPose_in, "world");
//            geometry_msgs::Pose part_pose_kit = StampedPose_out.pose;
//
//            string part_type = getPartTypeFromQC(current_parts_agv1_, camera_ID, part_pose_kit);
//
//            if (!part_type.empty()) {
//                ROS_INFO_STREAM("[qc_1_callback]: Inside if statement");
//                ROS_INFO_STREAM("[AriacSensorManager][qc_1_callback]: faulty part_type in Kit Frame: "<< part_type);
//                ROS_INFO_STREAM("[AriacSensorManager][qc_1_callback]: faulty part_pose_kit in Kit Frame: "<< part_pose_kit);
//                current_faulty_parts_on_tray1[part_type].emplace_back(part_pose_kit);
//            }
//        }
//    }
//}


//string AriacSensorManager::getPartTypeFromQC(const osrf_gear::LogicalCameraImage& lc_agv_img,
//                                             string camera_ID, geometry_msgs::Pose target_pose)
//{
//    string product_type;
//    for (auto& model : lc_agv_img.models)
//    {
//        auto part_type = model.type;
//
//        geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;
//        StampedPose_in.header.frame_id = camera_ID;
//        StampedPose_in.pose = model.pose;
//        StampedPose_out = tfBuffer.transform(StampedPose_in, "world");
//        geometry_msgs::Pose part_pose = StampedPose_out.pose;
//
//        ROS_WARN_STREAM("[getPartTypeFromQC]: Identify if " << part_type << " in " << camera_ID << " is faulty");
//        bool compare_result = comparePose(part_pose, target_pose);
//        if (compare_result) {
//            ROS_WARN_STREAM("[getPartTypeFromQC]: compared true and get product type: " << part_type);
//            return part_type;
//        }
//    }
//    return "";
//}