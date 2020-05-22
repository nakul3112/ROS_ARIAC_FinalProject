#include "RobotController.h"

/**
 * Constructor for the robot
 * Class attributes are initialized in the constructor init list
 * You can instantiate another robot by passing the correct parameter to the constructor
 */
RobotController::RobotController(std::string arm_id) :
    robot_controller_nh_("/ariac/"+arm_id),
    robot_controller_options("manipulator",
            "/ariac/"+arm_id+"/robot_description",
            robot_controller_nh_),
    robot_move_group_(robot_controller_options),
    id {arm_id}
{
    ros::AsyncSpinner armSpinner(0);
    armSpinner.start();
    ROS_WARN("[RobotController]:[Constructor]: Called Class");

    robot_move_group_.setPlanningTime(50);
    robot_move_group_.setNumPlanningAttempts(20);
    robot_move_group_.setPlannerId("RRTConnectkConfigDefault");
    robot_move_group_.setMaxVelocityScalingFactor(0.9);
    robot_move_group_.setMaxAccelerationScalingFactor(0.9);
    // robot_move_group_.setEndEffector("moveit_ee");
    robot_move_group_.allowReplanning(true);

    // -------------- Flipping Poses -------------------//
    RailRight["elbow_joint"] = 1.63;
    RailRight["linear_arm_actuator_joint"] = 0.26;
    RailRight["shoulder_lift_joint"] = -1.63;
    RailRight["shoulder_pan_joint"] = 1.63;
    RailRight["wrist_1_joint"] = 3.14;
    RailRight["wrist_2_joint"] = -1.63;
    RailRight["wrist_3_joint"] = 0;


    RailLeft["elbow_joint"] = -1.50;
    RailLeft["linear_arm_actuator_joint"] = -0.195;
    RailLeft["shoulder_lift_joint"] = -1.63;
    RailLeft["shoulder_pan_joint"] = 1.50;
    RailLeft["wrist_1_joint"] = 0.00;
    RailLeft["wrist_2_joint"] = 1.64;
    RailLeft["wrist_3_joint"] = 0.00;

    // -------------------------------------------------//


    if (arm_id == "arm1"){

        home_joint_pose_1["linear_arm_actuator_joint"] = 0.83;
        home_joint_pose_1["shoulder_pan_joint"] = 1.48;
        home_joint_pose_1["shoulder_lift_joint"] = -0.58;
        home_joint_pose_1["elbow_joint"] = 1.14;                //nakul
        home_joint_pose_1["wrist_1_joint"] = 4.18;
        home_joint_pose_1["wrist_2_joint"] = -1.51;
        home_joint_pose_1["wrist_3_joint"] = 0;

//        home_joint_pose_2["linear_arm_actuator_joint"] = -0.21;
//        home_joint_pose_2["shoulder_pan_joint"] = 2.86;
//        home_joint_pose_2["shoulder_lift_joint"] = -1.2;
//        home_joint_pose_2["elbow_joint"] = 2.01;
//        home_joint_pose_2["wrist_1_joint"] = 4.18;
//        home_joint_pose_2["wrist_2_joint"] = -1.55;
//        home_joint_pose_2["wrist_3_joint"] = 0;

        home_joint_pose_2["linear_arm_actuator_joint"] = -0.21;
        home_joint_pose_2["shoulder_pan_joint"] = 2.86;
        home_joint_pose_2["shoulder_lift_joint"] = -1.2;
        home_joint_pose_2["elbow_joint"] = 2.39;
        home_joint_pose_2["wrist_1_joint"] = 3.68;
        home_joint_pose_2["wrist_2_joint"] = -1.55;
        home_joint_pose_2["wrist_3_joint"] = 0;

        check_qc_pose = home_joint_pose_1;

        conveyer_pose["linear_arm_actuator_joint"] = -0.12;
        conveyer_pose["shoulder_pan_joint"] = 0.44;
        conveyer_pose["shoulder_lift_joint"] = -0.68;
        conveyer_pose["elbow_joint"] = 1.15;
        conveyer_pose["wrist_1_joint"] = 4.31;
        conveyer_pose["wrist_2_joint"] = -1.57;
        conveyer_pose["wrist_3_joint"] = -1.04;

        rail_pick_trans_pose["linear_arm_actuator_joint"] = 0.42;
//        rail_pick_trans_pose["shoulder_pan_joint"] = -1.57;
        rail_pick_trans_pose["shoulder_pan_joint"] = 4.71;
        rail_pick_trans_pose["shoulder_lift_joint"] = -1.07;
        rail_pick_trans_pose["elbow_joint"] = 1.75;
        rail_pick_trans_pose["wrist_1_joint"] = 4.02;
        rail_pick_trans_pose["wrist_2_joint"] = -1.57;
        rail_pick_trans_pose["wrist_3_joint"] = 0;

        belt_pickup_pose.position.x = 1.225;
        belt_pickup_pose.position.y = 1.418089;
        belt_pickup_pose.position.z = 0.912;
        belt_pickup_pose.orientation.x = 0;
        belt_pickup_pose.orientation.y = 0.707;
        belt_pickup_pose.orientation.z = 0;
        belt_pickup_pose.orientation.w = 0.707;

        throw_away_pose.position.x = 0.2-0.4;
        throw_away_pose.position.y = 3.0 ;
        throw_away_pose.position.z = 0.85 + 0.2;
        throw_away_pose.orientation.x = 0;
        throw_away_pose.orientation.y = 0;
        throw_away_pose.orientation.z = 0;
        throw_away_pose.orientation.w = 1;

        this->SendRobotTo(home_joint_pose_1);

        offset_ = 0.02;
//        offset_ = 0.005;
        Busy = false;
    }
    else{
        ROS_INFO_STREAM("Arm2 should go home");

//        home_joint_pose_2["linear_arm_actuator_joint"] = 0;
//        home_joint_pose_2["shoulder_pan_joint"] = 3.14;
//        home_joint_pose_2["shoulder_lift_joint"] = -1.2;
//        home_joint_pose_2["elbow_joint"] = 2.01;
//        home_joint_pose_2["wrist_1_joint"] = 4.15;
//        home_joint_pose_2["wrist_2_joint"] = -1.51;
//        home_joint_pose_2["wrist_3_joint"] = 0;

        home_joint_pose_2["linear_arm_actuator_joint"] = 0;
        home_joint_pose_2["shoulder_pan_joint"] = 3.14;
        home_joint_pose_2["shoulder_lift_joint"] = -1.2;
        home_joint_pose_2["elbow_joint"] = 2.39;
        home_joint_pose_2["wrist_1_joint"] = 3.68;
        home_joint_pose_2["wrist_2_joint"] = -1.51;
        home_joint_pose_2["wrist_3_joint"] = 0;

//        home_joint_pose_2["linear_arm_actuator_joint"] = -0.21;
//        home_joint_pose_2["shoulder_pan_joint"] = 2.86;
//        home_joint_pose_2["shoulder_lift_joint"] = -1.2;
//        home_joint_pose_2["elbow_joint"] = 2.39;
//        home_joint_pose_2["wrist_1_joint"] = 3.68;
//        home_joint_pose_2["wrist_2_joint"] = -1.55;
//        home_joint_pose_2["wrist_3_joint"] = 0;


        home_joint_pose_1["linear_arm_actuator_joint"] = -0.82;
        home_joint_pose_1["shoulder_pan_joint"] = 4.62;
        home_joint_pose_1["shoulder_lift_joint"] = -0.57;
        home_joint_pose_1["elbow_joint"] = 1.13;
        home_joint_pose_1["wrist_1_joint"] = 4.15;
        home_joint_pose_1["wrist_2_joint"] = -1.51;
        home_joint_pose_1["wrist_3_joint"] = 0;

        check_qc_pose = home_joint_pose_1;

        conveyer_pose["linear_arm_actuator_joint"] = -0.20;
        conveyer_pose["shoulder_pan_joint"] = 5.53;
        conveyer_pose["shoulder_lift_joint"] = -0.67;
        conveyer_pose["elbow_joint"] = 1.17;
        conveyer_pose["wrist_1_joint"] = 4.17;
        conveyer_pose["wrist_2_joint"] = -1.53;
        conveyer_pose["wrist_3_joint"] = 0.91;

        rail_pick_trans_pose["linear_arm_actuator_joint"] = -0.42;
        rail_pick_trans_pose["shoulder_pan_joint"] = 1.57;
        rail_pick_trans_pose["shoulder_lift_joint"] = -1.07;
        rail_pick_trans_pose["elbow_joint"] = 1.75;
        rail_pick_trans_pose["wrist_1_joint"] = 4.02;
        rail_pick_trans_pose["wrist_2_joint"] = -1.57;
        rail_pick_trans_pose["wrist_3_joint"] = 0;

        throw_away_pose.position.x = 0.2-0.4;
        throw_away_pose.position.y = -3.35 + 0.5;
        throw_away_pose.position.z = 0.85 + 0.2;
        throw_away_pose.orientation.x = 0;
        throw_away_pose.orientation.y = 0;
        throw_away_pose.orientation.z = 0;
        throw_away_pose.orientation.w = 1;

        belt_pickup_pose.position.x = 1.225;
        belt_pickup_pose.position.y = -1.75;
        belt_pickup_pose.position.z = 0.912;
        belt_pickup_pose.orientation.x = 0;
        belt_pickup_pose.orientation.y = 0.707;
        belt_pickup_pose.orientation.z = 0;
        belt_pickup_pose.orientation.w = 0.707;

//        offset_ = 0.0275;
//        offset_ = 0.005;
        offset_ = 0.02;

        this->SendRobotTo(home_joint_pose_1);
        Busy = false;
    }


    //-- offset used for picking up parts
    //-- For the pulley_part, the offset is different since the pulley is thicker

    //--topic used to get the status of the gripper
    gripper_subscriber_ = gripper_nh_.subscribe("/ariac/" + arm_id + "/gripper/state", 1000, 
            & RobotController::GripperCallback, this);

    /////////////modified
    // SendRobotTo();
    // SendRobotTo(home_joint_pose_0);


    robot_tf_listener_.waitForTransform(arm_id + "_linear_arm_actuator", arm_id + "_ee_link",
        ros::Time(0), ros::Duration(10));
    robot_tf_listener_.lookupTransform("/" + arm_id + "_linear_arm_actuator", 
        "/" + arm_id + "_ee_link", ros::Time(0), robot_tf_transform_);


    fixed_orientation_.x = robot_tf_transform_.getRotation().x();
    fixed_orientation_.y = robot_tf_transform_.getRotation().y();
    fixed_orientation_.z = robot_tf_transform_.getRotation().z();
    fixed_orientation_.w = robot_tf_transform_.getRotation().w();

    tf::quaternionMsgToTF(fixed_orientation_,q);
    tf::Matrix3x3(q).getRPY(roll_def_,pitch_def_,yaw_def_);


    end_position_ = home_joint_pose_1;
    //  end_position_[0] = 2.2;
    //  end_position_[1] = 4.5;
    //  end_position_[2] = 1.2;


    robot_tf_listener_.waitForTransform("world", arm_id + "_ee_link", ros::Time(0),
                                            ros::Duration(10));
    robot_tf_listener_.lookupTransform("/world", "/" + arm_id + "_ee_link", ros::Time(0),
                                           robot_tf_transform_);

    home_cart_pose_.position.x = robot_tf_transform_.getOrigin().x();
    home_cart_pose_.position.y = robot_tf_transform_.getOrigin().y();
    home_cart_pose_.position.z = robot_tf_transform_.getOrigin().z();
    home_cart_pose_.orientation.x = robot_tf_transform_.getRotation().x();
    home_cart_pose_.orientation.y = robot_tf_transform_.getRotation().y();
    home_cart_pose_.orientation.z = robot_tf_transform_.getRotation().z();
    home_cart_pose_.orientation.w = robot_tf_transform_.getRotation().w();

    std::string id_val = arm_id.substr(arm_id.size()-1, 1);
    // stringkit_tray_name = "kit_tray_" + id_val;
    agv_tf_listener_.waitForTransform("world", "kit_tray_" + id_val,
                                      ros::Time(0), ros::Duration(10));
    agv_tf_listener_.lookupTransform("/world", "/kit_tray_" + id_val,
                                     ros::Time(0), agv_tf_transform_);
    agv_position_.position.x = agv_tf_transform_.getOrigin().x();
    agv_position_.position.y = agv_tf_transform_.getOrigin().y();
    agv_position_.position.z = agv_tf_transform_.getOrigin().z();
    // agv_position_.position.z = agv_tf_transform_.getOrigin().z() + 4 * offset_;

    // define the store matrix in the kit_tray_i coordinates
    double x_separate = 0.23;
    double y_separate = 0.36;
    geometry_msgs::Pose first_tray_store_pose;

    first_tray_store_pose.position.x = x_separate/2;
    first_tray_store_pose.position.y = y_separate/2;
    first_tray_store_pose.position.z = 0;
    first_tray_store_pose.orientation.x = 0;
    first_tray_store_pose.orientation.y = 0;
    first_tray_store_pose.orientation.z = 0;
    first_tray_store_pose.orientation.w = 0;

    tray_store_poses.emplace_back(first_tray_store_pose);
    for (int i = 1; i < 4; ++i) {
        if (i == 1) {
            auto stroe_pose = tray_store_poses[0];
            stroe_pose.position.x -= x_separate;
            tray_store_poses.emplace_back(stroe_pose);
        }
        if (i == 2) {
            auto stroe_pose = tray_store_poses[0];
            stroe_pose.position.y -= y_separate;
            tray_store_poses.emplace_back(stroe_pose);
        }
        if (i == 3) {
            auto stroe_pose = tray_store_poses[0];
            stroe_pose.position.x -= x_separate;
            stroe_pose.position.y -= y_separate;
            tray_store_poses.emplace_back(stroe_pose);
        }
    }

    // first_tray_store_pose.position.x = agv_position_.position.x - x_separate/2;
    // first_tray_store_pose.position.y = agv_position_.position.y - y_separate/2;
    // first_tray_store_pose.position.z = agv_position_.position.z;

    // tray_store_poses.emplace_back(first_tray_store_pose);
    // for (int i = 1; i < 4; ++i) {
    //     if (i == 1) {
    //         auto stroe_pose = tray_store_poses[0];
    //         stroe_pose.position.x += x_separate;
    //         tray_store_poses.emplace_back(stroe_pose);
    //     }
    //     if (i == 2) {
    //         auto stroe_pose = tray_store_poses[0];
    //         stroe_pose.position.y += y_separate;
    //         tray_store_poses.emplace_back(stroe_pose);
    //     }
    //     if (i == 3) {
    //         auto stroe_pose = tray_store_poses[0];
    //         stroe_pose.position.x += x_separate;
    //         stroe_pose.position.y += y_separate;
    //         tray_store_poses.emplace_back(stroe_pose);
    //     }
    // }

    gripper_client_ = robot_controller_nh_.serviceClient<osrf_gear::VacuumGripperControl>(
            "/ariac/" + arm_id + "/gripper/control");
    counter_ = 0;
    drop_flag_ = false;
    y_comp = 0;
}

RobotController::~RobotController() {}


bool RobotController::Planner() {
    // ROS_INFO_STREAM("Planning started...");
    // ros::AsyncSpinner armSpinner;
    // armSpinner.start();
    if (robot_move_group_.plan(robot_planner_) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        plan_success_ = true;
        // ROS_INFO_STREAM("Planner succeeded!");
    } else {
        plan_success_ = false;
        ROS_WARN_STREAM(id + " Planner failed!");
    }

    return plan_success_;
}

void RobotController::Execute() {
    ros::AsyncSpinner armSpinner(0);
    armSpinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(1.0).sleep();
    }
}

void RobotController::GoToTarget1(const geometry_msgs::Pose& pose) {
    // ROS_INFO_STREAM("Inside GoToTarget");
    target_pose_.orientation = fixed_orientation_;
    target_pose_.position = pose.position;
    ros::AsyncSpinner armSpinner(0);
    robot_move_group_.setPoseTarget(target_pose_);
    armSpinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(1).sleep();
    }
    // ROS_INFO_STREAM(id + " Point reached...");
}

void RobotController::GoToTarget(
        std::initializer_list<geometry_msgs::Pose> list) {
    // ROS_INFO_STREAM("Inside GoToTarget by List");
    ros::AsyncSpinner armSpinner(0);
    armSpinner.start();

    std::vector<geometry_msgs::Pose> waypoints;
    for (auto i : list) {
        i.orientation.x = fixed_orientation_.x;
        i.orientation.y = fixed_orientation_.y;
        i.orientation.z = fixed_orientation_.z;
        i.orientation.w = fixed_orientation_.w;
        waypoints.emplace_back(i);
    }
    
    moveit_msgs::RobotTrajectory traj;
    auto fraction =
            robot_move_group_.computeCartesianPath(waypoints, 0.01, 0.0, traj, true);

    // ROS_WARN_STREAM("Fraction: " << fraction * 100);
    // ros::Duration(5.0).sleep();

    robot_planner_.trajectory_ = traj;
    //if (fraction >= 0.3) {
    robot_move_group_.execute(robot_planner_);
    ros::Duration(1.0).sleep();
}


void RobotController::SendRobotTo(std::map<std::string, double> desire_joint_states) {
    robot_move_group_.setJointValueTarget(desire_joint_states);
    // this->execute();
    ros::AsyncSpinner armSpinner(0);
    armSpinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(0.6).sleep();
    }
}

void RobotController::RobotGoHome() {
    robot_move_group_.setJointValueTarget(home_joint_pose_1);
    // this->execute();
    ros::AsyncSpinner armSpinner(0);
    armSpinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(0.5).sleep();
    }
}

void RobotController::SendRobotTo(std::string joint_name, double joint_value) {
    std::vector<std::string> constJointNameRef = robot_move_group_.getJointNames();
    std::vector<double> jointVals = robot_move_group_.getCurrentJointValues();
    std::map<std::string, double> cur_joint_status;
    for (size_t i = 0; i < constJointNameRef.size(); ++i){
        if (constJointNameRef[i] == joint_name) {
            cur_joint_status[constJointNameRef[i]] = joint_value;
        }
        else {
            cur_joint_status[constJointNameRef[i]] = jointVals[i];
        }
        ROS_INFO_STREAM("Joint '" <<  constJointNameRef[i] << "' has set to " 
            << cur_joint_status[constJointNameRef[i]]);
    }
    this->SendRobotTo(cur_joint_status);

    // robot_move_group_.setJointValueTarget(joint_name, joint_value);
    // // this->execute();
    // ros::AsyncSpinner armSpinner(0);
    // armSpinner.start();
    // if (this->Planner()) {
    //     robot_move_group_.move();
    //     ros::Duration(1).sleep();
    // }
}

void RobotController::GripperCallback(
        const osrf_gear::VacuumGripperState::ConstPtr& grip) {
    gripper_state_ = grip->attached;
}

void RobotController::GripperToggle(const bool& state) {
    gripper_service_.request.enable = state;
    gripper_client_.call(gripper_service_);
    // ros::Duration(1.0).sleep();
    // if (gripper_client_.call(gripper_service_)) {
    // if (gripper_service_.response.success) {
    //     ROS_INFO_STREAM("Gripper activated!");
    // } else {
    //     ROS_WARN_STREAM("Gripper activation failed!");
    // }
    if (!gripper_service_.response.success)
        ROS_WARN_STREAM(id + " Gripper activation failed!");
}

bool RobotController::DropPart(geometry_msgs::Pose part_pose) {
    // counter_++;

    drop_flag_ = true; // Dropping process in progress
    bool successful_drop = false;
    ros::spinOnce();
    ROS_INFO_STREAM("[RobotController]:[DropPart]: Dropping Part...");
    ROS_INFO_STREAM("[RobotController]:[DropPart]: Before dropping Gripper State : " << gripper_state_);
//    ROS_INFO_STREAM("[RobotController]:[DropPart]: Before dropping Activated or not? " << int(gripper_service_.response.success));

    if (gripper_state_){//--while the part is still attached to the gripper
        auto temp_pose = part_pose;
        part_pose.position.z += offset_;
        temp_pose.position.z = part_pose.position.z + 0.5;
        this->GoToTarget({temp_pose, part_pose});
//        ros::Duration(0.5).sleep();
        // ROS_INFO_STREAM("Actuating the gripper...");
        if (!gripper_state_){
            successful_drop = false;
        }else{
            successful_drop = true;
        }

        this->GripperToggle(false);
    }

    drop_flag_ = false; // Dropping process completed! Dropping successful
    ros::Duration(0.3).sleep(); // I hope in this period gripper_state_ gets updated.

    ROS_INFO_STREAM("[RobotController]:[DropPart]: After dropping Gripper State : " << gripper_state_);
//    ROS_INFO_STREAM("[RobotController]:[DropPart]: After dropping Activated or not? " << int(gripper_service_.response.success));
    return successful_drop;
}

double RobotController::getRotationCompensate (const geometry_msgs::Pose& part_pose, 
    const geometry_msgs::Pose& drop_pose) 
{
    tf2::Quaternion part_Q(
        part_pose.orientation.x,
        part_pose.orientation.y,
        part_pose.orientation.z,
        part_pose.orientation.w);

    tf2::Quaternion drop_Q(
        drop_pose.orientation.x,
        drop_pose.orientation.y,
        drop_pose.orientation.z,
        drop_pose.orientation.w);

    // transform quaternion to RPY
    tf2::Matrix3x3 m_part(part_Q);
    tf2::Matrix3x3 m_drop(drop_Q);

    double part_R, part_P, part_Y;
    double drop_R, drop_P, drop_Y;

    m_part.getRPY(part_R, part_P, part_Y);
    m_drop.getRPY(drop_R, drop_P, drop_Y);

    return (drop_Y - part_Y);
}


bool RobotController::DropPart2(geometry_msgs::Pose part_pose) {
    // counter_++;

    drop_flag_ = true; // Dropping process in progress
    ros::spinOnce();
    ROS_INFO_STREAM("[RobotController]:[DropPart]: Dropping Part...");
    ROS_INFO_STREAM("[RobotController]:[DropPart]: Before dropping Gripper State : " << gripper_state_);
//    ROS_INFO_STREAM("[RobotController]:[DropPart]: Before dropping Activated or not? " << int(gripper_service_.response.success));

    if (gripper_state_){//--while the part is still attached to the gripper
        auto temp_pose = part_pose;
        part_pose.position.z += offset_;
        temp_pose.position.z = part_pose.position.z + 0.5;
        this->GoToTarget({temp_pose, part_pose});
//        ros::Duration(1.0).sleep();
        ROS_INFO_STREAM("y_comp = " << y_comp);
        this->SendRobotTo("wrist_2_joint", - y_comp);
        this->GripperToggle(false);
    }

    drop_flag_ = false; // Dropping process completed! Dropping successful
    ros::Duration(0.3).sleep(); // I hope in this period gripper_state_ gets updated.

    ROS_INFO_STREAM("[RobotController]:[DropPart]: After dropping Gripper State : " << gripper_state_);
//    ROS_INFO_STREAM("[RobotController]:[DropPart]: After dropping Activated or not? " << int(gripper_service_.response.success));
    return gripper_state_;
}


bool RobotController::PickPart(const geometry_msgs::Pose& part_pose, bool pick_once) {
    // stand_by > contact > actual

    auto contact_pose = part_pose;
    contact_pose.position.z = part_pose.position.z + offset_;
    auto stand_by_pose = part_pose;
    stand_by_pose.position.z += 0.2;
    this->GripperToggle(true);
    this->GoToTarget({stand_by_pose, contact_pose});
    ros::spinOnce();
    while (!pick_once && !gripper_state_) {
        contact_pose.position.z -= 0.005;
        this->GripperToggle(true);
        this->GoToTarget({stand_by_pose, contact_pose});
        // ROS_INFO_STREAM("Actuating the gripper...");
        ros::spinOnce();
    }
    this->GoToTarget1(stand_by_pose);
    return gripper_state_;
}

bool RobotController::PickPart2(const geometry_msgs::Pose& part_pose, 
    const geometry_msgs::Pose& drop_pose) {
    // stand_by > contact > actual

    auto contact_pose = part_pose;
    contact_pose.position.z = part_pose.position.z + offset_;
    auto stand_by_pose = part_pose;
    stand_by_pose.position.z += 0.2;
    y_comp = this->getRotationCompensate(part_pose, drop_pose);
    this->GripperToggle(true);
    this->GoToTarget({stand_by_pose, contact_pose});
    ros::spinOnce();
    if (id == "arm2"){
        while (!gripper_state_) {
            // contact_pose.position.z -= 0.005;
            contact_pose.position.z -= 0.005;
            this->GripperToggle(true);
            this->GoToTarget({stand_by_pose, contact_pose});
            // ROS_INFO_STREAM("Actuating the gripper...");
            ros::spinOnce();
        }
    }
    // ROS_INFO_STREAM("Get things");

    // ROS_INFO_STREAM(id + "Going to waypoint...");
    this->GoToTarget1(stand_by_pose);
    return gripper_state_;
}

// void RobotController::BuiltKit(const geometry_msgs::Pose& pose, double yaw) {

// //    ROS_INFO_STREAM("[RobotController]:[BuiltKit]: pose recieved" << pose);
//     // ROS_INFO_STREAM("Inside GoToTarget");

//     geometry_msgs::Quaternion fixed_orientation_1;
//     tf2::Quaternion q_1;
//     geometry_msgs::Pose target_pose_kit;

//     double roll_def_1,pitch_def_1,yaw_def_1;

//     fixed_orientation_1 = fixed_orientation_;
//     tf2::fromMsg(fixed_orientation_1,q_1);
//     tf2::Matrix3x3(q_1).getRPY(roll_def_1,pitch_def_1,yaw_def_1);

//     q_1.setRPY(roll_def_1,pitch_def_1,yaw);
//     fixed_orientation_1 = tf2::toMsg(q_1);

//     target_pose_kit.orientation = fixed_orientation_1;
// //    target_pose_kit.orientation = pose.orientation;
//     target_pose_kit.position = pose.position;
//     ros::AsyncSpinner armSpinner(0);

//     if (gripper_state_){//--while the part is still attached to the gripper

//         robot_move_group_.setPoseTarget(target_pose_kit);
//         armSpinner.start();
//         if (this->Planner()) {
//             robot_move_group_.move();
//             ros::Duration(1).sleep();
//         }
//         ros::Duration(1.0).sleep();
//         // ROS_INFO_STREAM("Actuating the gripper...");
//         this->GripperToggle(false);

//     }
//     ROS_INFO_STREAM(id + " Orientation to place..."<< target_pose_kit.orientation);

// }