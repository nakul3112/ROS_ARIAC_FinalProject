// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//         http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


// #include "MyCompetitionClass.h"
#include "AriacSensorManager.h"
#include <osrf_gear/AGVControl.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf/tf.h>
#include <iostream>

 void start_competition(ros::NodeHandle & nh) {
     // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
     ros::ServiceClient start_client =
         nh.serviceClient<std_srvs::Trigger>("/ariac/start_competition");

     // If it's not already ready, wait for it to be ready.
     // Calling the Service using the client before the server is ready would fail.
     if (!start_client.exists()) {
         ROS_INFO("Waiting for the competition to be ready...");
         start_client.waitForExistence();
         ROS_INFO("Competition is now ready.");
     }
     ROS_INFO("Requesting competition start...");
     ros::Duration(10).sleep();
     std_srvs::Trigger srv;    // Combination of the "request" and the "response".
     start_client.call(srv);    // Call the start Service.
     if (!srv.response.success)    // If not successful, print out why.
         ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
     else
         ROS_INFO("Competition started!");
 }


int main(int argc, char ** argv) {
    // Last argument is the default name of the node.
    ros::init(argc, argv, "main_node");
    ros::NodeHandle main_nh;
    ros::AsyncSpinner spinner(0);
    AriacSensorManager sensor;

    // MyCompetitionClass comp_class(main_nh);

    ROS_INFO_STREAM("Wait for competition start...");
    start_competition(main_nh);

//    ros::Duration(5).sleep();
//    sensor.PickAndPlace();

//    ros::spin();
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
