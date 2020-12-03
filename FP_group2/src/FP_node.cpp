// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <algorithm>
#include <vector>

#include <ros/ros.h>

#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <nist_gear/Proximity.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //--needed for tf2::Matrix3x3
#include <nist_gear/AGVControl.h>
#include "competition.h"
#include "utils.h"
#include "gantry_control.h"

#include <tf2/LinearMath/Quaternion.h>

bool submitOrder(int AVG_id, std::string shipment_type){
    ROS_INFO("[submitOrder] Submitting order via AVG");

    // Create a node to call service from. Would be better to use one existing node
    // rather than creating a new one every time
    ros::NodeHandle node;

    // Create a Service client for the correct service, i.e. '/ariac/agv{AVG_id}'
    ros::ServiceClient avg_client;

    // Assign the service client to the correct service
    if(AVG_id == 1){
        avg_client = node.serviceClient<nist_gear::AGVControl>("/ariac/agv1");
    }else if(AVG_id == 2){
        avg_client = node.serviceClient<nist_gear::AGVControl>("/ariac/agv2");
    }else{
        ROS_ERROR_STREAM("[submitOrder] No AVG with id " << AVG_id <<". Valid ids are 1 and 2 only");
    }

    // Wait for client to start
    if (!avg_client.exists()) {
        avg_client.waitForExistence();
    }

    // Debug what you're doing
    ROS_INFO_STREAM("[submitOrder] Sending AVG " << AVG_id << " to submit order");

    // Create the message and assign the shipment type to it
    nist_gear::AGVControl srv;
    srv.request.shipment_type = shipment_type;

    // Send message and retrieve response
    avg_client.call(srv);
    if (!srv.response.success) {  // If not successful, print out why.
        ROS_ERROR_STREAM("[submitOrder]  Failed to submit: " << srv.response.message);
    } else {
        ROS_INFO("[submitOrder] Submitted");
    }

    return srv.response.success;
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "FP_node");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(8);
    spinner.start();

    Competition comp(node);
    comp.init();


    int Max_number_of_cameras = 17, Max_number_of_breakbeams = 29;
    std::ostringstream otopic;
    std::string topic;
    std::array<std::array<modelparam, 36>, 17> logicam, logicam2, logicam12;
    std::array < std::array < std::array < part, 10 >, 5 >, 5 > or_details, or_details_new, order_call;
    std::array < std::array < std::array < int, 10 >, 5 >, 5 > order_flag = {0};
    std::array<int, 5> completed2 = {0};
    part faulty_part, faulty_pose;
    double Model_adjust =0;     //Only required for AGV1 cases. offset of 0.17 observed in Camera and target.
    bool break_beam;
    std::string c_state = comp.getCompetitionState();
    comp.getClock();

    ros::Subscriber logical_camera_subscriber_[Max_number_of_cameras];


    for (int x = 0; x < Max_number_of_cameras; x++) {
        otopic.str("");
        otopic.clear();
        otopic << "/ariac/logical_camera_" << (x);
        topic = otopic.str();
        logical_camera_subscriber_[x] = node.subscribe<nist_gear::LogicalCameraImage>(topic, 10, boost::bind(
                &Competition::logical_camera_callback, &comp, _1, x));
    }

    ros::Subscriber breakbream_sensor_subscriber_[Max_number_of_breakbeams];
    for (int x = 0; x < Max_number_of_breakbeams; x++) {
        otopic.str("");
        otopic.clear();
        otopic << "/ariac/breakbeam_" << (x);
        topic = otopic.str();
        breakbream_sensor_subscriber_[x] = node.subscribe<nist_gear::Proximity>(topic, 10, boost::bind(
                &Competition::breakbeam_sensor_callback, &comp, _1, x));
    }

    GantryControl gantry(node);
    gantry.init();
    gantry.goToPresetLocation(gantry.start_);
    logicam = comp.getter_logicam_callback();
    order_call = comp.getter_part_callback();
    or_details[0] = order_call[0];
    int on_table_1 = 0, on_table_2 = 0, new_order = 0, index = 0, part_on_belt = 0;
    auto gap_id = comp.check_gaps();
    int x_loop = 0, check = 0;
    int on_belt = 0;
    std::array<std::array<int, 3>, 5> belt_part_arr = {0};
    comp.HumanDetection();

    // Initialization of variables and functions for move to preset location
    std::map <std::string, std::vector<PresetLocation>> presetLocation;
    std::string location;
    double loc_x, loc_y;
    gantry.initialPositions(presetLocation, comp.gap_nos, comp.Human, comp.Human_detected);
    ROS_INFO_STREAM("\nGap print:");
    for (auto i1=0; i1<3; i1++)
    {
        ROS_INFO_STREAM("\ngap "<<i1+1<<" "<<comp.gap_nos[i1]);
    }

    comp.PartonBeltCheck(comp.received_orders_, x_loop, logicam, belt_part_arr, on_belt);

    for (int i = comp.received_orders_.size() - 1; i >= 0; --i) {
        if (completed2[i]==1)
        {
            continue;
        }
        for (int j = 0; j < comp.received_orders_[i].shipments.size(); j++) {
            if (completed2[i] == 1)
                break;
            for (int k = 0; k < comp.received_orders_[i].shipments[j].products.size(); k++)
            {
                ROS_INFO_STREAM("\nIN k loop..");
                int count = 0, count1 = 0;
                ROS_INFO_STREAM("\n Print i="<<i<<", j="<<j<<", k="<<k);
                ROS_INFO_STREAM("\n Print comp.received_orders_.size()="<<comp.received_orders_.size());
                ROS_INFO_STREAM("\n Print comp.received_orders_[i].shipments.size()="<<comp.received_orders_[i].shipments.size());
                ROS_INFO_STREAM("\n Print comp.received_orders_[i].shipments[j].products.size()="<<comp.received_orders_[i].shipments[j].products.size());
                ROS_INFO_STREAM("\n AGV ID: "<<or_details[i][j][k].agv_id);
                ROS_INFO_STREAM("\n Order shipment name: "<<or_details[i][j][k].shipment);
                if (new_order)
                {
                    new_order = 0;
                    j=0;
                    k=0;
                }
                for (auto l = 0; l < comp.received_orders_[i].shipments[j].products.size(); l++)
                {
                    if (order_flag[i][j][l] == 1)
                        count1++;
                }
                if (count1 == comp.received_orders_[i].shipments[j].products.size())
                {
                    ROS_INFO_STREAM("shipment j=" << j << "completed..");
                    if (or_details[i][j][k].agv_id == "agv1")
                    {
                        ROS_INFO_STREAM("\n Submitting Order: " << or_details_new[i][j][k].shipment);
                        submitOrder(1, or_details[i][j][k].shipment);
                    }
                    else if (or_details[i][j][k].agv_id == "agv2")
                    {
                        ROS_INFO_STREAM("\n Submitting Order: " << or_details_new[i][j][k].shipment);
                        submitOrder(2, or_details[i][j][k].shipment);
                    }
                }
                if (order_flag[i][j][k] != 0)
                    continue;

                if (part_on_belt==0 && on_belt!=0)
                {
                    ROS_INFO_STREAM("Waiting for part to spawn on belt!!");
                    do {
                        //
                    }while(comp.beam_detect[0]==false);
                }
                logicam12 = comp.getter_logicam_callback();
                ROS_INFO_STREAM("\n Print i=" << i << ", j=" << j << ", k=" << k);
                ROS_INFO_STREAM("\n Print comp.received_orders_.size()=" << comp.received_orders_.size());
                ROS_INFO_STREAM("\n Print comp.received_orders_[i].shipments.size()="
                                        << comp.received_orders_[i].shipments.size());
                ROS_INFO_STREAM("\n Print comp.received_orders_[i].shipments[j].products.size()="
                                        << comp.received_orders_[i].shipments[j].products.size());
                ROS_INFO_STREAM("\n AGV ID: " << or_details[i][j][k].agv_id);
                ROS_INFO_STREAM("\n Order shipment name: " << or_details[i][j][k].shipment);
//                ROS_INFO_STREAM("\n parts on_belt : " << on_belt);

                //loop to pick up part from belt and place in bins 9 and 14 if required
                if ((part_on_belt < on_belt) && (comp.beam_detect[0]==true || !logicam12[12][0].type.empty()))
                {
                    on_belt = 2;
                    do
                        {
                        if (part_on_belt!=0)
                        {
                            ROS_INFO_STREAM("Waiting for part to spawn on belt!!");
                            do {
                                //
                            }while(comp.beam_detect[0]==false);
                        }
                        ROS_INFO_STREAM("\n Picking part from belt");
                        gantry.goToPresetLocation(gantry.belta_);
                        ROS_INFO_STREAM("\nWaiting for beam to turn off");
                        do {
                           //
                        } while (comp.beam_detect[0] == true);
                        logicam12 = comp.getter_logicam_callback();
                        if (part_on_belt!=0)
                        {
                            ros::Duration(2).sleep();
                        }
                        ROS_INFO_STREAM("\nWaiting to be detected by camera..");
                        do
                        {
                            logicam12 = comp.getter_logicam_callback();
                        }while(logicam12[12][0].type.empty());
                        ROS_INFO_STREAM("\n Detected by camera. Trying to pick up!!");
                        ros::Duration(0.2).sleep();
                        part belt_part;
                        belt_part.pose = logicam12[12][0].pose;
                        belt_part.type = logicam12[12][0].type;
                        if (logicam12[12][0].type == "piston_rod_part_red" || logicam12[12][0].type == "piston_rod_part_green" || logicam12[12][0].type == "piston_rod_part_blue")
                        {
                            gantry.goToPresetLocation(gantry.beltb1_);
                            do
                                {
                                gantry.activateGripper("left_arm");
                                ROS_INFO_STREAM("\n Trying to pick up!!");
                            } while (!gantry.getGripperState("left_arm").attached);
                            ros::Duration(1).sleep();
                            gantry.goToPresetLocation(gantry.beltb1_);
                            gantry.goToPresetLocation(gantry.belta_);
                            gantry.goToPresetLocation(gantry.start_);
                        }
                        if (logicam12[12][0].type == "pulley_part_red" || logicam12[12][0].type == "pulley_part_green" || logicam12[12][0].type == "pulley_part_blue")
                        {
                            gantry.goToPresetLocation(gantry.beltb2_);
                            do
                            {
                                gantry.activateGripper("left_arm");
                                ROS_INFO_STREAM("\n Trying to pick up!!");
                            } while (!gantry.getGripperState("left_arm").attached);
                            ros::Duration(1).sleep();
                            gantry.goToPresetLocation(gantry.beltb2_);
                            gantry.goToPresetLocation(gantry.belta_);
                            gantry.goToPresetLocation(gantry.start_);
                        }
                        if (logicam12[12][0].type == "disk_part_red" || logicam12[12][0].type == "disk_part_green" || logicam12[12][0].type == "disk_part_blue")
                        {
                            gantry.goToPresetLocation(gantry.beltc2_);
                            do
                            {
                                gantry.activateGripper("left_arm");
                                ROS_INFO_STREAM("\n Trying to pick up!!");
                            } while (!gantry.getGripperState("left_arm").attached);
                            ros::Duration(1).sleep();
                            gantry.goToPresetLocation(gantry.beltc2_);
                            gantry.goToPresetLocation(gantry.belta_);
                            gantry.goToPresetLocation(gantry.start_);
                        }
                        if (logicam12[12][0].type == "gasket_part_red" || logicam12[12][0].type == "gasket_part_green" || logicam12[12][0].type == "gasket_part_blue")
                        {
                            gantry.goToPresetLocation(gantry.beltd2_);
                            do
                            {
                                gantry.activateGripper("left_arm");
                                ROS_INFO_STREAM("\n Trying to pick up!!");
                            } while (!gantry.getGripperState("left_arm").attached);
                            ros::Duration(1).sleep();
                            gantry.goToPresetLocation(gantry.beltd2_);
                            gantry.goToPresetLocation(gantry.belta_);
                            gantry.goToPresetLocation(gantry.start_);
                        }

                        auto i1 = belt_part_arr[part_on_belt][0];
                        auto j1 = belt_part_arr[part_on_belt][1];
                        auto k1 = belt_part_arr[part_on_belt][2];
                        auto target_pose = gantry.getTargetWorldPose(or_details[i1][j1][k1].pose, "agv1");

                        if (or_details[i1][j1][k1].agv_id == "any" && j1==0)
                            or_details[i1][j1][k1].agv_id = "agv1";
                        else if (or_details[i1][j1][k1].agv_id == "any" && j1!=0)
                            or_details[i1][j1][k1].agv_id = "agv2";
                        if (part_on_belt == 0)
                        {
                            gantry.goToPresetLocation(gantry.bin9_);
                            gantry.deactivateGripper("left_arm");
                        }
                        else
                        {
                            gantry.goToPresetLocation(gantry.bin14_);
                            gantry.deactivateGripper("left_arm");
                        }
                        logicam12 = comp.getter_logicam_callback();
                        logicam[2] = logicam12[2];
                        part_on_belt++;
                        ROS_INFO_STREAM("\nPart on belt value has been incremented!!!!!!");
                        gantry.goToPresetLocation(gantry.start1_);
                        order_flag[i1][j1][k1] = 1;
                    } while (part_on_belt < on_belt);
                }

                if (or_details[i][j][k].agv_id == "any" && j==0)
                    or_details[i][j][k].agv_id = "agv1";
                else if (or_details[i][j][k].agv_id == "any" && j!=0)
                    or_details[i][j][k].agv_id = "agv2";

                //loop for checking for picking other parts from their locations
                for (int x = 0; x < 17; x++)
                {
                    if (count == 1)
                    {
                        ROS_INFO_STREAM("Count is 1");
                        if (new_order)
                        {
                            j=0;
                            k=-1;
                        }
                        break;
                    }
                    for (int y = 0; y < 36; y++)
                    {
                        if (logicam[x][y].type == comp.received_orders_[i].shipments[j].products[k].type && logicam[x][y].Shifted == false) {
                            ROS_INFO_STREAM("\n\nPart being taken " << logicam[x][y].type);
                            ROS_INFO_STREAM("\n\nlogical camera: " << x);
                            gantry.goToPresetLocation(gantry.start_);

//                            ROS_INFO_STREAM("\n Test run of move to preset location function.");
                            location = logicam[x][y].frame;
                            auto location1 = logicam[x][y].frame;
                            ROS_INFO_STREAM("X POSITION " << logicam[x][y].pose.position.x);
                            ROS_INFO_STREAM("Y POSITION " << logicam[x][y].pose.position.y);
                            ROS_INFO_STREAM("Z POSITION " << logicam[x][y].pose.position.z);
                            ROS_INFO_STREAM("Location: " << location);
                            auto target_pose = gantry.getTargetWorldPose(or_details[i][j][k].pose, "agv1");
                            loc_x = logicam[x][y].pose.position.x;
                            loc_y = logicam[x][y].pose.position.y;
                            gantry.moveToPresetLocation(presetLocation, location, loc_x, loc_y, 1, logicam[x][y].type,comp.gap_nos, comp);
                            ROS_INFO_STREAM("update Location: " << location);
                            part my_part;
                            my_part.type = logicam[x][y].type;
                            my_part.pose = logicam[x][y].pose;
//                            if ((loc_x<3.9 && loc_x> 3.2) && (loc_y>-2.4 && loc_y<-1.85))           //BIN14 alone, some moveit problem..
//                                my_part.pose.position.z -= 0.06;

                            if ((loc_x<3.9 && loc_x> 3.2) && (loc_y>-2.4 && loc_y<-1.85))           //BIN14 alone, some moveit problem..
                            my_part.pose.position.z -= 0.06;
                            else if ((loc_x>4.1 && loc_x<4.8) && (loc_y>-2.4 && loc_y<-1.85))
                                my_part.pose.position.z -= 0.02;

                            ros::Duration(1).sleep();
                            gantry.pickPart(my_part);
                            ros::Duration(1).sleep();
                            gantry.moveToPresetLocation(presetLocation, location1, loc_x, loc_y, 2, logicam[x][y].type,comp.gap_nos, comp);
                            ROS_INFO_STREAM("GOING TO START JUST TO BE SAFE!!!!!!");
                            gantry.goToPresetLocation(gantry.start_);
                            ROS_INFO_STREAM("Approaching AGV's to place object!!!");
                            if (or_details[i][j][k].agv_id == "agv1") {
                                gantry.goToPresetLocation(gantry.agv1_);
                                ROS_INFO_STREAM("\n Waypoint AGV1 reached\n");
                                if (or_details[i][j][k].pose.orientation.x != 0) {
                                    ROS_INFO_STREAM("Part is to be flipped");
                                    gantry.goToPresetLocation(gantry.agv1c_);
                                    ROS_INFO_STREAM("\n Waypoint AGV1 reached\n");
                                    gantry.goToPresetLocation(gantry.agv1flipa_);
                                    gantry.activateGripper("right_arm");
                                    ros::Duration(0.2).sleep();
                                    gantry.deactivateGripper("left_arm");
                                    ROS_INFO_STREAM("Part flipped");
                                    or_details[i][j][k].pose.orientation.x = 0.0;
                                    or_details[i][j][k].pose.orientation.y = 0;
                                    or_details[i][j][k].pose.orientation.z = 0.0;
                                    or_details[i][j][k].pose.orientation.w = 1;
                                    gantry.goToPresetLocation(gantry.agv1flipb_);
                                    gantry.placePartRight(or_details[i][j][k], "agv1");
                                    ROS_INFO_STREAM("\n Object placed!!!!!!!!!!\n");
                                    gantry.goToPresetLocation(gantry.agv1_);
                                } else
                                    gantry.placePart(or_details[i][j][k], "agv1");
                                logicam[x][y].Shifted = true;
                            } else if (or_details[i][j][k].agv_id == "agv2") {
                                gantry.goToPresetLocation(gantry.agv2_);
                                ROS_INFO_STREAM("\n Waypoint AGV2 reached\n");
                                if (or_details[i][j][k].pose.orientation.x != 0) {
                                    ROS_INFO_STREAM("Part is to be flipped");
                                    gantry.goToPresetLocation(gantry.agv2a_);
                                    ROS_INFO_STREAM("\n Waypoint AGV2 reached\n");
                                    gantry.activateGripper("right_arm");
                                    ros::Duration(0.2).sleep();
                                    gantry.deactivateGripper("left_arm");
                                    ROS_INFO_STREAM("Part flipped");
                                    or_details[i][j][k].pose.orientation.x = 0.0;
                                    or_details[i][j][k].pose.orientation.y = 0;
                                    or_details[i][j][k].pose.orientation.z = 0.0;
                                    or_details[i][j][k].pose.orientation.w = 1;
                                    gantry.goToPresetLocation(gantry.agv2b_);
                                    gantry.placePartRight(or_details[i][j][k], "agv2");
                                    ROS_INFO_STREAM("\n Object placed!!!!!!!!!!\n");
                                    gantry.goToPresetLocation(gantry.agv2_);
                                } else
                                    gantry.placePart(or_details[i][j][k], "agv2");
                                logicam[x][y].Shifted = true;
                                target_pose = gantry.getTargetWorldPose(or_details[i][j][k].pose, "agv2");
                            }

                            //Checking to erase in next logicam matrix..
                            for (int y1=0; y1<36; y1++)
                            {
                                if (x==Max_number_of_cameras-1)
                                    break;
                                else if((abs(logicam[x][y].pose.position.x - logicam[x+1][y1].pose.position.x) < 0.01) && (abs(logicam[x][y].pose.position.y - logicam[x+1][y1].pose.position.y) < 0.01)) {
                                    ROS_INFO_STREAM("\n Common part for 2 cameras, marking as completed for next iteration!");
                                    logicam[x + 1][y1].Shifted = true;
                                }
                            }

                            logicam2 = comp.getter_logicam_callback();
                            ROS_INFO_STREAM("\n After placing.");
                            ROS_INFO_STREAM("\n order name: "<<comp.received_orders_[i].shipments[j].products[k].type);
                            ROS_INFO_STREAM("\n order details: "<<or_details[i][j][k].pose);
                            ROS_INFO_STREAM("\n Target pose: "<<target_pose);
                            auto cam = logicam2[10][on_table_1].pose;
                            if (or_details[i][j][k].agv_id=="agv1")
                            {
                                for (auto ill=0; ill<=on_table_1; ill++)
                                {
                                    if (logicam2[10][ill].type == comp.received_orders_[i].shipments[j].products[k].type && abs(logicam2[10][ill].pose.position.x-target_pose.position.x)<0.1 && abs(logicam2[10][ill].pose.position.y-target_pose.position.y)<0.1)
                                    {
                                        ROS_INFO_STREAM("\n Printing agv1 index value: "<<ill<<"\n Also product type = "<<logicam2[10][ill].type);
                                        index=ill;
                                        break;
                                    }
                                }
                                ROS_INFO_STREAM("\n AGV camera details: "<<logicam2[10][index].pose);
                                cam = logicam2[10][index].pose;
                                ros::Duration(1).sleep();
                                faulty_part = comp.quality_sensor_status1();
                            }
                            else if (or_details[i][j][k].agv_id=="agv2")
                            {
                                for (auto ill=0; ill<=on_table_2; ill++)
                                {
                                    if (logicam2[11][ill].type == comp.received_orders_[i].shipments[j].products[k].type && abs(logicam2[11][ill].pose.position.x-target_pose.position.x)<0.1 && abs(logicam2[11][ill].pose.position.y-target_pose.position.y)<0.1)
                                    {
                                        ROS_INFO_STREAM("\n Printing agv2 index value: "<<ill<<"\n Also product type = "<<logicam2[11][ill].type);
                                        index=ill;
                                        break;
                                    }
                                }
                                ROS_INFO_STREAM("\n AGV camera details: "<<logicam2[11][index].pose);
                                cam = logicam2[11][index].pose;
                                ros::Duration(1).sleep();
                                faulty_part = comp.quality_sensor_status();
                            }
                            ROS_INFO_STREAM("\n X offset: "<<abs(cam.position.x-target_pose.position.x));
                            ROS_INFO_STREAM("\n Y offset: "<<abs(cam.position.y-target_pose.position.y));

                            Model_adjust = abs(cam.position.z-target_pose.position.z);

// Faulty part check
                            if(faulty_part.faulty == true)
                            {
                                ROS_INFO_STREAM("Faulty Part detected!!!");
                                faulty_part.type = my_part.type;
                                ROS_INFO_STREAM("\n Trying to compute path for "<<faulty_part.type);
                                ROS_INFO_STREAM("\n Pose at faulty part "<<cam);
                                faulty_part.pose = cam;
                                faulty_part.pose.position.z -= Model_adjust;
                                ROS_INFO_STREAM("\n Pose for Faulty part "<<faulty_part.pose);
                                if (or_details[i][j][k].agv_id=="agv2")
                                {
                                    gantry.goToPresetLocation(gantry.agv2f_);
                                    auto agv_faulty = gantry.agv2_;
                                    if (faulty_part.pose.position.x > 0 && faulty_part.pose.position.y < -7.26)
                                    {
                                        ROS_INFO_STREAM("Faulty part at Left top of tray!!");
                                        agv_faulty = gantry.agv2flt_;
                                    }
                                    else if (faulty_part.pose.position.x > 0 && faulty_part.pose.position.y > -7.26)
                                    {
                                        ROS_INFO_STREAM("Faulty part at Left bottom of tray!!");
                                        agv_faulty = gantry.agv2flb_;
                                    }
                                    else if (faulty_part.pose.position.x < 0 && faulty_part.pose.position.y < -7.26)
                                    {
                                        ROS_INFO_STREAM("Faulty part at Right top of tray!!");
                                        agv_faulty = gantry.agv2frt_;
                                    }
                                    else if (faulty_part.pose.position.x < 0 && faulty_part.pose.position.y > -7.26)
                                    {
                                        ROS_INFO_STREAM("Faulty part at Right bottom of tray!!");
                                        agv_faulty = gantry.agv2frb_;
                                    }
                                    gantry.goToPresetLocation(agv_faulty);
                                    gantry.pickPart(faulty_part);
                                    gantry.goToPresetLocation(agv_faulty);
                                }
                                else if (or_details[i][j][k].agv_id=="agv1")
                                {
                                    gantry.goToPresetLocation(gantry.agv1f_);
                                    auto agv_faulty = gantry.agv1_;
                                    if (faulty_part.pose.position.x < 0 && faulty_part.pose.position.y > 7.12)
                                    {
                                        ROS_INFO_STREAM("Faulty part at Left top of tray!!");
                                        agv_faulty = gantry.agv1flt_;
                                    }
                                    else if (faulty_part.pose.position.x < 0 && faulty_part.pose.position.y < 7.12)
                                    {
                                        ROS_INFO_STREAM("Faulty part at Left bottom of tray!!");
                                        agv_faulty = gantry.agv1flb_;
                                    }
                                    else if (faulty_part.pose.position.x > 0 && faulty_part.pose.position.y > 7.12)
                                    {
                                        ROS_INFO_STREAM("Faulty part at Right top of tray!!");
                                        agv_faulty = gantry.agv1frt_;
                                    }
                                    else if (faulty_part.pose.position.x > 0 && faulty_part.pose.position.y < 7.12)
                                    {
                                        ROS_INFO_STREAM("Faulty part at Right bottom of tray!!");
                                        agv_faulty = gantry.agv1frb_;
                                    }
                                    gantry.goToPresetLocation(agv_faulty);
                                    gantry.pickPart(faulty_part);
                                    gantry.goToPresetLocation(agv_faulty);
                                }
                                gantry.goToPresetLocation(gantry.start_);
                                gantry.goToPresetLocation(gantry.agv_faulty);
                                gantry.deactivateGripper("left_arm");
                                continue;
                            }

//Faulty pose correction
                            else if (abs(cam.position.x-target_pose.position.x)>0.03 || abs(cam.position.y-target_pose.position.y)>0.03)
                            {
                                if (abs(cam.position.x-target_pose.position.x)>0.03)
                                    ROS_INFO_STREAM("\n X offset detected");
                                if (abs(cam.position.y-target_pose.position.y)>0.03)
                                    ROS_INFO_STREAM("\n Y offset detected");
                                ROS_INFO_STREAM("\n Faulty Pose detected for part "<<logicam[x][y].type);
                                faulty_pose.type = my_part.type;
                                ROS_INFO_STREAM("\n Trying to compute path for "<<faulty_pose.type);
                                ROS_INFO_STREAM("\n Faulty pose "<<cam);
                                faulty_pose.pose = cam;
                                faulty_pose.pose.position.z -= Model_adjust;
                                if (or_details[i][j][k].agv_id=="agv2")
                                {
                                    gantry.goToPresetLocation(gantry.agv2f_);
                                    ROS_INFO_STREAM("\n Reconfiguring for better pickup...");
                                    auto agv_faulty = gantry.agv2_;
                                    if (faulty_pose.pose.position.x > 0 && faulty_pose.pose.position.y < -7.26)
                                    {
                                        ROS_INFO_STREAM("Faulty pose at Left top of tray!!");
                                        agv_faulty = gantry.agv2flt_;
                                    }
                                    else if (faulty_pose.pose.position.x > 0 && faulty_pose.pose.position.y > -7.26)
                                    {
                                        ROS_INFO_STREAM("Faulty pose at Left bottom of tray!!");
                                        agv_faulty = gantry.agv2flb_;
                                    }
                                    else if (faulty_pose.pose.position.x < 0 && faulty_pose.pose.position.y < -7.26)
                                    {
                                        ROS_INFO_STREAM("Faulty pose at Right top of tray!!");
                                        agv_faulty = gantry.agv2frt_;
                                    }
                                    else if (faulty_pose.pose.position.x < 0 && faulty_pose.pose.position.y > -7.26)
                                    {
                                        ROS_INFO_STREAM("Faulty pose at Right bottom of tray!!");
                                        agv_faulty = gantry.agv2frb_;
                                    }
                                    gantry.goToPresetLocation(agv_faulty);
                                    gantry.pickPart(faulty_pose);
                                    ros::Duration(0.2).sleep();
                                    ROS_INFO_STREAM("\nPart Picked!");
                                    gantry.goToPresetLocation(gantry.agv2_);
                                    gantry.placePart(or_details[i][j][k], "agv2");
                                    ROS_INFO_STREAM("\n Placed!!!");
                                    on_table_2++;
                                }
                                else if (or_details[i][j][k].agv_id=="agv1")
                                {
                                    gantry.goToPresetLocation(gantry.agv1f_);
                                    ROS_INFO_STREAM("\n Reconfiguring for better pickup...");
                                    auto agv_faulty = gantry.agv1_;
                                    if (faulty_pose.pose.position.x < 0 && faulty_pose.pose.position.y > 7.12)
                                    {
                                        ROS_INFO_STREAM("Faulty pose at Left top of tray!!");
                                        agv_faulty = gantry.agv1flt_;
                                    }
                                    else if (faulty_pose.pose.position.x < 0 && faulty_pose.pose.position.y < 7.12)
                                    {
                                        ROS_INFO_STREAM("Faulty pose at Left bottom of tray!!");
                                        agv_faulty = gantry.agv1flb_;
                                    }
                                    else if (faulty_pose.pose.position.x > 0 && faulty_pose.pose.position.y > 7.12)
                                    {
                                        ROS_INFO_STREAM("Faulty pose at Right top of tray!!");
                                        agv_faulty = gantry.agv1frt_;
                                    }
                                    else if (faulty_pose.pose.position.x > 0 && faulty_pose.pose.position.y < 7.12)
                                    {
                                        ROS_INFO_STREAM("Faulty pose at Right bottom of tray!!");
                                        agv_faulty = gantry.agv1frb_;
                                    }
                                    gantry.goToPresetLocation(agv_faulty);
                                    gantry.pickPart(faulty_pose);
                                    ros::Duration(0.2).sleep();
                                    ROS_INFO_STREAM("\nPart Picked!");
                                    gantry.goToPresetLocation(gantry.agv1_);
                                    gantry.placePart(or_details[i][j][k], "agv1");
                                    ROS_INFO_STREAM("\n Placed!!!");
                                    on_table_1++;
                                }
                            }
                            else
                            {
                                if (or_details[i][j][k].agv_id=="agv2")
                                    on_table_2++;
                                else if (or_details[i][j][k].agv_id=="agv1")
                                    on_table_1++;
                                ROS_INFO_STREAM("Part has been placed without any problem, moving onto next product!");
                            }
                            auto state = gantry.getGripperState("left_arm");
                            if (state.attached)
                                gantry.goToPresetLocation(gantry.start_);
                            count++;
                            order_flag[i][j][k]=1;

                            //Checking if this is the last product of the shipment, and submitting score
                            if (k==comp.received_orders_[i].shipments[j].products.size()-1)
                            {
                                completed2[i]=1;
                                if (or_details[i][j][k].agv_id=="agv1")
                                {
                                    ROS_INFO_STREAM("\n Submitting Order: "<<or_details_new[i][j][k].shipment);
                                    submitOrder(1, or_details_new[i][j][k].shipment);
                                }
                                else if (or_details[i][j][k].agv_id=="agv2")
                                {
                                    ROS_INFO_STREAM("\n Submitting Order: "<<or_details_new[i][j][k].shipment);
                                    submitOrder(2, or_details_new[i][j][k].shipment);
                                }
                            }
                            or_details_new = comp.getter_part_callback();
                            ros::Duration(0.2).sleep();
                            ROS_INFO_STREAM("\n Checking for high priority order insertion.. absent? (1 is true) "<<or_details_new[i+1][j][k].shipment.empty());
                            if (!or_details_new[i+1][j][k].shipment.empty())
                            {
                                ROS_INFO_STREAM("\n Order NEW shipment name 1: "<<or_details_new[i+1][j][k].shipment);
                                or_details[i+1]=or_details_new[i+1];
                                ROS_INFO_STREAM("\n Copied info details "<<or_details[i+1][j][k].shipment);
                                i=i+1;
                                ROS_INFO_STREAM("\n Value of i: "<<i);
                                ROS_INFO_STREAM("\n New order size detected.. breaking.. ");
                                new_order++;
                                ROS_INFO_STREAM("\n Value of new: "<<new_order);
                            }
                            if (completed2[i]==1)
                                i=0;
                            break;
                        }
                    }
                }
            }
        }
        gantry.goToPresetLocation(gantry.start_);
        comp.endCompetition();
        spinner.stop();
        ros::shutdown();
        return 0;
    }
}