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

//void logicam_presets(std::string logical_camera_4, std::string logical_camera_5, GantryControl& gantry)
//{
//    if (logical_camera_4 == "gasket_part_green")
//    {
//        gantry.goToPresetLocation(gantry.lc5la_);
//        gantry.goToPresetLocation(gantry.lc5lb_);
//        gantry.goToPresetLocation(gantry.lc5lc_);
//        gantry.goToPresetLocation(gantry.lc5ld_);
//        gantry.goToPresetLocation(gantry.lc5le_);
//        gantry.goToPresetLocation(gantry.lc5lf_);
//    }
//
//    else if (logical_camera_5 == "pulley_part_blue")
//    {
//        gantry.goToPresetLocation(gantry.lc4ra_);
//        gantry.goToPresetLocation(gantry.lc4rb_);
//        gantry.goToPresetLocation(gantry.lc4rc_);
//        gantry.goToPresetLocation(gantry.lc4rd_);
//        gantry.goToPresetLocation(gantry.lc4re_);
////        gantry.goToPresetLocation(gantry.lc4rf_);
//    }
//}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "rwa5_node");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(8);
    spinner.start();

    Competition comp(node);
    comp.init();
    int Max_number_of_cameras = 17, Max_number_of_breakbeams = 21;
    std::ostringstream otopic;
    std::string topic;
    std::array<std::array<modelparam, 36>, 17> logicam, logicam2, logicam12;
    std::array<std::array<std::array<part, 10>, 5>, 5> or_details, or_details_new, order_call;
    std::array<std::array<std::array<int, 10>, 5>, 5> order_flag = {0};
    std::array<int, 5> completed = {0};
    part faulty_part, faulty_pose;
    bool break_beam;
    std::string c_state = comp.getCompetitionState();
    comp.getClock();

    ros::Subscriber logical_camera_subscriber_[Max_number_of_cameras];


    for (int x=0; x<Max_number_of_cameras; x++)
    {
        otopic.str("");
        otopic.clear();
        otopic << "/ariac/logical_camera_" << (x);
        topic = otopic.str();
        logical_camera_subscriber_[x] = node.subscribe<nist_gear::LogicalCameraImage>(topic, 10, boost::bind(&Competition::logical_camera_callback, &comp, _1, x));
    }

    ros::Subscriber breakbream_sensor_subscriber_[Max_number_of_breakbeams];
    for (int x=0; x<Max_number_of_breakbeams; x++)
    {
        otopic.str("");
        otopic.clear();
        otopic << "/ariac/breakbeam_" << (x);
        topic = otopic.str();
        breakbream_sensor_subscriber_[x] = node.subscribe<nist_gear::Proximity>(topic, 10, boost::bind(&Competition::breakbeam_sensor_callback, &comp, _1, x));
    }

    GantryControl gantry(node);
    gantry.init();
    gantry.goToPresetLocation(gantry.start_);

    logicam = comp.getter_logicam_callback();
    order_call = comp.getter_part_callback();
    or_details[0] = order_call[0];
    int on_table_1=0, on_table_2=0, new_order=0, index=0, part_on_belt=0;
    comp.check_gaps();
    int x_loop = 0;
    int on_belt = 0;
    std::array<std::array<int, 3>, 5> belt_part_arr;

    for (int i=comp.received_orders_.size()-1; i >=0 ; i--)
    {
        for (int j=0; j < comp.received_orders_[i].shipments.size(); j++)
        {
            for (int k=0; k < comp.received_orders_[i].shipments[j].products.size(); k++)
            {
                for (int x=0; x<17; x++)
                {
                    if(x_loop != 0)
                    {
                        x_loop = 0;
                        break;
                    }
                    for (int y=0; y<36; y++)
                    {
                        if(logicam[x][y].type == comp.received_orders_[i].shipments[j].products[k].type)
                        {
                            ROS_INFO_STREAM("\n"<<comp.received_orders_[i].shipments[j].products[k].type<<" under logical camera "<<x);
                            ROS_INFO_STREAM("\n Part seen "<<logicam[x][y].type);
                            ROS_INFO_STREAM("\n order = "<<i<<", shipment = "<<j<<", products = "<<k);
                            x_loop++;
                            break;
                        }
                        if((x == 16) && (y == 35))
                        {
                            belt_part_arr[on_belt][0] = i;
                            belt_part_arr[on_belt][1] = j;
                            belt_part_arr[on_belt][2] = k;
                            on_belt++;
                            ROS_INFO_STREAM("\n Order details of the red part - i = "<<i<<", j = "<<j<<", k = "<<k);
                            ROS_INFO_STREAM("\n Part is on the belt");
                            ROS_INFO_STREAM("\n Number of parts on the belt "<<on_belt);
//                            ROS_INFO_STREAM("\n Array containing the order details of part on belt \n "<<belt_part_arr[on_belt][0]<<belt_part_arr[on_belt][1]<<belt_part_arr[on_belt][2]);
                        }
                    }
                }
            }
        }
    }

    for (int i=comp.received_orders_.size()-1; i >=0 ; i--)
    {
        if (completed[i]==1)
            continue;
        for (int j=0; j < comp.received_orders_[i].shipments.size(); j++)
        {
            if (completed[i]==1)
                break;
            if (new_order!=0)
            {
                new_order = 0;
                i = i+2;
                break;
            }
            for (int k=0; k < comp.received_orders_[i].shipments[j].products.size(); k++)
            {
                int count=0;
                if (new_order)
                    break;
                if (order_flag[i][j][k]!=0)
                    continue;
                //ros::Duration(15).sleep();
                logicam12 = comp.getter_logicam_callback();
                ROS_INFO_STREAM("\n Print i="<<i<<", j="<<j<<", k="<<k);
                ROS_INFO_STREAM("\n Print comp.received_orders_.size()="<<comp.received_orders_.size());
                ROS_INFO_STREAM("\n Print comp.received_orders_[i].shipments.size()="<<comp.received_orders_[i].shipments.size());
                ROS_INFO_STREAM("\n Print comp.received_orders_[i].shipments[j].products.size()="<<comp.received_orders_[i].shipments[j].products.size());
                ROS_INFO_STREAM("\n AGV ID: "<<or_details[i][j][k].agv_id);
                ROS_INFO_STREAM("\n Order shipment name: "<<or_details[i][j][k].shipment);
                if (!logicam12[12][0].type.empty())
                {
                    do {
                        if (!logicam12[12][0].type.empty())
                            ROS_INFO_STREAM("\nThere is a part on the belt..");
                        ROS_INFO_STREAM("\n Picking part from belt");
                        gantry.goToPresetLocation(gantry.belta_);
                        do
                        {
                            ROS_INFO_STREAM("\nWaiting for beam to turn off");
                        }while(comp.beam_detect[0]==true);
                        ROS_INFO_STREAM("\n Beam off. Trying to pick up!!");
                        gantry.goToPresetLocation(gantry.beltb_);logicam12
                        part belt_part;
                        belt_part.pose=[12][0].pose;
                        belt_part.type=logicam12[12][0].type;
                        do{
                            gantry.activateGripper("left_arm");
                            ROS_INFO_STREAM("\n Trying to pick up!!");
                        }while(!gantry.getGripperState("left_arm").attached);
                        ros::Duration(1).sleep();
                        gantry.goToPresetLocation(gantry.beltb_);
                        gantry.goToPresetLocation(gantry.belta_);

                        auto i1 = belt_part_arr[part_on_belt][0];
                        auto j1 = belt_part_arr[part_on_belt][1];
                        auto k1 = belt_part_arr[part_on_belt][2];
                        auto target_pose = gantry.getTargetWorldPose(or_details[i1][j1][k1].pose, "agv1");
                        if (or_details[i1][j1][k1].agv_id=="agv1")
                        {
                            gantry.goToPresetLocation(gantry.agv1_);
                            ROS_INFO_STREAM("\n Waypoint AGV1 reached\n");
                            gantry.placePart(or_details[i1][j1][k1], "agv1");
                            ROS_INFO_STREAM("\n Object placed!!!!!!!!!!\n");
                        }
                        else if (or_details[i1][j1][k1].agv_id=="agv2")
                        {
                            gantry.goToPresetLocation(gantry.agv2_);
                            ROS_INFO_STREAM("\n Waypoint AGV2 reached\n");
                            gantry.placePart(or_details[i1][j1][k1], "agv2");
                            ROS_INFO_STREAM("\n Object placed!!!!!!!!!!\n");
                            target_pose = gantry.getTargetWorldPose(or_details[i1][j1][k1].pose, "agv2");
                        }
                        logicam2 = comp.getter_logicam_callback();
                        ROS_INFO_STREAM("\n After placing.");
                        ROS_INFO_STREAM("\n order name: "<<comp.received_orders_[i1].shipments[j1].products[k1].type);
                        ROS_INFO_STREAM("\n order details: "<<or_details[i1][j1][k1].pose);
                        ROS_INFO_STREAM("\n Target pose: "<<target_pose);
                        auto cam = logicam2[10][on_table_1].pose;
                        ros::Duration(1).sleep();
                        if (or_details[i1][j1][k1].agv_id=="agv1")
                        {
                            for (auto ill=0; ill<=on_table_1; ill++)
                            {
                                if (logicam2[10][ill].type == comp.received_orders_[i1].shipments[j1].products[k1].type)
                                {
                                    ROS_INFO_STREAM("\n Printing ILL value: "<<ill<<"\n Also printing type = "<<logicam2[10][ill].type<<"\nAlso, shipment type"<<comp.received_orders_[i].shipments[j].products[k].type);
                                    index=ill;
                                    break;
                                }
                            }
                            ROS_INFO_STREAM("\n AGV camera details: "<<logicam2[10][index].pose);
                            cam = logicam2[10][index].pose;
                            faulty_part = comp.quality_sensor_status1();
                            if (faulty_part.faulty==true)
                            {
                                ROS_INFO_STREAM("\npart is faulty!!!!!!!");
                            }
                        }
                        else if (or_details[i1][j1][k1].agv_id=="agv2")
                        {
                            for (auto ill=0; ill<=on_table_2; ill++)
                            {
                                if (logicam2[11][ill].type == comp.received_orders_[i1].shipments[j1].products[k1].type)
                                {
                                    ROS_INFO_STREAM("\n Printing ILL value: "<<ill<<"\n Also printing type = "<<logicam2[11][ill].type<<"\nAlso, shipment type"<<comp.received_orders_[i].shipments[j].products[k].type);
                                    index=ill;
                                    break;
                                }
                            }
                            ROS_INFO_STREAM("\n AGV camera details: "<<logicam2[11][on_table_2].pose);
                            cam = logicam2[11][index].pose;
                            faulty_part = comp.quality_sensor_status();
                        }
                        ros::Duration(0.2).sleep();
                        ROS_INFO_STREAM("\n X offset: "<<abs(cam.position.x-target_pose.position.x));
                        ROS_INFO_STREAM("\n Y offset: "<<abs(cam.position.y-target_pose.position.y));
                        if(faulty_part.faulty == true)
                        {
                            ROS_INFO_STREAM("Faulty Part detected!!!");
                            faulty_part.type = belt_part.type;
                            ROS_INFO_STREAM("\n Trying to compute path for "<<faulty_part.type);
                            ROS_INFO_STREAM("\n Pose at faulty part "<<cam);
                            faulty_part.pose = cam;
                            faulty_part.pose.position.z -= 0.19;
                            ROS_INFO_STREAM("\n Pose for Faulty part "<<faulty_part.pose);
                            if (or_details[i1][j1][k1].agv_id=="agv2")
                            {
                                ROS_INFO_STREAM("\n AGV2 Loop");
                                gantry.goToPresetLocation(gantry.agv2_);
                                gantry.pickPart(faulty_part);
                                gantry.goToPresetLocation(gantry.agv2_);

                            }
                            else if (or_details[i1][j1][k1].agv_id=="agv1")
                            {
                                ROS_INFO_STREAM("\n AGV1 Loop");
                                gantry.goToPresetLocation(gantry.agv1_);
                                gantry.pickPart(faulty_part);
                                gantry.goToPresetLocation(gantry.agv1_);
                            }
                            gantry.goToPresetLocation(gantry.agv_faulty);
                            gantry.deactivateGripper("left_arm");
                            continue;
                        }
                        part_on_belt++;
                        gantry.goToPresetLocation(gantry.start_);
                    }while(part_on_belt<on_belt);

//                    faulty_part = comp.quality_sensor_status();
//                    if(faulty_part.faulty == true)
//                    {
//                        ROS_INFO_STREAM("Faulty Part detected!!!");
//                        faulty_part.type = my_part.type;
//                        ROS_INFO_STREAM("\n Trying to compute path for "<<faulty_part.type);
//                        ROS_INFO_STREAM("\n Pose at faulty part "<<cam);
//                        faulty_part.pose = cam;
//                        faulty_part.pose.position.z -= 0.19;
//                        ROS_INFO_STREAM("\n Pose for Faulty part "<<faulty_part.pose);
//                        if (or_details[i][j][k].agv_id=="agv2")
//                        {
//                            ROS_INFO_STREAM("\n AGV2 Loop");
//                            gantry.goToPresetLocation(gantry.agv2_);
//                            gantry.pickPart(faulty_part);
//                            gantry.goToPresetLocation(gantry.agv2_);
//
//                        }
//                        else if (or_details[i][j][k].agv_id=="agv1")
//                        {
//                            ROS_INFO_STREAM("\n AGV1 Loop");
//                            gantry.goToPresetLocation(gantry.agv1_);
//                            gantry.pickPart(faulty_part);
//                            gantry.goToPresetLocation(gantry.agv1_);
//                        }
//                        gantry.goToPresetLocation(gantry.agv_faulty);
//                        gantry.deactivateGripper("left_arm");
//                        continue;
//                    }
                }
                for(int x = 0; x < 17; x++)
                {
                    if (count==1)
                        break;
                    for(int y = 0; y < 36; y++)
                    {
                        if(logicam[x][y].type == comp.received_orders_[i].shipments[j].products[k].type &&  logicam[x][y].Shifted==false)
                        {
                            ROS_INFO_STREAM("\n\nPart being taken "<< logicam[x][y].type);
                            ROS_INFO_STREAM("\n\nlogical camera: "<<x);
                            gantry.goToPresetLocation(gantry.start_);

                            if (comp.received_orders_[i].shipments[j].products[k].type == "gasket_part_green")
                            {
                                gantry.goToPresetLocation(gantry.lc5la_);
                                gantry.goToPresetLocation(gantry.lc5lb_);
                                gantry.goToPresetLocation(gantry.lc5lc_);
                                gantry.goToPresetLocation(gantry.lc5ld_);
                                gantry.goToPresetLocation(gantry.lc5le_);
                                gantry.goToPresetLocation(gantry.lc5lf_);
                                part my_part;
                                my_part.type = logicam[x][y+1].type;
                                my_part.pose =logicam[x][y+1].pose;
                                auto target_pose = gantry.getTargetWorldPose(or_details[i][j][k].pose, "agv1");
                                gantry.pickPart(my_part);
                                ros::Duration(0.2).sleep();
                                gantry.goToPresetLocation(gantry.lc5lf_);
                                gantry.goToPresetLocation(gantry.lc5le_);
                                gantry.goToPresetLocation(gantry.lc5ld_);
                                gantry.goToPresetLocation(gantry.lc5lc_);
                                gantry.goToPresetLocation(gantry.lc5lb_);
                                gantry.goToPresetLocation(gantry.lc5la_);
                                if (or_details[i][j][k].agv_id=="agv1")
                                {
                                    gantry.goToPresetLocation(gantry.agv1_);
                                    ROS_INFO_STREAM("\n Waypoint AGV1 reached\n");
                                    gantry.placePart(or_details[i][j][k], "agv1");
                                    ROS_INFO_STREAM("\n Object placed!!!!!!!!!!\n");
                                    logicam[x][y+1].Shifted=true;
                                }
                                else if (or_details[i][j][k].agv_id=="agv2")
                                {
                                    gantry.goToPresetLocation(gantry.agv2_);
                                    ROS_INFO_STREAM("\n Waypoint AGV2 reached\n");
                                    gantry.placePart(or_details[i][j][k], "agv2");
                                    ROS_INFO_STREAM("\n Object placed!!!!!!!!!!\n");
                                    logicam[x][y+1].Shifted=true;
                                    target_pose = gantry.getTargetWorldPose(or_details[i][j][k].pose, "agv2");
                                }
                                logicam2 = comp.getter_logicam_callback();
                                ROS_INFO_STREAM("\n After placing.");
                                ROS_INFO_STREAM("\n order name: "<<comp.received_orders_[i].shipments[j].products[k].type);
                                ROS_INFO_STREAM("\n order details: "<<or_details[i][j][k].pose);
                                ROS_INFO_STREAM("\n Target pose: "<<target_pose);
                                auto cam = logicam2[10][on_table_1].pose;
                                ros::Duration(1).sleep();
                                if (or_details[i][j][k].agv_id=="agv1")
                                {
                                    for (auto ill=0; ill<=on_table_1; ill++)
                                    {
                                        if (logicam2[10][ill].type == comp.received_orders_[i].shipments[j].products[k].type)
                                        {
                                            ROS_INFO_STREAM("\n Printing ILL value: "<<ill<<"\n Also printing type = "<<logicam2[10][ill].type<<"\nAlso, shipment type"<<comp.received_orders_[i].shipments[j].products[k].type);
                                            index=ill;
                                            break;
                                        }
                                    }
                                    ROS_INFO_STREAM("\n AGV camera details: "<<logicam2[10][index].pose);
                                    cam = logicam2[10][index].pose;
                                    faulty_part = comp.quality_sensor_status1();
                                    if (faulty_part.faulty==true)
                                    {
                                        ROS_INFO_STREAM("\npart is faulty!!!!!!!");
                                    }
                                }
                                else if (or_details[i][j][k].agv_id=="agv2")
                                {
                                    for (auto ill=0; ill<=on_table_2; ill++)
                                    {
                                        if (logicam2[11][ill].type == comp.received_orders_[i].shipments[j].products[k].type)
                                        {
                                            ROS_INFO_STREAM("\n Printing ILL value: "<<ill<<"\n Also printing type = "<<logicam2[11][ill].type<<"\nAlso, shipment type"<<comp.received_orders_[i].shipments[j].products[k].type);
                                            index=ill;
                                            break;
                                        }
                                    }
                                    ROS_INFO_STREAM("\n AGV camera details: "<<logicam2[11][on_table_2].pose);
                                    cam = logicam2[11][index].pose;
                                    faulty_part = comp.quality_sensor_status();
                                }
                                ros::Duration(0.2).sleep();
                                ROS_INFO_STREAM("\n X offset: "<<abs(cam.position.x-target_pose.position.x));
                                ROS_INFO_STREAM("\n Y offset: "<<abs(cam.position.y-target_pose.position.y));
                                if(faulty_part.faulty == true)
                                {
                                    ROS_INFO_STREAM("Faulty Part detected!!!");
                                    faulty_part.type = my_part.type;
                                    ROS_INFO_STREAM("\n Trying to compute path for "<<faulty_part.type);
                                    ROS_INFO_STREAM("\n Pose at faulty part "<<cam);
                                    faulty_part.pose = cam;
                                    faulty_part.pose.position.z -= 0.19;
                                    ROS_INFO_STREAM("\n Pose for Faulty part "<<faulty_part.pose);
                                    if (or_details[i][j][k].agv_id=="agv2")
                                    {
                                        ROS_INFO_STREAM("\n AGV2 Loop");
                                        gantry.goToPresetLocation(gantry.agv2_);
                                        gantry.pickPart(faulty_part);
                                        gantry.goToPresetLocation(gantry.agv2_);

                                    }
                                    else if (or_details[i][j][k].agv_id=="agv1")
                                    {
                                        ROS_INFO_STREAM("\n AGV1 Loop");
                                        gantry.goToPresetLocation(gantry.agv1_);
                                        gantry.pickPart(faulty_part);
                                        gantry.goToPresetLocation(gantry.agv1_);
                                    }
                                    gantry.goToPresetLocation(gantry.agv_faulty);
                                    gantry.deactivateGripper("left_arm");
                                    continue;
                                }
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
                                    if (or_details[i][j][k].agv_id=="agv2")
                                    {
                                        gantry.goToPresetLocation(gantry.agv2c_);
                                        ROS_INFO_STREAM("\n Trying to pick up...");
                                        gantry.pickPart(faulty_pose);
                                        ROS_INFO_STREAM("\nPart Picked!");
                                        gantry.goToPresetLocation(gantry.agv2_);
                                        gantry.placePart(or_details[i][j][k], "agv2");
                                        ROS_INFO_STREAM("\n Placed!!!");
                                        on_table_2++;
                                    }
                                    else if (or_details[i][j][k].agv_id=="agv1")
                                    {
                                        gantry.goToPresetLocation(gantry.agv1a_);
                                        ROS_INFO_STREAM("\n Trying to pick up...");
                                        gantry.pickPart(faulty_pose);
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
                                }

                                auto state = gantry.getGripperState("left_arm");
                                if (state.attached)
                                    gantry.goToPresetLocation(gantry.start_);
                                ROS_INFO_STREAM("\n\nDISK part Green placed: "<<x);
                                or_details_new = comp.getter_part_callback();
                                ros::Duration(0.2).sleep();
                                ROS_INFO_STREAM("\n Order NEW shipment name 1: "<<or_details_new[i+1][j][k].shipment);
                                if (!or_details_new[i+1][j][k].shipment.empty())
                                {
                                    or_details[i+1]=or_details_new[i+1];
                                }
                                count++;
                                order_flag[i][j][k]=1;
                                if (k==comp.received_orders_[i].shipments[j].products.size()-1)
                                {
                                    completed[i]=1;
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
                                break;
                            }
//
                            else if (comp.received_orders_[i].shipments[j].products[k].type == "pulley_part_blue")
                            {
                                gantry.goToPresetLocation(gantry.lc4ra_);
                                gantry.goToPresetLocation(gantry.lc4rb_);
                                gantry.goToPresetLocation(gantry.lc4rc_);
                                gantry.goToPresetLocation(gantry.lc4rd_);
                                gantry.goToPresetLocation(gantry.lc4re_);
//                                gantry.goToPresetLocation(gantry.lc4rf_);
                                part my_part;
                                my_part.type = logicam[x][y].type;
                                my_part.pose =logicam[x][y].pose;
                                auto target_pose = gantry.getTargetWorldPose(or_details[i][j][k].pose, "agv1");
                                ROS_INFO_STREAM("my_part.type is" << my_part.type);
                                gantry.pickPart(my_part);
                                ros::Duration(0.2).sleep();
//                                gantry.goToPresetLocation(gantry.lc4rf_);
                                gantry.goToPresetLocation(gantry.lc4re_);
                                gantry.goToPresetLocation(gantry.lc4rd_);
                                gantry.goToPresetLocation(gantry.lc4rc_);
                                gantry.goToPresetLocation(gantry.lc4rb_);
                                gantry.goToPresetLocation(gantry.lc4ra_);
                                if (or_details[i][j][k].agv_id == "agv1")
                                {
                                    gantry.goToPresetLocation(gantry.agv1_);
                                    ROS_INFO_STREAM("\n Waypoint AGV1 reached\n");
                                    gantry.placePart(or_details[i][j][k], "agv1");
                                    ROS_INFO_STREAM("\n Object placed!!!!!!!!!!\n");
                                    logicam[x][y].Shifted=true;
                                }
                                else if (or_details[i][j][k].agv_id == "agv2")
                                {
                                    gantry.goToPresetLocation(gantry.agv2_);
                                    ROS_INFO_STREAM("\n Waypoint AGV2 reached\n");
                                    gantry.placePart(or_details[i][j][k], "agv2");
                                    ROS_INFO_STREAM("\n Object placed!!!!!!!!!!\n");
                                    logicam[x][y].Shifted=true;
                                    target_pose = gantry.getTargetWorldPose(or_details[i][j][k].pose, "agv2");
                                }
                                if (or_details[i][j][k].agv_id=="agv1")
                                {
                                    if(or_details[i][j][k].pose.orientation.x != 0)
                                    {
                                        ROS_INFO_STREAM("Part is to be flipped");
                                        gantry.goToPresetLocation(gantry.agv1_);
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
                                    }
                                    else
                                        gantry.placePart(or_details[i][j][k], "agv1");
                                    logicam[x][y].Shifted=true;
                                }
                                else if (or_details[i][j][k].agv_id=="agv2") {
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
                                    } else
                                        gantry.placePart(or_details[i][j][k], "agv2");
                                    logicam[x][y].Shifted = true;
                                    target_pose = gantry.getTargetWorldPose(or_details[i][j][k].pose, "agv2");
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
                                        if (logicam2[10][ill].type == comp.received_orders_[i].shipments[j].products[k].type)
                                        {
                                            ROS_INFO_STREAM("\n Printing ILL value: "<<ill<<"\n Also printing type = "<<logicam2[10][ill].type<<"\nAlso, shipment type"<<comp.received_orders_[i].shipments[j].products[k].type);
                                            index=ill;
                                            break;
                                        }
                                    }
                                    ROS_INFO_STREAM("\n AGV camera details: "<<logicam2[10][index].pose);
                                    cam = logicam2[10][index].pose;
                                    faulty_part = comp.quality_sensor_status1();
                                }
                                else if (or_details[i][j][k].agv_id=="agv2")
                                {
                                    for (auto ill=0; ill<=on_table_2; ill++)
                                    {
                                        if (logicam2[11][ill].type == comp.received_orders_[i].shipments[j].products[k].type)
                                        {
                                            ROS_INFO_STREAM("\n Printing ILL value: "<<ill<<"\n Also printing type = "<<logicam2[11][ill].type<<"\nAlso, shipment type"<<comp.received_orders_[i].shipments[j].products[k].type);
                                            index=ill;
                                            break;
                                        }
                                    }
                                    ROS_INFO_STREAM("\n AGV camera details: "<<logicam2[11][on_table_2].pose);
                                    cam = logicam2[11][index].pose;
                                    faulty_part = comp.quality_sensor_status();
                                }
                                ros::Duration(0.2).sleep();
                                ROS_INFO_STREAM("\n X offset: "<<abs(cam.position.x-target_pose.position.x));
                                ROS_INFO_STREAM("\n Y offset: "<<abs(cam.position.y-target_pose.position.y));
                                if(faulty_part.faulty == true)
                                {
                                    ROS_INFO_STREAM("Faulty Part detected!!!");
                                    faulty_part.type = my_part.type;
                                    ROS_INFO_STREAM("\n Trying to compute path for "<<faulty_part.type);
                                    ROS_INFO_STREAM("\n Pose at faulty part "<<cam);
                                    faulty_part.pose = cam;
                                    faulty_part.pose.position.z -= 0.19;
                                    ROS_INFO_STREAM("\n Pose for Faulty part "<<faulty_part.pose);
                                    if (or_details[i][j][k].agv_id=="agv2")
                                    {
                                        gantry.goToPresetLocation(gantry.agv2_);
                                        gantry.pickPart(faulty_part);
                                        gantry.goToPresetLocation(gantry.agv2_);

                                    }
                                    else if (or_details[i][j][k].agv_id=="agv1")
                                    {
                                        gantry.goToPresetLocation(gantry.agv1_);
                                        gantry.pickPart(faulty_part);
                                        gantry.goToPresetLocation(gantry.agv1_);
                                    }
                                    gantry.goToPresetLocation(gantry.agv_faulty);
                                    gantry.deactivateGripper("left_arm");
                                    continue;
                                }
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
                                    if (or_details[i][j][k].agv_id=="agv2")
                                    {
                                        gantry.goToPresetLocation(gantry.agv2c_);
                                        ROS_INFO_STREAM("\n Trying to pick up...");
                                        gantry.pickPart(faulty_pose);
                                        ROS_INFO_STREAM("\nPart Picked!");
                                        gantry.goToPresetLocation(gantry.agv2_);
                                        gantry.placePart(or_details[i][j][k], "agv2");
                                        ROS_INFO_STREAM("\n Placed!!!");
                                        on_table_2++;
                                    }
                                    else if (or_details[i][j][k].agv_id=="agv1")
                                    {
                                        gantry.goToPresetLocation(gantry.agv1a_);
                                        ROS_INFO_STREAM("\n Trying to pick up...");
                                        gantry.pickPart(faulty_pose);
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
                                }
                                auto state = gantry.getGripperState("left_arm");
                                if (state.attached)
                                    gantry.goToPresetLocation(gantry.start_);
                                count++;
                                order_flag[i][j][k]=1;
                                if (k==comp.received_orders_[i].shipments[j].products.size()-1)
                                {
                                    completed[i]=1;
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
                                ROS_INFO_STREAM("\n Order NEW shipment name 1: "<<or_details_new[i+1][j][k].shipment);
                                ROS_INFO_STREAM("\n is it empty??? "<<or_details_new[i+1][j][k].shipment.empty());
                                if (!or_details_new[i+1][j][k].shipment.empty())
                                {
                                    or_details[i+1]=or_details_new[i+1];
                                    ROS_INFO_STREAM("\n Copied info details "<<or_details[i+1][j][k].shipment);
                                    i = i+1;
                                    ROS_INFO_STREAM("\n Value of i: "<<i);
                                    ROS_INFO_STREAM("\n New order size detected.. breaking.. ");
                                    new_order++;
                                    ROS_INFO_STREAM("\n Value of new: "<<new_order);
                                }
                                break;
                            }
                        }
                        else if (part_on_belt==1)
                        {
                            if (comp.received_orders_[i].shipments[j].products[k].type == "pulley_part_red")
                            {
                                part_on_belt++;
                                gantry.goToPresetLocation(gantry.start_);
                                gantry.goToPresetLocation(gantry.bin3_);
                                part my_part;
                                my_part.type = logicam12[0][y].type;
                                my_part.pose =logicam12[0][y].pose;
                                auto target_pose = gantry.getTargetWorldPose(or_details[i][j][k].pose, "agv1");
                                gantry.pickPart(my_part);
                                ros::Duration(0.2).sleep();
                                gantry.goToPresetLocation(gantry.bin3_);
                                ros::Duration(0.2).sleep();
                                gantry.goToPresetLocation(gantry.start_);
                                if (or_details[i][j][k].agv_id=="agv1")
                                {
                                    if(or_details[i][j][k].pose.orientation.x != 0)
                                    {
                                        ROS_INFO_STREAM("Part is to be flipped");
                                        gantry.goToPresetLocation(gantry.agv1_);
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
                                    }
                                    else
                                        gantry.placePart(or_details[i][j][k], "agv1");
                                    logicam[x][y].Shifted=true;
                                }
                                else if (or_details[i][j][k].agv_id=="agv2")
                                {
                                    if(or_details[i][j][k].pose.orientation.x != 0)
                                    {
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
                                    }
                                    else
                                        gantry.placePart(or_details[i][j][k], "agv2");
                                    logicam[x][y].Shifted=true;
                                    target_pose = gantry.getTargetWorldPose(or_details[i][j][k].pose, "agv2");
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
                                        if (logicam2[10][ill].type == comp.received_orders_[i].shipments[j].products[k].type)
                                        {
                                            ROS_INFO_STREAM("\n Printing ILL value: "<<ill<<"\n Also printing type = "<<logicam2[10][ill].type<<"\nAlso, shipment type"<<comp.received_orders_[i].shipments[j].products[k].type);
                                            index=ill;
                                            break;
                                        }
                                    }
                                    ROS_INFO_STREAM("\n AGV camera details: "<<logicam2[10][index].pose);
                                    cam = logicam2[10][index].pose;
                                    faulty_part = comp.quality_sensor_status1();
                                }
                                else if (or_details[i][j][k].agv_id=="agv2")
                                {
                                    for (auto ill=0; ill<=on_table_2; ill++)
                                    {
                                        if (logicam2[11][ill].type == comp.received_orders_[i].shipments[j].products[k].type)
                                        {
                                            ROS_INFO_STREAM("\n Printing ILL value: "<<ill<<"\n Also printing type = "<<logicam2[11][ill].type<<"\nAlso, shipment type"<<comp.received_orders_[i].shipments[j].products[k].type);
                                            index=ill;
                                            break;
                                        }
                                    }
                                    ROS_INFO_STREAM("\n AGV camera details: "<<logicam2[11][on_table_2].pose);
                                    cam = logicam2[11][index].pose;
                                    faulty_part = comp.quality_sensor_status();
                                }
                                ros::Duration(0.2).sleep();
                                ROS_INFO_STREAM("\n X offset: "<<abs(cam.position.x-target_pose.position.x));
                                ROS_INFO_STREAM("\n Y offset: "<<abs(cam.position.y-target_pose.position.y));
                                if(faulty_part.faulty == true)
                                {
                                    ROS_INFO_STREAM("Faulty Part detected!!!");
                                    faulty_part.type = my_part.type;
                                    ROS_INFO_STREAM("\n Trying to compute path for "<<faulty_part.type);
                                    ROS_INFO_STREAM("\n Pose at faulty part "<<cam);
                                    faulty_part.pose = cam;
                                    faulty_part.pose.position.z -= 0.19;
                                    ROS_INFO_STREAM("\n Pose for Faulty part "<<faulty_part.pose);
                                    if (or_details[i][j][k].agv_id=="agv2")
                                    {
                                        gantry.goToPresetLocation(gantry.agv2_);
                                        gantry.pickPart(faulty_part);
                                        gantry.goToPresetLocation(gantry.agv2_);
                                    }
                                    else if (or_details[i][j][k].agv_id=="agv1")
                                    {
                                        gantry.goToPresetLocation(gantry.agv1_);
                                        gantry.pickPart(faulty_part);
                                        gantry.goToPresetLocation(gantry.agv1_);
                                    }
                                    gantry.goToPresetLocation(gantry.agv_faulty);
                                    gantry.deactivateGripper("left_arm");
                                    continue;
                                }
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
                                    if (or_details[i][j][k].agv_id=="agv2")
                                    {
                                        gantry.goToPresetLocation(gantry.agv2c_);
                                        ROS_INFO_STREAM("\n Trying to pick up...");
                                        gantry.pickPart(faulty_pose);
                                        ROS_INFO_STREAM("\nPart Picked!");
                                        gantry.goToPresetLocation(gantry.agv2_);
                                        gantry.placePart(or_details[i][j][k], "agv2");
                                        ROS_INFO_STREAM("\n Placed!!!");
                                        on_table_2++;
                                    }
                                    else if (or_details[i][j][k].agv_id=="agv1")
                                    {
                                        gantry.goToPresetLocation(gantry.agv1a_);
                                        ROS_INFO_STREAM("\n Trying to pick up...");
                                        gantry.pickPart(faulty_pose);
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
                                }

                                auto state = gantry.getGripperState("left_arm");
                                if (state.attached)
                                    gantry.goToPresetLocation(gantry.start_);
                                ROS_INFO_STREAM("\n\nDISK part Green placed: "<<x);
                                or_details_new = comp.getter_part_callback();
                                ros::Duration(0.2).sleep();
                                ROS_INFO_STREAM("\n Order NEW shipment name 1: "<<or_details_new[i+1][j][k].shipment);
                                if (!or_details_new[i+1][j][k].shipment.empty())
                                {
                                    or_details[i+1]=or_details_new[i+1];
                                }
                                count++;
                                order_flag[i][j][k]=1;
                                if (k==comp.received_orders_[i].shipments[j].products.size()-1)
                                {
                                    completed[i]=1;
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
                                break;
                            }
                        }
                    }
                }
                comp.check_gaps();
            }
        }
    }
    gantry.goToPresetLocation(gantry.start_);

    comp.endCompetition();
    spinner.stop();
    ros::shutdown();
    return 0;
}
