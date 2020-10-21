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

#include "competition.h"
#include "utils.h"
#include "gantry_control.h"

#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char ** argv) {
    ros::init(argc, argv, "rwa3_node");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(8);
    spinner.start();

    Competition comp(node);
    comp.init();
    int Max_number_of_cameras = 17;
    std::ostringstream otopic;
    std::string topic;
    std::array<std::array<modelparam, 36>, 17> logicam, logicam2;
    std::array<std::array<std::array<part, 10>, 5>, 5> or_details;
    part faulty_part;
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

    GantryControl gantry(node);
    gantry.init();
    gantry.goToPresetLocation(gantry.start_);

    //--1-Read order
    //--2-Look for parts in this order
    //--We go to this bin because a camera above
    //--this bin found one of the parts in the order

    logicam = comp.getter_logicam_callback();
    or_details = comp.getter_part_callback();
    int on_table=0;

    for (int i=0; i < comp.received_orders_.size(); i++)
    {
        for (int j=0; j < comp.received_orders_[i].shipments.size(); j++)
        {
            for (int k=0; k < comp.received_orders_[i].shipments[j].products.size(); k++)
            {

                int count=0;

                for(int x = 0; x < 17; x++)
                {
                    if (count==1)
                        break;
                    for(int y = 0; y < 36; y++)
                    {
                        if(logicam[x][y].type == comp.received_orders_[i].shipments[j].products[k].type &&  logicam[x][y].Shifted==false)
                        {
                            if (comp.received_orders_[i].shipments[j].products[k].type == "pulley_part_red")
                            {
                                gantry.goToPresetLocation(gantry.shelf5a_);
                                gantry.goToPresetLocation(gantry.shelf5b_);
                                gantry.goToPresetLocation(gantry.shelf5d_);
                                ROS_INFO_STREAM("\n Before picking.");
                                ROS_INFO_STREAM("\n order name: "<<comp.received_orders_[i].shipments[j].products[k].type);
                                ROS_INFO_STREAM("\n order details: "<<or_details[i][j][k].pose);
                                part my_part;
                                my_part.type = logicam[x][y].type;
                                my_part.pose = logicam[x][y].pose;
                                gantry.pickPart(my_part);
                                gantry.goToPresetLocation(gantry.shelf5e_);
                                gantry.goToPresetLocation(gantry.shelf5b_);
                                gantry.goToPresetLocation(gantry.shelf5a_);
                                gantry.goToPresetLocation(gantry.agv2_);
                                gantry.placePart(or_details[i][j][k], "agv2");
                                ROS_INFO_STREAM("\n After placing.");
                                ROS_INFO_STREAM("\n order name: "<<comp.received_orders_[i].shipments[j].products[k].type);
                                ROS_INFO_STREAM("\n order details: "<<or_details[i][j][k].pose);
                                ROS_INFO_STREAM("\n AGV camera details: "<<logicam2[11][on_table].pose);
                                logicam[x][y].Shifted=true;
                                logicam2 = comp.getter_logicam_callback();
                                ros::Duration(1.0).sleep();
                                faulty_part = comp.quality_sensor_status();
                                if(faulty_part.faulty == true)
                                {
                                    ROS_INFO_STREAM("Faulty Part");
                                    faulty_part.type = my_part.type;
                                    ROS_INFO_STREAM("\n Trying to compute path for "<<faulty_part.type);
                                    ROS_INFO_STREAM("\n Pose at faulty part "<<logicam2[11][on_table].pose);
                                    ROS_INFO_STREAM("\n logicam0 "<<logicam2[11][0].pose);
                                    ROS_INFO_STREAM("\n logicam1 "<<logicam2[11][1].pose);
                                    faulty_part.pose = logicam2[11][0].pose;
                                    faulty_part.pose.position.z -= 0.19;
                                    ROS_INFO_STREAM("\n Pose for Faulty path "<<faulty_part.pose);
                                    gantry.goToPresetLocation(gantry.agv2_);
                                    gantry.pickPart(faulty_part);
                                    gantry.goToPresetLocation(gantry.agv2_);
                                    gantry.goToPresetLocation(gantry.agv2_faulty);
                                    gantry.deactivateGripper("left_arm");
                                }
                                else
                                    on_table++;
                                auto state = gantry.getGripperState("left_arm");
                                if (state.attached)
                                    gantry.goToPresetLocation(gantry.start_);
                                count++;
                                break;
                            }
                            else if (comp.received_orders_[i].shipments[j].products[k].type == "disk_part_green" &&  logicam[x][y].Shifted==false)
                            {
                                gantry.goToPresetLocation(gantry.start_);
                                gantry.goToPresetLocation(gantry.bin16_);

                                ROS_INFO_STREAM("\n\ncamera details: "<< logicam[x][y].type);
                                part my_part;
                                my_part.type = logicam[x][y].type;
                                my_part.pose =logicam[x][y].pose;

                                gantry.pickPart(my_part);
                                gantry.goToPresetLocation(gantry.bin16_);
                                gantry.goToPresetLocation(gantry.start_);
                                gantry.goToPresetLocation(gantry.agv2_);
                                gantry.placePart(or_details[i][j][k], "agv2");

                                logicam[x][y].Shifted=true;
                                logicam2 = comp.getter_logicam_callback();
                                ROS_INFO_STREAM("\n After placing.");
                                ROS_INFO_STREAM("\n order name: "<<comp.received_orders_[i].shipments[j].products[k].type);
                                ROS_INFO_STREAM("\n order details: "<<or_details[i][j][k].pose);
                                ROS_INFO_STREAM("\n AGV camera details: "<<logicam2[11][on_table].pose);ros::Duration(1.0).sleep();
                                faulty_part = comp.quality_sensor_status();
                                if(faulty_part.faulty == true)
                                {
                                    ROS_INFO_STREAM("Faulty Part");
                                    faulty_part.type = my_part.type;
                                    ROS_INFO_STREAM("\n Trying to compute path for "<<faulty_part.type);
                                    ROS_INFO_STREAM("\n Pose at faulty part "<<logicam2[11][on_table].pose);
                                    ROS_INFO_STREAM("\n logicam0 "<<logicam2[11][0].pose);
                                    ROS_INFO_STREAM("\n logicam1 "<<logicam2[11][1].pose);
                                    faulty_part.pose = logicam2[11][on_table].pose;
                                    faulty_part.pose.position.z -= 0.19;
                                    ROS_INFO_STREAM("\n Pose for Faulty path "<<faulty_part.pose);
                                    gantry.goToPresetLocation(gantry.agv2_);
                                    gantry.pickPart(faulty_part);
                                    gantry.goToPresetLocation(gantry.agv2_);
                                    gantry.goToPresetLocation(gantry.agv2_faulty);
                                    gantry.deactivateGripper("left_arm");
                                }
                                else
                                    on_table++;

                                auto state = gantry.getGripperState("left_arm");
                                if (state.attached)
                                    gantry.goToPresetLocation(gantry.start_);
                                count++;
                                break;
                            }
                            else if (comp.received_orders_[i].shipments[j].products[k].type == "disk_part_blue" &&  logicam[x][y].Shifted==false)
                            {
                                gantry.goToPresetLocation(gantry.start_);
                                gantry.goToPresetLocation(gantry.bin13_);

                                ROS_INFO_STREAM("\n\ncamera details: "<< logicam[x][y].type);

                                part my_part;
                                my_part.type = logicam[x][y].type;
                                my_part.pose =logicam[x][y].pose;


                                gantry.pickPart(my_part);

                                gantry.goToPresetLocation(gantry.bin13_);
                                gantry.goToPresetLocation(gantry.start_);
                                gantry.goToPresetLocation(gantry.agv2_);

                                gantry.placePart(or_details[i][j][k], "agv2");
                                logicam[x][y].Shifted=true;
                                logicam2 = comp.getter_logicam_callback();
                                ROS_INFO_STREAM("\n After placing.");
                                ROS_INFO_STREAM("\n order name: "<<comp.received_orders_[i].shipments[j].products[k].type);
                                ROS_INFO_STREAM("\n order details: "<<or_details[i][j][k].pose);
                                ROS_INFO_STREAM("\n AGV camera details: "<<logicam2[11][on_table].pose);
                                ros::Duration(1.0).sleep();
                                faulty_part = comp.quality_sensor_status();
                                if(faulty_part.faulty == true)
                                {
                                    ROS_INFO_STREAM("Faulty Part");
                                    faulty_part.type = my_part.type;
                                    ROS_INFO_STREAM("\n Trying to compute path for "<<faulty_part.type);
                                    ROS_INFO_STREAM("\n Pose at faulty part "<<logicam2[11][on_table-1].pose);
                                    ROS_INFO_STREAM("\n logicam0 "<<logicam2[11][0].pose);
                                    ROS_INFO_STREAM("\n logicam1 "<<logicam2[11][1].pose);
                                    faulty_part.pose = logicam2[11][on_table].pose;
                                    faulty_part.pose.position.z -= 0.19;
                                    ROS_INFO_STREAM("\n Pose for Faulty path "<<faulty_part.pose);
                                    gantry.goToPresetLocation(gantry.agv2_);
                                    gantry.pickPart(faulty_part);
                                    gantry.goToPresetLocation(gantry.agv2_);
                                    gantry.goToPresetLocation(gantry.agv2_faulty);
                                    gantry.deactivateGripper("left_arm");
                                }
                                else
                                    on_table++;

                                ROS_INFO_STREAM("\norder name: "<<comp.received_orders_[i].shipments[j].products[k].type);
                                ROS_INFO_STREAM("\norder details: "<<or_details[i][j][k].pose);
                                ROS_INFO_STREAM("\n camera details: "<<logicam[x][y].pose);
                                auto state = gantry.getGripperState("left_arm");
                                if (state.attached)
                                    gantry.goToPresetLocation(gantry.start_);
                                count++;
                                break;
                            }

                        }
                    }

                }
            }
        }
    }


//    //--Go pick the part
//    gantry.pickPart(my_part);
//    //--Go place the part
//    gantry.placePart(part_in_tray, "agv2");

    comp.endCompetition();
    spinner.stop();
    ros::shutdown();
    return 0;
}