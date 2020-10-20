#include "competition.h"
#include "utils.h"

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

std::array<std::array<modelparam, 36>, 17> logical_cam;
std::array<std::array<std::array<part, 10>, 5>, 5> order_details;

Competition::Competition(ros::NodeHandle & node): current_score_(0)
{
  node_ = node;
}

void Competition::init() {
  // Subscribe to the '/ariac/current_score' topic.
  double time_called = ros::Time::now().toSec();
  competition_start_time_ = ros::Time::now().toSec();


    // Subscribe to the '/ariac/competition_state' topic.
//  ROS_INFO("Subscribe to the /ariac/competition_state topic...");
  competition_state_subscriber_ = node_.subscribe(
    "/ariac/competition_state", 10, &Competition::competition_state_callback, this);

  // Subscribe to the '/clock' topic.
//  ROS_INFO("Subscribe to the /clock...");
  competition_clock_subscriber_ = node_.subscribe(
    "/clock", 10, &Competition::competition_clock_callback, this);

//    ROS_INFO("Subscribe to the /orders...");
    orders_subscriber_ = node_.subscribe(
            "/ariac/orders", 10, &Competition::order_callback, this);



    startCompetition();

  init_.total_time += ros::Time::now().toSec() - time_called;

}

void Competition::logical_camera_callback(const nist_gear::LogicalCameraImage::ConstPtr &msg, int id)
{

    if (msg->models.size() > 0)
    {
//        ROS_INFO_STREAM("Logical camera " + std::to_string(id) + " detected '" << msg->models.size()<< "' objects.");
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        ros::Duration timeout(5.0);

        geometry_msgs::TransformStamped TfStamped;
        geometry_msgs::PoseStamped pose_target,pose_real;
        std::string frame_name = "logical_camera_" + std::to_string(id) + "_frame";

        try {
            TfStamped = tfBuffer.lookupTransform("world", frame_name,
                                                 ros::Time(0), timeout);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        for (int i = 0; i < msg->models.size(); i++) {
            pose_target.header.frame_id =
                    "logical_camera_" + std::to_string(id) + "_frame";
            pose_target.pose = msg->models[i].pose;
            tf2::doTransform(pose_target,pose_real, TfStamped);
            std::string partName = msg->models[i].type;
//            std::cout<<partName;
            logical_cam[id][i].frame = pose_target.header.frame_id.c_str();
            logical_cam[id][i].type = msg->models[i].type.c_str();
            logical_cam[id][i].pose = pose_real.pose;
//            ROS_INFO("%s in world frame:\t"
//                     "Position: [x,y,z] = [%f,%f,%f]\t"
//                     "Orientation: [x,y,z,w] = [%f,%f,%f,%f]\t",
//                     logical_cam[id][i].type.c_str(),
//                     logical_cam[id][i].pose_world.position.x,
//                     logical_cam[id][i].pose_world.position.y,
//                     logical_cam[id][i].pose_world.position.z,
//                     logical_cam[id][i].pose_world.orientation.x,
//                     logical_cam[id][i].pose_world.orientation.y,
//                     logical_cam[id][i].pose_world.orientation.z,
//                     logical_cam[id][i].pose_world.orientation.w);
        }
    }
}

/// Called when a new message is received.
void Competition::competition_state_callback(const std_msgs::String::ConstPtr & msg) {
  if (msg->data == "done" && competition_state_ != "done")
  {
//    ROS_INFO("Competition ended.");
  }
  competition_state_ = msg->data;
}

void Competition::order_callback(const nist_gear::Order::ConstPtr & msg) {
//    ROS_INFO_STREAM("Received order:\n" << *msg);
    received_orders_.push_back(*msg);
    for (int i=0; i<received_orders_.size(); i++)
    {
        for (int j=0; j<received_orders_[i].shipments.size(); j++)
        {
            for (int k=0; k<received_orders_[i].shipments[j].products.size(); k++)
            {
                order_details[i][j][k].type = received_orders_[i].shipments[j].products[k].type;
                order_details[i][j][k].pose.position.x = received_orders_[i].shipments[j].products[k].pose.position.x;
                order_details[i][j][k].pose.position.y = received_orders_[i].shipments[j].products[k].pose.position.y;
                order_details[i][j][k].pose.position.z = received_orders_[i].shipments[j].products[k].pose.position.z;
                order_details[i][j][k].pose.orientation.x = received_orders_[i].shipments[j].products[k].pose.orientation.x;
                order_details[i][j][k].pose.orientation.y = received_orders_[i].shipments[j].products[k].pose.orientation.y;
                order_details[i][j][k].pose.orientation.z = received_orders_[i].shipments[j].products[k].pose.orientation.z;
                order_details[i][j][k].pose.orientation.w = received_orders_[i].shipments[j].products[k].pose.orientation.w;
                order_details[i][j][k].agv_id = received_orders_[i].shipments[j].agv_id;
                order_details[i][j][k].Shifted = false;
            }
        }
    }
}


std::array<std::array<std::array<part, 10>, 5>, 5> Competition::getter_part_callback()
{
    return order_details;
}

std::array<std::array<modelparam, 36>, 17> Competition::getter_logicam_callback()
{
    return logical_cam;
}

/// Called when a new message is received.
void Competition::competition_clock_callback(const rosgraph_msgs::Clock::ConstPtr & msg) {
  competition_clock_ = msg->clock;
}


void Competition::startCompetition() {
  // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient start_client =
    node_.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!start_client.exists()) {
//    ROS_INFO("[competition][startCompetition] Waiting for the competition to be ready...");
    start_client.waitForExistence();
//    ROS_INFO("[competition][startCompetition] Competition is now ready.");
  }
//  ROS_INFO("[competition][startCompetition] Requesting competition start...");
  std_srvs::Trigger srv;
  start_client.call(srv);
  if (!srv.response.success) {  // If not successful, print out why.
    ROS_ERROR_STREAM("[competition][startCompetition] Failed to start the competition: " << srv.response.message);
  } else {
//    ROS_INFO("[competition][startCompetition] Competition started!");
  }
}


void Competition::endCompetition() {
  // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient end_client =
    node_.serviceClient<std_srvs::Trigger>("/ariac/end_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!end_client.exists()) {
//    ROS_INFO("[competition][endCompetition] Waiting for the end_competition to be ready...");
    end_client.waitForExistence();
//    ROS_INFO("[competition][endCompetition] end_competition is now ready.");
  }
//  ROS_INFO("[competition][endCompetition] Requesting competition end...");
  std_srvs::Trigger srv;
  end_client.call(srv);
  if (!srv.response.success) {  // If not successful, print out why.
    ROS_ERROR_STREAM("[competition][endCompetition] Failed to end the competition: " << srv.response.message);
  } else {
//    ROS_INFO("[competition][endCompetition] Competition ended!");
  }
}


stats Competition::getStats(std::string function) {
  if (function == "init") return init_;

}

double Competition::getStartTime() {
  return competition_start_time_;
}

double Competition::getClock() {
  double time_spent = competition_clock_.toSec();
  ROS_INFO_STREAM("[competition][getClock] competition time spent (getClock()) =" << time_spent);
  return time_spent;
}


std::string Competition::getCompetitionState() {
  return competition_state_;
}
