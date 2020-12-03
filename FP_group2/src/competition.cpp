#include "competition.h"
#include "utils.h"

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

std::array<std::array<modelparam, 36>, 17> logical_cam;
std::array<std::array<std::array<part, 10>, 5>, 5> order_details;
part faulty_part_agv2, faulty_part_agv1;

Competition::Competition(ros::NodeHandle &node): current_score_(0)
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

    fp_subscriber_ = node_.subscribe(
            "/ariac/quality_control_sensor_1", 10, &Competition::quality_sensor_status_callback, this);     //agv2

    fp_subscriber1_ = node_.subscribe(
            "/ariac/quality_control_sensor_2", 10, &Competition::quality_sensor_status_callback2, this);    //agv1


    startCompetition();

    init_.total_time += ros::Time::now().toSec() - time_called;

}


part Competition::quality_sensor_status(){
    return faulty_part_agv2;
}

part Competition::quality_sensor_status1(){
    return faulty_part_agv1;
}

void Competition::breakbeam_sensor_callback(const nist_gear::Proximity::ConstPtr &msg, int id)
{
    beam_detect[id] = msg->object_detected;
    beam_seq2[id] = msg->header.seq;
}

void Competition::breakbeam_sensing()
{
    for (int i =6; i<16; i++)
        if (beam_detect[i]==true)
        {
            do{
                ROS_INFO_STREAM("\nHuman is at breakbeam: "<<i);
            }while(beam_detect[i]==true);
            beam_seq[i]=beam_seq2[i];
            ROS_INFO_STREAM("\n Sequence id: "<<beam_seq[i]);
            break;
        }
}

void Competition::HumanDetection()
{
    for (auto i=0; i<4; i++)
    {
        if (beam_detect[i+21]==true || beam_detect[i+25]==true)
        {
            Human[i] = 1;
            continue;
        }
    }
    int count =0;
    for (auto j=0; j<4; j++) {
        if (Human[j] == 1) {
            ROS_INFO_STREAM("Human is at aisle: " << j + 1);
            count++;
        }
    }
    if (count)
        Human_detected = true;
    else
        Human_detected = false;
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
        }
    }
}

/*
 * Distance between sensors---Nishanth
 *
 */
geometry_msgs::TransformStamped Competition::shelf_pose_callback(std::string frame_name)
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Duration timeout(5.0);

    try {
        TfStamped = tfBuffer.lookupTransform("world", frame_name,
                                             ros::Time(0), timeout);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    //ROS_INFO_STREAM("shelf_poses" << TfStamped.transform.translation.x);
    return TfStamped;
//    }
}

double Competition::shelf_distance(std::string frame_id_1, std::string frame_id_2) {

    geometry_msgs::TransformStamped s1,s2;
    s1 = shelf_pose_callback(frame_id_1);
    s2 = shelf_pose_callback(frame_id_2);

    double distance =  double(abs(abs(s1.transform.translation.x)-abs(s2.transform.translation.x)));
    //ROS_INFO_STREAM("Distance between shelves = " << distance);

    return distance;
}

std::vector<std::string> Competition::check_gaps()
{
    std::vector<std::string> gap_id;
    for (int shelf_ind = 1;shelf_ind < 4; shelf_ind++)
    {
        std::string frame_id_1 = "shelf" + std::to_string(3*shelf_ind) + "_frame";
        std::string frame_id_2 = "shelf" + std::to_string((3*shelf_ind) + 1) + "_frame";
        std::string frame_id_3 = "shelf" + std::to_string((3*shelf_ind) + 2) + "_frame";
        auto shelf_dis_1 = shelf_distance(frame_id_1, frame_id_2);
        auto shelf_dis_2 = shelf_distance(frame_id_2, frame_id_3);

        if (shelf_dis_1 > 6)
        {
            gap_id.push_back("Gap between shelf" + std::to_string(3*shelf_ind) + " and shelf" + std::to_string((3*shelf_ind) + 1));
            gap_nos[shelf_ind-1]=33*shelf_ind+1;
//            ROS_INFO_STREAM("Distance between shelves = " << shelf_dis_1);

        }
        if (shelf_dis_2 > 6)
        {
            gap_id.push_back("Gap between shelf" + std::to_string((3*shelf_ind) + 1) + " and shelf" + std::to_string((3*shelf_ind) + 2));
            gap_nos[shelf_ind-1]=33*shelf_ind+12;
//            ROS_INFO_STREAM("Distance between shelves = " << shelf_dis_2);
        }
    }
    for(auto i: gap_id)
        ROS_INFO_STREAM(i);

    return gap_id;
}

void Competition::quality_sensor_status_callback(const nist_gear::LogicalCameraImage::ConstPtr &msg)
{
    if (msg->models.size() > 0)
    {
        faulty_part_agv2.faulty = true;
        faulty_part_agv2.pose = msg->models[0].pose;
    }
    else
        faulty_part_agv2.faulty = false;
}

void Competition::quality_sensor_status_callback2(const nist_gear::LogicalCameraImage::ConstPtr &msg)
{
    if (msg->models.size() > 0)
    {
        faulty_part_agv1.faulty = true;
        faulty_part_agv1.pose = msg->models[0].pose;
    }
    else
        faulty_part_agv1.faulty = false;
}

/// Called when a new message is received.
void Competition::competition_state_callback(const std_msgs::String::ConstPtr & msg) {
    if (msg->data == "done" && competition_state_ != "done")
    {
//    ROS_INFO("Competition ended.");
    }
    competition_state_ = msg->data;
}

void Competition::order_callback(const nist_gear:://                            ROS_INFO_STREAM("\n Array containing the order details of part on belt \n "<<belt_part_arr[on_belt][0]<<belt_part_arr[on_belt][1]<<belt_part_arr[on_belt][2]);
Order::ConstPtr & msg) {
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
                order_details[i][j][k].shipment = received_orders_[i].shipments[j].shipment_type;
                order_details[i][j][k].Shifted = false;
            }
        }
    }
}

void Competition::PartonBeltCheck(std::vector<nist_gear::Order> received, int x_loop, std::array<std::array<modelparam, 36>, 17> logicam, std::array<std::array<int, 3>, 5> &belt_part_arr, int &on_belt)
{
    for (int i = received.size() - 1; i >= 0; i--)
    {
        for (int j = 0; j < received[i].shipments.size(); j++)
        {
            for (int k = 0; k < received[i].shipments[j].products.size(); k++)
            {
                for (int x = 0; x < 17; x++)
                {
                    if (x_loop != 0)
                    {
                        x_loop = 0;
                        break;
                    }
                    for (int y = 0; y < 36; y++)
                    {
                        if (logicam[x][y].type == received[i].shipments[j].products[k].type)
                        {
                            ROS_INFO_STREAM("\n" << received[i].shipments[j].products[k].type
                                                 << " under logical camera " << x);
                            ROS_INFO_STREAM("\n Part seen " << logicam[x][y].type);
                            ROS_INFO_STREAM("\n order = " << i << ", shipment = " << j << ", products = " << k);
                            x_loop++;
                            break;
                        }
                        if ((x == 16) && (y == 35))
                        {
                            belt_part_arr[on_belt][0] = i;
                            belt_part_arr[on_belt][1] = j;
                            belt_part_arr[on_belt][2] = k;
                            on_belt++;
                            ROS_INFO_STREAM(
                                    "\n Order details of the " << received[i].shipments[j].products[k].type<<" part - i = " << i << ", j = " << j << ", k = " << k);
                            ROS_INFO_STREAM("\n Part is on the belt");
                            ROS_INFO_STREAM("\n Number of parts on the belt " << on_belt);
                        }
                    }
                }
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
