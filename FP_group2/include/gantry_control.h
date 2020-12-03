#ifndef GANTRYCONTROL_H
#define GANTRYCONTROL_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <array>


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

#include <nist_gear/VacuumGripperState.h>
#include <nist_gear/VacuumGripperControl.h>

#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "utils.h"
#include "competition.h"


class GantryControl {

public:
    GantryControl(ros::NodeHandle & node);

    void init();
    stats getStats(std::string function);

//    bool moveGantry(std::string waypoints);

//    bool pickPart(part part, std::string arm_name);
    bool pickPart(part part);
    void placePart(part part, std::string agv);


    /// Send command message to robot controller
    bool send_command(trajectory_msgs::JointTrajectory command_msg);
    void goToPresetLocation(PresetLocation location);
    void initialPositions(std::map<std::string,std::vector<PresetLocation>> &presetLocation, std::array<int, 3> gap_nos, std::array<int, 4> Human, bool Human_there);
    void moveToPresetLocation(std::map<std::string,std::vector<PresetLocation>> &presetLocation, std::string &location, double x, double y, int dir, std::string type, std::array<int, 3> gap_nos, Competition &comp);
    void placePartRight(part part, std::string agv);

    void activateGripper(std::string gripper_id);
    void deactivateGripper(std::string gripper_id);
    nist_gear::VacuumGripperState getGripperState(std::string arm_name);
    geometry_msgs::Pose getTargetWorldPose(geometry_msgs::Pose target, std::string agv);
    geometry_msgs::Pose getTargetWorldPoseRight(geometry_msgs::Pose target, std::string agv);
    //--preset locations;
    start start_, start1_;
    bin1 bin1_;bin2 bin2_, bin2b_;bin3 bin3_;bin4 bin4_;bin5 bin5_;bin6 bin6_;bin7 bin7_;bin8 bin8_;bin9 bin9_;bin10 bin10_;bin11 bin11_;bin12 bin12_;bin13 bin13_;bin14 bin14_;bin15 bin15_;bin16 bin16_;
    lc15lg lc15lg_;lc16lg lc16lg_; shelf2a  shelf2a_;lc15rg lc15rg_;lc16rg lc16rg_;
    lc13ra lc13ra_;lc14ra lc14ra_;lc13rb lc13rb_;lc14rb lc14rb_;shelf1a shelf1a_;
    belt belta_, beltb1_, beltb2_, beltc_, beltc2_, beltd2_;
    lc4r lc4ra_, lc4ra1_,lc4rb_,lc4rc_,lc4rd_,lc4rd1_,lc4re_,lc4rf_,lc5r_;
    lc5l lc5la_,lc6la_, lc6lb_,lc7l_,lc5la1_,lc5lb_,lc5lc_,lc5ld_,lc5ld3_,lc5ld2_,lc5ld1_,lc5le_,lc5lf_,lc5lf1_,lc5lg_, lc4l_;
    lc7r lc7ra_,lc7rb_,lc7rc_, lc6r_;
    shelf11 lc8la_, lc8lb_, lc8lc_, lc9l_, lc8ra_, lc8rb_, lc9r_;
    agv2 agv2_, agv2f_, agv2flt_, agv2flb_, agv2frt_, agv2frb_, agv2a_, agv2b_, agv2c_;
    agv2 agv_faulty;
    agv1 agv1_, agv1a_, agv1b_, agv1c_, agv1flipa_, agv1flipb_, agv1flt_, agv1flb_, agv1frt_, agv1frb_, agv1f_;
    left_gap_1_2 left_gap_1_2_;
    left_gap_1_3 left_gap_1_3_;
    logicam0    logicam0_;
    logicam1    logicam1_;
    logicam2    logicam2_;
    logicam3    logicam3_;
    logicam13   logicam13r_, logicam13l1_, logicam13l2_;
    logicam14   logicam14r_, logicam14ra_, logicam14l_;
    logicam15   logicam15r_, logicam15ra_, logicam15r1_, logicam15l_, logicam15la_;
    logicam16   logicam16r_, logicam16l_;
    shelf1 shelf1_fl,shelf1_bl,shelf1_fr,shelf1_br,shelf2_fl, shelf2_bl,shelf2_fr,shelf2_br;
    aisle2_1 aisle2_1_;
    aisle2_2 aisle2_2_;
    aisle2_3 aisle2_3_;
    aisle2_4 aisle2_4_;
    aisle2_5 aisle2_5_;
    aisle2_6 aisle2_6_;
    aisle4_6 aisle4_6_;
    aisle4_5 aisle4_5_;
    aisle4_4 aisle4_4_;
    aisle4_3 aisle4_3_;
    aisle4_2 aisle4_2_;
    aisle4_1 aisle4_1_;


private:
    std::vector<double> joint_group_positions_;
    ros::NodeHandle node_;
    std::string planning_group_;
    moveit::planning_interface::MoveGroupInterface::Options full_robot_options_;
    moveit::planning_interface::MoveGroupInterface::Options left_arm_options_;
    moveit::planning_interface::MoveGroupInterface::Options right_arm_options_;
    moveit::planning_interface::MoveGroupInterface::Options left_ee_link_options_;
    moveit::planning_interface::MoveGroupInterface::Options right_ee_link_options_;
    moveit::planning_interface::MoveGroupInterface full_robot_group_;
    moveit::planning_interface::MoveGroupInterface left_arm_group_;
    moveit::planning_interface::MoveGroupInterface right_arm_group_;
    moveit::planning_interface::MoveGroupInterface left_ee_link_group_;
    moveit::planning_interface::MoveGroupInterface right_ee_link_group_;

    double left_ee_roll_;
    double left_ee_pitch_;
    double left_ee_yaw_;
    std::array<float,4> left_ee_quaternion_;

    sensor_msgs::JointState current_joint_states_;


    nist_gear::VacuumGripperState current_left_gripper_state_;
    nist_gear::VacuumGripperState current_right_gripper_state_;

    control_msgs::JointTrajectoryControllerState current_gantry_controller_state_;
    control_msgs::JointTrajectoryControllerState current_left_arm_controller_state_;
    control_msgs::JointTrajectoryControllerState current_right_arm_controller_state_;

    ros::Publisher gantry_joint_trajectory_publisher_;
    ros::Publisher left_arm_joint_trajectory_publisher_;
    ros::Publisher right_arm_joint_trajectory_publisher_;

    ros::Subscriber joint_states_subscriber_;
    ros::Subscriber left_gripper_state_subscriber_;
    ros::Subscriber right_gripper_state_subscriber_;
    ros::Subscriber gantry_controller_state_subscriber_;
    ros::Subscriber left_arm_controller_state_subscriber_;
    ros::Subscriber right_arm_controller_state_subscriber_;

    ros::ServiceClient left_gripper_control_client;
    ros::ServiceClient right_gripper_control_client;

    // ---------- Callbacks ----------
    void joint_states_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg);
    void left_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr & msg);
    void right_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr & msg);
    void gantry_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg);
    void left_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg);
    void right_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg);




    // collect stats
    stats init_;
    stats moveJ_;
    stats IK_;
    stats moveGantry_;
    stats pickPart_;
    stats placePart_;
    stats dropPart_;
    stats gripFirmly_;
    stats gripFromBelt_;
    stats grip_;
};

#endif
