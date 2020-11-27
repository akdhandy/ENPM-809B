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
    void placePartRight(part part, std::string agv);
//    void logicam_presets(std::string logical_camera_4, std::string logical_camera_5);

    void activateGripper(std::string gripper_id);
    void deactivateGripper(std::string gripper_id);
    nist_gear::VacuumGripperState getGripperState(std::string arm_name);
    geometry_msgs::Pose getTargetWorldPose(geometry_msgs::Pose target, std::string agv);
    geometry_msgs::Pose getTargetWorldPoseRight(geometry_msgs::Pose target, std::string agv);
    //--preset locations;
    start start_, start1_;
    bin1 bin1_;bin2 bin2_;bin3 bin3_;bin4 bin4_;bin5 bin5_;bin6 bin6_;bin7 bin7_;bin8 bin8_;bin9 bin9_;bin10 bin10_;bin11 bin11_;bin12 bin12_;bin13 bin13_;bin14 bin14_;bin15 bin15_;bin16 bin16_;
    lc15lg lc15lg_;lc16lg lc16lg_; shelf2a  shelf2a_;lc15rg lc15rg_;lc16rg lc16rg_;
    lc13ra lc13ra_;lc14ra lc14ra_;lc13rb lc13rb_;lc14rb lc14rb_;shelf1a shelf1a_;
    belt belta_, beltb_, beltc_;
    lc4r lc4ra_,lc4rb_,lc4rc_,lc4rd_,lc4re_,lc4rf_;
    lc5l lc5la_,lc5lb_,lc5lc_,lc5ld_,lc5le_,lc5lf_,lc5lg_;
    shelf11 shelf11a_, shelf11b_, shelf11c_;
    agv2 agv2_, agv2a_, agv2b_, agv2c_;
    agv2 agv_faulty;
    agv1 agv1_, agv1a_, agv1b_, agv1c_, agv1flipa_, agv1flipb_;
    left_gap_1_2 left_gap_1_2_;
    left_gap_1_3 left_gap_1_3_;

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
