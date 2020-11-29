#include "gantry_control.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

GantryControl::GantryControl(ros::NodeHandle & node):
        node_("/ariac/gantry"),
        planning_group_ ("/ariac/gantry/robot_description"),
        full_robot_options_("Full_Robot",planning_group_,node_),
        left_arm_options_("Left_Arm",planning_group_,node_),
        right_arm_options_("Right_Arm",planning_group_,node_),
        left_ee_link_options_("Left_Endeffector",planning_group_,node_),
        right_ee_link_options_("Right_Endeffector",planning_group_,node_),
        full_robot_group_(full_robot_options_),
        left_arm_group_(left_arm_options_),
        right_arm_group_(right_arm_options_),
        left_ee_link_group_(left_ee_link_options_),
        right_ee_link_group_(right_ee_link_options_)
{
    ROS_INFO_STREAM("[GantryControl::GantryControl] constructor called... ");
}


void GantryControl::init() {
    ROS_INFO_STREAM("[GantryControl::init] init... ");
    double time_called = ros::Time::now().toSec();


    ROS_INFO_NAMED("init", "Planning frame: %s", left_arm_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", right_arm_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", full_robot_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", right_ee_link_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", left_ee_link_group_.getPlanningFrame().c_str());

    ROS_INFO_NAMED("init", "End effector link: %s", left_arm_group_.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("init", "End effector link: %s", right_arm_group_.getEndEffectorLink().c_str());

    left_arm_group_.setPoseReferenceFrame("world");

    //--start location
    start_.gantry = {0,0,0};
    start_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    start_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    start1_.gantry = {0,0,1.57};
    start1_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    start1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};


    logicam0_.gantry={5, -1.75,0};
    logicam0_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    logicam0_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};


    logicam1_.gantry={3.082,-1.75,0};
    logicam1_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    logicam1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    logicam2_.gantry={3.082,1.75,0};
    logicam2_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    logicam2_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    logicam3_.gantry={5,1.75,0};
    logicam3_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    logicam3_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};


    bin1_.gantry = {3.08,-1.82,-2.79};
    bin1_.left_arm = {0.18,-0.42,0.86,-0.40,1.70,0};
    bin1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    bin2_.gantry = {3.08, -1.82, -1.28};
    bin2_.left_arm = {0.18,-0.42,0.86,-0.40,1.70,0};
    bin2_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    bin3_.gantry = {5.18, -1.40, 3.14};
    bin3_.left_arm = {-0.13,-0.65,1.32,-0.65,1.44,0};
    bin3_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    //no parts on 4
    bin4_.gantry = {4.95, -1.1, 0.03};
    bin4_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    bin4_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    bin8_.gantry = {4.95, -1.54, 0.78};
    bin8_.left_arm = {0.18,-0.42,0.86,-0.40,1.70,0};
    bin8_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    //no parts on 7
    bin7_.gantry = {4.95, -1.96, 0.03};
    bin7_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    bin7_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    bin6_.gantry = {3.08,-1.82,0};
    bin6_.left_arm = {0.19,-0.42,0.86,-0.40,1.70,0};
    bin6_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    bin5_.gantry = {3.08,-1.82,-4.3};
    bin5_.left_arm = {0.18,-0.42,0.86,-0.40,1.70,0};
    bin5_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    bin16_.gantry = {5,1.82,-0.75};
    bin16_.left_arm = {-0.05,-0.67,1.36,-0.72,1.56,0};
    bin16_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    bin15_.gantry = {5,1.82,-2.45};
    bin15_.left_arm = {-0.05,-0.67,1.36,-0.72,1.56,0};
    bin15_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    bin14_.gantry = {3.08, 1.82, -0.78};
    bin14_.left_arm = {-0.05,-0.67,1.36,-0.72,1.56,0};
    bin14_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    bin13_.gantry = {3.08, 1.82,-2.45};
    bin13_.left_arm = {-0.05,-0.67,1.36,-0.72,1.56,0};
    bin13_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    bin12_.gantry = {5, 1.82,0};
    bin12_.left_arm = {0.18,-0.42,0.86,-0.40,1.70,0};
    bin12_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    bin11_.gantry = {5, 1.82,1.88};
    bin11_.left_arm = {0.18,-0.42,0.86,-0.40,1.70,0};
    bin11_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    bin10_.gantry = {3.08, 1.82,0};
    bin10_.left_arm = {0.18,-0.42,0.86,-0.47,1.70,0};
    bin10_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    //no parts on 9
    bin9_.gantry = {2.10, 1.5,0};
    bin9_.left_arm = {0.0, -PI/4, PI/2, -0.80, PI/2, 0};
    bin9_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    //-------------------------------------------------------------//


    //Green_Part-- shelf 8

    //--default for left
    lc5la_.gantry = {0, -4.48, 0};
    lc5la_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    lc5la_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    lc5lb_.gantry = {-11.58, -4.48, 0};
    lc5lb_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    lc5lb_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    lc5lc_.gantry = {-11.58, -4.48, 1.57};
    lc5lc_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    lc5lc_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    lc5ld_.gantry = {-11.58, -2.99, 0.73};
    lc5ld_.left_arm = {-PI/2, -PI/4, PI/2, -PI/4, -0.2, 0};
    lc5ld_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    lc5le_.gantry = {-11.47, -1.68, 0};
    lc5le_.left_arm = {-1.78, -PI/4, PI/2, -0.78, -0.2, 0};
    lc5le_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    lc5lf_.gantry = {-13.5, -1.6, 0};
    lc5lf_.left_arm = {-1.57, -PI/4, PI/2, -0.78, -0.2, 0};
    lc5lf_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    lc5lg_.gantry = {-14, -1.2, 0};
    lc5lg_.left_arm = {-1.78, -PI/4, PI/2, -0.78, -0.2, 0};
    lc5lg_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};


    //shelf2 -- frontward
    lc15lg_.gantry = {2.70, 2.4, 0};
    lc15lg_.left_arm = {-1.82, -0.40, 1.82, -1.41, -0.25, 0};
    lc5lg_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    // shelf2 under logical camera16
    lc16lg_.gantry = {4.72, 2.4, 0};
    lc16lg_.left_arm = {-1.82, -0.40, 1.82, -1.41, -0.25, 0};
    lc16lg_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    //shelf2 -- backward
    shelf2a_.gantry = {0.23, 5.2, 3.14};
    shelf2a_.left_arm = {-1.82, -0.40, 1.82, -1.41, -0.25, 0};
    shelf2a_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    lc15rg_.gantry = {3.15, 4.92, 3.14};
    lc15rg_.left_arm = {-1.82, -0.40, 1.82, -1.41, -0.25, 0};
    lc15rg_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    lc16rg_.gantry = {5.17,4.92,3.14};
    lc16rg_.left_arm = {-1.82, -0.40, 1.82, -1.41, -0.25, 0};
    lc16rg_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};


    // front side shelf -1
    lc13ra_.gantry = {3.23, -2.30, 3.14};
    lc13ra_.left_arm = {-1.82, -0.40, 1.82, -1.41, -0.25, 0};
    lc13ra_.right_arm = {0.13, -0.13, 0.00, 0.1, 0, 0};

    lc14ra_.gantry = {5.05, -2.30, 3.14};
    lc14ra_.left_arm = {-1.82, -0.40, 1.82, -1.41, -0.25, 0};
    lc14ra_.right_arm = {0.13, -0.13, 0.00, 0.1, 0, 0};

    // way point for backside
    shelf1a_.gantry = {1.00, -5.1, 0};
    shelf1a_.left_arm = {-1.82, -0.40, 1.82, -1.41, -0.25, 0};
    shelf1a_.right_arm = {0.13, -0.13, 0.00, 0.1, 0, 0};

    // back side shelf 1
    lc13rb_.gantry = {3.23, -5.1, 0};
    lc13rb_.left_arm = {-1.82, -0.40, 1.82, -1.41, -0.25, 0};
    lc13rb_.right_arm = {0.13, -0.13, 0.00, 0.1, 0, 0};

    lc14rb_.gantry = {5.05, -5.1, 0};
    lc14rb_.left_arm = {-1.82, -0.40, 1.82, -1.41, -0.25, 0};
    lc14rb_.right_arm = {0.13, -0.13, 0.00, 0.1, 0, 0};

    lc4ra_.gantry = {0.0, 5.18, 3.14};
    lc4ra_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    lc4ra_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    lc4rb_.gantry = {-11.3, 5.18, 3.14};
    lc4rb_.left_arm = {-1.82, -0.40, 1.82, -1.41, -0.25, 0};
    lc4rb_.right_arm = {0.15, -0.15, 0.00, 0.1, 0, 0};

    lc4rc_.gantry = {-11.3, 3.15, 3.14};
    lc4rc_.left_arm = {-1.82, -0.40, 1.82, -1.41, -0.25, 0};
    lc4rc_.right_arm = {0.15, -0.15, 0.00, 0.1, 0, 0};

    lc4rd_.gantry = {-11.3, 1.90, 3.14};
    lc4rd_.left_arm = {-1.83, -0.42, 1.82, -1.40, -0.26, 0};
    lc4rd_.right_arm = {0.15, -0.15, 0.00, 0.1, 0, 0};

    lc4re_.gantry = {-15.15, 1.32, 3.14};
    lc4re_.left_arm = {-1.82, -0.40, 1.82, -1.40, -0.25, 0};
    lc4re_.right_arm = {0.15, -0.15, 0.00, 0.1, 0, 0};

//    lc4re_.gantry = {-15.1, 1.32, 3.14};
//    lc4re_.left_arm = {-1.70, -0.40, 1.40, -1.16, 0, 0};
//    lc4re_.right_arm = {0.13, -0.13, 0.00, 0.1, 0, 0};
//
//    lc4rf_.gantry = {-15.00, 1.3, 3.14};
//    lc4rf_.left_arm = {-1.78, -PI/4, PI/2, -PI/4, -0.20, 0};
//    lc4rf_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI};

    shelf11a_.gantry = {0, 1.45, 0};
    shelf11a_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    shelf11a_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf11b_.gantry = {-14.7, 1.45, 0};
    shelf11b_.left_arm = {-1.5, -PI/4, PI/2, -PI/4, 0.08,0};
    shelf11b_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf11c_.gantry = {-14.7, 1.75, 0};
    shelf11c_.left_arm = {-1.5, -PI/4, PI/2, -PI/4, 0.08, 0};
    shelf11c_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    belta_.gantry = {0.15, -1.9, PI/2};
    belta_.left_arm = {0.0, -PI/4, 1.82, -1.03, PI/2, 0};
    belta_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    beltb_.gantry = {0.15, -1.9, PI/2};
    beltb_.left_arm = {0.0, -PI/4, 1.01, -0.22, PI/2, 0};
    beltb_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    beltc_.gantry = {0.15, -1.9, 0};
    beltc_.left_arm = {0.0, -PI/4, 1.82, -1.03, PI/2, 0};
    beltc_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv2_.gantry = {0.6, 6.9, PI};
    agv2_.left_arm = {0.0, -PI/4, 1.44, -0.65, PI/2, 0};
    agv2_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv2a_.gantry = {0.6, 6.5, PI};
    agv2a_.left_arm = {0.77, -0.20, 1.3, 0.49, 1.59, 0};
    agv2a_.right_arm = {-PI/4, -3.2, -1.5, -0.02, PI/2, -PI/4};

    agv2b_.gantry = {-0.6, 6.5, PI};
    agv2b_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv2b_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv2c_.gantry = {0.55, 6.9, PI};
    agv2c_.left_arm = {-0.15, -0.30, 0.95, -0.75, 1.44, 0};
    agv2c_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv_faulty.gantry = {0, 2.0, PI};//Faulty part dropoff
    agv_faulty.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv_faulty.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv1_.gantry = {-0.55, -6.95, 0.15};
    agv1_.left_arm = {0.0, -PI/4, 1.24, -0.5, PI/2, 0};
    agv1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv1a_.gantry = {-1, -6.7, 0};
    agv1a_.left_arm = {0.0, -PI/4, 0.94, -0.03, PI/2, 0};
    agv1a_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv1b_.gantry = {-0.55, -6.95, 0.15};
    agv1b_.left_arm = {0.14, -0.3, 0.8, -0.5, 1.57, 0};
    agv1b_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv1c_.gantry = {-0.55, -6.75, 0.15};
    agv1c_.left_arm = {0.0, -PI/4, 1.24, -0.5, PI/2, 0};
    agv1c_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv1flipa_.gantry = {-0.6, -6.5, 0};
    agv1flipa_.left_arm = {0.77, -0.20, 1.3, 0.49, 1.59, 0};
    agv1flipa_.right_arm = {-PI/4, -3.2, -1.5, -0.02, PI/2, -PI/4};;

    agv1flipb_.gantry = {0.6, -6.5, -0.25};
    agv1flipb_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv1flipb_.right_arm = {-0.2, -2.7, -1.3, 0.9, 1.7, 0};


    // Gap between shelf3 and shelf4
    left_gap_1_2_.gantry = {-7.25, -5.18, 0};
    left_gap_1_2_.left_arm = {-1.48, -2.89, -1.74, -1.72, 0, 0};
    left_gap_1_2_.right_arm = {1.49, -0.34, 1.74, -1.53, 3.14, 0};

    left_gap_1_3_.gantry = {-7.25, -3.08, 0};
    left_gap_1_3_.left_arm = {-1.48, -2.89, -1.74, -1.72, 0, 0};
    left_gap_1_3_.right_arm = {1.49, -0.34, 1.74, -1.53, 3.14, 0};




    //--Raw pointers are frequently used to refer to the planning group for improved performance.
    //--To start, we will create a pointer that references the current robot’s state.
    const moveit::core::JointModelGroup* joint_model_group =
            full_robot_group_.getCurrentState()->getJointModelGroup("Full_Robot");

    //--Let’s set a joint space goal and move towards it.
    moveit::core::RobotStatePtr current_state = full_robot_group_.getCurrentState();

    //--Next get the current set of joint values for the group.
//    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_);



    gantry_joint_trajectory_publisher_ =
            node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/gantry_controller/command", 10);

    left_arm_joint_trajectory_publisher_ =
            node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/left_arm_controller/command", 10);

    right_arm_joint_trajectory_publisher_ =
            node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/right_arm_controller/command", 10);

    joint_states_subscriber_ = node_.subscribe(
            "/ariac/gantry/joint_states", 10, &GantryControl::joint_states_callback, this);

    left_gripper_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/left_arm/gripper/state", 10, &GantryControl::left_gripper_state_callback, this);

    right_gripper_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/right_arm/gripper/state", 10, &GantryControl::right_gripper_state_callback, this);

    gantry_controller_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/gantry_controller/state", 10, &GantryControl::gantry_controller_state_callback, this);

    left_arm_controller_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/left_arm_controller/state", 10, &GantryControl::left_arm_controller_state_callback, this);

    right_arm_controller_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/right_arm_controller/state", 10, &GantryControl::right_arm_controller_state_callback, this);


    while( (current_gantry_controller_state_.joint_names.size() == 0)
           || (current_left_arm_controller_state_.joint_names.size() == 0)
           || (current_right_arm_controller_state_.joint_names.size() == 0) ) {
        ROS_WARN("[GantryControl::init] Waiting for first controller_state callbacks...");
        ros::Duration(0.1).sleep();
    }

    left_gripper_control_client =
            node_.serviceClient<nist_gear::VacuumGripperControl>("/ariac/gantry/left_arm/gripper/control");
    left_gripper_control_client.waitForExistence();

    right_gripper_control_client =
            node_.serviceClient<nist_gear::VacuumGripperControl>("/ariac/gantry/right_arm/gripper/control");
    right_gripper_control_client.waitForExistence();

    // Move robot to init position
    ROS_INFO("[GantryControl::init] Init position ready)...");
}



stats GantryControl::getStats(std::string function) {
    if (function == "init") return init_;
    if (function == "moveJ") return moveJ_;
    if (function == "IK") return IK_;
    if (function == "moveGantry") return moveGantry_;
    if (function == "pickPart") return pickPart_;
    if (function == "placePart") return placePart_;
    if (function == "dropPart") return dropPart_;
    if (function == "gripFirmly") return gripFirmly_;
    if (function == "gripFromBelt") return gripFromBelt_;
    if (function == "grip") return grip_;
}

geometry_msgs::Pose GantryControl::getTargetWorldPoseRight(geometry_msgs::Pose target, std::string agv)
{
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    std::string kit_tray;
    if (agv.compare("agv1")==0)
        kit_tray = "kit_tray_1";
    else
        kit_tray = "kit_tray_2";
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = kit_tray;
    transformStamped.child_frame_id = "target_frame";
    transformStamped.transform.translation.x = target.position.x;
    transformStamped.transform.translation.y = target.position.y;
    transformStamped.transform.translation.z = target.position.z;
    transformStamped.transform.rotation.x = target.orientation.x;
    transformStamped.transform.rotation.y = target.orientation.y;
    transformStamped.transform.rotation.z = target.orientation.z;
    transformStamped.transform.rotation.w = target.orientation.w;


    for (int i{0}; i<15; ++i)
        br.sendTransform(transformStamped);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10);
    ros::Duration timeout(5.0);


    geometry_msgs::TransformStamped world_target_tf;
    geometry_msgs::TransformStamped ee_target_tf;
    for (int i=0; i< 10; i++) {
        try {
            world_target_tf = tfBuffer.lookupTransform("world", "target_frame",
                                                       ros::Time(0), timeout);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        try {
            ee_target_tf = tfBuffer.lookupTransform("target_frame", "right_ee_link",
                                                    ros::Time(0), timeout);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }

    geometry_msgs::Pose world_target{target};
    world_target.position.x = world_target_tf.transform.translation.x;
    world_target.position.y = world_target_tf.transform.translation.y;
    world_target.position.z = world_target_tf.transform.translation.z;
    world_target.orientation.x = ee_target_tf.transform.rotation.x;
    world_target.orientation.y = ee_target_tf.transform.rotation.y;
    world_target.orientation.z = ee_target_tf.transform.rotation.z;
    world_target.orientation.w = ee_target_tf.transform.rotation.w;

    return world_target;
}

geometry_msgs::Pose GantryControl::getTargetWorldPose(geometry_msgs::Pose target,
                                                      std::string agv){
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    std::string kit_tray;
    if (agv.compare("agv1")==0)
        kit_tray = "kit_tray_1";
    else
        kit_tray = "kit_tray_2";
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = kit_tray;
    transformStamped.child_frame_id = "target_frame";
    transformStamped.transform.translation.x = target.position.x;
    transformStamped.transform.translation.y = target.position.y;
    transformStamped.transform.translation.z = target.position.z;
    transformStamped.transform.rotation.x = target.orientation.x;
    transformStamped.transform.rotation.y = target.orientation.y;
    transformStamped.transform.rotation.z = target.orientation.z;
    transformStamped.transform.rotation.w = target.orientation.w;


    for (int i{0}; i<15; ++i)
        br.sendTransform(transformStamped);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10);
    ros::Duration timeout(5.0);


    geometry_msgs::TransformStamped world_target_tf;
    geometry_msgs::TransformStamped ee_target_tf;
    for (int i=0; i< 10; i++) {
        try {
            world_target_tf = tfBuffer.lookupTransform("world", "target_frame",
                                                       ros::Time(0), timeout);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        try {
            ee_target_tf = tfBuffer.lookupTransform("target_frame", "left_ee_link",
                                                    ros::Time(0), timeout);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }

    geometry_msgs::Pose world_target{target};
    world_target.position.x = world_target_tf.transform.translation.x;
    world_target.position.y = world_target_tf.transform.translation.y;
    world_target.position.z = world_target_tf.transform.translation.z;
    world_target.orientation.x = ee_target_tf.transform.rotation.x;
    world_target.orientation.y = ee_target_tf.transform.rotation.y;
    world_target.orientation.z = ee_target_tf.transform.rotation.z;
    world_target.orientation.w = ee_target_tf.transform.rotation.w;

    return world_target;
}

bool GantryControl::pickPart(part part){
    //--Activate gripper
    activateGripper("left_arm");
//    ros::AsyncSpinner spinner(1);
//    spinner.start();

//    left_arm_group_.setPoseReferenceFrame("world");
    geometry_msgs::Pose currentPose = left_arm_group_.getCurrentPose().pose;

//    ROS_INFO_STREAM("[left_arm_group_]= " << currentPose.position.x << ", " << currentPose.position.y << "," << currentPose.position.z);

    part.pose.position.z = part.pose.position.z + model_height.at(part.type) + GRIPPER_HEIGHT - EPSILON;
    part.pose.orientation.x = currentPose.orientation.x;
    part.pose.orientation.y = currentPose.orientation.y;
    part.pose.orientation.z = currentPose.orientation.z;
    part.pose.orientation.w = currentPose.orientation.w;
//    ROS_INFO_STREAM("["<< part.type<<"]= " << part.pose.position.x << ", " << part.pose.position.y << "," << part.pose.position.z << "," << part.pose.orientation.x << "," << part.pose.orientation.y << "," << part.pose.orientation.z << "," << part.pose.orientation.w);


    auto state = getGripperState("left_arm");
    if (state.enabled) {
        ROS_INFO_STREAM("[Gripper] = enabled");
        //--Move arm to part
        left_arm_group_.setPoseTarget(part.pose);
        left_arm_group_.move();
        auto state = getGripperState("left_arm");
        if (state.attached) {
            ROS_INFO_STREAM("[Gripper] = object attached");
            //--Move arm to previous position
//            left_arm_group_.setPoseTarget(currentPose);
//            left_arm_group_.move();
//            goToPresetLocation(start_);
            return true;
        }
        else {
            ROS_INFO_STREAM("[Gripper] = object not attached");
            int max_attempts{5};
            int current_attempt{0};
            while(!state.attached) {
                left_arm_group_.setPoseTarget(currentPose);
                left_arm_group_.move();
                ros::Duration(0.5).sleep();
                left_arm_group_.setPoseTarget(part.pose);
                left_arm_group_.move();
                activateGripper("left_arm");
            }
        }
    }
    else {
        ROS_INFO_STREAM("[Gripper] = not enabled");
    }
    return false;

    /**
     * We want the Cartesian path to be interpolated at a resolution of 1 cm which is why
     * we will specify 0.01 as the max step in Cartesian translation.
     * We will specify the jump threshold as 0.0, effectively disabling it.
     */

}

void GantryControl::placePart(part part, std::string agv){
    auto target_pose_in_tray = getTargetWorldPose(part.pose, agv);
    if (agv=="agv1")
        goToPresetLocation(agv1_);
    else
        goToPresetLocation(agv2_);

    target_pose_in_tray.position.z += (ABOVE_TARGET + 1.5*model_height[part.type]);

    left_arm_group_.setPoseTarget(target_pose_in_tray);
    left_arm_group_.move();
    deactivateGripper("left_arm");
}

void GantryControl::placePartRight(part part, std::string agv){
    auto target_pose_in_tray = getTargetWorldPoseRight(part.pose, agv);
//    ros::Duration(3.0).sleep();
//    goToPresetLocation(agv2b_);
    target_pose_in_tray.position.z += (ABOVE_TARGET + 1.5*model_height[part.type]);

    right_arm_group_.setPoseTarget(target_pose_in_tray);
    right_arm_group_.move();
//    ros::Duration(3.0).sleep();
    deactivateGripper("right_arm");
//    auto state = getGripperState("right_arm");
//    if (state.attached)
//        goToPresetLocation(start_);
}


void GantryControl::goToPresetLocation(PresetLocation location) {
    //--gantry
    joint_group_positions_.at(0) = location.gantry.at(0);
    joint_group_positions_.at(1) = location.gantry.at(1);
    joint_group_positions_.at(2) = location.gantry.at(2);
    //--left arm
    joint_group_positions_.at(3) = location.left_arm.at(0);
    joint_group_positions_.at(4) = location.left_arm.at(1);
    joint_group_positions_.at(5) = location.left_arm.at(2);
    joint_group_positions_.at(6) = location.left_arm.at(3);
    joint_group_positions_.at(7) = location.left_arm.at(4);
    joint_group_positions_.at(8) = location.left_arm.at(5);
    //--right arm
    joint_group_positions_.at(9) = location.right_arm.at(0);
    joint_group_positions_.at(10) = location.right_arm.at(1);
    joint_group_positions_.at(11) = location.right_arm.at(2);
    joint_group_positions_.at(12) = location.right_arm.at(3);
    joint_group_positions_.at(13) = location.right_arm.at(4);
    joint_group_positions_.at(14) = location.right_arm.at(5);

    full_robot_group_.setJointValueTarget(joint_group_positions_);


    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (full_robot_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        full_robot_group_.move();
}

/// Turn on vacuum gripper
void GantryControl::activateGripper(std::string arm_name) {
    nist_gear::VacuumGripperControl srv;
    srv.request.enable = true;

    if (arm_name == "left_arm") {
        left_gripper_control_client.call(srv);
    } else {
        right_gripper_control_client.call(srv);
    }
    ROS_INFO_STREAM("[GantryControl][activateGripper] DEBUG: srv.response =" << srv.response);
}

/// Turn off vacuum gripper
void GantryControl::deactivateGripper(std::string arm_name) {
    nist_gear::VacuumGripperControl srv;
    srv.request.enable = false;

    if (arm_name == "left_arm") {
        left_gripper_control_client.call(srv);
    } else {
        right_gripper_control_client.call(srv);
    }
    ROS_INFO_STREAM("[GantryControl][deactivateGripper] DEBUG: srv.response =" << srv.response);
}

/// Retrieve gripper state
nist_gear::VacuumGripperState GantryControl::getGripperState(std::string arm_name) {
    if (arm_name == "left_arm") {
        return current_left_gripper_state_;
    } else {
        return current_right_gripper_state_;
    }
}

/// Called when a new VacuumGripperState message is received
void GantryControl::left_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr & gripper_state_msg) {
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Gripper States (throttled to 0.1 Hz):\n" << *gripper_state_msg);
    current_left_gripper_state_ = *gripper_state_msg;
}

void GantryControl::right_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr & gripper_state_msg) {
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Gripper States (throttled to 0.1 Hz):\n" << *gripper_state_msg);
    current_right_gripper_state_ = *gripper_state_msg;
}

/// Called when a new JointState message is received
void GantryControl::joint_states_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg) {
    if (joint_state_msg->position.size() == 0) {
        ROS_ERROR("[gantry_control][joint_states_callback] msg->position.size() == 0!");
    }
    current_joint_states_ = *joint_state_msg;
}


void GantryControl::gantry_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg) {
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Gantry controller states (throttled to 0.1 Hz):\n" << *msg);
    current_gantry_controller_state_ = *msg;
}

void GantryControl::left_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg) {
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Left arm controller states (throttled to 0.1 Hz):\n" << *msg);
    current_left_arm_controller_state_ = *msg;
}

void GantryControl::right_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg) {
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Right arm controller states (throttled to 0.1 Hz):\n" << *msg);
    current_right_arm_controller_state_ = *msg;
}


bool GantryControl::send_command(trajectory_msgs::JointTrajectory command_msg) {
    // ROS_INFO_STREAM("[gantry_control][send_command] called.");

    if(command_msg.points.size() == 0) {
        ROS_WARN("[gantry_control][send_command] Trajectory is empty or NAN, returning.");
        return false;
    }
    else if ((command_msg.joint_names[0] == "small_long_joint") // command is for gantry
             && (command_msg.points[0].positions[0] == command_msg.points[0].positions[0])) {

        gantry_joint_trajectory_publisher_.publish(command_msg);
        // ROS_INFO_STREAM("[gantry_control][send_command] gantry command published!");
        return true;
    }
    else if ((command_msg.joint_names[0].substr(0,4) == "left") // command is for left_arm
             && (command_msg.points[0].positions[0] == command_msg.points[0].positions[0])) {

        left_arm_joint_trajectory_publisher_.publish(command_msg);
        // ROS_INFO_STREAM("[gantry_control][send_command] left_arm command published!");
        return true;
    }
    else if ((command_msg.joint_names[0].substr(0,5) == "right") // command is for right arm
             && (command_msg.points[0].positions[0] == command_msg.points[0].positions[0])) {

        right_arm_joint_trajectory_publisher_.publish(command_msg);
        // ROS_INFO_STREAM("[gantry_control][send_command] right_arm command published!");
        return true;
    }
    else {
        return false;
    }
}

