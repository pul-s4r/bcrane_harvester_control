#ifndef CRANE_MANAGER_H_
#define CRANE_MANAGER_H_

#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <unistd.h>

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <crane_model_v02/SolvePositionInverse.h>
#include <crane_model_v02/ObjectIdentifier.h>

const double crane_base_elevation = 0.5;
const double l_a1_rod = sqrt(pow(2.1-0.2,2) + pow(0.3,2)) - 1.0;
// const double l_a2_rod = sqrt(pow(1.05,2) + pow(0.475,2));
const double l_a2_rod = sqrt(pow(0.95,2) + pow(0.5,2)) - 0.6;

bool transform_crane_to_origin(geometry_msgs::Pose & orig_pose);
bool transform_to_desired_elevation(geometry_msgs::Pose & orig_pose, double desired_elevation);
bool transform_origin_to_crane(geometry_msgs::Pose & transformed_pose);

std::vector<geometry_msgs::Pose> generate_waypoints_retrieval(moveit::planning_interface::MoveGroupInterface * main_plan_group, const moveit::core::JointModelGroup* main_joint_model_group, geometry_msgs::Pose desired_target, std::string ee_name = "jt3_l");
std::vector<geometry_msgs::Pose> generate_waypoints_retraction(moveit::planning_interface::MoveGroupInterface * main_plan_group, const moveit::core::JointModelGroup* main_joint_model_group, geometry_msgs::Pose desired_target);

double plan_crane_serial(moveit::planning_interface::MoveGroupInterface * main_plan_group, const moveit::core::JointModelGroup* main_joint_model_group, std::vector<geometry_msgs::Pose> & waypoints, moveit_msgs::RobotTrajectory & trajectory);

bool plan_slew(moveit::planning_interface::MoveGroupInterface * slew_plan_group, const moveit::core::JointModelGroup* slew_joint_model_group, double desired_value, moveit::planning_interface::MoveGroupInterface::Plan & joint_space_plan, std::vector<double> & joint_group_positions);
bool plan_actuator_1(moveit::planning_interface::MoveGroupInterface * act1_plan_group, const moveit::core::JointModelGroup* act1_joint_model_group, double desired_value, moveit::planning_interface::MoveGroupInterface::Plan & joint_space_plan, std::vector<double> & joint_group_positions);
bool plan_actuator_2(moveit::planning_interface::MoveGroupInterface * act2_plan_group, const moveit::core::JointModelGroup* act2_joint_model_group, double desired_value, moveit::planning_interface::MoveGroupInterface::Plan & joint_space_plan, std::vector<double> & joint_group_positions);

bool plan_actuator_3(moveit::planning_interface::MoveGroupInterface & act3_plan_group, moveit::core::JointModelGroup* act3_joint_model_group, double desired_value, moveit::planning_interface::MoveGroupInterface::Plan & joint_space_plan, std::vector<double> & joint_group_positions);

void retrieveRequestCallback(const crane_model_v02::ObjectIdentifier::ConstPtr& retrieve_req);

void publishStatus(const ros::TimerEvent& event);
enum Status {state_ready = 0, state_pending = 1, state_busy = 2};
static const char *enum_str[] = {"ready", "pending", "busy"};
std::string statusEnumToString(Status st) {
    return std::string(enum_str[st]);
}

bool getActuatorSpaceStates(crane_model_v02::SolvePositionInverse * ik_srv, ros::ServiceClient * ik_client, geometry_msgs::Pose & target, sensor_msgs::JointState & result, std::string frame_id = "/base_link");

bool planAndExecuteAllExtension(std::vector<double> desired_coords, ros::ServiceClient * ik_client, crane_model_v02::SolvePositionInverse * ik_srv, double desired_elevation = 0.5, bool returning_to_home = false);

bool planAndExecuteAllRetrieval(std::vector<double> desired_coords, ros::ServiceClient * ik_client, crane_model_v02::SolvePositionInverse * ik_srv, double desired_elevation = 0.5);

void setPGSettings();

#endif
