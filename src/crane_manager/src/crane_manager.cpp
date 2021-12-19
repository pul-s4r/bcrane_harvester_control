#include <crane_manager/crane_manager.h>


/* Setup - define planning groups, interfaces, planning scenes */
static const std::string MAIN_PG = "crane_arm_serial";
static const std::string SLEW_PG = "actuator_slew_controller";
static const std::string ACT1_PG = "actuator_base_controller";
static const std::string ACT2_PG = "actuator_jib_controller";
static const std::string ACT3_PG = "arm_extension_controller";

moveit::planning_interface::MoveGroupInterface * main_plan_group;
moveit::planning_interface::MoveGroupInterface * slew_plan_group;
moveit::planning_interface::MoveGroupInterface * act1_plan_group;
moveit::planning_interface::MoveGroupInterface * act2_plan_group;
moveit::planning_interface::MoveGroupInterface * act3_plan_group;

// moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

const moveit::core::JointModelGroup* main_joint_model_group;
const moveit::core::JointModelGroup* slew_joint_model_group;
const moveit::core::JointModelGroup* act1_joint_model_group;
const moveit::core::JointModelGroup* act2_joint_model_group;
const moveit::core::JointModelGroup* act3_joint_model_group;

ros::ServiceClient * ik_client;
crane_model_v02::SolvePositionInverse * ik_srv;
std::vector<double> home_position;

std::string retrieval_status = statusEnumToString(state_pending);
ros::Subscriber sub_target_object;
ros::Publisher pub_ready_status;
ros::Timer pub_ready_timer;

int main(int argc, char** argv) {
    ros::init(argc, argv, "crane_manager");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    main_plan_group = new moveit::planning_interface::MoveGroupInterface(MAIN_PG);
    slew_plan_group = new moveit::planning_interface::MoveGroupInterface(SLEW_PG); 
    act1_plan_group = new moveit::planning_interface::MoveGroupInterface(ACT1_PG);
    act2_plan_group = new moveit::planning_interface::MoveGroupInterface(ACT2_PG);
    act3_plan_group = new moveit::planning_interface::MoveGroupInterface(ACT3_PG);

    main_joint_model_group =
    main_plan_group->getCurrentState()->getJointModelGroup(MAIN_PG);
    act1_joint_model_group =
    act1_plan_group->getCurrentState()->getJointModelGroup(ACT1_PG);
    act2_joint_model_group =
    act2_plan_group->getCurrentState()->getJointModelGroup(ACT2_PG);
    act3_joint_model_group =
    act3_plan_group->getCurrentState()->getJointModelGroup(ACT3_PG);

    /* Setup - planning group */
    setPGSettings();

    /* Setup - joint to actuator space IK solver */
    ros::NodeHandle n;
    ros::ServiceClient ik_client_actual = n.serviceClient<crane_model_v02::SolvePositionInverse>("/CraneExtTools/crane_ik_service");
    ik_client = & ik_client_actual;
    ik_srv = new crane_model_v02::SolvePositionInverse();
    ik_srv->request.pose_stamped.resize(1);
    home_position = {0, -2.734, 0.676};

    // sub_target_object = n.subscribe("/object_data/current_target", 10, retrieveRequestCallback);
    pub_ready_status = n.advertise<std_msgs::String>("/manager/status", 10);
    retrieval_status = statusEnumToString(state_ready);
    pub_ready_timer = n.createTimer(ros::Duration(1.0), publishStatus);


    double desired_elevation = 0.5;
    // std::vector<double> desired_coords = {2.800, -0.500, 0.100};
    std::vector<double> desired_coords = {2.3+0.462+0.2, 0.0+0.2, 0.446+0.2};
    //
    planAndExecuteAllExtension(desired_coords, ik_client, ik_srv, desired_elevation);

    // ros::shutdown();
    ros::waitForShutdown();
    return 0;
}

/* Object designated message */
void retrieveRequestCallback(const crane_model_v02::ObjectIdentifier::ConstPtr& retrieve_req)
{
    ROS_INFO("Got: [%ld]", retrieve_req->index);
    retrieval_status = statusEnumToString(state_pending);
    std::vector<double> desired_coords = retrieve_req->coords;
    double desired_elevation = retrieve_req->elevation;
    retrieval_status = statusEnumToString(state_busy);
    planAndExecuteAllExtension(desired_coords, ik_client, ik_srv, desired_elevation, false);

    /* Move to desired position */
    moveit::core::RobotStatePtr current_state = main_plan_group->getCurrentState();
    const Eigen::Affine3d& ee_link_pose = current_state->getGlobalLinkTransform("jt3_l");
    Eigen::Vector3d ee_link_cart_pos = ee_link_pose.translation();

    std::cout << "Position for " << retrieve_req->index << ":(" << ee_link_cart_pos[0] << ", " << ee_link_cart_pos[1] << ", " << ee_link_cart_pos[2] << ")" << std::endl;

    /* Move to home position */
    planAndExecuteAllExtension(home_position, ik_client, ik_srv, 0, true);
    // Test this

    retrieval_status = statusEnumToString(state_ready);

    // Return status
}

void publishStatus(const ros::TimerEvent& event) {
    std_msgs::String msg;
    msg.data = retrieval_status;
    pub_ready_status.publish(msg);
}


bool getActuatorSpaceStates(crane_model_v02::SolvePositionInverse * ik_srv, ros::ServiceClient * ik_client, geometry_msgs::Pose & target, sensor_msgs::JointState & result, std::string frame_id) {
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = frame_id;
    target_pose.header.stamp = ros::Time::now();
    target_pose.pose.position = target.position;
    target_pose.pose.orientation = target.orientation;
    ik_srv->request.pose_stamped[0] = target_pose;

    std::cout << "Target pose: " << "(" << target_pose.pose.position.x << ", " << target_pose.pose.position.y << ", " << target_pose.pose.position.z << ")" << std::endl;

    bool call_success = ik_client->call(*ik_srv);
    if (call_success) {
        ROS_INFO("Joint positions: theta_0/%f d1/%f d2/%f", ik_srv->response.joints[0].position[0], ik_srv->response.joints[0].position[1], ik_srv->response.joints[0].position[2]);
        result = ik_srv->response.joints[0];
    } else {
        ROS_ERROR("Failed to call service crane_ik_service");
    }
    return call_success;
}

bool planAndExecuteAllExtension(std::vector<double> desired_coords, ros::ServiceClient * ik_client, crane_model_v02::SolvePositionInverse * ik_srv, double desired_elevation, bool returning_to_home) {
    /* Sample - plan to trajectory goal (waypoints) */
    geometry_msgs::Pose desired_target;
    desired_target.orientation.w = 1.0;
    desired_target.position.x = desired_coords[0];
    desired_target.position.y = desired_coords[1];
    desired_target.position.z = desired_coords[2];
    transform_crane_to_origin(desired_target);
    transform_to_desired_elevation(desired_target, desired_elevation);

    // std::cout << "End state: " << waypoints.back() << std::endl;

    /* Compute trajectory as waypoints in cartesian mode */
    // std::vector<geometry_msgs::Pose> waypoints = generate_waypoints_retrieval(main_plan_group, main_joint_model_group, desired_target);
    // std::vector<geometry_msgs::Pose> waypoints;
    // moveit::core::RobotStatePtr current_state = main_plan_group->getCurrentState();
    // const Eigen::Affine3d& ee_link_pose = current_state->getGlobalLinkTransform("jt3_l");
    // Eigen::Vector3d ee_link_cart_pos = ee_link_pose.translation();
    // geometry_msgs::Pose start_pose;
    // start_pose.orientation.w = 1.0;
    // start_pose.position.x = ee_link_cart_pos[0];
    // start_pose.position.y = ee_link_cart_pos[1];
    // start_pose.position.z = ee_link_cart_pos[2];
    // waypoints.push_back(start_pose);
    // // start_pose.position.y += 0.2;
    // waypoints.push_back(start_pose);
    // moveit_msgs::RobotTrajectory trajectory;
    // double fraction = plan_crane_serial(main_plan_group, main_joint_model_group, waypoints, trajectory);

    // std::cout << "Points [s0]: " << std::endl;
    // int num_waypoints = trajectory.joint_trajectory.points.size();
    // for (int i = 0; i < num_waypoints; i++) {
    //     std::cout << trajectory.joint_trajectory.points[i].positions[0] << ", ";
    // }
    // std::cout << std::endl;

    // std::cout << "Trajectory: " << trajectory << std::endl;

    sensor_msgs::JointState ik_result;
    bool ik_call_success = getActuatorSpaceStates(ik_srv, ik_client, desired_target, ik_result);

    /* Plan to joint space goal - base joint */
    std::vector<double> joint_group_positions;
    double s0_desired_position = (double) ik_result.position[0];
    moveit::planning_interface::MoveGroupInterface::Plan joint_space_plan_0;
    joint_group_positions.clear();
    bool s0_success = plan_slew(main_plan_group,  main_joint_model_group, s0_desired_position, joint_space_plan_0, joint_group_positions);
    std::cout << "Joint space positions [s0]: " << std::endl;
    for (int i = 0; i < joint_group_positions.size(); i++) {
        std::cout << joint_group_positions[i] << ", ";
    }
    std::cout << std::endl;

    /* Plan to joint space goal - actuator 1 */
    double a1_desired_position = (double) ik_result.position[1] - l_a1_rod;
    joint_group_positions.clear();
    moveit::planning_interface::MoveGroupInterface::Plan joint_space_plan_1;
    bool a1_success = plan_actuator_1(act1_plan_group,  act1_joint_model_group, a1_desired_position, joint_space_plan_1, joint_group_positions);

    std::cout << "Joint space positions [a1]: " << std::endl;
    for (int i = 0; i < joint_group_positions.size(); i++) {
        std::cout << joint_group_positions[i] << ", ";
    }
    std::cout << std::endl;


    double a2_desired_position = (double) ik_result.position[2] - l_a2_rod;
    /* Plan to joint space goal - actuator 2 */
    joint_group_positions.clear();
    moveit::planning_interface::MoveGroupInterface::Plan joint_space_plan_2;

    bool a2_success = plan_actuator_2(act2_plan_group, act2_joint_model_group, a2_desired_position, joint_space_plan_2, joint_group_positions);

    std::cout << "Joint space positions [a2]: " << std::endl;
    for (int i = 0; i < joint_group_positions.size(); i++) {
        std::cout << joint_group_positions[i] << ", ";
    }
    std::cout << std::endl;

    double a3_desired_position = (double) desired_elevation;
    joint_group_positions.clear();
    moveit::planning_interface::MoveGroupInterface::Plan joint_space_plan_3;

    bool a3_success = plan_actuator_2(act3_plan_group, act3_joint_model_group, a3_desired_position, joint_space_plan_3, joint_group_positions);

    std::cout << "Joint space positions [ext]: " << a3_desired_position << std::endl;

    if (returning_to_home) {
        // act3_plan_group->execute(joint_space_plan_3.trajectory_);
        // main_plan_group->execute(trajectory);
        act1_plan_group->execute(joint_space_plan_1.trajectory_);
        act2_plan_group->execute(joint_space_plan_2.trajectory_);
        // main_plan_group->execute(joint_space_plan_0.trajectory_);
    } else {
        // main_plan_group->execute(trajectory);
        act1_plan_group->execute(joint_space_plan_1.trajectory_);
        act2_plan_group->execute(joint_space_plan_2.trajectory_);
        joint_group_positions.clear();
        // bool s0_success = plan_slew(main_plan_group,  main_joint_model_group, s0_desired_position, joint_space_plan_0, joint_group_positions);

        // main_plan_group->execute(joint_space_plan_0.trajectory_);
        std::cout << "s0: Done" << std::endl;
        // act3_plan_group->execute(joint_space_plan_3.trajectory_);
    }

    ROS_INFO_NAMED("crane_manager", "Plan 1: " + a1_success ? "SUCCESS": "FAILED");
    ROS_INFO_NAMED("crane_manager", "Plan 2: " + a2_success ? "SUCCESS": "FAILED");

    return a1_success && a2_success;
}

void setPGSettings() {
    main_plan_group->setMaxVelocityScalingFactor(0.001);
    main_plan_group->setMaxAccelerationScalingFactor(0.001);
    act1_plan_group->setMaxVelocityScalingFactor(0.05);
    act1_plan_group->setMaxAccelerationScalingFactor(0.05);
    act1_plan_group->setGoalTolerance(0.05);
    act2_plan_group->setMaxVelocityScalingFactor(0.05);
    act2_plan_group->setMaxAccelerationScalingFactor(0.05);
    act2_plan_group->setGoalTolerance(0.05);
}

bool transform_crane_to_origin(geometry_msgs::Pose & orig_pose) {
    orig_pose.position.z -= crane_base_elevation;
    return true;
}
bool transform_to_desired_elevation(geometry_msgs::Pose & orig_pose, double desired_elevation) {
    orig_pose.position.z += desired_elevation;
    return true;
}
bool transform_origin_to_crane(geometry_msgs::PoseStamped & transformed_pose) {
    transformed_pose.pose.position.z += crane_base_elevation;
    return true;
}

std::vector<geometry_msgs::Pose> generate_waypoints_retrieval(moveit::planning_interface::MoveGroupInterface * main_plan_group, const moveit::core::JointModelGroup* main_joint_model_group, geometry_msgs::Pose desired_target,  std::string ee_name) {
    std::vector<geometry_msgs::Pose> waypoints;

    /* First waypoint is the current robot state */
    moveit::core::RobotStatePtr current_state = main_plan_group->getCurrentState();
    const Eigen::Affine3d& ee_link_pose = current_state->getGlobalLinkTransform("jt3_l");
    Eigen::Vector3d ee_link_cart_pos = ee_link_pose.translation();

    moveit::core::RobotState start_state(*main_plan_group->getCurrentState());
    geometry_msgs::Pose start_pose;
    start_pose.orientation.w = 1.0;
    start_pose.position.x = ee_link_cart_pos[0];
    start_pose.position.y = ee_link_cart_pos[1];
    start_pose.position.z = ee_link_cart_pos[2];
    start_state.setFromIK(main_joint_model_group, start_pose);
    main_plan_group->setStartState(start_state);
    waypoints.push_back(start_pose);

    /* [Provisional] One intermediate waypoint - midpoint */
    geometry_msgs::Pose intermediate_pose = desired_target;
    intermediate_pose.orientation.w = desired_target.orientation.w;
    intermediate_pose.position.x = start_pose.position.x + (desired_target.position.x - start_pose.position.x)/2;
    intermediate_pose.position.y = desired_target.position.y + (desired_target.position.y - start_pose.position.y)/2;
    intermediate_pose.position.z = desired_target.position.z + (desired_target.position.z - start_pose.position.z)/2;
    waypoints.push_back(intermediate_pose);

    /* Last waypoint is the desired position */
    waypoints.push_back(desired_target);

    return waypoints;
}

double plan_crane_serial(moveit::planning_interface::MoveGroupInterface * main_plan_group, const moveit::core::JointModelGroup* main_joint_model_group, std::vector<geometry_msgs::Pose> & waypoints, moveit_msgs::RobotTrajectory & trajectory){
    /* Compute trajectory as waypoints in cartesian mode */
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = main_plan_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    /* Reindex all timestamps */
    ros::Duration time_min = trajectory.joint_trajectory.points.front().time_from_start;
    double time_min_s = time_min.toSec();
    ros::Duration time_max = trajectory.joint_trajectory.points.back().time_from_start;
    double time_max_s = time_max.toSec();
    double time_diff = time_max_s - time_min_s;
    int num_waypoints = trajectory.joint_trajectory.points.size();
    for (int i = 0; i < num_waypoints; i++) {
        trajectory.joint_trajectory.points[i].time_from_start = ros::Duration(time_diff/num_waypoints * i);
    }

    return fraction;
}

bool plan_slew(moveit::planning_interface::MoveGroupInterface * slew_plan_group, const moveit::core::JointModelGroup* slew_joint_model_group, double desired_value, moveit::planning_interface::MoveGroupInterface::Plan & joint_space_plan, std::vector<double> & joint_group_positions) {
    moveit::core::RobotStatePtr current_state = slew_plan_group->getCurrentState();

    current_state = slew_plan_group->getCurrentState();
    current_state->copyJointGroupPositions(slew_joint_model_group, joint_group_positions);
    joint_group_positions[0] = desired_value; // prismatic - 0.5 metres
    slew_plan_group->setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveItErrorCode slew_plan_status = slew_plan_group->plan(joint_space_plan);

    bool success = ( slew_plan_status == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    return success;
}

bool plan_actuator_1(moveit::planning_interface::MoveGroupInterface * act1_plan_group, const moveit::core::JointModelGroup* act1_joint_model_group, double desired_value,  moveit::planning_interface::MoveGroupInterface::Plan & joint_space_plan, std::vector<double> & joint_group_positions) {
    moveit::core::RobotStatePtr current_state = act1_plan_group->getCurrentState();

    current_state = act1_plan_group->getCurrentState();
    current_state->copyJointGroupPositions(act1_joint_model_group, joint_group_positions);
    joint_group_positions[0] = desired_value; // prismatic - 0.5 metres
    act1_plan_group->setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveItErrorCode act1_plan_status = act1_plan_group->plan(joint_space_plan);

    bool success = ( act1_plan_status == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    return success;
}


bool plan_actuator_2(moveit::planning_interface::MoveGroupInterface * act2_plan_group, const moveit::core::JointModelGroup* act2_joint_model_group, double desired_value,  moveit::planning_interface::MoveGroupInterface::Plan & joint_space_plan, std::vector<double> & joint_group_positions) {
    moveit::core::RobotStatePtr current_state = act2_plan_group->getCurrentState();

    current_state->copyJointGroupPositions(act2_joint_model_group, joint_group_positions);
    joint_group_positions[0] = desired_value; // prismatic - 0.5 metres
    act2_plan_group->setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveItErrorCode act2_plan_status = act2_plan_group->plan(joint_space_plan);

    bool success = ( act2_plan_status == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    return success;
}

bool plan_actuator_3(moveit::planning_interface::MoveGroupInterface & act3_plan_group, moveit::core::JointModelGroup* act3_joint_model_group, double desired_value, moveit::planning_interface::MoveGroupInterface::Plan & joint_space_plan, std::vector<double> & joint_group_positions) {

    return false;
}
