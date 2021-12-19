#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <crane_model_v02/SolvePositionInverse.h>

int main (int argc, char **argv) {
    ros::init(argc, argv, "crane_test_client");
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " x y z" << std::endl;
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<crane_model_v02::SolvePositionInverse>("/CraneExtTools/crane_ik_service");

    crane_model_v02::SolvePositionInverse srv;
    srv.request.pose_stamped.resize(1);

    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = "/base_link";
    target_pose.header.stamp = ros::Time::now();

    // target_pose.pose.position.x =  2.300;
    // target_pose.pose.position.y =  0.000;
    // target_pose.pose.position.z =  0.175;
    char* end;
    target_pose.pose.position.x =  strtod(argv[1], &end);
    target_pose.pose.position.y =  strtod(argv[2], &end);
    target_pose.pose.position.z =  strtod(argv[3], &end);

    srv.request.pose_stamped[0] = target_pose;

    if (client.call(srv)) {
        ROS_INFO("Joint positions: theta_0/%f d1/%f d2/%f", srv.response.joints[0].position[0], srv.response.joints[0].position[1], srv.response.joints[0].position[2]);
    } else {
        ROS_ERROR("Failed to call service crane_ik_service");
        return 1;
    }

    return 0;
}
