#ifndef CRANE_IK_EQUATIONS_H_
#define CRANE_IK_EQUATIONS_H_

#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <unistd.h>

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

#include <crane_model_v02/SolvePositionInverse.h>

namespace crane_ik_equations
{
class CraneIKEquations {
public:
    typedef boost::shared_ptr<CraneIKEquations> cranekin_ptr;

    static cranekin_ptr create(std::string param_file) {
        cranekin_ptr ck_ptr = cranekin_ptr(new CraneIKEquations(param_file));
        return ck_ptr;
    }

    CraneIKEquations(std::string param_file);
    ~CraneIKEquations();
    bool loadParamsFromFile(std::string param_file);
    bool loadParamsDefault();

    double calc_gamma (double x_d, double y_d, double z_d, double theta_0);
    double calc_theta_1_p (double x_d, double y_d, double z_d,
        double theta_0);
    double calc_rho(double x_d, double y_d, double z_d, double theta_0);

    double calc_cos_theta_1 (double gamma, double rho);
    double calc_theta_1 (double x_d, double y_d, double z_d, double theta_0);
    double calc_theta_2 (double x_d, double y_d, double z_d, double theta_0, double theta_1);
    double calc_g_1 (double theta_1);
    double calc_g_2 (double theta_2);

    double calc_theta_0 (double x_d, double y_d);
    double calc_d_1 (double theta_1);
    double calc_d_2 (double theta_2);

    bool IKCallback (crane_model_v02::SolvePositionInverse::Request &req,
        crane_model_v02::SolvePositionInverse::Response &res);

    void run() {
        // Run loop
        ros::spin();
        // Shut down running processes then attempt proper shutdown
        closeServices();
        ros::shutdown();
    }

    void closeServices() {
        crane_ikService.shutdown();
    }

private:
    bool loadConstantsCommon ();

    double l_0;
    double l_1;
    double l_2;
    double l_b1x;
    double l_b1z;
    double l_b4x;
    double l_b4y;
    double l_c1x;
    double l_c1y;
    double l_c3x;
    double l_c3y;
    double l_co;
    double l_b2;
    double l_c2;

    double a_1;
    double b_1;
    double a_2;
    double b_2;

    double angle_a1;
    double angle_a2;

    ros::ServiceServer crane_ikService;
    // ros::Subscriber sub_jointStates, sub_robotStates;
    // ros::Publisher pub_endPointState;
};
};

#endif /* CRANE_IK_EQUATIONS_H_ */
