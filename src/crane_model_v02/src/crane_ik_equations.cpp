#include <crane_ik_equations/crane_ik_equations.h>

namespace crane_ik_equations {

CraneIKEquations::CraneIKEquations (std::string param_file) {
    try {
        std::ifstream f(param_file.c_str());

        if (!param_file.empty() && f.good()) {
            CraneIKEquations::loadParamsFromFile(param_file);
        } else {
            std::cerr << "Filename.empty: " << param_file.empty() << std::endl;
            std::cerr << "Access: " << f.good() << std::endl;
            std::cerr << param_file << " not found, loading defaults" << std::endl;
            CraneIKEquations::loadParamsDefault();
        }
        CraneIKEquations::loadConstantsCommon();

        std::string node_path = "/CraneExtTools/";
        ros::NodeHandle nh(node_path);
        crane_ikService = nh.advertiseService("crane_ik_service", &crane_ik_equations::CraneIKEquations::IKCallback, this);

    } catch (const std::exception& e) {
        std::cerr << param_file << " - loading failed due to: " << e.what() << std::endl;
        throw e;
    }
}

CraneIKEquations::~CraneIKEquations () {

}

bool CraneIKEquations::loadParamsFromFile (std::string param_file) {
    YAML::Node config = YAML::LoadFile(param_file);
    l_0 = config["l_0"].as<double>();
    l_1 = config["l_1"].as<double>();
    l_2 = config["l_2"].as<double>();
    l_b1x = config["l_b1x"].as<double>();
    l_b1z = config["l_b1z"].as<double>();
    l_b4x = config["l_b4x"].as<double>();
    l_b4y = config["l_b4y"].as<double>();
    l_c1x = config["l_c1x"].as<double>();
    l_c1y = config["l_c1y"].as<double>();
    l_c3x = config["l_c3x"].as<double>();
    l_c3y = config["l_c3y"].as<double>();
    l_co = config["l_co"].as<double>();
    l_b2 = config["l_b2"].as<double>();
    l_c2 = config["l_c2"].as<double>();

    return true;
}

bool CraneIKEquations::loadParamsDefault () {
    l_0 = 2.1;
    l_1 = 2.3;
    l_2 = 2.1;
    l_b1x = 0.3;
    l_b1z = 0.2;
    l_b4x = 0.5;
    l_b4y = 0.15;
    l_c1x = 0.5;
    l_c1y = 0.5;
    l_c3x = 0.5;
    l_c3y = 0.2;
    l_co = 1.1;
    l_b2 = 1.0;
    l_c2 = 0.6;

    return true;
}

bool CraneIKEquations::loadConstantsCommon () {
    a_1 = sqrt(pow(l_b4x,2)+pow(l_b4y,2));
    b_1 = sqrt(pow(l_b1x,2)+pow(l_0,2));
    a_2 = sqrt(pow(l_c1x,2)+pow(l_c1y,2))+0.05;
    b_2 = sqrt(pow(l_c3x,2)+pow(l_c3y,2))+0.05;

    angle_a1 = atan((l_b4x-l_b1x)/(l_0));
    angle_a2 = atan(l_c3x/l_co);

    // see calc_g_1 - do not subtract l_b1z from l_0

    return true;
}

double CraneIKEquations::calc_gamma (double x_d, double y_d, double z_d, double theta_0) {
    return pow(l_2, 2) - pow(l_1, 2) - pow(l_0 - z_d,2)
    - pow(x_d*cos(theta_0)+y_d*sin(theta_0),2);
}

double CraneIKEquations::calc_theta_1_p (double x_d, double y_d, double z_d,
    double theta_0) {
    return atan2(x_d*cos(theta_0)+y_d*sin(theta_0), l_0-z_d);
}

double CraneIKEquations::calc_rho (double x_d, double y_d, double z_d, double theta_0) {
    return 2*l_1*sqrt(pow(x_d*cos(theta_0)+y_d*sin(theta_0),2) + pow(l_0-z_d,2));
}

double CraneIKEquations::calc_cos_theta_1 (double gamma, double rho) {
    return atan2(gamma/rho, -sqrt(1-pow(gamma/rho,2)));  // sin / cos
}

double CraneIKEquations::calc_theta_1 (double x_d, double y_d, double z_d, double theta_0) {
    double gamma = CraneIKEquations::calc_gamma(x_d, y_d, z_d, theta_0);
    double rho = CraneIKEquations::calc_rho(x_d, y_d, z_d, theta_0);
    double theta_1_p = CraneIKEquations::calc_theta_1_p(x_d, y_d, z_d, theta_0);
    std::cout << "Gamma: " << gamma << ", rho: " << rho << ", theta_1_p (deg): " << 180/M_PI * theta_1_p << std::endl;
    std::cout << "Dec: " << 180/M_PI * CraneIKEquations::calc_cos_theta_1(gamma, rho) << std::endl;

    double theta_1 = M_PI + CraneIKEquations::calc_cos_theta_1(gamma, rho) - theta_1_p;

    return -theta_1;
}

double CraneIKEquations::calc_theta_2 (double x_d, double y_d, double z_d, double theta_0, double theta_1) {
    double t2_atan2 = atan2(l_0-l_1*sin(theta_1)-z_d, x_d*cos(theta_0)+y_d*sin(theta_0)-l_1*cos(theta_1)) - theta_1;
    double t2_atan = atan((l_0-l_1*sin(theta_1)-z_d)/(x_d*cos(theta_0)+y_d*sin(theta_0)-l_1*cos(theta_1))) - theta_1;
    std::cout << "Theta_2 (with atan2): " << t2_atan2 << std::endl;
    std::cout << "Theta_2 (with only atan): " << t2_atan << std::endl;
    return t2_atan2;
}

double CraneIKEquations::calc_g_1 (double theta_1) {
    return M_PI/2 - theta_1 - atan(l_b1x/(l_0)) - atan(l_b4y/l_b4x);
    // do not subtract l_b1z as l_0 refers to link height not height from ground.
}

double CraneIKEquations::calc_g_2 (double theta_2) {
    return M_PI - theta_2 - atan(l_c1y/l_c1x) - atan(l_c3y/l_c3x);
}

double CraneIKEquations::calc_theta_0 (double x_d, double y_d) {
    return atan2(y_d, x_d);
}

double CraneIKEquations::calc_d_1 (double theta_1) {
    double g_1 = calc_g_1(theta_1);
    return sqrt(pow(a_1, 2) + pow(b_1, 2) - 2*a_1*b_1*cos(g_1)) - l_b2;
}
double CraneIKEquations::calc_d_2 (double theta_2) {
    double g_2 = calc_g_2(theta_2);
    std::cout << "gamma_2 " << g_2 << std::endl;
    return sqrt(pow(a_2,2) + pow(b_2,2) - 2*a_2*b_2*cos(M_PI/2 + g_2)) - l_c2;
}

bool CraneIKEquations::IKCallback
    (crane_model_v02::SolvePositionInverse::Request &req,
    crane_model_v02::SolvePositionInverse::Response &res) {
    ros::Rate loop_rate(100);
    res.joints.resize(req.pose_stamped.size());
    res.result_type.resize(req.pose_stamped.size());
    // Calculate joint states for each target position received
    for (size_t req_i = 0; req_i < req.pose_stamped.size(); req_i++) {
        // Get target position cartesian coordinates
        sensor_msgs::JointState joint_positions;
        geometry_msgs::Point eepose = req.pose_stamped[req_i].pose.position;
        double theta_0 = calc_theta_0(eepose.x, eepose.y);
        double theta_1 = calc_theta_1(eepose.x, eepose.y, eepose.z, theta_0);
        double theta_2 = calc_theta_2(eepose.x, eepose.y, eepose.z, theta_0, theta_1);
        // Populate fields
        // joint_positions.name =
        //     crane_model_v02::SolvePositionInverseRequest::ACTUATOR_NAME;
        std::cout << "Angles (deg): " << theta_0 * 180/M_PI << " " << theta_1 * 180/M_PI << " " << theta_2 * 180/M_PI << std::endl;
        joint_positions.position.push_back(theta_0);
        joint_positions.position.push_back(CraneIKEquations::calc_d_1(theta_1));
        joint_positions.position.push_back(CraneIKEquations::calc_d_2(theta_2));
        for (size_t j = 0; j < 3; j++) {
            joint_positions.velocity.push_back(0.0);
            joint_positions.effort.push_back(0.0);
        }
        std::cout << "Final position: (" << joint_positions.position[0] << ", " << joint_positions.position[1] << ", " << joint_positions.position[2] << ")" << std::endl;
        res.joints[req_i] = joint_positions;
        res.result_type[req_i] = crane_model_v02::SolvePositionInverseResponse::RESULT_VALID;
    }
    loop_rate.sleep();
    return true;
}

}

crane_ik_equations::CraneIKEquations::cranekin_ptr c_pNode;

/* Main entry point */
int main(int argc, char* argv[]) {
    std::string param_file = argc > 1 ? argv[1] : "";
    if (argc < 1) {
        std::cerr << "Usage: " << argv[0] << " <filename_of_config>.yaml" << std::endl;
        return 1;
    }

    ros::init(argc, argv, "crane_ik_node");

    c_pNode = crane_ik_equations::CraneIKEquations::create(param_file);

    if (c_pNode) {
        c_pNode->run();
    }

    return 0;
}
