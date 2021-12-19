#include <pluginlib/class_list_macros.hpp>

#include <crane_control_adaptive/joint_trajectory_adaptive.h>

namespace position_controllers {
  typedef crane_control_adaptive::JointTrajectoryAdaptive<trajectory_interface::QuinticSplineSegment<double>, hardware_interface::PositionJointInterface>
    JointTrajectoryAdaptive;
}

// Generic: controller_ns
PLUGINLIB_EXPORT_CLASS(position_controllers::JointTrajectoryAdaptive, controller_interface::ControllerBase)
