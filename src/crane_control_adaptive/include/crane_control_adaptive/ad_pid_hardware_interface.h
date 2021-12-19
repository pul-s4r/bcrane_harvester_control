#ifndef AD_PID_HARDWARE_INTERFACE
#define AD_PID_HARDWARE_INTERFACE

// #include <cassert>
// #include <string>
// #include <vector>
// #include <memory>

// #include <ros/node_handle.h>
// #include <ros/time.h>

// #include <control_toolbox/pid.h>
// #include <hardware_interface/joint_command_interface.h>
// #include <hardware_interface/posvel_command_interface.h>
// #include <hardware_interface/posvelacc_command_interface.h>

#include <joint_trajectory_controller/hardware_interface_adapter.h>

/* [TODO] Replacement PID implementation */
/* Adapted from https://github.com/ros-controls/ros_controllers/blob/noetic-devel/joint_trajectory_controller/include/joint_trajectory_controller/hardware_interface_adapter.h*/

/* Replacement class template for generic HardwareInterfaceAdapter */
template <class HardwareInterface, class State>
class PositionHIA
{
public:
  bool init(std::vector<typename HardwareInterface::ResourceHandleType>& /*joint_handles*/, ros::NodeHandle& /*controller_nh*/)
  {
    return false;
  }

  void starting(const ros::Time& /*time*/) {}
  void stopping(const ros::Time& /*time*/) {}

  void updateCommand(const ros::Time&     /*time*/,
                     const ros::Duration& /*period*/,
                     const State&         /*desired_state*/,
                     const State&         /*state_error*/) {}
};

/* [Trial] replacement class for ClosedLoopHardwareInterfaceAdapter */
template <class State>
class ClosedLoopPositionHIA
{
public:
  ClosedLoopPositionHIA() : joint_handles_ptr_(nullptr) {}

  bool init(std::vector<hardware_interface::JointHandle>& joint_handles, ros::NodeHandle& controller_nh)
  {
    // Store pointer to joint handles
    joint_handles_ptr_ = &joint_handles;

    // Initialize PIDs
    pids_.resize(joint_handles.size());
    for (unsigned int i = 0; i < pids_.size(); ++i)
    {
      // Node handle to PID gains
      ros::NodeHandle joint_nh(controller_nh, std::string("gains/") + joint_handles[i].getName());

      // Init PID gains from ROS parameter server
      pids_[i].reset(new control_toolbox::Pid());
      if (!pids_[i]->init(joint_nh))
      {
        ROS_WARN_STREAM("Failed to initialize PID gains from ROS parameter server.");
        return false;
      }
    }

    // Load velocity feedforward gains from parameter server
    velocity_ff_.resize(joint_handles.size());
    for (unsigned int i = 0; i < velocity_ff_.size(); ++i)
    {
      controller_nh.param(std::string("velocity_ff/") + joint_handles[i].getName(), velocity_ff_[i], 0.0);
    }

    return true;
  }

  void starting(const ros::Time& /*time*/)
  {
    if (!joint_handles_ptr_) {return;}

    // Reset PIDs, zero commands
    for (unsigned int i = 0; i < pids_.size(); ++i)
    {
      pids_[i]->reset();
      (*joint_handles_ptr_)[i].setCommand(0.0);
    }
  }

  void stopping(const ros::Time& /*time*/) {}

  void updateCommand(const ros::Time&     /*time*/,
                     const ros::Duration& period,
                     const State&         desired_state,
                     const State&         state_error)
  {
    const unsigned int n_joints = joint_handles_ptr_->size();

    // Preconditions
    if (!joint_handles_ptr_)
      return;
    assert(n_joints == state_error.position.size());
    assert(n_joints == state_error.velocity.size());

    // Update PIDs
    for (unsigned int i = 0; i < n_joints; ++i)
    {
      const double command = (desired_state.velocity[i] * velocity_ff_[i]) + pids_[i] ->computeCommand(state_error.position[i], state_error.velocity[i], period);
      (*joint_handles_ptr_)[i].setCommand(command);
    }
  }

  void getPidGains(int joint_idx, double &p, double &i, double &d, double &i_max, double &i_min) {
    bool antiwindup;
    pids_[joint_idx]->getGains(p, i, d, i_max, i_min, antiwindup);
  }

  void setPidGains(int joint_idx, double p, double i, double d, double i_max, double i_min) {
    pids_[joint_idx]->setGains(p, i, d, i_max, i_min);
  }

private:
  typedef std::shared_ptr<control_toolbox::Pid> PidPtr;
  std::vector<PidPtr> pids_;

  std::vector<double> velocity_ff_;

  std::vector<hardware_interface::JointHandle>* joint_handles_ptr_;
};

// Starting point
template <class State>
class PositionHIA<hardware_interface::PositionJointInterface, State> : public ClosedLoopPositionHIA<State> {};

#endif
