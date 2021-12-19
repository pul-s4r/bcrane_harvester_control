#ifndef JOINT_TRAJECTORY_ADAPTIVE_H
#define JOINT_TRAJECTORY_ADAPTIVE_H

// #include <vector>
// #include <stdexcept>
#include <map>
#include <cmath>

#include <controller_interface/controller.h>
#include <trajectory_interface/quintic_spline_segment.h>
#include <joint_trajectory_controller/joint_trajectory_segment.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>

#include "ad_pid_hardware_interface.h"

namespace crane_control_adaptive {


template <class SegmentImpl, class HardwareInterface>
class JointTrajectoryAdaptive : public joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface> {
public:
    // /* [Not real-time safe] Initialize */
    bool init(HardwareInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
        joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface>::init(hw, root_nh, controller_nh);

        for (unsigned int i = 0; i < this->joints_.size(); ++i)
        {
            ros::NodeHandle joint_nh(controller_nh, std::string("gains/") + this->joints_[i].getName());
            RegularGains reg_gain;
            joint_nh.param("p", reg_gain.p_, (double) 100.0);
            joint_nh.param("i", reg_gain.i_, (double) 100.0);
            joint_nh.param("d", reg_gain.d_, (double) 100.0);
            joint_nh.param("i_clamp", reg_gain.i_max_, (double) 100.0);
            reg_gain.i_min_ = -reg_gain.i_max_;
            reg_gains.insert(std::pair<std::string, RegularGains>(this->joints_[i].getName(), reg_gain));

            ros::NodeHandle joint_nh_e(controller_nh, std::string("exp_tune_params/") + this->joints_[i].getName());
            ExpTunerGains et_gain;
            joint_nh_e.param("p_mult", et_gain.p_mult_, (double) 1.25);
            joint_nh_e.param("i_mult", et_gain.i_mult_, (double) 1.25);
            joint_nh_e.param("d_mult", et_gain.d_mult_, (double) 1.25);
            joint_nh_e.param("p_chf", et_gain.p_chf_, (double) 30.0);
            joint_nh_e.param("i_chf", et_gain.i_chf_, (double) 30.0);
            joint_nh_e.param("d_chf", et_gain.d_chf_, (double) 30.0);
            et_gains.insert(std::pair<std::string, ExpTunerGains>(this->joints_[i].getName(), et_gain));
        }

        try {
            ExpTunerGains sample = et_gains["joint1"];
            RegularGains sample2 = reg_gains["joint1"];
            ROS_INFO_STREAM_NAMED(this->name_, "Gains: " << sample2.p_ << ", " << sample2.i_ << ", " << sample2.d_);
            ROS_INFO_STREAM_NAMED(this->name_, "Gain multipliers: " << sample.p_mult_ << ", " << sample.i_mult_ << ", " << sample.d_mult_);
            ROS_INFO_STREAM_NAMED(this->name_, "Change factors: " << sample.p_chf_ << ", " << sample.i_chf_ << ", " << sample.d_chf_);
        } catch (const std::exception& ex) {
            ROS_ERROR_STREAM_NAMED(this->name_, "Could not find specified joint or other error: " << ex.what() );
        }

        return true;
    }
    // /* Holds the current position */
    // void starting(const ros::Time& time) { };
    // /* Cancels active action goal if it exists*/
    // void stopping(const ros::Time& time) { };
    // /* Updates current trajectory, time data, current state/error/tolerances
    // and issues commands via interface adapter */
    void update(const ros::Time& time, const ros::Duration& period) {
        joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface>::update(time, period);
        if (this->rt_active_goal_) {
            setExpGains();
        }
    }

    double calculateGain(double gainBase, double mult, double chf, double err) {
        return gainBase + gainBase * mult * exp(- chf * abs(err));
    }

    bool setExpGainByJoint(int i) {
        // get errors: state_error_.position, state_error_.velocity
        // assign: this->hw_iface_adapter_.pids[0]
        RegularGains jointG = reg_gains[this->joints_[i].getName()];
        double p_error = this->state_joint_error_.position[i]; // state_error_/state_joint_error_?
        ExpTunerGains jointGE = et_gains[this->joints_[i].getName()];
        double p_gain_d = calculateGain(jointG.p_, jointGE.p_mult_, jointGE.p_chf_, p_error);
        double i_gain_d = calculateGain(jointG.i_, jointGE.p_mult_, jointGE.p_chf_, p_error);
        double d_gain_d = calculateGain(jointG.d_, jointGE.p_mult_, jointGE.p_chf_, p_error);
        ROS_INFO_STREAM_NAMED(this->name_, "Joint: " << this->joints_[i].getName() << ", Error: " << p_error << ", Gains: " << p_gain_d << ", " << i_gain_d << ", " << d_gain_d);
        // setPidGains(i, p_gain_d, i_gain_d, d_gain_d, jointG.i_max_, jointG.i_min_);
        return true;
    }

    bool setExpGains() {
      for (unsigned int i = 0; i < this->joints_.size(); ++i)
      {
        setExpGainByJoint(i);
      }
      return true;
    }

protected:
    typedef joint_trajectory_controller::JointTrajectorySegment<SegmentImpl> Segment;
    typedef PositionHIA<HardwareInterface, typename Segment::State> HwIfaceAdapter;

    struct ExpTunerGains {
        double p_mult_;
        double i_mult_;
        double d_mult_;
        double p_chf_;
        double i_chf_;
        double d_chf_;
    };

    struct RegularGains {
        double p_;
        double i_;
        double d_;
        double i_max_;
        double i_min_;
    };

    std::map<std::string, ExpTunerGains> et_gains;
    std::map<std::string, RegularGains> reg_gains;
private:
  // control_toolbox::Pid pid_controller_;
};

// [TODO] calculate gains method
// [TODO] setGains method
/*
void JointTrajectoryAdaptive::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup)
{
pid_controller_.setGains(p,i,d,i_max,i_min,antiwindup);
}
*/
/* void JointPositionController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup)
{
  pid_controller_.setGains(p,i,d,i_max,i_min,antiwindup);
} */
}

// Implementation here, separate namespace.


#endif
