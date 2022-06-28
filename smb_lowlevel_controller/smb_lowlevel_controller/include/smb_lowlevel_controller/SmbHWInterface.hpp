/*!
 * @file     SmbHWInterface.hpp
 * @author   Oliver Harley
 * @date     2021-03-1
 */

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <ros/ros.h>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
// #include <transmission_interface/RobotTransmissions.h>
// #include <transmission_interface/transmission_interface_loader.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64.h>

#include <smb_lowlevel_controller/SmbModes.hpp>
#include <smb_driver/SmbController.h>

namespace smb_lowlevel_controller {

  class WheelVelocityControl {
public:

  WheelVelocityControl(double* const setPoint, double* const command, const double * const processValue);
  ~WheelVelocityControl();

  bool init(ros::NodeHandle& n, const std::string& nhprefix, bool publishControllerState=false);

  void starting(const ros::Time& time);

  void update(const ros::Time& time, const ros::Duration& period);

  control_toolbox::Pid pid_controller_;
  double* const setPoint_;
  double* const command_;
  const double * const processValue_;
private:
  realtime_tools::RealtimePublisher<control_msgs::JointControllerState>* controller_state_publisher_;
  int loop_count_;
  ros::Time lastUpdate_;

 };


  class SmbHWInterface : public hardware_interface::RobotHW
  {
  public:
    SmbHWInterface();
    virtual ~SmbHWInterface();

    bool init(ros::NodeHandle& /*root_nh*/, ros::NodeHandle & /*robot_hw_nh*/) override;
    bool registerLimits(hardware_interface::JointHandle&);
    bool registerControlInterfaces();
    void enforceLimits(const ros::Duration& period);

    void read(const ros::Time&, const ros::Duration& elapsed_time) override;
    void write(const ros::Time&, const ros::Duration& elapsed_time) override;



  private:
    static constexpr std::size_t nActuators = 2;


    // Hardware Driver
    const int controllerTimeoutUs_ = 500; // [us]
    std::mutex smbDriverMutex_;
    std::shared_ptr<smb_driver::SmbController> smb_;

    SmbMode controlMode_, desiredControlMode_ = SmbMode::FREEZE;

    SmbMode getDriverMode();
    void setDriverMode(SmbMode mode);

    // ROS
    ros::NodeHandle nh_, private_nh_;

    ros::Publisher batteryStatePublisher_;
    ros::Timer batteryStatePublishingTimer_;
    std::mutex batteryVoltageMutex_;
    double batteryVoltage_ = -1;
    
    // Hardware interfaces
    // hardware_interface::EffortJointInterface current_joint_interface_;
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    // hardware_interface::EffortJointInterface torque_joint_interface;

    // Limits
    bool limitsSet;

    //! Only effort soft limts are implemented here
    // Joint limits interfaces - Soft limits
    // joint_limits_interface::EffortJointSoftLimitsInterface current_jnt_soft_limits_;
    joint_limits_interface::VelocityJointSoftLimitsInterface velocitySoftLimitsInterface_;
    const double DEFAULT_MAX_VEL_ = 10;
    const double DEFAULT_MIN_VEL_ = -10;
    const double DEFAULT_MAX_EFF_ = 1000;

    // Transmissions
    // RobotTransmissions robot_transmissions_;
    // std::unique_ptr<TransmissionInterfaceLoader> transmission_loader_;
    // ROS Parameters
    double wheel_diameter_, max_accel_, max_speed_;
    double lin_vel_scale, ang_vel_scale;

    // State and commands
    struct Joint {
      double pos, vel, eff;
      Joint() :  pos(0), vel(0), eff(0) {}
    };
    std::array<Joint, nActuators> wheels_;
    std::array<double, nActuators> velCmd_;
    std::array<double, nActuators> iCmd_;
    std::array<double, nActuators> torqCmd_;
    std::vector<WheelVelocityControl> currentPIDs_;
  };

} /* smb_lowlevel_controller */
