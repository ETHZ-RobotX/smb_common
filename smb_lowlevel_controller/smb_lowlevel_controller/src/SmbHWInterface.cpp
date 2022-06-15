/*!
 * @file     SmbHWInterface.cpp
 * @author   Oliver Harley
 * @date     2021-02-18
 */

#include <smb_lowlevel_controller/SmbHWInterface.hpp>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <urdf_parser/urdf_parser.h>
#include <sensor_msgs/BatteryState.h>
#include <algorithm>

namespace smb_lowlevel_controller {

SmbHWInterface::SmbHWInterface() {
    currentPIDs_.emplace_back(&(velCmd_[0]), &(iCmd_[0]), &(wheels_[0].vel));
    currentPIDs_.emplace_back(&(velCmd_[1]), &(iCmd_[1]), &(wheels_[1].vel));
}


SmbMode SmbHWInterface::getDriverMode(){
  auto hyambMode = smb_->getMode();

  SmbMode smbMode;
  switch (hyambMode) {
    case smb_driver::CLOSED_LOOP_SPEED_POSITION:
    case smb_driver::CLOSED_LOOP_POSITION_RELATIVE:
    case smb_driver::CLOSED_LOOP_COUNT_POSITION:
    case smb_driver::CLOSED_LOOP_POSITION_TRACKING:
      throw std::runtime_error("Found system in unimplemented motor control.");
      break;
    case smb_driver::TORQUE:
      smbMode = MODE_TORQUE;
      break;
    case smb_driver::OPEN_LOOP:
      smbMode = MODE_DC_CMD;
      break;
    case smb_driver::CLOSED_LOOP_SPEED:
      smbMode = SmbMode::MODE_VELOCITY;
      break;
  }
  return smbMode;
}

SmbHWInterface::~SmbHWInterface(){
        smb_->stopAcquisition();
}

// sets the driver's mode, requires the lock
void SmbHWInterface::setDriverMode(SmbMode mode){
  smb_driver::HYAMBModes driverMode;
  switch (mode) {
    case MODE_TORQUE:
      driverMode = smb_driver::TORQUE;
      break;
    case  MODE_DC_CMD:
      driverMode = smb_driver::OPEN_LOOP;
      break;
    case SmbMode::MODE_VELOCITY:
      driverMode = smb_driver::CLOSED_LOOP_SPEED;
      break;
    default:
      ROS_ERROR_THROTTLE(1.0, "[SmbHWInterface] Specified SmbMode cannot be handled by the controller. mode=%d", desiredControlMode_);
      break;
  }
  smb_->setMode(driverMode);
  controlMode_ = mode;
}

  bool SmbHWInterface::init(ros::NodeHandle& nh, ros::NodeHandle & private_nh) {
      nh_ = nh;
      private_nh_= private_nh;
      // private_nh_.param<double>("wheel_diameter", wheel_diameter_, 0.3302);
      // private_nh_.param<double>("max_accel", max_accel_, 5.0);
      // private_nh_.param<double>("max_speed", max_speed_, 1.0);

      std::scoped_lock lock{smbDriverMutex_};
      std::string port;
      bool command_smb;
      private_nh_.param<std::string>("port", port, "/dev/ttySMB");
      private_nh_.param<bool>("command_smb", command_smb, true);

      // RC velocity scale
      private_nh_.param<double>("smb/lin_vel_scale", lin_vel_scale, 1.5);
      private_nh_.param<double>("smb/ang_vel_scale", ang_vel_scale, 1.5);
      smb_ = std::make_shared<smb_driver::SmbController>(port, private_nh_, 10, command_smb, lin_vel_scale, ang_vel_scale);
      registerControlInterfaces();

      // TODO(oharley): expose a method to set the desiredControlMode_
      controlMode_ = SmbMode::MODE_DC_CMD;
      desiredControlMode_ = SmbMode::MODE_DC_CMD;
      try {
          // Initializes communication to the smb
          smb_->startAcquisition();
          setDriverMode(desiredControlMode_);
          controlMode_ = getDriverMode();
          ROS_INFO_STREAM("[SmbHWInterface]: Initialized successfully.");
      } catch (...) {
          ROS_ERROR_STREAM("[SmbHWInterface]: Caught an unspecified exception.");
          return false;
      }

      batteryStatePublisher_ = nh_.advertise<sensor_msgs::BatteryState>("base_battery_state", 1, true);
      batteryStatePublishingTimer_ = nh_.createTimer(ros::Duration(1.0), [this](const auto &){
        sensor_msgs::BatteryState state;
        {
        std::lock_guard<std::mutex> lock(batteryVoltageMutex_);
        state.voltage = batteryVoltage_;
        }
        state.header.stamp = ros::Time::now();
        const double maxVoltage = 40.0;
        const double minVoltage = 34.0;
        state.percentage = (state.voltage - minVoltage)/(maxVoltage - minVoltage);
        state.percentage = std::clamp((double)state.percentage, (double)0.0, (double)1.0);
        state.power_supply_status = state.POWER_SUPPLY_STATUS_DISCHARGING;
        state.present = true;
        batteryStatePublisher_.publish(state);
      });
      return true;
  }

  bool SmbHWInterface::registerControlInterfaces() {
      // Initialize interfaces for each joint
      std::vector<std::string> wheelNames = {"LF_WHEEL_JOINT", "RF_WHEEL_JOINT", "LH_WHEEL_JOINT", "RH_WHEEL_JOINT"};
      std::string controller_ns;
      private_nh_.getParam("controller_namespace",controller_ns );
      if (!controller_ns.empty()) controller_ns += '/';

      for (std::size_t i = 0; i < nActuators; ++i) {
          const auto& name = wheelNames[i];
          ROS_DEBUG_STREAM("Loading joint: " << name);

          // Create joint state interface
          hardware_interface::JointStateHandle jointStateHandle(name, &wheels_[i].pos, &wheels_[i].vel, &wheels_[i].eff);
          joint_state_interface_.registerHandle(jointStateHandle);

          hardware_interface::JointHandle
            jointVelHandle(joint_state_interface_.getHandle(name), &velCmd_[i]); //claims Handle
          velocity_joint_interface_.registerHandle(jointVelHandle);
          if (!registerLimits(jointVelHandle))
            ROS_WARN_STREAM("Could not load controller joint limits, are they set?");

          currentPIDs_[i].init(nh_, controller_ns + name + "_dc_controller");

      }  // end for each joint

      for (size_t i = 0; i < 2; i++){
          // Create mock joint state interface
          hardware_interface::JointStateHandle jointStateHandle(wheelNames[i+2], &wheels_[i].pos, &wheels_[i].vel, &wheels_[i].eff);
          joint_state_interface_.registerHandle(jointStateHandle);
      }

      // Only MODE_DC_CMD==smb_driver::OPEN_LOOP is actually implemented in the driver
      // registerInterface(&current_joint_interface_);
      registerInterface(&joint_state_interface_);
      registerInterface(&velocity_joint_interface_);
      // registerInterface(&torque_joint_interface);


      return true;
  }


  bool SmbHWInterface::registerLimits(hardware_interface::JointHandle& jHandle) {

    using namespace joint_limits_interface;
    using joint_limits_interface::VelocityJointSoftLimitsInterface;

    const auto& name = jHandle.getName();
    // Parse robot description
    std::string urdf_string;
    if(!nh_.getParam("robot_description", urdf_string)){
      ROS_ERROR("Could not parse the parameter robot_description");
      return false;
    }
    auto urdf = urdf::parseURDF(urdf_string);
    auto urdf_joint = urdf->getJoint(name);
    JointLimits limits;
    limits.max_velocity = DEFAULT_MAX_VEL_;
    limits.max_effort = DEFAULT_MAX_EFF_;
    const bool urdf_limits_ok = getJointLimits(urdf_joint, limits); // getJointLimits(urdf_joint, limits);
    SoftJointLimits softLimits;
    const bool urdf_soft_limits_ok = getSoftJointLimits(urdf_joint, softLimits);// getSoftJointLimits(urdf_joint, soft_limits);

    //! Parameter server limits override
    const bool rosparam_limits_ok = getJointLimits(name, nh_, limits);
    const bool rosparam_soft_limits_ok = getSoftJointLimits(name, nh_, softLimits);
    ROS_INFO("Read URDF limits? %s.", urdf_limits_ok ? "YES" : "NO");
    ROS_INFO("Read URDF soft limits? %s.", urdf_soft_limits_ok ? "YES" : "NO");
    ROS_INFO("Read Parameter Server limits? %s.", rosparam_limits_ok ? "YES" : "NO");
    ROS_INFO("Read Parameter Server limits? %s.", rosparam_soft_limits_ok ? "YES" : "NO");

bool reglimits = ((urdf_limits_ok && urdf_soft_limits_ok) || (rosparam_limits_ok && rosparam_soft_limits_ok));
    if (reglimits) {
	    joint_limits_interface::VelocityJointSoftLimitsHandle velocitySoftLimitsHandle(jHandle, limits, softLimits);
	    velocitySoftLimitsInterface_.registerHandle(velocitySoftLimitsHandle);
	    ROS_INFO("Registered limits.");
    }
    else{ ROS_INFO("Did NOT register limits.");}
    return  reglimits;
  }


  void SmbHWInterface::read(const ros::Time &time, const ros::Duration& elapsedTime) {
    std::scoped_lock lock{smbDriverMutex_};
    if (!smb_->getWheelSpeeds(wheels_[0].vel, wheels_[1].vel, controllerTimeoutUs_)) {
      ROS_WARN_STREAM("[SmbHWInterface] Failed to get wheel speeds.");
    } else {
      wheels_[0].pos += wheels_[0].vel * elapsedTime.toSec();
      wheels_[1].pos += wheels_[1].vel * elapsedTime.toSec();
      currentPIDs_[0].update(time, elapsedTime);
      currentPIDs_[1].update(time, elapsedTime);
    }
    double batteryVoltage = 0;
    if (smb_->getBatteryVoltage(batteryVoltage, 1000)){
      std::lock_guard<std::mutex> lock(batteryVoltageMutex_);
      batteryVoltage_ = batteryVoltage;
    }
  }

  void SmbHWInterface::write(const ros::Time &time, const ros::Duration& elapsedTime) {
    std::scoped_lock lock{smbDriverMutex_};
    //! At the moment there is no means to change the underlying driver controll method
    bool noWrite = false;
    if (desiredControlMode_ == SmbMode::FREEZE){
      controlMode_ = SmbMode::FREEZE;
      smb_->setFreeze();
    }
    if (controlMode_ == SmbMode::FREEZE){
      if (desiredControlMode_ != SmbMode::THAW) {
        smb_->setFreeze();
      } else //if (desiredControlMode_ == SmbMode::THAW) {
      { controlMode_ = SmbMode::THAW; }
      noWrite = true;
    }
    if (noWrite){
      return;
    }

    // Only change mode if necessary
    if (desiredControlMode_ != controlMode_) { setDriverMode(desiredControlMode_); }

    // Write the actual values
    // ROS_DEBUG("[SmbHWInterface] %f %f %f %f %f %f.", velCmd_[0], iCmd_[0], torqCmd_[0], velCmd_[1], iCmd_[1], torqCmd_[1]);
    for (auto i=0; i < nActuators; ++i) {
      switch(controlMode_){
        case SmbMode::MODE_DC_CMD:
          velocitySoftLimitsInterface_.enforceLimits(elapsedTime);
          // ROS_DEBUG("[SmbHWInterface] %f ", iCmd_[i]);
          currentPIDs_[i].update(time, elapsedTime);
          //! Note that we explicitly switch the order here to make the turning directions correct
          smb_->setMotorPower(iCmd_[i], 2-i);
          break;
        default:
          ROS_WARN("[SmbHWInterface] Specified SmbMode's values cannot be written to the driver. mode=%d", controlMode_);
          break;
      }
    }
    }


    WheelVelocityControl::WheelVelocityControl(double* const setPoint, double* const command, const double * const processValue) :
     setPoint_(setPoint),  command_(command),  processValue_(processValue), loop_count_(0)
    {}

    WheelVelocityControl::~WheelVelocityControl() {delete controller_state_publisher_;};

    bool WheelVelocityControl::init(ros::NodeHandle& nh, const std::string& nhprefix, bool publishControllerState) {
      if (publishControllerState) {
        controller_state_publisher_ = new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(nh, "iPID_state", 1);
      }
      pid_controller_.initParam(nhprefix);
      return true;
    }


    void WheelVelocityControl::starting(const ros::Time& time)
    {
      *command_ = 0.0;
      pid_controller_.reset();
    }

    void WheelVelocityControl::update(const ros::Time& time, const ros::Duration& period)
    {
      double error = *setPoint_ - *processValue_;

      // Set the PID error and compute the PID command with nonuniform time
      // step size. The derivative error is computed from the change in the error
      // and the timestep dt.
      *command_ = pid_controller_.computeCommand(error, period);

      if(loop_count_ % 20 == 0)
      {
        if(controller_state_publisher_ && controller_state_publisher_->trylock())
        {
          controller_state_publisher_->msg_.header.stamp = time;
          controller_state_publisher_->msg_.set_point = *setPoint_;
          controller_state_publisher_->msg_.process_value = *processValue_;
          controller_state_publisher_->msg_.error = error;
          controller_state_publisher_->msg_.time_step = period.toSec();
          controller_state_publisher_->msg_.command = *command_;

          double dummy;
          bool antiwindup;
          pid_controller_.getGains(controller_state_publisher_->msg_.p,
              controller_state_publisher_->msg_.i,
              controller_state_publisher_->msg_.d,
              controller_state_publisher_->msg_.i_clamp,
              dummy,
              antiwindup);
          controller_state_publisher_->msg_.antiwindup = static_cast<char>(antiwindup);
          controller_state_publisher_->unlockAndPublish();
        }
      }
      loop_count_++;
    }

} // namespace smb_lowlevel_controller
