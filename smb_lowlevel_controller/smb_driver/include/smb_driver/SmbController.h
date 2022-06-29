/*
 * Controller.h
 *
 *  Created on: May 26, 2017
 *      Author: sasutosh
 */

#ifndef INCLUDE_CONTROLLER_H_
#define INCLUDE_CONTROLLER_H_

#include <ros/ros.h>
#include <cstring>
#include <iostream>
#include <map>
#include <vector>
#include <memory>
#include <unistd.h>
#include <math.h> //For M_PI
#include <boost/circular_buffer.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>

#include <smb_driver/auxiliaries/interProcessCommunication.h>

#include <smb_driver/RoboteqDevice.h>
#include <smb_driver/ErrorCodes.h>
#include <smb_driver/Constants.h>

//#include <smb_common/SmbModes.h>

#include<chrono>

// Define values for RC twist publisher
#define RC_UPPER_LIMIT 10
#define RC_LOWER_LIMIT -10
#define RC_MAX_COUNT 8

//Motor one is the LEFT one
//Both wheel speeds and torques are handled such that positive values are in the forward direction (away from the power switch)
//The controller's internal watchdog will set the commands to zero if no serial command is received after 500 msec

namespace smb_driver {

class SmbController {

public:

    enum CommandType {
        Velocity = 0,
        Torque,
        Motor_Power,
        Mode,
        INVALID
    };

    struct CommandPacket {
        CommandType cmdType;
        double value;
        int motor;
    };

    //When sendCommands is false, commands aren't sent to the motor controller, only base measurements are received
    SmbController(std::string port, ros::NodeHandle &nh, size_t vecSize = 10, bool sendCommands = true, double lin_vel_scale = 1, double ang_vel_scale = 1);

    ~SmbController();

    bool connect();

    bool getWheelSpeeds(double &leftSpeed, double &rightSpeed, int timeoutUs);

    bool getBatteryVoltage(double &batteryVoltage, int timeoutUs);

    double getVelocity(int motor);        // returns the motor velocity in RPM
    void setVelocity(double vel);        // sets the velocity for closed loop speed mode for both motors
    void setVelocity(double vel, int motor);  // sets the velocity for closed loop speed mode for the motor
    void setTorque(double tau);          // sets the velocity for closed loop torque mode for both motors
    void setTorque(double tau, int motor);    // sets the velocity for closed loop torque mode for thr motor
    void setMotorPower(double pow);        // sets the motor power for open loop mode for both motors
    void setMotorPower(double pow, int motor);  // sets the motor power for open loop mode for the motor
    void setFreeze(); //set the commands to zero, regardless of the current command mode

    HYAMBModes getMode();            // returns the current mode
    void setMode(HYAMBModes mod);        // sets the mode for both motors
    void setMode(HYAMBModes mod, int motor);  // sets the mode for the specified motor
    void eStop();                // activate Emergency stop
    void eStopRelease();            // Release emergency stop
    void stopAcquisition();            // clean up after the task is done
    bool isConnected();              // returns true if the controller is connected & ready for communication

    bool setDesiredCommands();

    void startAcquisition();

private:
    // static so that this function can be used while spawning a xenomai thread
    static void receiveData(void *context);

    bool readWheelSpeeds();
    bool readBatteryVoltage();
    bool readRCInputs();

    const bool sendCommands_;

    mutex_t cmdBufferMutex_;                  // for all other commands
    mutex_t desiredCmdMutex_;                // for all desired commands
    mutex_t dataMutex_;                    // for reading the parameters from the roboteq controller
    size_t vecSize_;
    boost::circular_buffer <CommandPacket> command_packets_; // Mutex-protected buffer

    bool isReady_ = false;
    bool requestAvailable_ = false;
    bool stopAcquisition_;

    thread_t hyambRoboteqSerialThread_;
//	RoboteqDevice* serialDevice;
    std::shared_ptr <RoboteqDevice> serialDevice;

    double leftMotorSpeed_ = 0; //[rad/sec]
    double rightMotorSpeed_ = 0; //[rad/sec]
    double batteryVoltage_ = 0; //[volts]

    // Joystick axes
    float x_rc_ = 0.0;
    float y_rc_ = 0.0;
    double lin_vel_scale_, ang_vel_scale_;
    unsigned int rc_count = 0;

    HYAMBModes mode_ = OPEN_LOOP;

    double des_velocity_motor1_ = 0;
    double des_velocity_motor2_ = 0;

    double des_tau_motor1_ = 0;
    double des_tau_motor2_ = 0;

    double des_pow_motor1_ = 0;
    double des_pow_motor2_ = 0;

    // Looping helpers
//	RTIME minPeriod_ = 20000000; //[nanosecs]
    double min_cycle_time_Us_ = 1e4; //[microsec] Sets the max desired io rate
    double cycleCountPeriod_Us_ = 5000000.0; //[microsec] Count the number of cycles over this duration for timing diagnostics
    double batteryVoltageUpdateInterval_ns_ = 500000000.0; //[nano_sec] Update the battery voltage at this rate
    double communicationDropoutTime_Us_ = 1000000.0; //[microsec] Try reconnecting to the motor controller if this amount of time has passed since the last successful read/write cycle
    std::chrono::time_point<std::chrono::high_resolution_clock> t_lastVoltageUpdate_;
    std::chrono::time_point<std::chrono::high_resolution_clock> t_lastSuccessfulCycle_;
    double lowBatteryVoltageWarningLevel_ = 36.0; //[V] todo Need to set this to a good value

    int sendGoCommand(double value, int motor);

    bool setVelocityImpl(double vel);

    bool setVelocityImpl(double vel, int motor);

    bool setTorqueImpl(double tau);

    bool setTorqueImpl(double tau, int motor);

    bool setMotorPowerImpl(double pow);

    bool setMotorPowerImpl(double pow, int motor);

    bool setModeImpl(int mod);

    bool setModeImpl(int mod, int motor);

    const double config_rps_to_cmd_ = 47.0; // this number was calculated by measuring the traveled distance with a commanded speed of 0.1m/s. Accurate to ~ 10%

    std::string port_;

    ros::NodeHandle &nh_;
    ros::Publisher wheelSpeedPub_;
    ros::Publisher rcTwistPub_;
    std_msgs::Float64MultiArray wheelSpeedMsg_;

    //Convert RPM to rad/sec
    double rpmToRps_;
};

} //namespace smb_driver

#endif /* INCLUDE_CONTROLLER_H_ */
