#ifndef PIDCONTROLLERNODE_HPP
#define PIDCONTROLLERNODE_HPP

#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <dynamic_reconfigure/server.h>
#include <smb_control/PIDConfig.h>

class PIDController {
public:
    PIDController(double kp = 0.0, double ki = 0.0, double kd = 0.0, double max_integral = 2.0)
        : kp_(kp), ki_(ki), kd_(kd), integral_(0.0), prev_error_(0.0), max_integral_(max_integral) {}

    void setGains(double kp, double ki, double kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    void setMaxIntegral(double max_integral) {
        max_integral_ = max_integral;
    }

    double update(double error, double dt) {
        if (dt <= 0.0001) return 0.0; // Prevent division by zero

        integral_ += error * dt;
        
        // Clamp the integral value to the maximum limit
        if (integral_ > max_integral_) integral_ = max_integral_;
        if (integral_ < -max_integral_) integral_ = -max_integral_;
        
        double derivative = (error - prev_error_) / dt;
        prev_error_ = error;

        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

    void reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
    }

    void resetIfZero(double cmd) {
        if (std::abs(cmd) < 0.05) { // Check if the command is approximately zero
            reset();
        }
    }

private:
    double kp_, ki_, kd_;
    double integral_, prev_error_;
    double max_integral_;
};

class PIDControllerNode
{
public:
    // Constructor
    PIDControllerNode();

    // Destructor
    ~PIDControllerNode();

    // Main loop function
    void run();

    void configCallback(smb_control::PIDConfig &config, uint32_t level);

    void ekfConfigCallback(smb_control::PIDConfig &config, uint32_t level);

private:
    ros::NodeHandle nh_;

    // Publisher
    ros::Publisher cmd_vel_pub_;

    // Subscriber
    ros::Subscriber twist_sub_;
    ros::Subscriber odom_sub_;

    bool pure_feed_forward_{true};

    // Subscriber callback
    void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    // PID
    PIDController angular_velocity_pid_;
    PIDController linear_velocity_pid_;
    bool use_diff_drive_estimation_{false};
    bool use_vio_{false};
    bool use_imu_{false};

    std::shared_ptr<dynamic_reconfigure::Server<smb_control::PIDConfig>> config_server_;

    geometry_msgs::Twist cmd_vel_requested_;
    geometry_msgs::Twist cmd_vel_;

    ros::Time last_twist_time_;
    ros::Time last_odom_time_;
    double dt_{0.01};
    double odom_timeout_{0.0};
    double twist_timeout_{0.0};

    void checkTimeout();
    void executeFeedforwardControl();
};

#endif // PIDCONTROLLERNODE_HPP
