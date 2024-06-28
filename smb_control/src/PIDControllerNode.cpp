#include "smb_control/PIDControllerNode.hpp"
#include <dynamic_reconfigure/server.h>
#include <smb_control/PIDConfig.h>

PIDControllerNode::PIDControllerNode() 
: nh_("~"), 
  last_odom_time_(ros::Time(0)), 
  angular_velocity_pid_(1.0, 0.1, 0.0), 
  linear_velocity_pid_(1.0, 0.0, 0.0),

  config_server_(std::make_shared<dynamic_reconfigure::Server<smb_control::PIDConfig>>(nh_)),
  odom_timeout_(0.5), 
  twist_timeout_(0.2) 
{
    // Initialize the publisher
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/control/smb_diff_drive/cmd_vel", 1);

    // Initialize the subscriber
    twist_sub_ = nh_.subscribe("/control/twist_mux/output", 1, &PIDControllerNode::twistCallback, this);
    odom_sub_ = nh_.subscribe("/odometry/filtered", 1, &PIDControllerNode::odomCallback, this);

    // Initialize dynamic reconfigure server for PID parameters
    config_server_->setCallback(boost::bind(&PIDControllerNode::configCallback, this, _1, _2));
}

PIDControllerNode::~PIDControllerNode()
{
    // Cleanup can be handled here
}

void PIDControllerNode::run()
{
    ros::Rate loop_rate(100);  // 100 Hz
    while (ros::ok())
    {
        ros::spinOnce();
        checkTimeout();
        loop_rate.sleep();
    }
}

void PIDControllerNode::configCallback(smb_control::PIDConfig &config, uint32_t level)
{
    // Update PID gains
    angular_velocity_pid_.setGains(config.kp_angular, config.ki_angular, config.kd_angular);
    linear_velocity_pid_.setGains(config.kp_linear, config.ki_linear, config.kd_linear);

    // Update EKF parameters
    std::string ns = "/ekf_se"; // Ensure the namespace matches your EKF node's namespace

    // General EKF parameters
    ros::param::set(ns + "/odom_frame", "odom");
    ros::param::set(ns + "/base_link_frame", "base_link");
    ros::param::set(ns + "/world_frame", "odom");
    ros::param::set(ns + "/two_d_mode", true);
    ros::param::set(ns + "/frequency", 50);
    ros::param::set(ns + "/publish_tf", true);
    ros::param::set(ns + "/print_diagnostics", true);

    use_diff_drive_estimation_ = config.use_differential_drive_estimation;
    use_vio_ = config.use_vio;
    use_imu_ = config.use_imu;

    // Handle odom0 and odom1 logic
    if (use_diff_drive_estimation_)
    {
        ros::param::set(ns + "/odom0", "/control/smb_diff_drive/odom");
        ros::param::set(ns + "/odom0_config", std::vector<bool>{false, false, false, false, false, false, true, true, true, false, false, true, false, false, false});
        ros::param::set(ns + "/odom0_queue_size", 100);
        ros::param::set(ns + "/odom0_differential", false);
        ros::param::set(ns + "/odom0_relative", false);

        if (use_vio_)
        {
            ros::param::set(ns + "/odom1", "/tracking_camera/odom/sample");
            ros::param::set(ns + "/odom1_config", std::vector<bool>{false, false, false, false, false, false, true, true, true, true, true, true, false, false, false});
            ros::param::set(ns + "/odom1_queue_size", 100);
            ros::param::set(ns + "/odom1_differential", false);
            ros::param::set(ns + "/odom1_relative", false);
        }
        else
        {
            ros::param::del(ns + "/odom1");
            ros::param::del(ns + "/odom1_config");
            ros::param::del(ns + "/odom1_queue_size");
            ros::param::del(ns + "/odom1_differential");
            ros::param::del(ns + "/odom1_relative");
        }
    }
    else if (use_vio_)
    {
        // If only VIO is used, configure it as odom0
        ros::param::set(ns + "/odom0", "/tracking_camera/odom/sample");
        ros::param::set(ns + "/odom0_config", std::vector<bool>{false, false, false, false, false, false, true, true, true, true, true, true, false, false, false});
        ros::param::set(ns + "/odom0_queue_size", 100);
        ros::param::set(ns + "/odom0_differential", false);
        ros::param::set(ns + "/odom0_relative", false);

        ros::param::del(ns + "/odom1");
        ros::param::del(ns + "/odom1_config");
        ros::param::del(ns + "/odom1_queue_size");
        ros::param::del(ns + "/odom1_differential");
        ros::param::del(ns + "/odom1_relative");
    }
    else
    {
        // Neither differential drive nor VIO is used, delete both odom configurations
        ros::param::del(ns + "/odom0");
        ros::param::del(ns + "/odom0_config");
        ros::param::del(ns + "/odom0_queue_size");
        ros::param::del(ns + "/odom0_differential");
        ros::param::del(ns + "/odom0_relative");

        ros::param::del(ns + "/odom1");
        ros::param::del(ns + "/odom1_config");
        ros::param::del(ns + "/odom1_queue_size");
        ros::param::del(ns + "/odom1_differential");
        ros::param::del(ns + "/odom1_relative");

        if(!use_imu_)
        {
            // If no sensors are used, delete the EKF node
            pure_feed_forward_ = true;
        }
    }

    // IMU0 parameters
    if (use_imu_)
    {
        ros::param::set(ns + "/imu0", "/imu");
        ros::param::set(ns + "/imu0_config", std::vector<bool>{false, false, false, true, true, true, false, false, false, true, true, true, false, false, false});
        ros::param::set(ns + "/imu0_differential", true);
        ros::param::set(ns + "/imu0_queue_size", 10);
        ros::param::set(ns + "/imu0_remove_gravitational_acceleration", true);
    }
    else
    {
        ros::param::del(ns + "/imu0");
        ros::param::del(ns + "/imu0_config");
        ros::param::del(ns + "/imu0_differential");
        ros::param::del(ns + "/imu0_queue_size");
        ros::param::del(ns + "/imu0_remove_gravitational_acceleration");
    }

    // Diagnostic configuration
    ros::param::set(ns + "/diagnostic_updater/frequency", 1.0);
    ros::param::set(ns + "/diagnostic_updater/min_frequency", 0.1);
    ros::param::set(ns + "/diagnostic_updater/max_frequency", 100.0);
}

void PIDControllerNode::twistCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // Get the current angular velocity from the Odom
    cmd_vel_requested_ = *msg;
    last_twist_time_ = ros::Time::now();
}

void PIDControllerNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double angular_velocity = msg->twist.twist.angular.z;
    double linear_velocity = msg->twist.twist.linear.x; // Assuming x is the forward direction

    // Apply noise threshold
    if (std::abs(angular_velocity) < 0.01) angular_velocity = 0;
    if (std::abs(linear_velocity) < 0.01) linear_velocity = 0;

    if (!last_odom_time_.isZero()) { // Check if not the first message
        dt_ = (msg->header.stamp - last_odom_time_).toSec();
    }
    last_odom_time_ = msg->header.stamp; // Update last_odom_time_ for the next call

    // Calculate the error
    double angular_error = cmd_vel_requested_.angular.z - angular_velocity;
    double linear_error = cmd_vel_requested_.linear.x - linear_velocity;

    // Reset the integrator if the command should be zero
    angular_velocity_pid_.resetIfZero(cmd_vel_requested_.angular.z);
    linear_velocity_pid_.resetIfZero(cmd_vel_requested_.linear.x);

    // Update the PID controller
    cmd_vel_.angular.z = cmd_vel_requested_.angular.z;
    cmd_vel_.linear.x = cmd_vel_requested_.linear.x;

    if(!pure_feed_forward_)
    {
        if (abs(cmd_vel_requested_.angular.z) > 0.05f){
            cmd_vel_.angular.z += angular_velocity_pid_.update(angular_error, dt_);
        } else { 
            cmd_vel_.angular.z = 0.0;
        }
        cmd_vel_.linear.x += linear_velocity_pid_.update(linear_error, dt_);
    }

    // Publish the control output
    cmd_vel_pub_.publish(cmd_vel_);
}

void PIDControllerNode::checkTimeout()
{
    ros::Time current_time = ros::Time::now();

    if ((current_time - last_twist_time_).toSec() > twist_timeout_) {
        cmd_vel_requested_ = geometry_msgs::Twist();
    }

    if ((current_time - last_odom_time_).toSec() > odom_timeout_) {
        if (!pure_feed_forward_) {
            pure_feed_forward_ = true;
            ROS_WARN("Odometry data timeout, switching to feedforward control.");
        }
        executeFeedforwardControl();
    } else {
        if (pure_feed_forward_) {
            pure_feed_forward_ = false;
            ROS_INFO("Odometry data regained, switching to PID control.");
        }
    }
}

void PIDControllerNode::executeFeedforwardControl()
{
    cmd_vel_.angular.z = cmd_vel_requested_.angular.z;
    cmd_vel_.linear.x = cmd_vel_requested_.linear.x;

    // Publish the control output
    cmd_vel_pub_.publish(cmd_vel_);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "PIDControllerNode");
    PIDControllerNode node;
    node.run();
    return 0;
}
