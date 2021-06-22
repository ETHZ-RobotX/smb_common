// smb lowlevel controller
#include <functional>
#include <chrono>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "smb_lowlevel_controller/SmbHWInterface.hpp"
#include "controller_manager/controller_manager.h"

using time_source = std::chrono::steady_clock ;

void controlLoop(smb_lowlevel_controller::SmbHWInterface &smb,
                 controller_manager::ControllerManager &cm,
                 time_source::time_point &last_time) {

  // Calculate monotonic time difference
  auto now = time_source::now();
  time_source::duration elapsed_duration = now - last_time;
  double durationSecs = std::chrono::duration_cast<std::chrono::microseconds>(elapsed_duration).count()/1e6;
  ros::Duration elapsed{durationSecs};
  last_time = now;

  // Process control loop
  smb.read(ros::Time::now(), elapsed);
  cm.update(ros::Time::now(), elapsed);
  // cm.update(robot.get_time(), robot.get_period());
  smb.write(ros::Time::now(), elapsed);
}

int main(int argc, char** argv)
/*
 * {
 *   any_node::Nodewrap<smb_lowlevel_controller::SmbHWInterface> node(argc, argv, "smb_lowlevel_controller", 2); // use 2 spinner thread
 *   node.execute(); // update thread won't be executed
 *   return 0;
 * }
 */
{
  ros::init(argc, argv, "smb_base");
  ros::NodeHandle nh, private_nh("~");

  double control_frequency;
  private_nh.param<double>("control_frequency", control_frequency, 50.0);

  smb_lowlevel_controller::SmbHWInterface robot{};
  std::string controller_ns;
  nh.param<std::string>("controller_namespace", controller_ns, "");
  ros::NodeHandle robot_nh;
  if (controller_ns.empty())
	  robot_nh = nh;
  else {
	  robot_nh = ros::NodeHandle(nh, controller_ns);
	  ROS_INFO("Using relative ns: '%s'", controller_ns);
  }
  robot.init(robot_nh, private_nh);
  controller_manager::ControllerManager cm(&robot, nh);


  ros::CallbackQueue smb_queue;
  ros::AsyncSpinner smb_spinner(1, &smb_queue);

  time_source::time_point last_time = time_source::now();
  std::function<void(const ros::TimerEvent&)> callback = [&robot, &cm, &last_time](const ros::TimerEvent&){controlLoop(robot, cm, last_time);};
  ros::TimerOptions control_timer(
    ros::Duration(1.0 / control_frequency),
    callback,
    &smb_queue);
  ros::Timer control_loop = nh.createTimer(control_timer);

  smb_spinner.start();

  // Process remainder of ROS callbacks separately, mainly ControlManager related
  ros::spin();
}
