/*!
 * @file 	  SmbController.hpp
 * @author   Johannes Pankert
 * @date		  03/05/2019
 * @version 	1.0
 * @brief    A controller that ...
 */

#pragma once

#include <mutex>

// smb_path_following
#include <smb_mpc/SmbConversions.h>
#include <smb_mpc/SmbInterface.h>

// ros
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <ocs2_mpc/MPC_MRT_Interface.h>

namespace smb_path_following {

class SmbController {
public:
  typedef ocs2::SystemObservation Observation;
  typedef ocs2::MPC_MRT_Interface MpcInterface;
  typedef ocs2::CostDesiredTrajectories CostDesiredTrajectories;

  //! Construct SmbController.
  SmbController(ros::NodeHandle &nh);

  //! Destruct SmbController.
  virtual ~SmbController();

protected:
  SmbInterface mmInterface_;
  std::shared_ptr<MpcInterface> mpcInterface_;

  bool planAvailable_;
  bool observationAvailable_;
  bool desiredTrajectoryAvailable_;
  int lastSequence_;
  double maxLinearVelocity_ = 1.0;
  double maxAngularVelocity_ = 1.0;
  bool adjustTimeStamps_ = true;

  std::string baseFrame_ = "base";
  std::string controlFrame_ = "odom";

  std::mutex observationMutex_;
  Observation observation_;

  std::mutex costDesiredTrajectoriesMutex_;
  CostDesiredTrajectories costDesiredTrajectories_;

  ros::Timer mpcUpdateWorker_;

  ros::Timer trackingControllerWorker_;

  std::mutex pathMutex_;
  nav_msgs::Path path_;
  ros::Subscriber pathSubscriber_;
  ros::Subscriber joyTwistInterventionSubscriber_;

  ros::Publisher commandTwistStampedPublisher_;
  ros::Publisher commandTwistPublisher_;
  // visualization for debugging
  ros::Timer rosPublishingWorker_;
  ros::Publisher currentPosePublisher_;
  ros::Publisher optimalPosePublisher_;
  ros::Publisher optimalPathPublisher_;

  ros::NodeHandle nh_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tfListener_;

protected:
  virtual bool initialize();

  virtual bool advance();

protected:
  void writeDesiredTrajectory(
      const nav_msgs::Path &path,
      SmbController::CostDesiredTrajectories &costDesiredTrajectories,
      const Observation &currentObservation);

  bool mpcUpdate();

  bool publishRos();

  void pathCallback(const nav_msgs::PathConstPtr &path);
  void joyTwistInverventionCallback(const geometry_msgs::TwistConstPtr &twist);
  void adjustTimeStamps(nav_msgs::Path &path);
  void resetMpc();
};
} // namespace smb_path_following