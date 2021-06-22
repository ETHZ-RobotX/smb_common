#pragma once

#include "Eigen/Eigen"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

namespace odometry_conversion {
class OdometryConversion {
 public:
  OdometryConversion(ros::NodeHandle& nh);

 private:
  void odometryInCallback(const nav_msgs::Odometry& odomIn);

  Eigen::Matrix4d toHomTransform(const geometry_msgs::Transform& transform) const;
  Eigen::Matrix4d toHomTransform(const geometry_msgs::Pose& transform) const;
  geometry_msgs::Transform fromHomTransform(const Eigen::Matrix4d& homTransform) const;
  geometry_msgs::Pose fromHomTransformToPose(const Eigen::Matrix4d& homTransform) const;

  std::string inOdomFrame_ = "camera_pose_frame";
  std::string inSensorFrame_ = "camera_pose_frame";
  std::string outOdomFrame_ = "tracking_camera_odom";
  std::string outSensorFrame_ = "base_link";

  Eigen::Matrix4d odomTransformHom_;
  Eigen::Matrix4d sensorTransformHom_;

  ros::Subscriber odometryInSubscriber_;
  ros::Publisher odometryPublisher_;

  tf2_ros::StaticTransformBroadcaster odomCameraOdomTransformPublisher_;
  tf2_ros::TransformBroadcaster odomPublisher_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener transformListener_;
};
}  // namespace odometry_conversion