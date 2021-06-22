#include <eigen_conversions/eigen_msg.h>

#include <odometry_conversion/OdometryConversion.hpp>

using namespace odometry_conversion;

OdometryConversion::OdometryConversion(ros::NodeHandle& nh) : buffer_(), transformListener_(buffer_) {
  inOdomFrame_ = nh.param<std::string>("in_odom_frame", inOdomFrame_);
  outOdomFrame_ = nh.param<std::string>("out_odom_frame", outOdomFrame_);
  inSensorFrame_ = nh.param<std::string>("in_sensor_frame", inSensorFrame_);
  outSensorFrame_ = nh.param<std::string>("out_sensor_frame", outSensorFrame_);

  auto sensorTransform = buffer_.lookupTransform(outSensorFrame_, inSensorFrame_, ros::Time(0), ros::Duration(10));
  sensorTransformHom_ = toHomTransform(sensorTransform.transform);
  sensorTransformHom_.block<3, 3>(0, 0) = sensorTransformHom_.block<3, 3>(0, 0).transpose();
  sensorTransformHom_.block<3, 1>(0, 3) = -sensorTransformHom_.block<3, 3>(0, 0) * sensorTransformHom_.block<3, 1>(0, 3);

  auto odomTransform = buffer_.lookupTransform(outSensorFrame_, inOdomFrame_, ros::Time(0), ros::Duration(10));
  odomTransformHom_ = toHomTransform(odomTransform.transform);

  // the odom frame of the tracking camera is gravity aligned. In case the camera is mounted with an angle, remove pitch and roll of this
  // transformation
  odomTransform.header.frame_id = outOdomFrame_;
  odomTransform.child_frame_id = inOdomFrame_ + "_odom";
  odomCameraOdomTransformPublisher_.sendTransform(odomTransform);

  odometryPublisher_ = nh.advertise<nav_msgs::Odometry>("/base_odom", 1, false);
  odometryInSubscriber_ = nh.subscribe("/camera/odom/sample", 1, &OdometryConversion::odometryInCallback, this);
}

Eigen::Matrix4d OdometryConversion::toHomTransform(const geometry_msgs::Transform& transform) const {
  Eigen::Matrix4d homTransform = Eigen::Matrix4d::Identity();
  Eigen::Vector3d translation;
  tf::vectorMsgToEigen(transform.translation, translation);
  homTransform.block<3, 1>(0, 3) = translation;
  Eigen::Quaterniond rotation;
  tf::quaternionMsgToEigen(transform.rotation, rotation);
  homTransform.block<3, 3>(0, 0) = rotation.toRotationMatrix();
  return homTransform;
}

Eigen::Matrix4d OdometryConversion::toHomTransform(const geometry_msgs::Pose& transform) const {
  Eigen::Matrix4d homTransform = Eigen::Matrix4d::Identity();
  Eigen::Vector3d translation;
  tf::pointMsgToEigen(transform.position, translation);
  homTransform.block<3, 1>(0, 3) = translation;
  Eigen::Quaterniond rotation;
  tf::quaternionMsgToEigen(transform.orientation, rotation);
  homTransform.block<3, 3>(0, 0) = rotation.toRotationMatrix();
  return homTransform;
}

geometry_msgs::Transform OdometryConversion::fromHomTransform(const Eigen::Matrix4d& homTransform) const {
  geometry_msgs::Transform transform;
  Eigen::Vector3d translation = homTransform.block<3, 1>(0, 3);
  tf::vectorEigenToMsg(translation, transform.translation);
  Eigen::Quaterniond rotation = Eigen::Quaterniond(homTransform.block<3, 3>(0, 0));
  tf::quaternionEigenToMsg(rotation, transform.rotation);
  return transform;
}

geometry_msgs::Pose OdometryConversion::fromHomTransformToPose(const Eigen::Matrix4d& homTransform) const {
  geometry_msgs::Pose transform;
  Eigen::Vector3d translation = homTransform.block<3, 1>(0, 3);
  tf::pointEigenToMsg(translation, transform.position);
  Eigen::Quaterniond rotation = Eigen::Quaterniond(homTransform.block<3, 3>(0, 0));
  tf::quaternionEigenToMsg(rotation, transform.orientation);
  return transform;
}

void OdometryConversion::odometryInCallback(const nav_msgs::Odometry& odomIn) {
  nav_msgs::Odometry odomOut;
  odomOut.header.stamp = odomIn.header.stamp;
  odomOut.header.frame_id = outOdomFrame_;
  odomOut.child_frame_id = outSensorFrame_;

  Eigen::Matrix4d inHom = toHomTransform(odomIn.pose.pose);
  Eigen::Matrix4d outHom = odomTransformHom_ * inHom * sensorTransformHom_;
  odomOut.pose.pose = fromHomTransformToPose(outHom);

  Eigen::Vector3d inRotVel;
  tf::vectorMsgToEigen(odomIn.twist.twist.angular, inRotVel);
  Eigen::Vector3d outRotVel = sensorTransformHom_.block<3, 3>(0, 0).transpose() * inRotVel;
  tf::vectorEigenToMsg(outRotVel, odomOut.twist.twist.angular);

  Eigen::Vector3d inLinVel;
  tf::vectorMsgToEigen(odomIn.twist.twist.linear, inLinVel);
  Eigen::Vector3d inLinVelOutFrame = sensorTransformHom_.block<3, 3>(0, 0).transpose() * inLinVel;
  Eigen::Vector3d outLinVel = inLinVelOutFrame + outRotVel.cross(sensorTransformHom_.block<3, 1>(0, 3));
  tf::vectorEigenToMsg(outLinVel, odomOut.twist.twist.linear);

  // covariance is symbolic for RS-T265. We do not need to transform this properly.
  odomOut.pose.covariance = odomIn.pose.covariance;
  odomOut.twist.covariance = odomIn.twist.covariance;

  odometryPublisher_.publish(odomOut);

  // publish base odom frame via tf
  geometry_msgs::TransformStamped odomTransform;
  odomTransform.header.stamp = odomIn.header.stamp;
  odomTransform.header.frame_id = outSensorFrame_;
  odomTransform.child_frame_id = outOdomFrame_;
  Eigen::Matrix3d odomTransformRot = outHom.block<3, 3>(0, 0).transpose();
  tf::quaternionEigenToMsg(Eigen::Quaterniond(odomTransformRot), odomTransform.transform.rotation);
  tf::vectorEigenToMsg(Eigen::Vector3d(-odomTransformRot * outHom.block<3, 1>(0, 3)), odomTransform.transform.translation);
  odomPublisher_.sendTransform(odomTransform);

  Eigen::Affine3d odomTransformEigen;
  tf::transformMsgToEigen(odomTransform.transform, odomTransformEigen);

  // ROS_WARN_STREAM_THROTTLE(5.0, std::endl << "inHom:" << std::endl << inHom << std::endl << std::endl
  // << "odomTransformHom_:" << std::endl << odomTransformHom_ << std::endl << std::endl
  // << "sensorTransformHom_:" << std::endl << sensorTransformHom_ << std::endl << std::endl
  // << "outHom:" << std::endl << outHom << std::endl << std::endl
  // << "odomTransformEigen:" << std::endl << odomTransformEigen.matrix() << std::endl << std::endl);
}
