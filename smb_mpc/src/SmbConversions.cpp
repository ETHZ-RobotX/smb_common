//
// Created by johannes on 13.06.19.
//
#include <eigen_conversions/eigen_msg.h>

#include <smb_mpc/SmbConversions.h>

namespace smb_mpc
{
  void SmbConversions::writeMpcObservation(
      ocs2::SystemObservation &observation,
      const geometry_msgs::TransformStamped &transformStamped)
  {
    observation.time = ros::Time(transformStamped.header.stamp).toSec();
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    tf::vectorMsgToEigen(transformStamped.transform.translation, position);
    tf::quaternionMsgToEigen(transformStamped.transform.rotation, orientation);
    observation.state = Eigen::VectorXd(7);
    observation.state.head(3) = position;
    observation.state.tail(4) = orientation.coeffs();
  }

  void SmbConversions::readMpcObservation(const ocs2::SystemObservation &observation,
                                          geometry_msgs::PoseStamped &poseStamped)
  {
    SmbConversions::readMpcState(observation.state, poseStamped.pose);
    poseStamped.header.stamp = ros::Time(observation.time);
  }

  void SmbConversions::writeMpcState(
      ocs2::vector_t &stateVector, const geometry_msgs::Pose &pose)
  {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    tf::pointMsgToEigen(pose.position, position);
    tf::quaternionMsgToEigen(pose.orientation, orientation);
    stateVector = Eigen::VectorXd(7);
    stateVector.head(3) = position;
    stateVector.tail(4) = orientation.coeffs();
  }

  void SmbConversions::readMpcState(
      const ocs2::vector_t &stateVector, geometry_msgs::Pose &pose)
  {
    Eigen::Vector3d position(stateVector.head<3>());
    Eigen::Quaterniond orientation(stateVector.tail<4>());

    tf::pointEigenToMsg(position, pose.position);
    tf::quaternionEigenToMsg(orientation, pose.orientation);
  }

  void SmbConversions::readMpcInput(
      const ocs2::vector_t &inputVector, geometry_msgs::Twist &twist)
  {
    twist.linear.x = inputVector[0];
    twist.angular.z = inputVector[1];
  }
}