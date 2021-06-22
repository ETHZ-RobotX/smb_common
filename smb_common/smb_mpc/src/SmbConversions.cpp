//
// Created by johannes on 13.06.19.
//
#include <eigen_conversions/eigen_msg.h>

#include <smb_mpc/SmbConversions.h>

using namespace smb_path_following;

// void SmbConversions::writeMpcState(ocs2::vector_t& stateVector,
//                                                 const
//                                                 kindr::HomTransformQuatD&
//                                                 pose) {
//   // write x and y position into the first 2 elements
//   stateVector.head<2>() = pose.getPosition().toImplementation().head<2>();
//   // convert current quaternion to Euler angles and use the yaw rotation
//   kindr::EulerAnglesRpyD rpy(pose.getRotation());

//   kindr::Velocity3D v1(1, 0, 0);
//   kindr::Velocity3D v2 = pose.getRotation().rotate(v1);
//   stateVector[2] = std::atan2(v2(1), v2(0));
// }

// void SmbConversions::readMpcState(const ocs2::vector_t&
// stateVector,
//                                                kindr::HomTransformQuatD&
//                                                pose) {
//   kindr::Position3D position;
//   position.x() = stateVector[0];
//   position.y() = stateVector[1];
//   position.z() = 0;
//   kindr::EulerAnglesRpyD rotation;
//   rotation.setYaw(stateVector[2]);

//   pose.getPosition() = position;
//   pose.getRotation() = kindr::RotationQuaternionD(rotation);
// }

// void SmbConversions::readMpcInput(const ocs2::vector_t&
// inputVector, kindr::TwistLocalD& twist) {
//   kindr::Velocity3D linearVelocity;
//   linearVelocity(0) = inputVector[0];

//   kindr::LocalAngularVelocityD angularVelocity;
//   angularVelocity(2) = inputVector[1];

//   twist.getTranslationalVelocity() = linearVelocity;
//   twist.getRotationalVelocity() = angularVelocity;
// }

void SmbConversions::writeMpcObservation(
    ocs2::SystemObservation &observation,
    const geometry_msgs::TransformStamped &transformStamped) {
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
                        geometry_msgs::PoseStamped &poseStamped) {
  SmbConversions::readMpcState(observation.state, poseStamped.pose);
  poseStamped.header.stamp = ros::Time(observation.time);
}

void SmbConversions::writeMpcState(
    ocs2::vector_t &stateVector, const geometry_msgs::Pose &pose) {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  tf::pointMsgToEigen(pose.position, position);
  tf::quaternionMsgToEigen(pose.orientation, orientation);
  stateVector = Eigen::VectorXd(7);
  stateVector.head(3) = position;
  stateVector.tail(4) = orientation.coeffs();
}

void SmbConversions::readMpcState(
    const ocs2::vector_t &stateVector, geometry_msgs::Pose &pose) {
  Eigen::Vector3d position(stateVector.head<3>());
  Eigen::Quaterniond orientation(stateVector.tail<4>());

  tf::pointEigenToMsg(position, pose.position);
  tf::quaternionEigenToMsg(orientation, pose.orientation);
}

void SmbConversions::readMpcInput(
    const ocs2::vector_t &inputVector, geometry_msgs::Twist &twist) {
  twist.linear.x = inputVector[0];
  twist.angular.z = inputVector[1];
}
