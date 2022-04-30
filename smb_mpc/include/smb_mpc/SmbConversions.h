//
// Created by johannes on 13.06.19.
//

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

#include <ocs2_core/Types.h>
#include <ocs2_mpc/SystemObservation.h>

namespace smb_mpc {

class SmbConversions {
public:
  static void
  writeMpcObservation(ocs2::SystemObservation &observation,
                      const geometry_msgs::TransformStamped &transformStamped);
  static void readMpcObservation(const ocs2::SystemObservation &observation,
                                 geometry_msgs::PoseStamped &poseStamped);

  static void writeMpcState(ocs2::vector_t &stateVector,
                            const geometry_msgs::Pose &pose);
  static void readMpcState(const ocs2::vector_t &stateVector,
                           geometry_msgs::Pose &pose);

  static void readMpcInput(const ocs2::vector_t &inputVector,
                           geometry_msgs::Twist &twist);

  // static void writeMpcObservation(ocs2::SystemObservation& observation, const
  // any_measurements::Pose& pose);
};

} // namespace smb_path_following