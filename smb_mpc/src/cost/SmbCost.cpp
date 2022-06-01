//
// Created by johannes on 01.05.19.
//
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <smb_mpc/SmbCost.h>

using namespace smb_mpc;
using namespace ocs2;

SmbCost::SmbCost(ocs2::matrix_t QPosition, ocs2::matrix_t QOrientation,
                 ocs2::matrix_t R, const std::string &libraryFolder,
                 bool recompileLibraries)
    : Base(), QPosition_(std::move(QPosition)),
      QOrientation_(std::move(QOrientation)), R_(std::move(R)) {
  initialize(SmbDefinitions::STATE_DIM, SmbDefinitions::INPUT_DIM,
             SmbDefinitions::STATE_DIM, "smb", libraryFolder,
             recompileLibraries, true);
}

ad_vector_t SmbCost::costVectorFunction(ad_scalar_t time,
                                        const ad_vector_t &state,
                                        const ad_vector_t &input,
                                        const ad_vector_t &parameters) const {

  using ad_quat_t = Eigen::Quaternion<ad_scalar_t>;

  const ad_vector_t currentPosition = SmbConversions::readPosition(state);
  const ad_quat_t currentOrientation = SmbConversions::readRotation(state);

  const ad_vector_t desiredPosition = SmbConversions::readPosition(parameters);
  const ad_quat_t desiredOrientation = SmbConversions::readRotation(parameters);

  ad_vector_t positionError = ad_vector_t::Zero(3);
  ad_vector_t orientationError = ad_vector_t::Zero(3);
  ad_vector_t totalCost(positionError.size() + orientationError.size() +
                        input.size());

  /// todo Compute totalCost here. You can use the weight matricies
  /// QPosition_, QOrientation_ and R_

  return totalCost;
}

vector_t
SmbCost::getParameters(scalar_t time,
                       const TargetTrajectories &costDesiredTrajectory) const {
  const auto &desiredStateTrajectory = costDesiredTrajectory.stateTrajectory;
  const auto &desiredTimeTrajectory = costDesiredTrajectory.timeTrajectory;
  Eigen::Vector3d referencePosition = Eigen::Vector3d::Zero();
  Eigen::Quaterniond referenceOrientation = Eigen::Quaterniond::Identity();

  if (desiredStateTrajectory.size() > 1) {
    // TODO: Compute  referencePosition and referenceOrientation implementation
    // function here. You can use
    // SmbConversions::readRotation(desiredStateTrajectory[index]) -->
    // Eigen::Quaterniond to get the reference quaternion and
    // SmbConversions::readPosition(desiredStateTrajectory[index]) -->
    // Eigen::Vector3d to get the reference position.
    // desiredTimeTrajectory is an std::vector<double> of the reference timestamps.

  } else { // desiredStateTrajectory.size() == 1, Do not change this
    referencePosition = SmbConversions::readPosition(desiredStateTrajectory[0]);
    referenceOrientation = SmbConversions::readRotation(desiredStateTrajectory[0]);
  }

  vector_t reference(SmbDefinitions::STATE_DIM);
  reference.head(3) = referencePosition;
  reference.tail(4) = referenceOrientation.coeffs();
  return reference;
}

SmbCost *SmbCost::clone() const { return new SmbCost(*this); }
