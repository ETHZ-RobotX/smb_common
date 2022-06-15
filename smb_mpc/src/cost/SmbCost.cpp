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
  using ad_mat_t = Eigen::Matrix<ad_scalar_t, -1, -1>;
  using ad_angle_axis_t = Eigen::AngleAxis<ad_scalar_t>;
  using ad_vec3_t = Eigen::Matrix<ad_scalar_t, 3, 1>;

  const ad_vector_t currentPosition = SmbConversions::readPosition(state);
  const ad_quat_t currentOrientation = SmbConversions::readRotation(state);

  ad_vector_t desiredPosition = SmbConversions::readPosition(parameters);
  ad_quat_t desiredOrientation = SmbConversions::readRotation(parameters);

  const ad_mat_t QPosition = QPosition_.cast<ad_scalar_t>();
  const ad_mat_t QOrientation = QOrientation_.cast<ad_scalar_t>();
  const ad_mat_t R = R_.cast<ad_scalar_t>();

  ad_vector_t positionError = ad_vector_t::Zero(3);
  ad_vector_t orientationError = ad_vector_t::Zero(3);
  ad_vector_t inputError = ad_vector_t::Zero(2);

  /// TODO: Compute cost terms positionError, orientationError, and inputError
  /// here. For Task 2, overwrite currentPosition and currentOrientation with
  /// the desired setpoint. For Task 3, keep the values set in L33-34 You can
  /// use the weight matricies QPosition, QOrientation and R for Task 4.


  ad_vector_t totalCost(positionError.size() + orientationError.size() +
                        inputError.size());
  totalCost << positionError, orientationError, inputError;

  return totalCost;
}

vector_t
SmbCost::getParameters(scalar_t time,
                       const TargetTrajectories &costDesiredTrajectory) const {
  const auto &desiredStateTrajectory = costDesiredTrajectory.stateTrajectory;
  const auto &desiredTimeTrajectory = costDesiredTrajectory.timeTrajectory;
  int numPoses = desiredTimeTrajectory.size();
  Eigen::Vector3d referencePosition = Eigen::Vector3d::Zero();
  Eigen::Quaterniond referenceOrientation = Eigen::Quaterniond::Identity();

  if (desiredStateTrajectory.size() > 1) {
    // TODO: Compute  referencePosition and referenceOrientation implementation
    // function here. You can use
    // SmbConversions::readRotation(desiredStateTrajectory[index]) -->
    // Eigen::Quaterniond to get the reference quaternion and
    // SmbConversions::readPosition(desiredStateTrajectory[index]) -->
    // Eigen::Vector3d to get the reference position.
    // desiredTimeTrajectory is an std::vector<double> of the reference
    // timestamps.

  } else { // desiredStateTrajectory.size() == 1, Do not change this
    referencePosition = SmbConversions::readPosition(desiredStateTrajectory[0]);
    referenceOrientation =
        SmbConversions::readRotation(desiredStateTrajectory[0]);
  }

  vector_t reference(SmbDefinitions::STATE_DIM);
  reference.head(3) = referencePosition;
  reference.tail(4) = referenceOrientation.coeffs();
  return reference;
}

SmbCost *SmbCost::clone() const { return new SmbCost(*this); }
