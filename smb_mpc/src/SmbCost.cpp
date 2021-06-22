//
// Created by johannes on 01.05.19.
//
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <smb_mpc/SmbCost.h>

using namespace smb_path_following;
using namespace ocs2;

SmbCost::SmbCost(ocs2::matrix_t QPosition,
                                           ocs2::matrix_t QOrientation,
                                           ocs2::matrix_t R)
    : Base(), QPosition_(std::move(QPosition)),
        QOrientation_(std::move(QOrientation)), R_(std::move(R)) {}

ad_vector_t SmbCost::costVectorFunction(
    ad_scalar_t time, const ad_vector_t &state, const ad_vector_t &input,
    const ad_vector_t &parameters) const {

  using ad_quat_t = Eigen::Quaternion<ad_scalar_t>;

  const ad_vector_t &currentPosition = state.head(3);
  const ad_quat_t currentOrientation(state.tail<4>());

  const ad_vector_t &desiredPosition = parameters.head(3);
  const ad_quat_t desiredOrientation(parameters.tail<4>());

  ad_vector_t positionError = desiredPosition - currentPosition;
  ad_vector_t orientationError =
      ocs2::quaternionDistance(desiredOrientation, currentOrientation);

  ad_vector_t totalCost(positionError.size() + orientationError.size() + input.size());
  totalCost.head(positionError.size()) = QPosition_.cast<ad_scalar_t>() * positionError;
  totalCost.segment(positionError.size(), orientationError.size()) = QOrientation_.cast<ad_scalar_t>() * orientationError;
  totalCost.tail(input.size()) = R_.cast<ad_scalar_t>() * input;
  return totalCost;
}

vector_t SmbCost::getParameters(
    scalar_t time, const CostDesiredTrajectories &costDesiredTrajectory) const {
  const auto &desiredTrajectory =
      costDesiredTrajectory.desiredStateTrajectory();
  vector_t reference(SmbDefinitions::STATE_DIM);

  if (desiredTrajectory.size() > 1) {
    // Normal interpolation case
    int index;
    scalar_t alpha;
    std::tie(index, alpha) = LinearInterpolation::timeSegment(
        time, costDesiredTrajectory.desiredTimeTrajectory());

    const auto &lhs = desiredTrajectory[index];
    const auto &rhs = desiredTrajectory[index + 1];
    const Eigen::Quaterniond quaternionA(lhs.tail<4>());
    const Eigen::Quaterniond quaternionB(rhs.tail<4>());

    reference.head(3) = alpha * lhs.head<3>() + (1.0 - alpha) * rhs.head<3>();
    reference.tail(4) = (quaternionA.slerp((1 - alpha), quaternionB)).coeffs();
  } else { // desiredTrajectory.size() == 1
    reference.head(3) = desiredTrajectory[0].head<3>();
    reference.tail(4) = desiredTrajectory[0].tail<4>();
  }

  return reference;
}

SmbCost *SmbCost::clone() const {
  return new SmbCost(*this);
}
