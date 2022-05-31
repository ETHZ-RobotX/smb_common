//
// Created by johannes on 01.05.19.
//
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <smb_mpc/SmbCost.h>

using namespace smb_mpc;
using namespace ocs2;

SmbCost::SmbCost(ocs2::matrix_t QPosition, ocs2::matrix_t QOrientation, ocs2::matrix_t R,
                 const std::string &libraryFolder, bool recompileLibraries) : Base(), QPosition_(std::move(QPosition)), QOrientation_(std::move(QOrientation)), R_(std::move(R))
{
  initialize(SmbDefinitions::STATE_DIM, SmbDefinitions::INPUT_DIM, SmbDefinitions::STATE_DIM, "smb", libraryFolder, recompileLibraries, true);
}

ad_vector_t SmbCost::costVectorFunction(ad_scalar_t time, const ad_vector_t &state, const ad_vector_t &input, const ad_vector_t &parameters) const
{

  using ad_quat_t = Eigen::Quaternion<ad_scalar_t>;

  const ad_vector_t currentPosition = SmbConversions::readPosition(state);
  const ad_quat_t currentOrientation = SmbConversions::readRotation(state);

  const ad_vector_t desiredPosition = SmbConversions::readPosition(parameters);
  const ad_quat_t desiredOrientation = SmbConversions::readRotation(parameters);

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
    scalar_t time, const TargetTrajectories &costDesiredTrajectory) const
{
  const auto &desiredTrajectory =
      costDesiredTrajectory.stateTrajectory;
  vector_t reference(SmbDefinitions::STATE_DIM);

  if (desiredTrajectory.size() > 1)
  {
    // Normal interpolation case
    int index;
    scalar_t alpha;
    std::tie(index, alpha) = LinearInterpolation::timeSegment(
        time, costDesiredTrajectory.timeTrajectory);

    const auto &lhs = desiredTrajectory[index];
    const auto &rhs = desiredTrajectory[index + 1];
    const Eigen::Quaterniond quaternionA = SmbConversions::readRotation(lhs);
    const Eigen::Quaterniond quaternionB = SmbConversions::readRotation(rhs);

    reference.head(3) = alpha * SmbConversions::readPosition(lhs) + (1.0 - alpha) * SmbConversions::readPosition(rhs);
    reference.tail(4) = (quaternionA.slerp((1 - alpha), quaternionB)).coeffs();
  }
  else
  { // desiredTrajectory.size() == 1
    reference.head(3) = SmbConversions::readPosition(desiredTrajectory[0]);
    reference.tail(4) = SmbConversions::readRotation(desiredTrajectory[0]).coeffs();
  }

  return reference;
}

SmbCost *SmbCost::clone() const
{
  return new SmbCost(*this);
}
