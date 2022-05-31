//
// Created by johannes on 01.05.19.
//
#include <smb_mpc/SmbSystemDynamics.h>

using namespace ocs2;

namespace smb_mpc
{
  SmbSystemDynamics::SmbSystemDynamics(const std::string &modelName,
                                       const std::string &modelFolder, bool recompileLibraries, bool verbose)
  {
    this->initialize(SmbDefinitions::STATE_DIM, SmbDefinitions::INPUT_DIM, modelName, modelFolder, recompileLibraries, verbose);
  }
  ad_vector_t SmbSystemDynamics::systemFlowMap(ocs2::ad_scalar_t time, const ocs2::ad_vector_t &state, const ocs2::ad_vector_t &input,
                                               const ocs2::ad_vector_t &parameters) const
  {
    ad_vector_t stateDerivative(SmbDefinitions::STATE_DIM);

    using ad_quat_t = Eigen::Quaternion<ad_scalar_t>;

    ad_scalar_t linearVelocity = SmbConversions::readLinVel(input);
    ad_scalar_t omega = SmbConversions::readAngVel(input);

    ad_quat_t currentRotation = SmbConversions::readRotation(state);

    // derivative of orientation quaternion: https://fgiesen.wordpress.com/2012/08/24/quaternion-differentiation/
    ad_quat_t deltaRotation;
    deltaRotation.w() = 0;
    deltaRotation.x() = 0;
    deltaRotation.y() = 0;
    deltaRotation.z() = omega / 2;
    stateDerivative.tail(4) = (currentRotation * deltaRotation).coeffs();

    ad_vector_t linearVelocityVector = ad_vector_t::Zero(3);
    linearVelocityVector[0] = linearVelocity;
    stateDerivative.head(3) = currentRotation * linearVelocityVector;
    // stateDerivative[2] = 0 // force derivative in z direction to 0

    return stateDerivative;
  }
} // namespace smb_mpc
