//
// Created by johannes on 29.04.19.
//

#include <smb_mpc/SmbDefinitions.h>
namespace smb_mpc {
const size_t SmbDefinitions::STATE_DIM;
const size_t SmbDefinitions::INPUT_DIM;
const size_t SmbDefinitions::REFERENCE_STATE_DIM_;

Eigen::VectorXd SmbDefinitions::defaultState() {
  Eigen::VectorXd defaultState =
      Eigen::VectorXd::Zero(SmbDefinitions::STATE_DIM);
  defaultState.tail(4) = Eigen::Quaterniond::Identity().coeffs();
  return defaultState;
}

} // namespace smb_path_following
