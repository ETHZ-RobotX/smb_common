//
// Created by johannes on 02.05.19.
//
#include <smb_mpc/SmbOperatingPoint.h>

using namespace smb_path_following;
using namespace ocs2;

SmbOperatingPoint::SmbOperatingPoint()
    : Base(SmbDefinitions::defaultState(),
           vector_t::Zero(SmbDefinitions::INPUT_DIM)) {}

void SmbOperatingPoint::getSystemOperatingTrajectories(
    const ocs2::vector_t &initialState, ocs2::scalar_t startTime,
    ocs2::scalar_t finalTime, ocs2::scalar_array_t &timeTrajectory,
    ocs2::vector_array_t &stateTrajectory,
    ocs2::vector_array_t &inputTrajectory, bool concatOutput) {
  if (concatOutput == false) {
    timeTrajectory.clear();
    stateTrajectory.clear();
    inputTrajectory.clear();
  }

  timeTrajectory.emplace_back(startTime);
  timeTrajectory.emplace_back(finalTime);

  stateTrajectory.emplace_back(initialState);
  stateTrajectory.emplace_back(initialState);

  inputTrajectory.emplace_back(SmbDefinitions::INPUT_DIM);
  inputTrajectory.emplace_back(SmbDefinitions::INPUT_DIM);
}
