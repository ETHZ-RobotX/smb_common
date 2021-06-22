//
// Created by johannes on 29.04.19.
//

#pragma once

#include <memory>

#include <ocs2_core/initialization/OperatingPoints.h>
#include "smb_mpc/SmbDefinitions.h"

namespace smb_path_following {

class SmbOperatingPoint : public ocs2::OperatingPoints {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<SmbOperatingPoint> Ptr;
  typedef std::shared_ptr<const SmbOperatingPoint> ConstPtr;

  typedef ocs2::OperatingPoints Base;

  /**
   * Constructor
   *
   */
  SmbOperatingPoint();

  /**
   * Destructor
   */
  ~SmbOperatingPoint() = default;

  /**
   * Returns pointer to the class.
   *
   * @return A raw pointer to the class.
   */
  SmbOperatingPoint* clone() const final { return new SmbOperatingPoint(*this); }

  /**
   * Gets the operating points for the system in time interval [startTime, finalTime] where there is
   * no intermediate switches except possibly the end time.
   *
   * @param [in] initialState: Initial state.
   * @param [in] startTime: Initial time.
   * @param [in] finalTime: Final time.
   * @param [out] timeTrajectory: Output time stamp trajectory.
   * @param [out] stateTrajectory: Output state trajectory.
   * @param [out] inputTrajectory: Output control input trajectory.
   * @param [in] concatOutput: Whether to concatenate the output to the input trajectories or
   * override (default).
   */
  void getSystemOperatingTrajectories(const ocs2::vector_t& initialState, ocs2::scalar_t startTime, ocs2::scalar_t finalTime,
                                      ocs2::scalar_array_t& timeTrajectory, ocs2::vector_array_t& stateTrajectory,
                                      ocs2::vector_array_t& inputTrajectory, bool concatOutput = false) final;

 private:
};

}  // namespace smb_path_following
