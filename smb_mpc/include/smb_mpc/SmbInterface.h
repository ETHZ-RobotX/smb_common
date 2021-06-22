//
// Created by johannes on 29.04.19.
//

#pragma once

// C++
#include <iostream>
#include <stdlib.h>
#include <string>

// OCS2
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_mpc/MPC_DDP.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>

// MM
#include "smb_mpc/SmbDefinitions.h"
#include "smb_mpc/SmbModelSettings.h"
#include <smb_mpc/SmbOperatingPoint.h>

// ros
#include <ros/package.h>
#include <ros/ros.h>

namespace smb_path_following {

class SmbInterface : public ocs2::RobotInterface {
public:
  using operating_point_t = SmbOperatingPoint;

  using constraint_t = ocs2::ConstraintBase;
  using rollout_base_t = ocs2::RolloutBase;
  using time_triggered_rollout_t = ocs2::TimeTriggeredRollout;

  /**
   * Constructor
   */
  SmbInterface();

  /**
   * Destructor
   */
  ~SmbInterface() = default;

  /**
   * setup all optimizes.
   *
   */
  void setupOptimizer(const std::string &taskFile) { resetMpc(); };

  /**
   * reset the mpc module
   */
  void resetMpc();

  /**
   * Gets a pointer to the internal SLQ-MPC class.
   *
   * @return Pointer to the internal MPC
   */
  std::unique_ptr<ocs2::MPC_DDP> &getMPCPtr();

  ocs2::mpc::Settings &mpcSettings();

  const ocs2::SystemDynamicsBase &getDynamics() const override {
    return *dynamicsPtr_;
  }

  const ocs2::CostFunctionBase &getCost() const override { return *costPtr_; }

  const operating_point_t &getOperatingPoints() const override {
    return *operatingPointPtr_;
  }

  /**
   * Get the model settings.
   *
   * @return Model settings.
   */
  SmbModelSettings &modelSettings();

protected:
  /**
   * Loads the settings from the path file.
   *
   * @param [in] taskFile: Task's file full path.
   */
  void loadSettings(const std::string &taskFile);

  void definePartitioningTimes(const std::string &taskFile,
                               ocs2::scalar_t &timeHorizon,
                               size_t &numPartitions,
                               ocs2::scalar_array_t &partitioningTimes,
                               bool verbose = false);

  void loadMpcTimeHorizon(const std::string &taskFile,
                          ocs2::scalar_t &timeHorizon, size_t &numPartitions,
                          bool verbose = false);

public:
  std::string taskFile_;
  std::string libraryFolder_;

  std::unique_ptr<ocs2::MPC_DDP> mpcPtr_;

  SmbModelSettings modelSettings_;

  std::unique_ptr<rollout_base_t> ddpSmbRolloutPtr_;
  std::unique_ptr<ocs2::SystemDynamicsBase> dynamicsPtr_;
  std::unique_ptr<ocs2::CostFunctionBase> costPtr_;
  SmbOperatingPoint::Ptr operatingPointPtr_;
  std::unique_ptr<constraint_t> constraintPtr_;

  size_t numPartitions_ = 0;
  /*
   * Time partitioning which defines the time horizon and the number of data
   * partitioning
   */
  ocs2::scalar_t timeHorizon_ = 1;
  ocs2::scalar_array_t partitioningTimes_;

  ocs2::mpc::Settings mpcSettings_;
  ocs2::ddp::Settings ddpSettings_;
};

} // namespace smb_path_following
