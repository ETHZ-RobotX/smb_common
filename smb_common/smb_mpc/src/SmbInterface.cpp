#include "smb_mpc/SmbInterface.h"

#include "smb_mpc/SmbCostWrapper.h"
#include "smb_mpc/SmbCost.h"
#include "smb_mpc/SmbSystemDynamics.h"

namespace smb_path_following {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SmbInterface::SmbInterface() {
  std::string packagePath =
      ros::package::getPath("smb_mpc");

  taskFile_ = packagePath + "/config/task.info";
  std::cerr << "Loading task file: " << taskFile_ << std::endl;

  libraryFolder_ = packagePath + "/auto_generated";
  std::cerr << "Generated library path: " << libraryFolder_ << std::endl;

  // load setting from loading file
  loadSettings(taskFile_);

  // MPC
  resetMpc();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SmbInterface::loadSettings(const std::string &taskFile) {
  /*
   * Default initial condition
   */

  /*
   * SLQ-MPC settings
   */
  ddpSettings_ = ocs2::ddp::loadSettings(taskFile);
  mpcSettings_ = ocs2::mpc::loadSettings(taskFile);

  /*
   * Model settings
   */
  modelSettings_.loadSettings(taskFile_, true);

  /*
   * Dynamics
   */
  auto dynamics = std::make_unique<SmbSystemDynamics>();
  std::cerr << "The system model name is " << modelSettings_.systemName_
            << std::endl;
  dynamics->initialize(modelSettings_.systemName_, "tmp/ocs2",
                       modelSettings_.recompileLibraries_, true);
  dynamicsPtr_ = std::move(dynamics);

  /*
   * Rollout
   */
  ocs2::rollout::Settings rolloutSettings =
      ocs2::rollout::loadSettings(taskFile, "rollout");
  ddpSmbRolloutPtr_.reset(
      new time_triggered_rollout_t(*dynamicsPtr_, rolloutSettings));

  /*
   * Cost function
   */

  // joint space tracking cost
  ocs2::matrix_t QPosition(3, 3);
  ocs2::matrix_t QOrientation(3, 3);
  ocs2::matrix_t R(SmbDefinitions::INPUT_DIM, SmbDefinitions::INPUT_DIM);

  ocs2::loadData::loadEigenMatrix(taskFile, "QPosition", QPosition);
  ocs2::loadData::loadEigenMatrix(taskFile, "QOrientation", QOrientation);
  ocs2::loadData::loadEigenMatrix(taskFile, "R", R);

  std::cerr << "QPosition:       \n" << QPosition << std::endl;
  std::cerr << "QOrientation:       \n" << QOrientation << std::endl;
  std::cerr << "R: \n" << R << std::endl;

  auto baseTrackingCost =
      std::make_unique<SmbCost>(QPosition, QOrientation, R);

  baseTrackingCost->initialize(7, 2, 7, "base_tracking_cost", "tmp/ocs2",
                               modelSettings_.recompileLibraries_);

  costPtr_ = std::make_unique<SmbCostWrapper>(std::move(baseTrackingCost));

  constraintPtr_.reset(new constraint_t());

  operatingPointPtr_.reset(new SmbOperatingPoint());

  definePartitioningTimes(taskFile, timeHorizon_, numPartitions_,
                          partitioningTimes_, true);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SmbInterface::resetMpc() {
  mpcPtr_.reset(
      new ocs2::MPC_DDP(ddpSmbRolloutPtr_.get(), dynamicsPtr_.get(),
                        constraintPtr_.get(), costPtr_.get(),
                        operatingPointPtr_.get(), ddpSettings_, mpcSettings_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<ocs2::MPC_DDP> &SmbInterface::getMPCPtr() {
  return mpcPtr_;
}

ocs2::mpc::Settings &SmbInterface::mpcSettings() {
  return mpcSettings_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SmbModelSettings &SmbInterface::modelSettings() {
  return modelSettings_;
}

void SmbInterface::definePartitioningTimes(
    const std::string &taskFile, ocs2::scalar_t &timeHorizon,
    size_t &numPartitions, ocs2::scalar_array_t &partitioningTimes,
    bool verbose /*= false*/) {
  // load from task file
  loadMpcTimeHorizon(taskFile, timeHorizon, numPartitions);

  if (numPartitions == 0) {
    throw std::runtime_error("mpcTimeHorizon field is not defined.");
  }

  partitioningTimes.resize(numPartitions + 1);
  partitioningTimes[0] = 0.0;
  for (size_t i = 0; i < numPartitions; i++) {
    partitioningTimes[i + 1] =
        partitioningTimes[i] + timeHorizon / numPartitions;
  }
  partitioningTimes[numPartitions] = timeHorizon;
}

void SmbInterface::loadMpcTimeHorizon(const std::string &taskFile,
                                                   ocs2::scalar_t &timeHorizon,
                                                   size_t &numPartitions,
                                                   bool verbose /*= false*/) {
  ocs2::loadData::loadCppDataType(taskFile, "mpc.timeHorizon",
                                  timeHorizon);
  ocs2::loadData::loadCppDataType(taskFile, "mpc.numPartitions",
                                  numPartitions);

  if (true) {
    std::cerr << "Time Horizon Settings: " << std::endl;
    std::cerr << "=====================================" << std::endl;
    std::cerr << "Time Horizon .................. " << timeHorizon << std::endl;
    std::cerr << "Number of Partitions .......... " << numPartitions
              << std::endl
              << std::endl;
  }
}

} // namespace smb_path_following
