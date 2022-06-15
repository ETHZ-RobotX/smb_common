#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>

#include "smb_mpc/SmbDefinitions.h"

namespace smb_mpc
{
  using namespace ocs2;

  class SmbInterface final : public RobotInterface
  {
  public:
    SmbInterface(const std::string &taskFile, const std::string &libraryFolder);

    const vector_t &getInitialState() { return initialState_; }

    ddp::Settings &ddpSettings() { return ddpSettings_; }

    mpc::Settings &mpcSettings() { return mpcSettings_; }

    const OptimalControlProblem &getOptimalControlProblem() const override { return problem_; }

    std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }

    const Initializer &getInitializer() const override { return *initializerPtr_; }

    const RolloutBase &getRollout() const { return *rolloutPtr_; }

  private:
    std::unique_ptr<StateInputCost> getPositionCost(const std::string &taskFile, const std::string &libraryFolder, bool recompileLibraries);

    ddp::Settings ddpSettings_;
    mpc::Settings mpcSettings_;

    OptimalControlProblem problem_;
    std::shared_ptr<ReferenceManager> referenceManagerPtr_;

    std::unique_ptr<RolloutBase> rolloutPtr_;
    std::unique_ptr<Initializer> initializerPtr_;

    vector_t initialState_;
  };

}