#include <string>

#include "smb_mpc/SmbInterface.h"
#include "smb_mpc/SmbSystemDynamics.h"
#include "smb_mpc/cost/QuadraticInputCost.h"

#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

using namespace ocs2;

namespace smb_mpc
{

  SmbInterface::SmbInterface(const std::string &taskFile, const std::string &libraryFolder,
                             const std::string &urdfFile)
  {
    // check that task file exists
    boost::filesystem::path taskFilePath(taskFile);
    if (boost::filesystem::exists(taskFilePath))
    {
      std::cerr << "[SmbInterface] Loading task file: " << taskFilePath << std::endl;
    }
    else
    {
      throw std::invalid_argument("[SmbInterface] Task file not found: " + taskFilePath.string());
    }

    // create library folder if it does not exist
    boost::filesystem::path libraryFolderPath(libraryFolder);
    boost::filesystem::create_directories(libraryFolderPath);
    std::cerr << "[SmbInterface] Generated library path: " << libraryFolderPath << std::endl;

    // read the task file
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);

    bool recompileLibraries = true;
    std::cerr << "\n #### Model Settings:";
    std::cerr << "\n #### =============================================================================\n";
    loadData::loadPtreeValue(pt, recompileLibraries, "model_settings.recompileLibraries", true);
    std::cerr << " #### =============================================================================\n";

    // Default initial state
    initialState_.setZero(SmbDefinitions::STATE_DIM);

    // DDP-MPC settings
    ddpSettings_ = ddp::loadSettings(taskFile, "ddp");
    mpcSettings_ = mpc::loadSettings(taskFile, "mpc");

    // Reference Manager
    referenceManagerPtr_.reset(new ReferenceManager);

    /*
     * Optimal control problem
     */
    // Cost
    problem_.costPtr->add("inputCost", getQuadraticInputCost(taskFile));

    problem_.dynamicsPtr.reset(new SmbSystemDynamics("dynamics", libraryFolder, recompileLibraries, true));


    // Rollout
    const auto rolloutSettings = rollout::loadSettings(taskFile, "rollout");
    rolloutPtr_.reset(new TimeTriggeredRollout(*problem_.dynamicsPtr, rolloutSettings));

    // Initialization
    initializerPtr_.reset(new DefaultInitializer(SmbDefinitions::INPUT_DIM));
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::unique_ptr<StateInputCost> SmbInterface::getQuadraticInputCost(const std::string &taskFile)
  {
    matrix_t R = matrix_t::Zero(SmbDefinitions::INPUT_DIM, SmbDefinitions::INPUT_DIM);

    loadData::loadEigenMatrix(taskFile, "inputCost.R", R);

    return std::unique_ptr<StateInputCost>(new QuadraticInputCost(std::move(R), SmbDefinitions::STATE_DIM));
  }

}