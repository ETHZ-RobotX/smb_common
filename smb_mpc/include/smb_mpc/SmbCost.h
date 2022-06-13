#include <utility>

/*
 * WacoWeightedCost.h
 *
 *  Created on: May 1, 2019
 *      Author: Johannes Pankert
 */

#pragma once

#include "smb_mpc/SmbDefinitions.h"
#include "smb_mpc/SmbConversions.h"
#include <ocs2_core/cost/StateInputGaussNewtonCostAd.h>

#include <cmath>

// CPPAD stuff
#include "cppad/cg/math.hpp"

namespace smb_mpc
{

  class SmbCost : public ocs2::StateInputCostGaussNewtonAd
  {
  public:
    using vector_t = ocs2::vector_t;
    using scalar_t = ocs2::scalar_t;

    using ad_vector_t = ocs2::ad_vector_t;
    using ad_scalar_t = ocs2::ad_scalar_t;

    typedef std::shared_ptr<SmbCost> Ptr;
    typedef std::shared_ptr<const SmbCost> ConstPtr;

    using Base = ocs2::StateInputCostGaussNewtonAd;

    /**
     * Constructor
     */
    SmbCost(ocs2::matrix_t QPosition, ocs2::matrix_t QOrientation, ocs2::matrix_t R, const std::string &libraryFolder, bool recompileLibraries);

    /**
     * Copy constructor
     * @param rhs
     */
    SmbCost(const SmbCost &rhs) = default;

    /**
     * Default destructor
     */
    ~SmbCost() override = default;

    /** Get the parameter vector */
    vector_t getParameters(scalar_t time, const ocs2::TargetTrajectories &desiredTrajectory) const override;

  protected:
    ad_vector_t costVectorFunction(ad_scalar_t time, const ad_vector_t &state,
                                   const ad_vector_t &input,
                                   const ad_vector_t &parameters) const override;

    SmbCost *clone() const override;

  private:
    ocs2::matrix_t QPosition_;
    ocs2::matrix_t QOrientation_;
    ocs2::matrix_t R_;
  };

} // namespace smb_path_following
