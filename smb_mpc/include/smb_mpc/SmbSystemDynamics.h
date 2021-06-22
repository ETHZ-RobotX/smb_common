//
// Created by johannes on 29.04.19.
//

#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>

#include <cppad/cg.hpp>
#include "smb_mpc/SmbDefinitions.h"

namespace smb_path_following {

class SmbSystemDynamics : public ocs2::SystemDynamicsBaseAD{
 public:

  using Base = ocs2::SystemDynamicsBaseAD;
  using ad_scalar_t = ocs2::ad_scalar_t;
  using ad_vector_t = ocs2::ad_vector_t;

  /**
   * Constructor
   *
   * @param [in] dynamicLibraryIsCompiled: Whether the library has been already complied.
   */
  SmbSystemDynamics() : Base(SmbDefinitions::STATE_DIM, SmbDefinitions::INPUT_DIM) {}

  /**
   * Destructor
   */
  ~SmbSystemDynamics() = default;

  /**
   * Interface method to the state flow map of the hybrid system. This method should be implemented by the derived class.
   *
   * @tparam scalar type. All the floating point operations should be with this type.
   * @param [in] time: time.
   * @param [in] state: state vector.
   * @param [in] input: input vector
   * @param [out] stateDerivative: state vector time derivative.
   */
  ad_vector_t systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                            const ad_vector_t& parameters) const override;

  SmbSystemDynamics* clone() const override;
};

}  // namespace smb_path_following
