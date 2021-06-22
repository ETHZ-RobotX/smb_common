#pragma once

#include <ocs2_core/cost/CostCollection.h>
#include <ocs2_core/cost/CostFunctionBase.h>

#include <smb_mpc/SmbCost.h>

namespace smb_path_following {

class SmbCostWrapper : public ocs2::CostFunctionBase {
public:
  using scalar_t = ocs2::scalar_t;
  using vector_t = ocs2::vector_t;
  using ScalarFunctionQuadraticApproximation =
      ocs2::ScalarFunctionQuadraticApproximation;

  SmbCostWrapper(std::unique_ptr<SmbCost> cost);

  ~SmbCostWrapper() override = default;
  SmbCostWrapper *clone() const override { return new SmbCostWrapper(*this); }

  scalar_t cost(scalar_t t, const vector_t &x, const vector_t &u) override;
  scalar_t finalCost(scalar_t t, const vector_t &x) override;
  ScalarFunctionQuadraticApproximation
  costQuadraticApproximation(scalar_t t, const vector_t &x,
                             const vector_t &u) override;
  ScalarFunctionQuadraticApproximation
  finalCostQuadraticApproximation(scalar_t t, const vector_t &x) override;

private:
  ocs2::CostCollection<ocs2::StateInputCost> costCollection_;
};
} // namespace smb_path_following
