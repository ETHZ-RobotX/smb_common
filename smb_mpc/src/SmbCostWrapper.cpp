#include <smb_mpc/SmbCostWrapper.h>

using namespace smb_path_following;
using namespace ocs2;

SmbCostWrapper::SmbCostWrapper(std::unique_ptr<SmbCost> cost)
    : costCollection_() {
  costCollection_.add("tracking_cost", std::move(cost));
}

scalar_t SmbCostWrapper::cost(scalar_t t, const vector_t &x,
                              const vector_t &u) {
  return costCollection_.getValue(t, x, u, *costDesiredTrajectoriesPtr_);
}

scalar_t SmbCostWrapper::finalCost(scalar_t t, const vector_t &x) { return 0; }

ScalarFunctionQuadraticApproximation
SmbCostWrapper::costQuadraticApproximation(scalar_t t, const vector_t &x,
                                           const vector_t &u) {
  return costCollection_.getQuadraticApproximation(
      t, x, u, *costDesiredTrajectoriesPtr_);
}

ScalarFunctionQuadraticApproximation
SmbCostWrapper::finalCostQuadraticApproximation(scalar_t t, const vector_t &x) {
  return ScalarFunctionQuadraticApproximation::Zero(x.size(), 0);
}