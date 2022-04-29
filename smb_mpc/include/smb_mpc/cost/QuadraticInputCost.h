#pragma once

#include <ocs2_core/cost/QuadraticStateInputCost.h>

using namespace ocs2;
namespace smb_mpc {

class QuadraticInputCost final : public QuadraticStateInputCost {
 public:
  QuadraticInputCost(matrix_t R, size_t stateDim)
      : QuadraticStateInputCost(matrix_t::Zero(stateDim, stateDim), std::move(R)), stateDim_(stateDim) {}

  ~QuadraticInputCost() override = default;

  QuadraticInputCost(const QuadraticInputCost& rhs) = default;
  QuadraticInputCost* clone() const override { return new QuadraticInputCost(*this); }

  std::pair<vector_t, vector_t> getStateInputDeviation(scalar_t time, const vector_t& state, const vector_t& input,
                                                       const TargetTrajectories& targetTrajectories) const override {
    const vector_t inputDeviation = input - targetTrajectories.getDesiredInput(time);
    return {vector_t::Zero(stateDim_), inputDeviation};
  }

 private:
  const size_t stateDim_;
};

}  // namespace smb_mpc