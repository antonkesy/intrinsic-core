// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "intrinsic/math/linear_systems/discrete_algebraic_riccati_equation.h"

#include <limits>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/linear_systems/dynamic_riccati_equation.h"

namespace intrinsic {

absl::StatusOr<DiscreteAlgebraicRiccatiEquationSolution>
SolveDiscreteAlgebraicRiccatiEquationIteratively(
    const eigenmath::MatrixXd& P_init, const eigenmath::MatrixXd& Q,
    const eigenmath::MatrixXd& R, const eigenmath::MatrixXd& A,
    const eigenmath::MatrixXd& B,
    const DiscreteAlgebraicRiccatiSolverSettings& settings) {
  DiscreteAlgebraicRiccatiEquationSolution solution;
  eigenmath::MatrixXd P_prev;

  solution.P = P_init;
  solution.step_size_infinity_norm = std::numeric_limits<double>::max();
  solution.iteration_count = 0;

  while (solution.step_size_infinity_norm > settings.accuracy &&
         solution.iteration_count < settings.max_iterations) {
    P_prev = solution.P;

    if (settings.strategy == DiscreteAlgebraicRiccatiSolverSettings::kRobust) {
      auto status = ComputeRobustDynamicRiccatiEquationIterate(
          P_prev, Q, R, A, B, settings.regularizer_epsilon, solution.P,
          solution.K);
      if (!status.ok()) {
        return absl::Status(
            status.code(),
            absl::StrCat("DiscreteAlgebraicRiccatiEquation failed because "
                         "ComputeRobustDynamicRiccatiEquationIterate() failed "
                         "with error: ",
                         status.message()));
      }
    } else {
      auto status = ComputeNaiveDynamicRiccatiEquationIterate(
          P_prev, Q, R, A, B, solution.P, solution.K);
      if (!status.ok()) {
        return absl::Status(
            status.code(),
            absl::StrCat("DiscreteAlgebraicRiccatiEquation failed because "
                         "ComputeNaiveDynamicRiccatiEquationIterate() failed "
                         "with error: ",
                         status.message()));
      }
    }

    if (!solution.K.allFinite()) {
      return absl::InternalError(
          "DiscreteAlgebraicRiccatiEquation failed to converge, instability "
          "detected.");
    }

    solution.step_size_infinity_norm =
        (solution.P - P_prev).lpNorm<Eigen::Infinity>();
    solution.iteration_count++;
  }

  if (solution.step_size_infinity_norm > settings.accuracy) {
    return absl::InternalError(absl::StrCat(
        "DiscreteAlgebraicRiccatiEquation failed to converge, iteration limit "
        "reached, current step size infinity norm: ",
        solution.step_size_infinity_norm));
  }

  return solution;
}

}  // namespace intrinsic
