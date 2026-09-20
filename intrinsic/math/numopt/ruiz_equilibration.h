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

#ifndef INTRINSIC_MATH_NUMOPT_RUIZ_EQUILIBRATION_H_
#define INTRINSIC_MATH_NUMOPT_RUIZ_EQUILIBRATION_H_

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"

namespace intrinsic {

// Default maximum number of iterations for Ruiz Equilibration.
constexpr int kMaximumIterationsForRuizEquilibration = 5;

// Contains the scaled problem data resulting from Ruiz equilibration. This
// struct holds the well-conditioned matrices and vectors to be passed to the
// numerical solver. It also contains the accumulated column scaling factors,
// which are strictly required to unscale the solver's resulting primal
// variables back to the original problem space.
struct RuizEquilibrationResult {
  // The scaled linear objective vector.
  eigenmath::VectorXd c;

  // The scaled inequality constraint matrix (`A_ineq * x <= b_ineq`).
  eigenmath::MatrixXd A_ineq;

  // The scaled inequality constraint bounds.
  eigenmath::VectorXd b_ineq;

  // The scaled equality constraint matrix (`A_eq * x = b_eq`).
  eigenmath::MatrixXd A_eq;

  // The scaled equality constraint bounds.
  eigenmath::VectorXd b_eq;

  // The accumulated scaling factors applied to the columns (variables).
  // To recover the original unscaled primal solution `x_orig` from the solver's
  // scaled solution `x_scaled`, apply:
  //       `x_orig = x_scaled.cwiseProduct(columns_scaling);`
  eigenmath::VectorXd columns_scaling;
};

// Applies Ruiz equilibration (infinity-norm scaling) to precondition a linear
// programming problem. This algorithm iteratively scales the rows and columns
// of the constraint matrices by the square root of their maximum absolute
// values. This drives the maximum element of every row and column toward 1.0,
// improving the condition number of the matrices and increasing solver
// stability.
//
// The objective vector `c` and bound vectors `b_ineq`, `b_eq` are scaled to
// mathematically match the matrix transformations, but they do not influence
// the calculation of the scaling factors.
//
// - `c` is the original linear objective vector.
// - `A_ineq` is the original inequality constraint matrix.
// - `b_ineq` is the original inequality bounds vector.
// - `A_eq` is the original equality constraint matrix.
// - `b_eq` is the original equality bounds vector.
// - `max_iterations` is the number of scaling iterations to perform. Typically
//   5 to 15 iterations are sufficient for optimal conditioning.
//
//  Returns a `RuizEquilibrationResult` containing the scaled variables on
//  success, or an error if matrix/vector dimensions are mismatched or
//  `max_iterations` is invalid (<= 0).
absl::StatusOr<RuizEquilibrationResult> ApplyRuizEquilibration(
    Eigen::Ref<const eigenmath::VectorXd> c,
    Eigen::Ref<const eigenmath::MatrixXd> A_ineq,
    Eigen::Ref<const eigenmath::VectorXd> b_ineq,
    Eigen::Ref<const eigenmath::MatrixXd> A_eq,
    Eigen::Ref<const eigenmath::VectorXd> b_eq,
    int max_iterations = kMaximumIterationsForRuizEquilibration);

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_NUMOPT_RUIZ_EQUILIBRATION_H_
