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

#ifndef INTRINSIC_MATH_NUMOPT_SIMPLEX_H_
#define INTRINSIC_MATH_NUMOPT_SIMPLEX_H_

#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/numopt/ruiz_equilibration.h"

namespace intrinsic {

// Infinity value used in the simplex method. Constraints whose right hand side
// is larger or equal than this value are considered unbounded.
static constexpr double kSimplexInfinity = 1.0e30;

// Possible termination statuses of the simplex method.
enum class SimplexStatus {
  kOptimal,     // The problem was solved to optimality.
  kUnbounded,   // The problem has no solution, as the objective value can
                // increase without bound.
  kInfeasible,  // No solution could be found that satisfies all constraints.
  kMaxIterationsReached,  // The maximum number of iterations was reached.
  kInternalFailure,
};

// Result of the simplex method. It contains the termination status of the
// simplex algorithm on the input problem, the objective value and the optimal
// solution vector. Note that the `solution` vector is optimal if the
// termination status is `kOptimal`, it is feasible if the termination status is
// only `kMaxIterationsReached`. The user should check the termination status
// to determine if the solution can be used or not.
struct SimplexResult {
  SimplexStatus status;
  double objective_value;
  eigenmath::VectorXd solution;
};

// Options to configure what the Simplex method does.
struct SimplexOptions {
  // Maximum allowed number of Simplex iterations.
  static constexpr int kDefaultMaxIterations = 1000;
  int max_iterations = kDefaultMaxIterations;

  // List of the optimization variables' indices not subject to the constraint
  // `x >= 0` (typically enforced by the Simplex solver to all variables).
  std::vector<int> free_primal_variables = {};

  // The floating-point tolerance used to evaluate if Phase 1 successfully drove
  // all artificial variables to zero. If the Phase 1 objective value is greater
  // than or equal to this threshold, the problem is considered infeasible.
  static constexpr double kPhaseOneFeasibilityTolerance = 1.0e-13;
  double phase_one_feasibility_tolerance = kPhaseOneFeasibilityTolerance;

  // If not `nullopt`, enforces running `num_ruiz_equilibration_iterations` to
  // improve the conditioning of the constraints geometry of the optimization
  // problem.
  std::optional<int> num_ruiz_equilibration_iterations =
      kMaximumIterationsForRuizEquilibration;
};

// Solves using the simplex method the following linear programming problem
//   maximize    `c^T * x`
//   subject to  `A * x <= b`
//              `x >= 0`
// where `c` is the objective vector, `A` is the constraint matrix and `b` is
// the constraint vector. The `options` contain indices of free variables, which
// are variables that are not bounded and can take any value. This means that
// for those variables the constraint `x >= 0` is not enforced. To make that
// happen, the free variables are replaced by a difference of non-negative
// variables, which implies adding additional non-negative variables to the
// constraint matrix `A` and the objective vector `c`.
// Returns the termination status, objective value and optimal solution vector.
absl::StatusOr<SimplexResult> SimplexSolve(
    Eigen::Ref<const eigenmath::VectorXd> c,
    Eigen::Ref<const eigenmath::MatrixXd> A,
    Eigen::Ref<const eigenmath::VectorXd> b,
    const SimplexOptions& options = {});

// Solves linear programming problems with both inequality and equality
// constraints:
//   maximize    `c^T * x`
//   subject to  `A_ineq * x <= b_ineq`
//               `A_eq * x == b_eq`
//               `x >= 0`
absl::StatusOr<SimplexResult> SimplexSolve(
    Eigen::Ref<const eigenmath::VectorXd> c,
    Eigen::Ref<const eigenmath::MatrixXd> A_ineq,
    Eigen::Ref<const eigenmath::VectorXd> b_ineq,
    Eigen::Ref<const eigenmath::MatrixXd> A_eq,
    Eigen::Ref<const eigenmath::VectorXd> b_eq,
    const SimplexOptions& options = {});

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_NUMOPT_SIMPLEX_H_
