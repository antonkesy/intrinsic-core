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

#ifndef INTRINSIC_MATH_LINEAR_SYSTEMS_DISCRETE_ALGEBRAIC_RICCATI_EQUATION_H_
#define INTRINSIC_MATH_LINEAR_SYSTEMS_DISCRETE_ALGEBRAIC_RICCATI_EQUATION_H_

#include <cstddef>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"

namespace intrinsic {

struct DiscreteAlgebraicRiccatiSolverSettings {
  enum IterationStrategy { kRobust, kNaive };

  IterationStrategy strategy = kRobust;
  size_t max_iterations = 250;  // Number of iterations the solver attempts.
  double accuracy = 1e-4;  // Convergence criterion for P-matrix infinity norm.
  double regularizer_epsilon =
      1e-5;  // Hessian regularizer offset, ignored if strategy == kNaive.
};

struct DiscreteAlgebraicRiccatiEquationSolution {
  // Covariance matrix.
  eigenmath::MatrixXd P;  // NOLINT(google3-readability-class-member-naming).
  // Gain matrix.
  eigenmath::MatrixXd K;  // NOLINT(google3-readability-class-member-naming).
  size_t iteration_count = 0;
  double step_size_infinity_norm = -1;
};

// Solves the Discrete Algebraic Riccati Equation iteratively. Solves the system
// of equations
//
// P = Q + A^T ( P - P B ( R + B^T P B )^-1 B^T P) A.
//   = Q + A^T ( P - P B H_inverse B^T P) A
//   = Q + A^T S A
// where H_inverse = (R + B^T P B)^-1 and S = P - P B H_inverse B^T * P using
// recursions over the Dynamic Riccati Equation.
//
// The solver parameters can be `settings`. Takes an initial guess for a Riccati
// matrix P_init, process state weighting Q, input/output weighting R, and
// linear system matrices A and B.
//
// See R.F. Stengel, "Optimal Control and Estimation", Chapter 6.
//
// Returns a solution struct with Riccati matrix P and gain matrix object K.
// Returns kFailedPrecondition in case of dimension mismatches, returns
// kInternalError in case of numerical instability or exceeding admissible
// iteration count.
absl::StatusOr<DiscreteAlgebraicRiccatiEquationSolution>
SolveDiscreteAlgebraicRiccatiEquationIteratively(
    const eigenmath::MatrixXd& P_init, const eigenmath::MatrixXd& Q,
    const eigenmath::MatrixXd& R, const eigenmath::MatrixXd& A,
    const eigenmath::MatrixXd& B,
    const DiscreteAlgebraicRiccatiSolverSettings& settings);

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_LINEAR_SYSTEMS_DISCRETE_ALGEBRAIC_RICCATI_EQUATION_H_
