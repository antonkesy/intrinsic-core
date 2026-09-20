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

#ifndef INTRINSIC_MATH_LINEAR_SYSTEMS_DYNAMIC_RICCATI_EQUATION_H_
#define INTRINSIC_MATH_LINEAR_SYSTEMS_DYNAMIC_RICCATI_EQUATION_H_

#include "absl/status/status.h"
#include "intrinsic/eigenmath/types.h"

namespace intrinsic {

// Evaluates the discrete-time Dynamic Riccati Equation using naive, direct
// implementation. Evaluates the left-hand-side of
// P = Q + A^T ( P - P B ( R + B^T P B )^-1 B^T P) A.
//   = Q + A^T ( P - P B H_inverse B^T P) A
//   = Q + A^T S A
// where H_inverse = (R + B^T P B)^-1 and S = P - P B H_inverse B^T * P.
// Returns P and feedback gain K = -Hinverse B^T P A. Returns
// FailedPreconditionError in case of dimensional mismatches. See R.F. Stengel,
// "Optimal Control and Estimation", Chapter 6, or
// https://en.wikipedia.org/wiki/Algebraic_Riccati_equation
absl::Status ComputeNaiveDynamicRiccatiEquationIterate(
    const eigenmath::MatrixXd& P, const eigenmath::MatrixXd& Q,
    const eigenmath::MatrixXd& R, const eigenmath::MatrixXd& A,
    const eigenmath::MatrixXd& B, eigenmath::MatrixXd& P_next,
    eigenmath::MatrixXd& K);

// Evaluates the discrete-time Dynamic Riccati Equation including a Hessian
// matrix regularization with damping parameter 'epsilon'. Computes P = Q + A^T
// ( P - P B ( R + B^T P B )^-1 B^T P) A.
//            = Q + A^T ( P - P B H_inverse B^T P) A
//            = Q + A^T S A
// where H_inverse = (R + B^T P B)^-1 and S = P - P B H_inverse B^T * P in a
// robust manner. This method is computationally more expensive than the naive
// iterate but is more reliable for badly conditioned control problems. //
// Returns P and feedback gain K = -Hinverse B^T P A. The Hessian regularizer
// constant `epsilon` should be chosen = 1e-5 or similar. Returns
// FailedPreconditionError in case of dimensional mismatches. See R.F. Stengel,
// "Optimal Control and Estimation", Chapter 6, or
// https://en.wikipedia.org/wiki/Algebraic_Riccati_equation
absl::Status ComputeRobustDynamicRiccatiEquationIterate(
    const eigenmath::MatrixXd& P, const eigenmath::MatrixXd& Q,
    const eigenmath::MatrixXd& R, const eigenmath::MatrixXd& A,
    const eigenmath::MatrixXd& B, double epsilon, eigenmath::MatrixXd& P_next,
    eigenmath::MatrixXd& K);

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_LINEAR_SYSTEMS_DYNAMIC_RICCATI_EQUATION_H_
