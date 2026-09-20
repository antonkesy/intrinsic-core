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

#ifndef INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_INTEGRATE_CUBIC_SQUARED_PATH_VELOCITY_POLYNOMIAL_H_
#define INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_INTEGRATE_CUBIC_SQUARED_PATH_VELOCITY_POLYNOMIAL_H_

#include "absl/status/statusor.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/topp_polynomials.h"

namespace intrinsic::topp {

namespace integrate_cubic_squared_path_velocity_polynomial_internal {

// Computes numerically the following integral for a cubic `polynomial`:
//       range_end          1.0
//  integral         ------------------ dx
//   x = range_start  sqrt(polynomial)
absl::StatusOr<double> NumericallyIntegrateOneOverSqrtOfCubicPolynomial(
    const CubicPolynomial& polynomial);

}  // namespace integrate_cubic_squared_path_velocity_polynomial_internal

// Integrates a cubic representation of a squared path velocity between
// `polynomial.range_start` and `polynomial.range_end`. The polynomial
// coefficients are given by `polynomial.coeffs`.
// Denoting: x = coeffs[0] * s^3 + coeffs[1] * s^2 + coeffs[2] * s + coeffs[3].
// The integral takes the following form:
//          range_end      1.0
//   integral           --------- ds
//      s = range_start  sqrt(x)
// where `s` denotes the path variable along which to integrate the squared path
// velocity cubic polynomial. It returns an error if the integration range is
// invalid, or the solver failed to compute the integral due to the value of `x`
// being negative.
absl::StatusOr<double> IntegrateCubicSquaredPathVelocityPolynomial(
    const CubicPolynomial& polynomial);

}  // namespace intrinsic::topp

#endif  // INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_INTEGRATE_CUBIC_SQUARED_PATH_VELOCITY_POLYNOMIAL_H_
