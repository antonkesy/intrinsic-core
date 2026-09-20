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

#ifndef INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_INTEGRATE_SQUARED_PATH_VELOCITY_POLYNOMIAL_H_
#define INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_INTEGRATE_SQUARED_PATH_VELOCITY_POLYNOMIAL_H_

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/topp_polynomials.h"

namespace intrinsic::topp {

// Integrates a quadratic expression between `polynomial.range_start` and
// `polynomial.range_end`. The polynomial coefficients are given by
// `polynomial.coeffs`. The integral takes the following form:
//         range_end                           1.0
//   integral          --------------------------------------------------- ds
//     s = range_start  sqrt(coeffs[0] * s^2 + coeffs[1] * s + coeffs[2])
// It handles the following cases:
//  - a quadratic concave expression (negative leading coefficient).
//  - a quadratic convex expression (positive leading coefficient).
//  - a degenerate quadratic expression (zero leading coefficient).
// The boolean flag `truncate` allows to truncate the integration of the
// polynomial to avoid ill-conditioning of the integral, i.e. to compute the
// integral in the well-defined range of the polynomial.
absl::StatusOr<double> IntegrateSquaredPathVelocityPolynomial(
    const QuadraticPolynomial& polynomial, bool truncate = false);

}  // namespace intrinsic::topp

#endif  // INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_INTEGRATE_SQUARED_PATH_VELOCITY_POLYNOMIAL_H_
