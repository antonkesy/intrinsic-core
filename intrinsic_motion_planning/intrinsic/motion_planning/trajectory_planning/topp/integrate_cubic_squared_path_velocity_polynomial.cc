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

#include "intrinsic/motion_planning/trajectory_planning/topp/integrate_cubic_squared_path_velocity_polynomial.h"

#include <cmath>
#include <functional>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/math/signals/gauss_legendre_quadrature.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/topp_analytical_polynomial_integrals.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/topp_polynomials.h"

namespace intrinsic::topp {

namespace integrate_cubic_squared_path_velocity_polynomial_internal {

absl::StatusOr<double> NumericallyIntegrateOneOverSqrtOfCubicPolynomial(
    const CubicPolynomial& polynomial) {
  if (polynomial.range_end <= polynomial.range_start) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The integration range is invalid, The `range_start` "
        "should be less than `range_end`. Got ",
        polynomial.range_start, " vs. ", polynomial.range_end, "."));
  }

  // Squared path velocity integrand for cubic polynomial. It computes
  //  `1.0/sqrt(cubic polynomial value)`. Returns an error if the squared path
  //  velocity becomes negative.
  std::function<absl::StatusOr<double>(double knot_curve_parameter)> integrand =
      [&polynomial](double knot_curve_parameter) -> absl::StatusOr<double> {
    const double squared_path_velocity =
        polynomial.EvaluatePolynomial(knot_curve_parameter);
    if (squared_path_velocity < 0.0) {
      return absl::InternalError(absl::StrCat(
          "The squared path velocity is negative: ", squared_path_velocity,
          ", and cannot thus be integrated."));
    }
    return absl::StatusOr<double>(
        1.0 / std::sqrt(polynomial.EvaluatePolynomial(knot_curve_parameter)));
  };

  // TODO(b/387482079): Optimize number of nodes for computation efficiency.
  const int kNumNodes = 1000;
  return GaussLegendreQuadrature(integrand,
                                 /*start=*/polynomial.range_start,
                                 /*end=*/polynomial.range_end,
                                 /*num_nodes=*/kNumNodes);
}

}  // namespace integrate_cubic_squared_path_velocity_polynomial_internal

absl::StatusOr<double> IntegrateCubicSquaredPathVelocityPolynomial(
    const CubicPolynomial& polynomial) {
  // Attempt to compute the integral analytically.
  icon::RealtimeStatusOr<double> definite_integral_status_or;
  const double integral_valid_lower_bound_sec = 0.0;
  const double integral_valid_upper_bound_sec = 1.0;
  definite_integral_status_or =
      IntegrateOneOverSqrtOfStructuredCubicPolynomial(polynomial);
  if (definite_integral_status_or.ok()) {
    // Safety check: If the integral is within `integral_valid_lower_bound_sec`
    // and `integral_valid_upper_bound_sec` seconds, we will trust its value and
    // use it. This should likely never be used, but it is a safety mechanism in
    // place for now.
    if (definite_integral_status_or.value() > integral_valid_lower_bound_sec &&
        definite_integral_status_or.value() < integral_valid_upper_bound_sec) {
      return definite_integral_status_or.value();
    }
  }

  // The computation of the numerical integral is only a fallback strategy, for
  // cases where the analytical integrals fail to be computed.
  return integrate_cubic_squared_path_velocity_polynomial_internal::
      NumericallyIntegrateOneOverSqrtOfCubicPolynomial(polynomial);
}

}  // namespace intrinsic::topp
