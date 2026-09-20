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

#ifndef INTRINSIC_MATH_SPLINE_BSPLINE_PARAMETER_TRANSFORM_FUNCTION_INTEGRAND_H_
#define INTRINSIC_MATH_SPLINE_BSPLINE_PARAMETER_TRANSFORM_FUNCTION_INTEGRAND_H_

#include <cmath>
#include <functional>
#include <limits>
#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/math/almost_equals.h"
#include "intrinsic/math/spline/bspline.h"
#include "intrinsic/math/spline/bspline_utils.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

namespace internal {

constexpr double kKnotRangeTolerance = 1e-6;

// Default value for the upper bound of the velocity inflection integrand
// function. This is used to regularize the integrand function by soft clipping.
// The value has been empirically tuned through extensive testing for B-spline
// adaptive sampling use case, in the context of path refinement. See
// go/intrinsic-evaluation-bspline-adaptive-sampling for more details.
constexpr double kDefaultMaxVelocityInflectionIntegrandValue = 1000.0;

// Returns the ratio `numerator` / `denominator`, applying a variation of
// the Tikhonov regularization in case the denominator approaches zero.
// Computes the ratio as
//
//         `numerator` * `denominator`
// -------------------------------------------
// `denominator`^2 + `regularization_factor`^2
//
// where the `regularization_factor` is zero if `denominator` >
// `min_denominator_for_regularization`, and approaches
// `max_regularization_factor` as `denominator` approaches zero.
absl::StatusOr<double> RegularizedRatio(
    double numerator, double denominator,
    double min_denominator_for_regularization,
    double max_regularization_factor);

}  // namespace internal

// The integrand g(t) of a spline parameter transform function of the form
//               u
// s = f(u) = integral g(t) dt
//              u_0
// with u in [u_0, u_n] being the knot curve parameter of the B-spline. Note
// that an std::function is intentionally used here to allow copies of the
// function in consumers of the integrand.
template <typename Traits>
using BSplineParameterTransformFunctionIntegrand =
    std::function<absl::StatusOr<double>(const BSplineT<Traits>& spline,
                                         double knot_curve_parameter)>;

// Returns the parameter transform function integrand g(t) = 1. This corresponds
// to an identity parameter transform function f(u) = u, which simply returns
// the value of the input B-spline knot curve parameter.
template <typename Traits>
BSplineParameterTransformFunctionIntegrand<Traits>
BSplineKnotParameterIntegrand() {
  return
      [](const BSplineT<Traits>& spline,
         double knot_curve_parameter) -> absl::StatusOr<double> { return 1.0; };
}

// Returns the parameter transform function integrand g(t) = ||C'(t)|| of the
// arc-length parameterization, where ||.|| is the norm operator, based on the
// inner product defined by the B-spline traits, and C'(t) is the curve first
// path derivative at point t.
template <typename Traits>
BSplineParameterTransformFunctionIntegrand<Traits> BSplineArcLengthIntegrand() {
  return [](const BSplineT<Traits>& spline,
            double knot_curve_parameter) -> absl::StatusOr<double> {
    // The spline degree must be > 0, since we need to evaluate the first
    // path derivative.
    if (spline.Degree() < 1) {
      return absl::InvalidArgumentError(absl::StrCat(
          "The spline degree must be positive. Got ", spline.Degree(), "."));
    }
    INTR_ASSIGN_OR_RETURN(
        knot_curve_parameter,
        ValidateAndClampKnotCurveParameter(spline, knot_curve_parameter,
                                           internal::kKnotRangeTolerance));

    // TODO(b/351978904) here we exploit implementation details of B-spline
    // Traits to guarantee points are initialized with correct size also for
    // dynamic-size types.
    typename Traits::Point zero_point(spline.PointDim());
    Traits::Zero(zero_point);
    std::vector<typename Traits::Point> curve_point_and_derivative(2,
                                                                   zero_point);
    if (!spline.EvalCurveAndDerivatives(knot_curve_parameter,
                                        &curve_point_and_derivative)) {
      return absl::StatusOr<double>(
          absl::InternalError("Could not evaluate the B-spline."));
    }

    icon::RealtimeStatusOr<double> derivative_norm =
        Norm<Traits>(curve_point_and_derivative[1]);
    if (!derivative_norm.ok()) {
      return derivative_norm.status();
    }
    return derivative_norm.value();
  };
}

// Returns the parameter transform function integrand g(t) = k(t)||C'(t)|| of
// the curvature parameterization, where k(t) is the curvature of the spline
// curve at point t, ||.|| is the norm operator, based on the inner product
// defined by the B-spline traits, and C'(t) is the curve first path derivative
// at point t.
template <typename Traits>
BSplineParameterTransformFunctionIntegrand<Traits> BSplineCurvatureIntegrand() {
  return [](const BSplineT<Traits>& spline,
            double knot_curve_parameter) -> absl::StatusOr<double> {
    // The spline degree must be > 1, since we need to evaluate the first
    // and second path derivative.
    if (spline.Degree() < 2) {
      return absl::InvalidArgumentError(absl::StrCat(
          "The spline degree must be > 1. Got ", spline.Degree(), "."));
    }
    INTR_ASSIGN_OR_RETURN(
        const double clamped_knot_curve_parameter,
        ValidateAndClampKnotCurveParameter(spline, knot_curve_parameter,
                                           internal::kKnotRangeTolerance));

    // TODO(b/351978904) here we exploit implementation details of B-spline
    // Traits to guarantee points are initialized with correct size also for
    // dynamic-size types.
    typename Traits::Point zero_point(spline.PointDim());
    Traits::Zero(zero_point);
    std::vector<typename Traits::Point> curve_point_and_derivatives(3,
                                                                    zero_point);
    if (!spline.EvalCurveAndDerivatives(clamped_knot_curve_parameter,
                                        &curve_point_and_derivatives)) {
      return absl::StatusOr<double>(
          absl::InternalError("Could not evaluate the B-spline."));
    }

    icon::RealtimeStatusOr<double> first_derivative_norm =
        Norm<Traits>(curve_point_and_derivatives[1]);
    if (!first_derivative_norm.ok()) {
      return first_derivative_norm.status();
    }
    INTR_ASSIGN_OR_RETURN(
        const double curvature,
        BSplineCurvature<Traits>(curve_point_and_derivatives[1],
                                 curve_point_and_derivatives[2]));

    return curvature * first_derivative_norm.value();
  };
}

// Returns the parameter transform function integrand g(t) = ||C''(t)|| /
// ||C'(t)||, where||.|| is the norm operator, based on the inner product
// defined by the B-spline traits, and C'(t), C''(t) are the curve first and
// second path derivative at point t, respectively. This integrand approximates
// the curvature parameterization k(t)||C'(t)||, where k(t) is the curvature of
// the spline curve at point t. Such curvature can be expressed as k(t) =
// ||C'(t)||
// * ||C''(t)|| * sin(alpha) / ||C'(t)||^3, where alpha is the angle between
// C'(t) and C''(t). Thus, the approximation implies the assumption sin(alpha) =
// 1, which helps smoothing localized curvature spikes, charactherized by zero
// alpha value before and after the abrupt curvature change. As further
// regularization measure, the integrand g(t) is formulated as (Thikonov
// regularization)  g(t) = ||C''(t)|| * ||C'(t)|| / (||C'(t)||^2 + lambda^2). We
// use an adaptive lambda to activate the regularization only in the proximity
// of a singularity ||C'(t)|| = 0. See go/intrinsic-bspline-adaptive-sampling
// for more details.
template <typename Traits>
BSplineParameterTransformFunctionIntegrand<Traits>
BSplineRegularizedCurvatureIntegrand() {
  return [](const BSplineT<Traits>& spline,
            double knot_curve_parameter) -> absl::StatusOr<double> {
    // The spline degree must be > 1, since we need to evaluate the first
    // and second path derivative.
    if (spline.Degree() < 2) {
      return absl::InvalidArgumentError(absl::StrCat(
          "The spline degree must be > 1. Got ", spline.Degree(), "."));
    }
    INTR_ASSIGN_OR_RETURN(
        const double clamped_knot_curve_parameter,
        ValidateAndClampKnotCurveParameter(spline, knot_curve_parameter,
                                           internal::kKnotRangeTolerance));

    // TODO(b/351978904) here we exploit implementation details of B-spline
    // Traits to guarantee that points are initialized with the correct size
    // also for dynamic-size types.
    typename Traits::Point zero_point(spline.PointDim());
    Traits::Zero(zero_point);
    std::vector<typename Traits::Point> curve_point_and_derivatives(3,
                                                                    zero_point);
    if (!spline.EvalCurveAndDerivatives(clamped_knot_curve_parameter,
                                        &curve_point_and_derivatives)) {
      return absl::StatusOr<double>(
          absl::InternalError("Could not evaluate the B-spline."));
    }

    icon::RealtimeStatusOr<double> first_derivative_norm =
        Norm<Traits>(curve_point_and_derivatives[1]);
    if (!first_derivative_norm.ok()) {
      return first_derivative_norm.status();
    }
    icon::RealtimeStatusOr<double> second_derivative_norm =
        Norm<Traits>(curve_point_and_derivatives[2]);
    if (!second_derivative_norm.ok()) {
      return second_derivative_norm.status();
    }

    constexpr double kMinFirstDerivativeNormForRegularization = 0.005;
    constexpr double kMaxRegularizationFactor = 0.001;
    return internal::RegularizedRatio(
        second_derivative_norm.value(), first_derivative_norm.value(),
        kMinFirstDerivativeNormForRegularization, kMaxRegularizationFactor);
  };
}

// Returns the parameter transform function integrand g(t) = ||C'''(t)|| /
// ||C''(t)||, where ||.|| is the norm operator, based on the inner product
// defined by the B-spline traits, and C''(t), C'''(t) are the curve second and
// third path derivative at point t, respectively. This integrand aims at
// capturing areas where the third path derivative is large, an more
// specifically areas around velocity inflection points, i.e., points where the
// second path derivative goes to zero through zero. As a regularization
// measure, the actual integrand function is formulated as (Tikhonov
// regularization) g(t) = ||C'''(t)|| *
// ||C''(t)|| / (||C''(t)||^2 + lambda^2). We use an adaptive lambda to activate
// the regularization only in the proximity of a singularity ||C''(t)|| = 0.
template <typename Traits>
BSplineParameterTransformFunctionIntegrand<Traits>
BSplineVelocityInflectionsIntegrand(
    const double max_integrand_value =
        internal::kDefaultMaxVelocityInflectionIntegrandValue) {
  return [max_integrand_value](
             const BSplineT<Traits>& spline,
             double knot_curve_parameter) -> absl::StatusOr<double> {
    // The spline degree must be > 2, since we need to evaluate the first
    // and second path derivatives.
    if (spline.Degree() < 3) {
      return absl::InvalidArgumentError(absl::StrCat(
          "The spline degree must be > 2. Got ", spline.Degree(), "."));
    }
    INTR_ASSIGN_OR_RETURN(
        const double clamped_knot_curve_parameter,
        ValidateAndClampKnotCurveParameter(spline, knot_curve_parameter,
                                           internal::kKnotRangeTolerance));

    if (max_integrand_value <= 0.0) {
      return absl::InvalidArgumentError(
          absl::StrCat("The max integrand value must be > 0. Got ",
                       max_integrand_value, "."));
    }

    // TODO(b/351978904) here we exploit implementation details of B-spline
    // Traits to guarantee that points are initialized with the correct size
    // also for dynamic-size types.
    typename Traits::Point zero_point(spline.PointDim());
    Traits::Zero(zero_point);
    constexpr int kNumRequiredDerivatives = 4;
    std::vector<typename Traits::Point> curve_point_and_derivatives(
        kNumRequiredDerivatives, zero_point);
    if (!spline.EvalCurveAndDerivatives(clamped_knot_curve_parameter,
                                        &curve_point_and_derivatives)) {
      return absl::StatusOr<double>(
          absl::InternalError("Could not evaluate the B-spline."));
    }

    icon::RealtimeStatusOr<double> second_derivative_norm =
        Norm<Traits>(curve_point_and_derivatives[2]);
    if (!second_derivative_norm.ok()) {
      return second_derivative_norm.status();
    }
    icon::RealtimeStatusOr<double> third_derivative_norm =
        Norm<Traits>(curve_point_and_derivatives[3]);
    if (!third_derivative_norm.ok()) {
      return third_derivative_norm.status();
    }
    const double third_to_second_derivative_ratio =
        (AlmostEquals(second_derivative_norm.value(), 0.0)
             ? std::numeric_limits<double>::max()
             : third_derivative_norm.value() / second_derivative_norm.value());

    // Regularization by soft clipping.
    return max_integrand_value *
           std::tanh(third_to_second_derivative_ratio / max_integrand_value);
  };
}

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_SPLINE_BSPLINE_PARAMETER_TRANSFORM_FUNCTION_INTEGRAND_H_
