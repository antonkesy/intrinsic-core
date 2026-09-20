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

#ifndef INTRINSIC_MATH_SPLINE_BSPLINE_PARAMETER_INTEGRAL_TRANSFORM_FUNCTION_H_
#define INTRINSIC_MATH_SPLINE_BSPLINE_PARAMETER_INTEGRAL_TRANSFORM_FUNCTION_H_

#include <algorithm>
#include <cstdlib>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/base/attributes.h"
#include "absl/functional/any_invocable.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "intrinsic/math/almost_equals.h"
#include "intrinsic/math/linspace.h"
#include "intrinsic/math/signals/adaptive_quadrature.h"
#include "intrinsic/math/signals/gauss_kronrod_quadrature.h"
#include "intrinsic/math/signals/gauss_legendre_quadrature.h"
#include "intrinsic/math/spline/bspline.h"
#include "intrinsic/math/spline/bspline_parameter_transform_function_integrand.h"
#include "intrinsic/math/spline/bspline_utils.h"
#include "intrinsic/math/spline/monotonically_increasing_spline.h"
#include "intrinsic/math/spline/spline_parameter_transform_function.h"
#include "intrinsic/math/time_series_utils.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace internal {

// Returns the indices of the equal elements in `values` except for the first
// and last elements, that we want to keep.
std::vector<int> GetIndicesOfDuplicatedValues(absl::Span<const double> values);

// Removes the elements at the indices given in `indices_to_remove` from
// `values`.
absl::Status RemoveDuplicates(absl::Span<const int> indices_to_remove,
                              std::vector<double>& values);

}  // namespace internal

constexpr int kDebugLogLevel = 2;

// The settings used to configure the numerical integration of the
// parameter transform function.
struct BSplineParameterIntegralTransformFunctionSettings {
  // The key to select the order of the Gauss-Kronrod rule used in the adaptive
  // quadrature scheme. High-order rules are more accurate and might require
  // less subdivisions of the integration domain, but take longer to compute.
  GaussKronrodKey gauss_kronrod_key = GaussKronrodKey::kGaussKronrod15;

  // The options used to configure the adaptive quadrature rule used to
  // precompute the data points of the integral parameter transform function.
  // TODO(b/425285411) Retune error thresholds and max number of
  // subdivisions once monotonicity of evaluations is guaranteed. Do not change
  // the current setting without prior consulting with mdfiore@.
  AdaptiveQuadratureOptions adaptive_quadrature_options = {
      .desired_absolute_error = 1e-8,
      .desired_relative_error = 1e-7,
      .max_num_subintervals = 100,
      .max_subinterval_depth = 20};
};

// A parameter transform function s = f(u) for B-splines, where u in [u_0, u_n]
// is the standard knot curve parameter, u_0 and u_n being the first and last
// knots of the B-spline, and f has the form
//           u
// f(u) = integral g(t) dt.
//          u_0
// The integral operation is here performed numerically, dividing [u_0, u_n]
// into smaller intervals via an adaptive quadrature based on a Gauss-Kronrod
// rule. Note that an object of this class will only hold a const-reference to
// the B-spline it operates on. The user is therefore responsible for ensuring
// that the B-spline object outlives the parameter transform object.
template <typename Traits>
class BSplineParameterIntegralTransformFunction
    : public SplineParameterTransformFunction {
 public:
  // Creates a BSplineParameterIntegralTransformFunction for the given `spline`,
  // which integrates the function provided by the `integrand`. The numerical
  // integration is performed via an adaptive quadrature scheme based on a
  // Gauss- Kronrod rule, which can be configured via the provided `settings`.
  // The BSplineParameterIntegralTransformFunction does not take ownership of
  // the provided `spline`, which must outlive the
  // BSplineParameterIntegralTransformFunction. Additionally, the provided
  // `spline` is assumed not to change during the lifetime of the
  // BSplineParameterIntegralTransformFunction (internal computations might be
  // invalidated otherwise). Returns an error if the provided `spline` is a
  // null-pointer or not valid, i.e., it has no control points or invalid (empty
  // or with all knots coinciding) knot vector. Additionally, returns an error
  // if the provided `settings` are invalid.
  static absl::StatusOr<
      std::unique_ptr<BSplineParameterIntegralTransformFunction>>
  Create(
      const BSplineT<Traits>* spline,
      const BSplineParameterTransformFunctionIntegrand<Traits>& integrand,
      const BSplineParameterIntegralTransformFunctionSettings& settings = {});

  absl::StatusOr<double> Evaluate(double knot_curve_parameter) const override;

  // Batch evaluation method. Returns an error if the input
  // `knot_curve_parameters` are not sorted. Evaluation accuracy and performance
  // might improve when using this method compared to the non-batch evaluation
  // method used in a loop. In particular, this method can overcome small
  // integration errors that might occur for integrands that are not
  // "well-behaved", and guarantees monotonicity of the output parameters.
  absl::StatusOr<std::vector<double>> Evaluate(
      absl::Span<const double> knot_curve_parameters) const override;

  absl::StatusOr<double> EvaluateFirstDerivative(
      double knot_curve_parameter) const override;

  std::vector<double> GetKnotSpansEndpoints() const override {
    return knot_spans_endpoints_;
  }

  // The method uses the internal data points of the parameter transform
  // function arising from the Gauss-Kronrod rule used in its construction to
  // improve the accuracy of the approximation. Returns an error if the
  // approximation cannot be computed, i.e., if the parameter transform function
  // is not monotonically-increasing or if the provided minimum number of data
  // points is smaller than 2.
  absl::StatusOr<std::unique_ptr<MonotonicallyIncreasingSpline>>
  ComputeApproximatingSpline(int min_num_data_points) const override;

 private:
  // An internal data structure to hold a data point of the parameter transform
  // function s = f(u), namely a pair (u*, s*).
  struct DataPoint {
    double knot_parameter;
    double transformed_domain_parameter;
  };

  // Same as above, but it additionally contains the `curve_derivative_value` at
  // the data point.
  struct ExtendedDataPoint {
    double curve_parameter;
    double curve_value;
    double curve_derivative_value;
  };

  BSplineParameterIntegralTransformFunction(
      const BSplineT<Traits>* spline ABSL_ATTRIBUTE_LIFETIME_BOUND,
      absl::Span<const double> knot_spans_endpoints,
      absl::AnyInvocable<absl::StatusOr<double>(double) const> integrand_fcn,
      std::unique_ptr<GaussKronrodQuadrature> gauss_kronrod_rule,
      absl::Span<const DataPoint> data_points)
      : spline_(*spline),
        knot_spans_endpoints_(
            {knot_spans_endpoints.begin(), knot_spans_endpoints.end()}),
        integrand_fcn_(std::move(integrand_fcn)),
        gauss_kronrod_rule_(std::move(gauss_kronrod_rule)),
        data_points_({data_points.begin(), data_points.end()}) {}

  const BSplineT<Traits>& spline_;
  std::vector<double> knot_spans_endpoints_;

  // The scalar function to be integrated.
  absl::AnyInvocable<absl::StatusOr<double>(double) const> integrand_fcn_;

  // The Gauss-Kronrod quadrature rule used as base of the adaptive quadrature.
  std::unique_ptr<GaussKronrodQuadrature> gauss_kronrod_rule_;

  // A set of data points for which the parameter transform function is
  // precomputed.
  std::vector<DataPoint> data_points_;

  // Computes the first derivative of the `transform_function` at the given
  // `knot_curve_parameter`. Ensures that the derivative is strictly positive by
  // adding a small value to it if it is close to zero.
  absl::StatusOr<double> ComputeAndRegularizeDerivative(
      double knot_curve_parameter) const;

  // Refines spline data points recursively between `left` and `right` if the
  // difference in derivatives exceeds `derivative_threshold`.
  // `current_depth` tracks recursion depth, limited by `max_depth`.
  // New points are added to `refined_points`.
  absl::Status RecursiveRefine(
      const ExtendedDataPoint& left, const ExtendedDataPoint& right,
      int current_depth, int max_depth, double derivative_threshold,
      std::vector<ExtendedDataPoint>& refined_points) const;

  // Refines the approximating spline by adding elements to `curve_parameters`,
  // `curve_values` and `curve_derivative_values`. The refinement is done by
  // using binary search to strategically place new data points when a change
  // in the curve derivatives greater than the provided `derivative_threshold`
  // is detected. The new data points are directly inserted into the
  // `curve_parameters`, `curve_values` and `curve_derivative_values` inputs.
  // Returns an error if there is a mismatch in the size of the input vectors,
  // or if their size is smaller than 2.
  absl::Status RefineApproximatingSplineByDerivative(
      double derivative_threshold, std::vector<double>& curve_parameters,
      std::vector<double>& curve_values,
      std::vector<double>& curve_derivative_values) const;
};

template <typename Traits>
absl::StatusOr<double> BSplineParameterIntegralTransformFunction<
    Traits>::ComputeAndRegularizeDerivative(double knot_curve_parameter) const {
  double first_derivative_value = 0.0;
  INTR_ASSIGN_OR_RETURN(first_derivative_value,
                        EvaluateFirstDerivative(knot_curve_parameter));
  first_derivative_value =
      std::max(first_derivative_value, std::numeric_limits<double>::epsilon());

  return first_derivative_value;
}

template <typename Traits>
absl::StatusOr<
    std::unique_ptr<BSplineParameterIntegralTransformFunction<Traits>>>
BSplineParameterIntegralTransformFunction<Traits>::Create(
    const BSplineT<Traits>* spline,
    const BSplineParameterTransformFunctionIntegrand<Traits>& integrand,
    const BSplineParameterIntegralTransformFunctionSettings& settings) {
  if (!spline) {
    return absl::InvalidArgumentError("B-spline must not be a nullptr.");
  }
  INTR_RETURN_IF_ERROR(
      ValidateBSpline(*spline, BSplineBase::kDefaultKnotEqualityThreshold));

  // Get the vector of valid unique B-spline knots to set the knot spans
  // endpoints.
  std::vector<double> unique_knots;
  if (!spline->GetValidUniqueKnots(unique_knots)) {
    // We should never get here, since we already checked the validity of the
    // spline above.
    return absl::InternalError("Failed to get unique knots vector.");
  }

  // Starting from the provided `integrand`, create a scalar function g:t->g(t),
  // which can be directly passed to numerical integrators.
  absl::AnyInvocable<absl::StatusOr<double>(double) const> integrand_fcn =
      [spline, integrand](double u) -> absl::StatusOr<double> {
    return integrand(*spline, u);
  };

  // For a more accurate and efficient evaluation of the parameter transform
  // function, we precompute a set of data points. To do so, we run an adaptive
  // quadrature rule in each knot span. The adaptive quadrature rule adaptively
  // subdivides the knot span into smaller intervals to refine the integral
  // result. This operation provides data points for the parameter transform
  // function.
  INTR_ASSIGN_OR_RETURN(
      std::unique_ptr<GaussKronrodQuadrature> gauss_kronrod_rule,
      GaussKronrodQuadrature::Create(settings.gauss_kronrod_key));
  std::vector<DataPoint> data_points;
  data_points.reserve(
      unique_knots.size() *
      settings.adaptive_quadrature_options.max_num_subintervals);
  data_points.push_back(DataPoint{.knot_parameter = unique_knots.front(),
                                  .transformed_domain_parameter = 0.0});

  for (int i = 1; i < unique_knots.size(); ++i) {
    INTR_ASSIGN_OR_RETURN(
        AdaptiveQuadratureResult quadrature_result,
        AdaptiveQuadrature(*gauss_kronrod_rule, integrand_fcn,
                           /*start=*/unique_knots[i - 1],
                           /*end=*/unique_knots[i],
                           settings.adaptive_quadrature_options));

    // To generate sorted data points, we need to sort the generated
    // subintervals by their start elements.
    quadrature_result.subinterval_results.sort(
        [](const AdaptiveQuadratureSubinterval& a,
           const AdaptiveQuadratureSubinterval& b) {
          return a.start < b.start;
        });
    for (const auto& subinterval : quadrature_result.subinterval_results) {
      data_points.push_back({
          .knot_parameter = subinterval.end,
          .transformed_domain_parameter =
              subinterval.integral +
              data_points.back().transformed_domain_parameter,
      });
    }
  };

  // WrapUnique due to private constructor.
  return absl::WrapUnique(new BSplineParameterIntegralTransformFunction(
      spline, /*knot_spans_endpoints=*/unique_knots, std::move(integrand_fcn),
      std::move(gauss_kronrod_rule), data_points));
}

template <typename Traits>
absl::StatusOr<double>
BSplineParameterIntegralTransformFunction<Traits>::Evaluate(
    double knot_curve_parameter) const {
  INTR_ASSIGN_OR_RETURN(
      const double clamped_knot_curve_parameter,
      ValidateAndClampKnotCurveParameter(spline_, knot_curve_parameter,
                                         internal::kKnotRangeTolerance));

  // To evaluate the parameter transform function at the given knot curve
  // parameter, we start from its value at the closest (on the left) data
  // point and only compute the relative increment.
  const int previous_data_point_index = GetLowerIndexForValue(
      absl::MakeConstSpan(data_points_), clamped_knot_curve_parameter,
      [](const double knot_parameter, const DataPoint& data_point) {
        return knot_parameter < data_point.knot_parameter;
      });
  const DataPoint& previous_data_point =
      data_points_[previous_data_point_index];

  // Early return if the knot curve parameter is close enough to the knot
  // parameter of the data point.
  if (AlmostEquals(clamped_knot_curve_parameter,
                   previous_data_point.knot_parameter,
                   BSplineBase::kDefaultKnotEqualityThreshold)) {
    return previous_data_point.transformed_domain_parameter;
  }

  // Since the data points are generated by an adaptive quadrature rule, the
  // integrand function is supposed to be "well-behaved" between data points.
  // Thus, we can integrate with the lower-order (embedded) rule of the
  // Gauss-Kronrod quadrature to optimize the performance.
  INTR_ASSIGN_OR_RETURN(
      const double integral_increment,
      GaussLegendreQuadrature(
          integrand_fcn_, /*start=*/previous_data_point.knot_parameter,
          /*end=*/clamped_knot_curve_parameter,
          /*num_nodes=*/gauss_kronrod_rule_->NumNodesEmbeddedRule()));
  const double result =
      previous_data_point.transformed_domain_parameter + integral_increment;

  // Prevent the result to be larger than the value of the following data point.
  // This might happen due to numerics or small integration errors when
  // integrating up to a point that is close to the following data point.
  return std::min(
      result,
      data_points_[previous_data_point_index + 1].transformed_domain_parameter);
}

template <typename Traits>
absl::StatusOr<std::vector<double>>
BSplineParameterIntegralTransformFunction<Traits>::Evaluate(
    absl::Span<const double> knot_curve_parameters) const {
  if (!absl::c_is_sorted(knot_curve_parameters)) {
    return absl::InvalidArgumentError(
        "Curve parameters must be strictly monotonically increasing.");
  }

  std::vector<double> transformed_domain_parameters;
  transformed_domain_parameters.reserve(knot_curve_parameters.size());
  DataPoint last_evaluated_point = data_points_.front();
  for (double knot_curve_parameter : knot_curve_parameters) {
    INTR_ASSIGN_OR_RETURN(
        const double clamped_knot_curve_parameter,
        ValidateAndClampKnotCurveParameter(spline_, knot_curve_parameter,
                                           internal::kKnotRangeTolerance));

    // To evaluate the change-of-variable function at the given knot curve
    // parameter, we start from its value at the closest (on the left) data
    // point or the previous knot curve parameter (whichever of the two is
    // closer), and only compute the relative increment.
    const int closest_data_point_index = GetLowerIndexForValue(
        absl::MakeConstSpan(data_points_), clamped_knot_curve_parameter,
        [](const double knot_parameter, const DataPoint& data_point) {
          return knot_parameter < data_point.knot_parameter;
        });
    const DataPoint& closest_point =
        data_points_[closest_data_point_index].knot_parameter >
                last_evaluated_point.knot_parameter
            ? data_points_[closest_data_point_index]
            : last_evaluated_point;

    // "Early return" if the current knot curve parameter is close enough to the
    // closest point.
    if (AlmostEquals(clamped_knot_curve_parameter, closest_point.knot_parameter,
                     BSplineBase::kDefaultKnotEqualityThreshold)) {
      transformed_domain_parameters.push_back(
          closest_point.transformed_domain_parameter);
      last_evaluated_point = closest_point;
      continue;
    }

    // Since the data points are generated by an adaptive quadrature rule, the
    // integrand function is supposed to be "well-behaved" between data points.
    // Thus, we can integrate with the lower-order (embedded) rule of the
    // Gauss-Kronrod quadrature to optimize the performance.
    INTR_ASSIGN_OR_RETURN(
        const double integral_increment,
        GaussLegendreQuadrature(
            integrand_fcn_, /*start=*/closest_point.knot_parameter,
            /*end=*/clamped_knot_curve_parameter,
            /*num_nodes=*/gauss_kronrod_rule_->NumNodesEmbeddedRule()));

    // Prevent the result to be larger than the value of the following data
    // point. This might happen due to numerics or small integration errors when
    // integrating up to a point that is close to the following data point.
    // TODO (b/425285411) This can cause transformed-domain parameters to
    // coincide, thus not being strictly monotonically increasing.
    const double integral_result = std::min(
        closest_point.transformed_domain_parameter + integral_increment,
        data_points_[closest_data_point_index + 1]
            .transformed_domain_parameter);

    transformed_domain_parameters.push_back(integral_result);
    last_evaluated_point = {
        .knot_parameter = knot_curve_parameter,
        .transformed_domain_parameter = integral_result,
    };
  }

  return transformed_domain_parameters;
}

template <typename Traits>
absl::StatusOr<double>
BSplineParameterIntegralTransformFunction<Traits>::EvaluateFirstDerivative(
    double knot_curve_parameter) const {
  INTR_ASSIGN_OR_RETURN(
      const double clamped_knot_curve_parameter,
      ValidateAndClampKnotCurveParameter(spline_, knot_curve_parameter,
                                         internal::kKnotRangeTolerance));

  // The first derivative of an integral parameter transform function is its
  // integrand function (https://en.wikipedia.org/wiki/Leibniz_integral_rule).
  return integrand_fcn_(clamped_knot_curve_parameter);
}

template <typename Traits>
absl::Status BSplineParameterIntegralTransformFunction<Traits>::RecursiveRefine(
    const ExtendedDataPoint& left, const ExtendedDataPoint& right,
    const int current_depth, const int max_depth,
    const double derivative_threshold,
    std::vector<ExtendedDataPoint>& refined_points) const {
  if (current_depth >= max_depth ||
      std::abs(right.curve_derivative_value - left.curve_derivative_value) <=
          derivative_threshold) {
    return absl::OkStatus();
  }

  const double mid_curve_parameter =
      (left.curve_parameter + right.curve_parameter) / 2.0;

  const double kIntervalTolerance = 1.0e-6;
  // If the interval is too small, we stop refining.
  if (AlmostEquals(right.curve_parameter, left.curve_parameter,
                   kIntervalTolerance)) {
    return absl::OkStatus();
  }

  INTR_ASSIGN_OR_RETURN(const double mid_curve_value,
                        Evaluate(mid_curve_parameter));

  // Due to numerics or small integration errors, the additional curve
  // values might not be strictly monotonically increasing. In this case,
  // we stop refining.
  if (mid_curve_value >= right.curve_value ||
      mid_curve_value <= left.curve_value) {
    return absl::OkStatus();
  }

  INTR_ASSIGN_OR_RETURN(const double mid_derivative,
                        ComputeAndRegularizeDerivative(mid_curve_parameter));

  ExtendedDataPoint mid{
      .curve_parameter = mid_curve_parameter,
      .curve_value = mid_curve_value,
      .curve_derivative_value = mid_derivative,
  };

  INTR_RETURN_IF_ERROR(RecursiveRefine(left, mid, current_depth + 1, max_depth,
                                       derivative_threshold, refined_points));
  refined_points.push_back(mid);
  INTR_RETURN_IF_ERROR(RecursiveRefine(mid, right, current_depth + 1, max_depth,
                                       derivative_threshold, refined_points));
  return absl::OkStatus();
}

template <typename Traits>
absl::Status BSplineParameterIntegralTransformFunction<Traits>::
    RefineApproximatingSplineByDerivative(
        const double derivative_threshold,
        std::vector<double>& curve_parameters,
        std::vector<double>& curve_values,
        std::vector<double>& curve_derivative_values) const {
  if (curve_parameters.size() != curve_values.size() ||
      curve_parameters.size() != curve_derivative_values.size()) {
    return absl::InvalidArgumentError(
        "The vectors curve_parameters, curve_values, and "
        "curve_derivative_values must have the same size.");
  }
  if (curve_parameters.size() < 2) {
    return absl::InvalidArgumentError(
        "The vectors curve_parameters, curve_values, and "
        "curve_derivative_values must have at least 2 elements.");
  }

  constexpr int kMaxRecursionDepth = 5;

  for (int i = 0; i < curve_parameters.size() - 1; ++i) {
    // If the change in the curve derivatives is greater than the provided
    // threshold, we use a recursive approach to insert new data points until
    // the change in the curve derivatives is small enough.
    const double derivatives_diff =
        std::abs(curve_derivative_values[i + 1] - curve_derivative_values[i]);
    if (derivatives_diff > derivative_threshold) {
      std::vector<ExtendedDataPoint> refined_points;
      refined_points.reserve((1 << kMaxRecursionDepth) - 1);

      const ExtendedDataPoint left = {
          .curve_parameter = curve_parameters[i],
          .curve_value = curve_values[i],
          .curve_derivative_value = curve_derivative_values[i]};
      const ExtendedDataPoint right = {
          .curve_parameter = curve_parameters[i + 1],
          .curve_value = curve_values[i + 1],
          .curve_derivative_value = curve_derivative_values[i + 1]};

      INTR_RETURN_IF_ERROR(
          RecursiveRefine(left, right,
                          /*current_depth=*/0, kMaxRecursionDepth,
                          derivative_threshold, refined_points));

      if (refined_points.empty()) {
        continue;
      }

      // Sort the refined data points depending on the curve parameters.
      absl::c_sort(refined_points,
                   [](const ExtendedDataPoint& a, const ExtendedDataPoint& b) {
                     return a.curve_parameter < b.curve_parameter;
                   });

      // Extract the new data points to insert them into the curve parameters,
      // values, and derivatives vectors.
      std::vector<double> refined_curve_parameters;
      std::vector<double> refined_curve_values;
      std::vector<double> refined_curve_derivative_values;
      refined_curve_parameters.reserve(refined_points.size());
      refined_curve_values.reserve(refined_points.size());
      refined_curve_derivative_values.reserve(refined_points.size());

      for (const ExtendedDataPoint& data_point : refined_points) {
        refined_curve_parameters.push_back(data_point.curve_parameter);
        refined_curve_values.push_back(data_point.curve_value);
        refined_curve_derivative_values.push_back(
            data_point.curve_derivative_value);
      }
      curve_parameters.insert(curve_parameters.begin() + i + 1,
                              refined_curve_parameters.begin(),
                              refined_curve_parameters.end());

      curve_values.insert(curve_values.begin() + i + 1,
                          refined_curve_values.begin(),
                          refined_curve_values.end());

      curve_derivative_values.insert(curve_derivative_values.begin() + i + 1,
                                     refined_curve_derivative_values.begin(),
                                     refined_curve_derivative_values.end());

      i += refined_curve_parameters.size();
    }
  }
  return absl::OkStatus();
}

template <typename Traits>
absl::StatusOr<std::unique_ptr<MonotonicallyIncreasingSpline>>
BSplineParameterIntegralTransformFunction<Traits>::ComputeApproximatingSpline(
    int min_num_data_points) const {
  if (min_num_data_points < 2) {
    return absl::InvalidArgumentError(
        absl::StrCat("Minimum number of data points must be at least 2. Got,  ",
                     min_num_data_points, "."));
  }

  // Support variables for the approximation of the parameter transform
  // function.
  int num_existing_knot_intervals = data_points_.size() - 1;

  int num_data_points_to_add =
      std::max(0, static_cast<int>(min_num_data_points - data_points_.size()));

  // The number of data points to add is rounded up to the nearest multiple of
  // the number of existing knot intervals.
  if (num_data_points_to_add > 0) {
    num_data_points_to_add +=
        num_existing_knot_intervals -
        (num_data_points_to_add % num_existing_knot_intervals);
  }
  int num_data_points_to_add_per_interval =
      num_data_points_to_add / num_existing_knot_intervals;

  // Depending on the number of the existing `data_points_` and the number of
  // data points to be added, we can pre-allocate the support vectors.
  const int kTotalNumDataPoints = num_data_points_to_add + data_points_.size();
  std::vector<double> curve_parameters;
  std::vector<double> curve_values;
  std::vector<double> curve_derivative_values;
  curve_parameters.reserve(kTotalNumDataPoints);
  curve_values.reserve(kTotalNumDataPoints);
  curve_derivative_values.reserve(kTotalNumDataPoints);

  // Fill the curve_parameters, curve_values, and curve_value_derivatives
  // vectors with the existing `data_points_` and
  // uniformly add additional data points in each subinterval.
  for (int internal_data_point_idx = 0;
       internal_data_point_idx < data_points_.size() - 1;
       ++internal_data_point_idx) {
    INTR_ASSIGN_OR_RETURN(
        std::vector<double> curve_parameters_knot_interval,
        Linspace(data_points_[internal_data_point_idx].knot_parameter,
                 data_points_[internal_data_point_idx + 1].knot_parameter,
                 num_data_points_to_add_per_interval + 2));

    // Use the batch evaluation method for data points consistency.
    INTR_ASSIGN_OR_RETURN(std::vector<double> curve_values_to_add,
                          Evaluate(curve_parameters_knot_interval));

    // We fill the support vectors with the
    // `curve_parameters_knot_interval` except the last element, which is
    // inserted as the first element in the next iteration.
    curve_values.insert(curve_values.end(), curve_values_to_add.begin(),
                        curve_values_to_add.end() - 1);
    curve_parameters.insert(curve_parameters.end(),
                            curve_parameters_knot_interval.begin(),
                            curve_parameters_knot_interval.end() - 1);

    for (int i = 0; i < curve_parameters_knot_interval.size() - 1; ++i) {
      INTR_ASSIGN_OR_RETURN(
          const double curve_derivative_value,
          ComputeAndRegularizeDerivative(curve_parameters_knot_interval[i]));
      curve_derivative_values.push_back(curve_derivative_value);
    }
  }

  // Add the last data point
  curve_parameters.push_back(data_points_.back().knot_parameter);
  curve_values.push_back(data_points_.back().transformed_domain_parameter);
  INTR_ASSIGN_OR_RETURN(
      double curve_derivative_value,
      ComputeAndRegularizeDerivative(data_points_.back().knot_parameter));
  curve_derivative_values.push_back(curve_derivative_value);

  // Refine the approximating spline by adding new data points if the change in
  // two consecutive curve derivatives is greater than `kDerivativeThreshold`.
  // The threshold has been determined empirically based on internal motion
  // planning tests and proved to provide a good trade-off between accuracy and
  // number of data points. Refer to go/intrinsic-bs-path-refinement for more
  // details.
  const double kDerivativeThreshold = 80.0;
  INTR_RETURN_IF_ERROR(RefineApproximatingSplineByDerivative(
      kDerivativeThreshold, curve_parameters, curve_values,
      curve_derivative_values));

  // Due to numerics or small integration errors when integrating up to a
  // point that is close to the following data point, the output parameters
  // might not be strictly monotonically increasing. To avoid this, we
  // compute the indices of the duplicated curve values and remove the
  // corresponding elements from the curve values and curve parameters.
  const std::vector<int> indices_to_remove =
      internal::GetIndicesOfDuplicatedValues(curve_values);
  INTR_RETURN_IF_ERROR(
      internal::RemoveDuplicates(indices_to_remove, curve_parameters));
  INTR_RETURN_IF_ERROR(
      internal::RemoveDuplicates(indices_to_remove, curve_values));
  INTR_RETURN_IF_ERROR(
      internal::RemoveDuplicates(indices_to_remove, curve_derivative_values));

  if (curve_parameters.size() < min_num_data_points) {
    DVLOG(kDebugLogLevel)
        << "A smaller number of data points has "
           "been used to ensure the spline is monotonically increasing.";
  }

  INTR_ASSIGN_OR_RETURN(
      std::unique_ptr<MonotonicallyIncreasingSpline>
          approximating_parameter_transform_function,
      MonotonicallyIncreasingSpline::Create(curve_parameters, curve_values,
                                            curve_derivative_values));
  return approximating_parameter_transform_function;
}

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_SPLINE_BSPLINE_PARAMETER_INTEGRAL_TRANSFORM_FUNCTION_H_
