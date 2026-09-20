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

#include "intrinsic/math/spline/spline_parameter_converter.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "intrinsic/math/almost_equals.h"
#include "intrinsic/math/ipow.h"
#include "intrinsic/math/linspace.h"
#include "intrinsic/math/spline/monotonically_increasing_spline.h"
#include "intrinsic/math/spline/spline_parameter_transform_function.h"
#include "intrinsic/math/within_margin.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

namespace {

// Equality threshold for comparing transformed-domain parameters.
constexpr double kTransformedDomainParameterEqualityThreshold = 1e-6;

// Computes the L2 norm error between two vectors `x` and `y`. Returns an error
// status if the vectors have different sizes.
absl::StatusOr<double> ComputeL2NormErrorSquared(
    const absl::Span<const double> x, const absl::Span<const double> y) {
  if (x.size() != y.size()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "L2 norm error cannot be computed. Vectors have different sizes: ",
        x.size(), " vs ", y.size(), "."));
  }

  double total_error_squared = 0.0;
  for (int i = 0; i < x.size(); ++i) {
    const double error = x[i] - y[i];
    total_error_squared += error * error;
  }

  return total_error_squared;
}

// Returns an Ok status if the provided `curve_parameter` is within the valid
// range [`lower_bound`, `upper_bound`] with the given `allowed_margin`. Returns
// an out-of-range error status otherwise.
absl::Status ValidateCurveParameter(const double curve_parameter,
                                    const double lower_bound,
                                    const double upper_bound,
                                    const double allowed_margin) {
  if (curve_parameter < lower_bound - allowed_margin ||
      curve_parameter > upper_bound + allowed_margin) {
    return absl::OutOfRangeError(
        absl::StrCat("Got curve parameter ", curve_parameter,
                     ", but valid range is [", lower_bound, ",", upper_bound,
                     "], with allowed margin ", allowed_margin, "."));
  }

  return absl::OkStatus();
}

// A struct that holds the lower and upper bounds of the knot and
// transformed-domain curve parameters for a given
// `SplineParameterTransformFunction` object.
struct SplineParameterTransformFunctionBounds {
  double knot_parameter_lower_end;
  double knot_parameter_upper_end;
  double transformed_domain_parameter_lower_end;
  double transformed_domain_parameter_upper_end;
};

// Returns the lower and upper bounds of the knot and transformed-domain curve
// parameters for the given `parameter_transform_function` object. Returns an
// error status if the evaluation of the parameter transform function at the
// knot parameter bounds fails.
absl::StatusOr<SplineParameterTransformFunctionBounds>
GetSplineChangeOfVariableBounds(
    const SplineParameterTransformFunction& parameter_transform_function) {
  SplineParameterTransformFunctionBounds bounds;

  bounds.knot_parameter_lower_end =
      parameter_transform_function.GetKnotSpansEndpoints().front();
  bounds.knot_parameter_upper_end =
      parameter_transform_function.GetKnotSpansEndpoints().back();

  INTR_ASSIGN_OR_RETURN(
      bounds.transformed_domain_parameter_lower_end,
      parameter_transform_function.Evaluate(bounds.knot_parameter_lower_end));
  INTR_ASSIGN_OR_RETURN(
      bounds.transformed_domain_parameter_upper_end,
      parameter_transform_function.Evaluate(bounds.knot_parameter_upper_end));

  return bounds;
}

// A struct that holds the result of a conversion from a set of
// transformed-domain to knot curve parameters. Contains the
// `knot_curve_parameters` computed as the result of the conversion, and the
// actual `transformed_domain_parameters` corresponding to the
// `knot_curve_parameters`, as well as the `conversion_error` (L2 norm of the
// error between the transformed-domain parameters to be converted and the
// actual `transformed_domain_parameters`).
struct ConversionToKnotParametersResult {
  std::vector<double> knot_curve_parameters;
  std::vector<double> transformed_domain_parameters;
  double conversion_error = std::numeric_limits<double>::max();

  // Initializes the result struct reserving `reserved_size` for the
  // `knot_curve_parameters` and `transformed_domain_parameters` vectors.
  explicit ConversionToKnotParametersResult(const size_t reserved_size) {
    knot_curve_parameters.reserve(reserved_size);
    transformed_domain_parameters.reserve(reserved_size);
  }
};

// Performs a binary search to find a knot parameter that produces the given
// `transformed_domain_curve_parameter` through the
// `parameter_transform_function` function with an error smaller than the
// provided `error_threshold`.
absl::StatusOr<ConversionToKnotParametersResult> KnotParameterViaBinarySearch(
    const SplineParameterTransformFunction& parameter_transform_function,
    const double transformed_domain_curve_parameter,
    const double error_threshold) {
  INTR_ASSIGN_OR_RETURN(
      const SplineParameterTransformFunctionBounds
          spline_parameter_transform_function_bounds,
      GetSplineChangeOfVariableBounds(parameter_transform_function));
  INTR_RETURN_IF_ERROR(ValidateCurveParameter(
      transformed_domain_curve_parameter,
      spline_parameter_transform_function_bounds
          .transformed_domain_parameter_lower_end,
      spline_parameter_transform_function_bounds
          .transformed_domain_parameter_upper_end,
      /*allowed_margin=*/kTransformedDomainParameterEqualityThreshold))
      << "Transformed-domain curve parameter out of valid range.";

  // Binary search initialization.
  double knot_param_search_lower_bound =
      spline_parameter_transform_function_bounds.knot_parameter_lower_end;
  double knot_param_search_upper_bound =
      spline_parameter_transform_function_bounds.knot_parameter_upper_end;
  double knot_param_candidate =
      0.5 * (knot_param_search_lower_bound + knot_param_search_upper_bound);
  INTR_ASSIGN_OR_RETURN(
      double transformed_domain_param_candidate,
      parameter_transform_function.Evaluate(knot_param_candidate));
  double error = std::abs(transformed_domain_param_candidate -
                          transformed_domain_curve_parameter);
  double error_best_candidate = error;
  ConversionToKnotParametersResult result(/*reserved_size=*/1);
  result.knot_curve_parameters.push_back(knot_param_candidate);
  result.transformed_domain_parameters.push_back(
      transformed_domain_param_candidate);
  result.conversion_error = error;

  constexpr int kMaxNumIterations = 100;
  int num_iterations = 0;
  while (error > error_threshold && num_iterations < kMaxNumIterations) {
    if (transformed_domain_param_candidate <
        transformed_domain_curve_parameter) {
      knot_param_search_lower_bound = knot_param_candidate;
    } else {
      knot_param_search_upper_bound = knot_param_candidate;
    }
    knot_param_candidate =
        0.5 * (knot_param_search_lower_bound + knot_param_search_upper_bound);

    INTR_ASSIGN_OR_RETURN(
        transformed_domain_param_candidate,
        parameter_transform_function.Evaluate(knot_param_candidate));
    error = std::abs(transformed_domain_param_candidate -
                     transformed_domain_curve_parameter);

    if (error < error_best_candidate) {
      // Update the result with the best candidate.
      result.knot_curve_parameters.front() = knot_param_candidate;
      result.transformed_domain_parameters.front() =
          transformed_domain_param_candidate;
      result.conversion_error = error;
      error_best_candidate = error;
    }
    ++num_iterations;
  }

  if (error > error_threshold) {
    DVLOG(2) << absl::StrCat(
        "Binary search failed to converge for transformed-domain parameter ",
        transformed_domain_curve_parameter, " after ", num_iterations,
        " iterations.", " Last knot parameter candidate produced an error of ",
        error, ". Best knot paramereter candidate produced an error of ",
        error_best_candidate, ". Error threshold is ", error_threshold,
        ". Last binary search lower/upper bounds are [",
        knot_param_search_lower_bound, ", ", knot_param_search_upper_bound,
        "].");
  }

  return result;
}

// A struct that holds the result of inserting the lower and/or upper endpoint
// of a set to a vector. The `output` vector contains the result vector, while
// the `lower_endpoint_inserted` and `upper_endpoint_inserted` flags indicate
// whether the endpoints have been inserted at the start/end of the `output`
// vector.
struct EndpointInsertionResult {
  bool lower_endpoint_inserted = false;
  bool upper_endpoint_inserted = false;
  std::vector<double> output;

  // Initializes the result struct reserving `output_size` for the `output`
  // vector.
  explicit EndpointInsertionResult(const size_t reserved_output_size) {
    output.reserve(reserved_output_size);
  }
};

// Inserts the `lower_endpoint`/`upper_endpoint` at the start/end of an `input`
// vector, if it is not already present. `insertion_margin` is used as the
// equality threshold to check whether the endpoints are already present at the
// start/end of the `input` vector.
absl::StatusOr<EndpointInsertionResult> InsertEndpointsIfMissing(
    const absl::Span<const double> input, const double lower_endpoint,
    const double upper_endpoint, const double insertion_margin) {
  EndpointInsertionResult result(/*reserved_output_size=*/input.size() + 2);

  if (!WithinMargin(input.front(), lower_endpoint, insertion_margin)) {
    result.output.push_back(lower_endpoint);
    result.lower_endpoint_inserted = true;
  }
  result.output.insert(result.output.end(), input.begin(), input.end());
  if (!WithinMargin(input.back(), upper_endpoint, insertion_margin)) {
    result.output.push_back(upper_endpoint);
    result.upper_endpoint_inserted = true;
  }

  return result;
}

// For the provided `spline_parameter_transform_function` s = f(u), computes the
// derivative of the inverse function f^-1(s) for the transformed-domain
// parameter s corresponding to the given `knot_parameter` through the
// `spline_parameter_transform_function` itself. This corresponds to the inverse
// of the derivative of the parameter transform function f(u).
absl::StatusOr<double> ComputeKnotParameterDerivative(
    const SplineParameterTransformFunction& parameter_transform_function,
    const double knot_parameter) {
  INTR_ASSIGN_OR_RETURN(
      const double d_transformed_domain_param_vs_knot_param,
      parameter_transform_function.EvaluateFirstDerivative(knot_parameter));

  // A spline change-of-variable function is monotonically increasing. Thus, if
  // the derivative approaches zero, we return a large positive value.
  return (AlmostEquals(d_transformed_domain_param_vs_knot_param, 0.0)
              ? std::numeric_limits<double>::max()
              : 1.0 / d_transformed_domain_param_vs_knot_param);
}

// Computes a set of knot curve parameters producing the given
// `transformed_domain_curve_parameters` through the provided spline
// `parameter_transform_function` with a conversion error of `error_threshold`.
// The search is done iteratively using a monotonically increasing spline
// approximating the inverse of the parameter transform function.
absl::StatusOr<ConversionToKnotParametersResult>
KnotParameterViaMonotoneSplineApproximations(
    const SplineParameterTransformFunction& parameter_transform_function,
    const absl::Span<const double> transformed_domain_curve_parameters,
    const double error_threshold) {
  INTR_ASSIGN_OR_RETURN(
      const SplineParameterTransformFunctionBounds
          spline_parameter_transform_function_bounds,
      GetSplineChangeOfVariableBounds(parameter_transform_function));
  INTR_RETURN_IF_ERROR(
      ValidateCurveParameter(transformed_domain_curve_parameters.front(),
                             spline_parameter_transform_function_bounds
                                 .transformed_domain_parameter_lower_end,
                             spline_parameter_transform_function_bounds
                                 .transformed_domain_parameter_upper_end,
                             kTransformedDomainParameterEqualityThreshold))
      << "First transformed-domain curve parameter out of valid range.";
  INTR_RETURN_IF_ERROR(
      ValidateCurveParameter(transformed_domain_curve_parameters.back(),
                             spline_parameter_transform_function_bounds
                                 .transformed_domain_parameter_lower_end,
                             spline_parameter_transform_function_bounds
                                 .transformed_domain_parameter_upper_end,
                             kTransformedDomainParameterEqualityThreshold))
      << "Last transformed-domain curve parameter out of valid range.";

  // We use the method described in
  //
  // Hernández-Mederos V, Estrada-Sarlabous J. "Sampling points on regular
  // parametric curves with control of their distribution" Computer Aided
  // Geometric Design. Section 3. 2003 Sep 1;20(6):363-82.
  //
  // For the conversion algorithm to succeed, the transformed-domain parameter
  // lower and upper ends must always be included in the set of parameters to be
  // converted.
  INTR_ASSIGN_OR_RETURN(
      const EndpointInsertionResult endpoint_insertion_result,
      InsertEndpointsIfMissing(transformed_domain_curve_parameters,
                               spline_parameter_transform_function_bounds
                                   .transformed_domain_parameter_lower_end,
                               spline_parameter_transform_function_bounds
                                   .transformed_domain_parameter_upper_end,
                               kTransformedDomainParameterEqualityThreshold));
  const std::vector<double>& transformed_domain_params_to_convert =
      endpoint_insertion_result.output;

  // The maximum number of iterations for the conversion algorithm. The value
  // has been empirically tuned through extensive testing. The rationale behind
  // a relative small max number of iterations is to return early in case of
  // slow convergence, and let binary search refine the conversion of the
  // (typically few) critical parameters. This results in a better runtime. See
  // go/intrinsic-evaluation-bspline-adaptive-sampling for more details.
  constexpr int kMaxNumIterations = 5;
  int num_iterations = 0;
  const int num_points = transformed_domain_params_to_convert.size();

  // Initial guess: equally distributed knot parameters. The algorithm
  // additionally needs the derivatives of the knot parameter w.r.t the
  // transformed-domain parameter computed at the transformed-domain parameters
  // corresponding to the guess knot parameters.
  INTR_ASSIGN_OR_RETURN(
      std::vector<double> knot_params_guess,
      Linspace(
          spline_parameter_transform_function_bounds.knot_parameter_lower_end,
          spline_parameter_transform_function_bounds.knot_parameter_upper_end,
          num_points));
  INTR_ASSIGN_OR_RETURN(
      std::vector<double> transformed_domain_params_guess,
      parameter_transform_function.Evaluate(knot_params_guess));
  std::vector<double> d_knot_params_guess(num_points);
  for (int i = 0; i < num_points; ++i) {
    INTR_ASSIGN_OR_RETURN(
        d_knot_params_guess[i],
        ComputeKnotParameterDerivative(parameter_transform_function,
                                       knot_params_guess[i]));
  }

  const double error_threshold_squared = error_threshold * error_threshold;
  INTR_ASSIGN_OR_RETURN(
      double error_squared,
      ComputeL2NormErrorSquared(transformed_domain_params_to_convert,
                                transformed_domain_params_guess));
  double error_squared_best_candidate = error_squared;
  std::vector<double> knot_params_best_candidate = knot_params_guess;
  std::vector<double> transformed_domain_params_best_candidate =
      transformed_domain_params_guess;

  while (error_squared > error_threshold_squared &&
         num_iterations < kMaxNumIterations) {
    ++num_iterations;

    // At each iteration, we approximate the inverse of the change-of-variable
    // function with a monotonically increasing spline passing through the
    // guess data points.
    INTR_ASSIGN_OR_RETURN(const auto spline,
                          MonotonicallyIncreasingSpline::Create(
                              transformed_domain_params_guess,
                              knot_params_guess, d_knot_params_guess));

    // Then we evaluate the approximating spline at the transformed-domain
    // parameters to be converted and use the resulting knot parameters as a new
    // guess for the next iteration. Note that the first and last parameters
    // (and their respective derivatives) are excluded from the evaluation, as
    // they correctly map to the endpoints of the knot parameter domain already
    // in the first guess. This way (besides saving computation) we also avoid
    // small numerical errors on the endpoints conversion, which might arise
    // through the iterative process.
    for (int i = 1; i < num_points - 1; ++i) {
      INTR_ASSIGN_OR_RETURN(
          const PointOfMonotonicallyIncreasingSpline monotone_spline_point,
          spline->Evaluate(transformed_domain_params_to_convert[i]));

      knot_params_guess[i] = monotone_spline_point.curve_value;
      INTR_ASSIGN_OR_RETURN(
          d_knot_params_guess[i],
          ComputeKnotParameterDerivative(parameter_transform_function,
                                         knot_params_guess[i]));
    }
    INTR_ASSIGN_OR_RETURN(
        transformed_domain_params_guess,
        parameter_transform_function.Evaluate(knot_params_guess));

    INTR_ASSIGN_OR_RETURN(
        error_squared,
        ComputeL2NormErrorSquared(transformed_domain_params_to_convert,
                                  transformed_domain_params_guess));

    if (error_squared < error_squared_best_candidate) {
      error_squared_best_candidate = error_squared;
      knot_params_best_candidate = knot_params_guess;
      transformed_domain_params_best_candidate =
          transformed_domain_params_guess;
    }
  }

  DVLOG(2) << absl::StrCat(
      "Result from monotone spline approximation method after ", num_iterations,
      "/", kMaxNumIterations,
      " iterations: last knot parameter candidate produced an error of ",
      std::sqrt(error_squared),
      ". Best knot parameter candidate produced an error of ",
      std::sqrt(error_squared_best_candidate), ". Error threshold is ",
      error_threshold, ".");

  // If endpoints were inserted to the set of parameters to convert, we need to
  // remove the corresponding endpoints from
  //  the output vector.
  if (endpoint_insertion_result.lower_endpoint_inserted) {
    knot_params_best_candidate.erase(knot_params_best_candidate.begin());
    transformed_domain_params_best_candidate.erase(
        transformed_domain_params_best_candidate.begin());
  }
  if (endpoint_insertion_result.upper_endpoint_inserted) {
    knot_params_best_candidate.erase(knot_params_best_candidate.end() - 1);
    transformed_domain_params_best_candidate.erase(
        transformed_domain_params_best_candidate.end() - 1);
  }

  ConversionToKnotParametersResult result(num_points);
  result.knot_curve_parameters = std::move(knot_params_best_candidate);
  result.transformed_domain_parameters =
      std::move(transformed_domain_params_best_candidate);
  result.conversion_error = std::sqrt(error_squared_best_candidate);

  return result;
}

// Stores the result of a call to `GetErrorsBelowThreshold`. Holds the number of
// conversion errors that are below a given threshold, and the average of
// those errors.
struct ErrorsBelowThresholdResult {
  int num_errors_below_threshold;
  double avg_error_below_threshold;
};

// Compares the `desired_transformed_domain_parameters` against the
// `actual_transformed_domain_parameters` and returns the number errors that are
// below the given `error_threshold`, and the average of those
// errors.
absl::StatusOr<ErrorsBelowThresholdResult> GetErrorsBelowThreshold(
    absl::Span<const double> desired_transformed_domain_parameters,
    absl::Span<const double> actual_transformed_domain_parameters,
    const double error_threshold) {
  if (desired_transformed_domain_parameters.size() !=
      actual_transformed_domain_parameters.size()) {
    return absl::InvalidArgumentError(
        "Desired and actual transformed-domain parameters must have the same "
        "size.");
  }
  const int num_params = desired_transformed_domain_parameters.size();
  std::vector<double> conversion_errors;
  conversion_errors.reserve(num_params);

  int num_errors_below_threshold = 0;
  double errors_below_threshold_avg = 0.0;
  for (int i = 0; i < num_params; ++i) {
    conversion_errors.push_back(
        std::abs(actual_transformed_domain_parameters[i] -
                 desired_transformed_domain_parameters[i]));
    if (conversion_errors.back() <= error_threshold) {
      ++num_errors_below_threshold;
      errors_below_threshold_avg += conversion_errors.back();
    }
  }
  errors_below_threshold_avg =
      (num_errors_below_threshold > 0)
          ? errors_below_threshold_avg /
                static_cast<double>(num_errors_below_threshold)
          : 0.0;

  return ErrorsBelowThresholdResult{
      .num_errors_below_threshold = num_errors_below_threshold,
      .avg_error_below_threshold = errors_below_threshold_avg};
}

// Analyzes the conversion error of the single parameters in
// `conversion_result`, namely,
// abs(`conversion_result.transformed_domain_parameters[i]` -
// `transformed_domain_parameters_to_convert[i]`), and computes an error
// threshold for a subsequent conversion refinement step. The computed
// threshold is such that, if all parameters with a conversion error higher
// than the threshold are refined to eventually fulfill the threshold, the
// total batch conversion error (l2 norm of the error vector
// `conversion_result.transformed_domain_parameters` -
// `transformed_domain_parameters_to_convert`) fulfills
// `l2_batch_conversion_error_threshold`.
absl::StatusOr<double> ComputeConversionErrorThresholdForConversionRefinement(
    absl::Span<const double> transformed_domain_parameters_to_convert,
    const ConversionToKnotParametersResult& conversion_result,
    const double l2_batch_conversion_error_threshold) {
  const int num_params = conversion_result.knot_curve_parameters.size();
  if (transformed_domain_parameters_to_convert.size() != num_params) {
    return absl::InvalidArgumentError(
        "Conversion result and transformed-domain curve parameters to convert "
        "must have the same size.");
  }

  // From the input batch conversion error threshold, derive a threshold for
  // the conversion error of a single parameter averaging on the number of
  // parameters. If all parameters had a conversion error below this
  // threshold, `l2_batch_conversion_error_threshold` would be already
  // fulfilled (note that this condition is sufficient but not necessary).
  const double kAvgSingleConversionErrorThreshold =
      l2_batch_conversion_error_threshold / std::sqrt(num_params);

  // Analyze the conversion errors and store the number of parameters whose
  // conversion errors are below `kAvgSingleConversionErrorThreshold`.
  // Additionally, compute the average of those errors.
  INTR_ASSIGN_OR_RETURN(
      const ErrorsBelowThresholdResult errors_below_threshold_result,
      GetErrorsBelowThreshold(transformed_domain_parameters_to_convert,
                              conversion_result.transformed_domain_parameters,
                              kAvgSingleConversionErrorThreshold));

  // There are `errors_below_threshold_result.num_errors_below_threshold`
  // parameters whose conversion error can now be considered to be equal to
  // `errors_below_threshold_result.avg_error_below_threshold` <=
  // `kAvgSingleConversionErrorThreshold`. Thus, the remaining parameters might
  // have conversion errors that are >= `kAvgSingleConversionErrorThreshold`
  // without violating the constraint on
  // the`l2_batch_conversion_error_threshold`. Assuming a fixed maximum error
  // for the remaining parameters, the maximum error is simply computed via
  // Pythagorean rule.
  const double num_errors_below_threshold_ratio =
      static_cast<double>(
          errors_below_threshold_result.num_errors_below_threshold) /
      static_cast<double>(num_params);
  const double errors_below_threshold_avg_ratio =
      (kAvgSingleConversionErrorThreshold > 0.0)
          ? errors_below_threshold_result.avg_error_below_threshold /
                kAvgSingleConversionErrorThreshold
          : 0.0;
  const double max_error_above_threshold_ratio =
      std::sqrt((1 - num_errors_below_threshold_ratio *
                         intrinsic::IPow(errors_below_threshold_avg_ratio, 2)) /
                (1 - num_errors_below_threshold_ratio));

  const double max_error_above_threshold =
      max_error_above_threshold_ratio * kAvgSingleConversionErrorThreshold;

  // The threshold computed above is the max value for the curve parameters
  // with conversion errors > `kAvgSingleConversionErrorThreshold` that
  // guarantees the fulfillment of the provided batch conversion error. We
  // want the final threshold for the refinement of these parameters to be
  // neither too low (might cause numerical issues) nor too high (might lead
  // to inconsistent sequence of parameters, e.g. non-monotonic). Thus, we
  // strike a balance between `kAvgSingleConversionErrorThreshold` and
  // `max_error_above_threshold`.
  constexpr double kNumericalSafetyFactor = 0.999;
  constexpr double kAvgSingleConversionErrorThresholdScaleFactor = 2.0;
  return std::min(kNumericalSafetyFactor * max_error_above_threshold,
                  kAvgSingleConversionErrorThresholdScaleFactor *
                      kAvgSingleConversionErrorThreshold);
}

// Improves the accuracy of the provided knot parameters in
// `conversion_result` using binary search. The knot parameters are
// evaluated through the `spline_parameter_transform_function`, and the
// obtained transformed-domain parameters are compared with the given
// `transformed_domain_parameters_to_convert`. Then, the knot parameters are
// refined until the all the obtained transformed-domain parameters have a
// conversion error below `conversion_error_threshold`.
absl::Status ImproveConversionAccuracyViaBinarySearch(
    absl::Span<const double> transformed_domain_parameters_to_convert,
    const SplineParameterTransformFunction& spline_parameter_transform_function,
    const double conversion_error_threshold,
    ConversionToKnotParametersResult& conversion_result) {
  std::vector<double>& knot_curve_parameters =
      conversion_result.knot_curve_parameters;
  if (knot_curve_parameters.size() !=
      transformed_domain_parameters_to_convert.size()) {
    return absl::InvalidArgumentError(
        "Conversion result and desired transformed-domain curve parameters "
        "must have the same size.");
  }

  int num_refined_parameters = 0;
  for (int i = 0; i < knot_curve_parameters.size(); ++i) {
    const double conversion_error =
        std::abs(conversion_result.transformed_domain_parameters[i] -
                 transformed_domain_parameters_to_convert[i]);
    if (conversion_error > conversion_error_threshold) {
      INTR_ASSIGN_OR_RETURN(
          const ConversionToKnotParametersResult refined_parameter_result,
          KnotParameterViaBinarySearch(
              spline_parameter_transform_function,
              transformed_domain_parameters_to_convert[i],
              conversion_error_threshold));

      if (refined_parameter_result.conversion_error < conversion_error) {
        const double refined_knot_parameter =
            refined_parameter_result.knot_curve_parameters.front();
        knot_curve_parameters[i] = refined_knot_parameter;
        INTR_ASSIGN_OR_RETURN(
            conversion_result.transformed_domain_parameters[i],
            spline_parameter_transform_function.Evaluate(
                refined_knot_parameter));
        ++num_refined_parameters;
      }
    }
  }

  if (num_refined_parameters > 0) {
    // Recompute the conversion error after refinement.
    const double prev_conversion_error = conversion_result.conversion_error;
    INTR_ASSIGN_OR_RETURN(const double error_squared,
                          ComputeL2NormErrorSquared(
                              conversion_result.transformed_domain_parameters,
                              transformed_domain_parameters_to_convert));
    conversion_result.conversion_error = std::sqrt(error_squared);

    LOG(INFO) << absl::StrCat(
        num_refined_parameters,
        " parameters refined via binary search. Error before refinement: ",
        prev_conversion_error,
        ". After refinement: ", conversion_result.conversion_error);
  }

  return absl::OkStatus();
}

}  // namespace

absl::StatusOr<std::vector<double>>
ConvertToKnotCurveParameterViaIterativeSearch(
    absl::Span<const double> transformed_domain_curve_parameters,
    const SplineParameterTransformFunction& spline_parameter_transform_function,
    const double batch_conversion_error_threshold,
    const bool return_best_estimate_if_search_fails) {
  if (transformed_domain_curve_parameters.empty()) {
    return absl::InvalidArgumentError(
        "Transformed-domain curve parameters must not be empty.");
  }

  INTR_ASSIGN_OR_RETURN(ConversionToKnotParametersResult conversion_result,
                        KnotParameterViaMonotoneSplineApproximations(
                            spline_parameter_transform_function,
                            transformed_domain_curve_parameters,
                            batch_conversion_error_threshold));

  if (conversion_result.conversion_error > batch_conversion_error_threshold) {
    INTR_ASSIGN_OR_RETURN(
        const double conversion_error_threshold_for_binary_search,
        ComputeConversionErrorThresholdForConversionRefinement(
            transformed_domain_curve_parameters, conversion_result,
            batch_conversion_error_threshold));

    INTR_RETURN_IF_ERROR(ImproveConversionAccuracyViaBinarySearch(
        transformed_domain_curve_parameters,
        spline_parameter_transform_function,
        conversion_error_threshold_for_binary_search, conversion_result));
  }

  if (conversion_result.conversion_error > batch_conversion_error_threshold &&
      !return_best_estimate_if_search_fails) {
    return absl::InternalError(
        absl::StrCat("Spline parameter conversion search failed to converge. "
                     "Best candidate produced a conversion error of ",
                     conversion_result.conversion_error,
                     " (threshold = ", batch_conversion_error_threshold, ")."));
  }

  // Conversion was successful or it is desired to return the best estimate
  // found during the search.
  return conversion_result.knot_curve_parameters;
}

absl::StatusOr<std::vector<double>>
ConvertToKnotCurveParameterViaSingleApproximation(
    absl::Span<const double> transformed_domain_curve_parameters,
    const SplineParameterTransformFunction&
        spline_parameter_transform_function) {
  if (transformed_domain_curve_parameters.empty()) {
    return absl::InvalidArgumentError(
        "Transformed-domain curve parameters must not be empty.");
  }

  // Approximate the parameter transform function with a monotonically
  // increasing spline using a minimum number of data points computed as a
  // percentage of the number of transformed-domain curve parameters. However,
  // to ensure that the quality of the approximation is good enough when
  // `transformed_domain_curve_parameters` has small size, the number of data
  // points is always at least the minimum between `kMinNumDataPoints` and
  // `kMinNumDataPoints` scaled by twice the knot domain length. This allows to
  // reduce the number of data points for short paths.
  const int kMinNumDataPoints = 350;
  double knot_domain_length =
      spline_parameter_transform_function.GetKnotSpansEndpoints().back() -
      spline_parameter_transform_function.GetKnotSpansEndpoints().front();

  const int kMinScaledNumDataPoints =
      std::min(kMinNumDataPoints,
               static_cast<int>(2 * kMinNumDataPoints * knot_domain_length));
  const int num_data_points_percentage =
      static_cast<int>(0.3 * transformed_domain_curve_parameters.size());

  const int num_data_points_for_approximation =
      std::max(kMinScaledNumDataPoints, num_data_points_percentage);

  INTR_ASSIGN_OR_RETURN(
      const auto approximated_parameter_transform_function,
      spline_parameter_transform_function.ComputeApproximatingSpline(
          num_data_points_for_approximation));

  // Since the approximation is a monotonically increasing spline, we can
  // compute its inverse function by swapping the curve parameters with the
  // curve values, and evaluating its derivatives as the reciprocal of the
  // derivatives of the original approximated parameter transform function.
  // The new vectors can be used to construct the inverse function as a
  // monotonically increasing spline.
  const int num_data_points =
      approximated_parameter_transform_function->GetDataPoints().size();
  std::vector<double> curve_parameters(num_data_points);
  std::vector<double> curve_values(num_data_points);
  std::vector<double> inverse_curve_value_derivatives(num_data_points);
  for (int i = 0; i < curve_values.size(); ++i) {
    curve_parameters[i] =
        approximated_parameter_transform_function->GetDataPoints()[i]
            .curve_parameter;
    curve_values[i] =
        approximated_parameter_transform_function->GetDataPoints()[i]
            .curve_value;

    // Check if the derivative is strictly positive. In that case, we can safely
    // compute its reciprocal, otherwise we return an error.
    // Note that we should not expect the derivative to be less or equal to
    // zero, as the approximated parameter transform function is a monotonically
    // increasing spline which requires strictly positive derivatives.
    const double curve_value_derivative =
        approximated_parameter_transform_function->GetDataPoints()[i]
            .curve_value_derivative;
    if (curve_value_derivative <= 0.0) {
      return absl::InternalError(
          "Approximated parameter transform function has a non-positive "
          "derivative.");
    }

    inverse_curve_value_derivatives[i] = 1.0 / curve_value_derivative;
  }

  INTR_ASSIGN_OR_RETURN(
      auto inverse_parameter_transform_function,
      MonotonicallyIncreasingSpline::Create(curve_values, curve_parameters,
                                            inverse_curve_value_derivatives));

  // Evaluate the approximated inverse function at the transformed-domain curve
  // parameters to get the corresponding knot curve parameters.
  std::vector<double> knot_curve_parameters;
  knot_curve_parameters.reserve(transformed_domain_curve_parameters.size());

  for (const double transformed_domain_parameter :
       transformed_domain_curve_parameters) {
    INTR_ASSIGN_OR_RETURN(
        const PointOfMonotonicallyIncreasingSpline monotone_spline_point,
        inverse_parameter_transform_function->Evaluate(
            transformed_domain_parameter));
    knot_curve_parameters.push_back(monotone_spline_point.curve_value);
  }

  return knot_curve_parameters;
}

}  // namespace intrinsic
