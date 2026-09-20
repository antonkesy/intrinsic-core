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

#include "intrinsic/math/signals/adaptive_quadrature.h"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <list>
#include <string>
#include <utility>

#include "absl/algorithm/container.h"
#include "absl/functional/function_ref.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/math/signals/nested_quadrature.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

namespace {

absl::Status ValidateOptions(const AdaptiveQuadratureOptions& options) {
  const std::string error_prefix = "Invalid adaptive quadrature options. ";

  if (options.desired_absolute_error < 0.0) {
    return absl::InvalidArgumentError(absl::StrCat(
        error_prefix, "The desired absolute error must be non-negative. Got ",
        options.desired_absolute_error, "."));
  }
  if (options.desired_relative_error < 0.0) {
    return absl::InvalidArgumentError(absl::StrCat(
        error_prefix, "The desired relative error must be non-negative. Got ",
        options.desired_relative_error, "."));
  }
  if (options.max_num_subintervals < 1) {
    return absl::InvalidArgumentError(absl::StrCat(
        error_prefix, "The max number of subintervals must be positive. Got ",
        options.max_num_subintervals, "."));
  }
  if (options.max_subinterval_depth < 0) {
    return absl::InvalidArgumentError(absl::StrCat(
        error_prefix, "The max subinterval depth must be non-negative. Got ",
        options.max_subinterval_depth, "."));
  }

  return absl::OkStatus();
}

// Returns true if any of the error-based termination criteria for the adaptive
// quadrature is met, i.e., if the `error_estimate` is smaller than
// `absolute_tolerance` or  negligible w.r.t. the `integral` with the given
// `relative_tolerance` (i.e., if `error_estimate` < `relative_tolerance` *
// |`integral`|).
absl::StatusOr<bool> IsErrorTerminationCriterionMet(
    const double integral, const double error_estimate,
    const double absolute_tolerance, const double relative_tolerance) {
  if (error_estimate < 0.0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The error estimate must be non-negative. Got ", error_estimate, "."));
  }
  if (absolute_tolerance < 0.0 || relative_tolerance < 0.0) {
    return absl::InvalidArgumentError(
        "The absolute and relative tolerances must be non-negative.");
  }

  if (error_estimate < absolute_tolerance) {
    return true;
  }

  // To check the relative error , we use the machine-independent termination
  // criterion from: Gander, Walter, and Walter Gautschi. "Adaptive
  // quadrature—revisited." BIT Numerical Mathematics 40 (2000): 84-101.
  const double integral_scaled =
      integral * relative_tolerance / std::numeric_limits<double>::epsilon();

  return integral_scaled + error_estimate == integral_scaled;
}

// Returns true if the subinterval [`subinterval_start`, `subinterval_end`] is
// too small for further splitting due to machine precision.
bool IsSubintervalDivisionTerminationCriterionMet(
    const double subinterval_start, const double subinterval_end) {
  const double subinterval_middle = 0.5 * (subinterval_start + subinterval_end);

  return subinterval_middle <= subinterval_start ||
         subinterval_middle >= subinterval_end;
}

}  // namespace

absl::StatusOr<AdaptiveQuadratureResult> AdaptiveQuadrature(
    const NestedQuadratureRule& nested_quadrature_rule,
    absl::FunctionRef<absl::StatusOr<double>(double)> function, double start,
    double end, const AdaptiveQuadratureOptions& options) {
  if (start > end) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Invalid integration interval. Got start > end: ", start, " > ", end));
  }
  INTR_RETURN_IF_ERROR(ValidateOptions(options));

  // Compute the initial estimate of the integral over the interval [`start`,
  // `end`] and use the result to initialize the subintervals list.
  INTR_ASSIGN_OR_RETURN(const NestedQuadratureResult initial_estimate,
                        nested_quadrature_rule.Integrate(function, start, end));
  double integral = initial_estimate.integral;
  double error_estimate = initial_estimate.error_estimate;

  AdaptiveQuadratureSubinterval initial_subinterval = {
      .start = start,
      .end = end,
      .integral = integral,
      .error_estimate = error_estimate,
      .depth = 0};
  std::list<AdaptiveQuadratureSubinterval> subintervals;
  subintervals.push_back(initial_subinterval);
  std::list<AdaptiveQuadratureSubinterval>::iterator subinterval_to_split =
      subintervals.begin();
  int max_depth_reached = 0;
  INTR_ASSIGN_OR_RETURN(
      bool is_error_termination_criterion_met,
      IsErrorTerminationCriterionMet(integral, error_estimate,
                                     options.desired_absolute_error,
                                     options.desired_relative_error));
  while (!is_error_termination_criterion_met &&
         subinterval_to_split->depth < options.max_subinterval_depth &&
         subintervals.size() < options.max_num_subintervals &&
         !IsSubintervalDivisionTerminationCriterionMet(
             subinterval_to_split->start, subinterval_to_split->end)) {
    // Subdivision of the integration interval.
    const double subinterval_middle =
        0.5 * (subinterval_to_split->start + subinterval_to_split->end);
    const size_t subinterval_depth = subinterval_to_split->depth;

    INTR_ASSIGN_OR_RETURN(
        const NestedQuadratureResult result_start_middle,
        nested_quadrature_rule.Integrate(function, subinterval_to_split->start,
                                         subinterval_middle));
    INTR_ASSIGN_OR_RETURN(
        const NestedQuadratureResult result_middle_end,
        nested_quadrature_rule.Integrate(function, subinterval_middle,
                                         subinterval_to_split->end));

    // Create the two subintervals that will replace the one that was just
    // split.
    const int depth = subinterval_depth + 1;
    const AdaptiveQuadratureSubinterval subinterval_start_middle = {
        .start = subinterval_to_split->start,
        .end = subinterval_middle,
        .integral = result_start_middle.integral,
        .error_estimate = result_start_middle.error_estimate,
        .depth = depth};
    const AdaptiveQuadratureSubinterval subinterval_middle_end = {
        .start = subinterval_middle,
        .end = subinterval_to_split->end,
        .integral = result_middle_end.integral,
        .error_estimate = result_middle_end.error_estimate,
        .depth = depth};

    // Update the total integral and error estimate, and the maximum depth
    // reached.
    integral += result_start_middle.integral + result_middle_end.integral -
                subinterval_to_split->integral;
    error_estimate += result_start_middle.error_estimate +
                      result_middle_end.error_estimate -
                      subinterval_to_split->error_estimate;
    max_depth_reached = std::max(max_depth_reached, depth);

    // Replace the subinterval that was just split with the two new
    // subintervals.
    subintervals.erase(subinterval_to_split);
    subintervals.push_back(subinterval_start_middle);
    subintervals.push_back(subinterval_middle_end);

    // At each new cycle, we want to split the subinterval with the largest
    // error estimate.
    subinterval_to_split = absl::c_max_element(
        subintervals, [](const AdaptiveQuadratureSubinterval& a,
                         const AdaptiveQuadratureSubinterval& b) {
          return a.error_estimate < b.error_estimate;
        });

    INTR_ASSIGN_OR_RETURN(
        is_error_termination_criterion_met,
        IsErrorTerminationCriterionMet(integral, error_estimate,
                                       options.desired_absolute_error,
                                       options.desired_relative_error));
  }

  return AdaptiveQuadratureResult{
      .integral = integral,
      .error_estimate = error_estimate,
      .max_subinterval_depth_reached = max_depth_reached,
      .subinterval_results = std::move(subintervals)};
}

}  // namespace intrinsic
