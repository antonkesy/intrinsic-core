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

#ifndef INTRINSIC_MATH_SPLINE_SPLINE_PARAMETER_CONVERTER_H_
#define INTRINSIC_MATH_SPLINE_SPLINE_PARAMETER_CONVERTER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/math/spline/spline_parameter_transform_function.h"

namespace intrinsic {

// Converts `transformed_domain_curve_parameters` from the codomain of a
// `spline_parameter_transform_function` to their corresponding knot curve
// parameters in the domain of the `spline_parameter_transform_function`. The
// conversion is achieved by numerically inverting the
// `spline_parameter_transform_function`, using an iterative search
// method. The search stops when the conversion error, defined as the L2-norm of
// the difference vector between the provided
// `transformed_domain_curve_parameters` and the ones corresponding to the
// computed knot curve parameters, is less than
// `batch_conversion_error_threshold`. Thus, the final error on each individual
// parameter might depend on the number of input parameters, but it is
// neverthless guaranteed to be smaller than `batch_conversion_error_threshold`.
// If the search fails to converge an error status is returned, unless
// `return_best_estimate_if_search_fails` is true (defaults to false). In such
// case, the method returns the best estimate (smallest
absl::StatusOr<std::vector<double>>
ConvertToKnotCurveParameterViaIterativeSearch(
    absl::Span<const double> transformed_domain_curve_parameters,
    const SplineParameterTransformFunction& spline_parameter_transform_function,
    double batch_conversion_error_threshold,
    bool return_best_estimate_if_search_fails = false);

// Converts `transformed_domain_curve_parameters` from the co-domain of a
// `spline_parameter_transform_function` to their corresponding knot curve
// parameters in the domain of the `spline_parameter_transform_function`. The
// conversion is performed by evaluating a monotonically increasing spline,
// which approximates the inverse of the `spline_parameter_transform_function`.
// Compared to the method above, the inversion of the
// `spline_parameter_transform_function` is therefore achieved by constructing a
// single approximation of the transform function, rather than by iterative
// search. As a result, the computation of the knot curve parameters is more
// efficient, although the parameter conversion might be less accurate.
absl::StatusOr<std::vector<double>>
ConvertToKnotCurveParameterViaSingleApproximation(
    absl::Span<const double> transformed_domain_curve_parameters,
    const SplineParameterTransformFunction&
        spline_parameter_transform_function);

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_SPLINE_SPLINE_PARAMETER_CONVERTER_H_
