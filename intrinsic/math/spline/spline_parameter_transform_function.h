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

#ifndef INTRINSIC_MATH_SPLINE_SPLINE_PARAMETER_TRANSFORM_FUNCTION_H_
#define INTRINSIC_MATH_SPLINE_SPLINE_PARAMETER_TRANSFORM_FUNCTION_H_

#include <memory>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/math/spline/monotonically_increasing_spline.h"

namespace intrinsic {

// An interface for functions used to express a spline curve in a different
// parameterization space, i.e. with respect to a curve parameter different from
// the standard knot-vector based parameter. This can be interpreted as a change
// of variables s=f(u), where u is the knot curve parameter, s is the new curve
// parameter and f a bijective function mapping u into s. The knot curve
// parameter is defined in U = [u_0, u_n], u_0 and u_n being the first and last
// knot of the B-spline, respectively.
class SplineParameterTransformFunction {
 public:
  virtual ~SplineParameterTransformFunction() = default;

  // Evaluates the parameter transform function at the given
  // `knot_curve_parameter`. Returns the value of the new curve parameter
  // corresponding to `knot_curve_parameter`.
  virtual absl::StatusOr<double> Evaluate(
      double knot_curve_parameter) const = 0;

  // A batch version of the above method. Implementations of this method may
  // have improved performance when evaluation is batched.
  virtual absl::StatusOr<std::vector<double>> Evaluate(
      absl::Span<const double> knot_curve_parameters) const = 0;

  // Evaluates the first derivative of the parameter transform function at the
  // given `knot_curve_parameter`. Returns the value of the first derivative of
  // the new curve parameter corresponding to `knot_curve_parameter`.
  virtual absl::StatusOr<double> EvaluateFirstDerivative(
      double knot_curve_parameter) const = 0;

  // Returns the sequence of knot spans endpoints composing the domain of
  // the parameter transform function. The first and last value of the returned
  // vector defines the domain of the function.
  virtual std::vector<double> GetKnotSpansEndpoints() const = 0;

  // Computes the approximation of the parameter transform function by
  // constructing a monotonically increasing spline with at least
  // `min_num_data points`.
  virtual absl::StatusOr<std::unique_ptr<MonotonicallyIncreasingSpline>>
  ComputeApproximatingSpline(int min_num_data_points) const = 0;
};

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_SPLINE_SPLINE_PARAMETER_TRANSFORM_FUNCTION_H_
