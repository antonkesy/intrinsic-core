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

#ifndef INTRINSIC_MATH_SPLINE_MONOTONICALLY_INCREASING_SPLINE_H_
#define INTRINSIC_MATH_SPLINE_MONOTONICALLY_INCREASING_SPLINE_H_

#include <memory>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"

namespace intrinsic {

// A point of a monotonically increasing spline, including the `curve_parameter`
// (which measures the progress along the spling from the start to the point),
// its corresponding `curve_value`, and the first derivative
// `curve_value_derivative` of the `curve_value` w.r.t. the `curve_parameter`.
struct PointOfMonotonicallyIncreasingSpline {
  double curve_parameter;
  double curve_value;
  double curve_value_derivative;
};

// A one-dimensional, C1-continuous, strictly monotonically increasing spline
// based on linear rational functions. The implementation follows the steps
// described in
//
// Hernández-Mederos V, Estrada-Sarlabous J. "Sampling points on regular
// parametric curves with control of their distribution" Computer Aided
// Geometric Design. Section 5. 2003 Sep 1;20(6):363-82.
//
// and comes as a specialization of the monotone linear rational spline
// construction method introduced in
//
// Fuhr, R.D. and Kallay, M., 1992. Monotone linear rational spline
// interpolation. Computer Aided Geometric Design, 9(4), pp.313-319.
class MonotonicallyIncreasingSpline {
 public:
  // Creates a monotonically increasing spline from the given data points,
  // provided as the triplets (`curve_parameters`[i], `curve_values`[i],
  // `curve_value_derivatives`[i]). The provided `curve_parameters` and
  // `curve_values` must be strictly increasing, due to the monotonicity
  // property of the spline. Hence, the `curve_value_derivatives` must be
  // strictly positive. An error status is returned otherwise. An error status
  // is also returned if less than two data points are provided, and if the
  // input vectors have inconsistent sizes. Note that the first and last curve
  // parameters additionally define the domain of the spline.
  static absl::StatusOr<std::unique_ptr<MonotonicallyIncreasingSpline>> Create(
      absl::Span<const double> curve_parameters,
      absl::Span<const double> curve_values,
      absl::Span<const double> curve_value_derivatives);

  // Evaluates the monotonically increasing spline at the given
  // `curve_parameter`. An error status is returned if the `curve_parameter` is
  // out of the valid range defined by the data points used to construct the
  // spline.
  absl::StatusOr<PointOfMonotonicallyIncreasingSpline> Evaluate(
      double curve_parameter) const;

  // Returns the data points of the spline.
  const std::vector<PointOfMonotonicallyIncreasingSpline>& GetDataPoints()
      const {
    return data_points_;
  }

 private:
  explicit MonotonicallyIncreasingSpline(
      absl::Span<const PointOfMonotonicallyIncreasingSpline> data_points)
      : data_points_(data_points.begin(), data_points.end()) {}

  std::vector<PointOfMonotonicallyIncreasingSpline> data_points_;
};

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_SPLINE_MONOTONICALLY_INCREASING_SPLINE_H_
