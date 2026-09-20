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

#include "intrinsic/math/spline/monotonically_increasing_spline.h"

#include <sys/stat.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "intrinsic/math/almost_equals.h"
#include "intrinsic/math/ipow.h"
#include "intrinsic/math/time_series_utils.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

namespace {

// The minimum number of data points required to construct a spline.
constexpr int kMinNumberOfCurvePoints = 2;

// Returns an Ok status if the provided `curve_parameters`, `curve_values`, and
// `curve_value_derivatives` have consistent sizes, and the common size is >=
// `kMinCurvePoints`. Returns an error status otherwise.
absl::Status ValidateSizeConsistency(
    const absl::Span<const double> curve_parameters,
    const absl::Span<const double> curve_values,
    const absl::Span<const double> curve_value_derivatives) {
  if (curve_parameters.size() < kMinNumberOfCurvePoints) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Not enough curve parameters to construct the spline. Got ",
        curve_parameters.size(), " required ", kMinNumberOfCurvePoints, "."));
  }
  if (curve_parameters.size() != curve_values.size() ||
      curve_parameters.size() != curve_value_derivatives.size()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Size mismatch: curve parameters, values and value derivatives must "
        "have the same size. Got ",
        curve_parameters.size(), ", ", curve_values.size(), ", and ",
        curve_value_derivatives.size(), " respectively."));
  }

  return absl::OkStatus();
}

absl::Status AreAllPositive(const absl::Span<const double> values) {
  for (int i = 0; i < values.size(); ++i) {
    if (values[i] <= 0.0) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Got non-positive value ", values[i], " at index ", i, "."));
    }
  }

  return absl::OkStatus();
}

// Constructs the sequence of spline data points from the provided
// `curve_parameters`, `curve_values`, and `curve_value_derivatives`. Returns an
// error status if the input vectors have inconsistent sizes or break the
// monotonicity constraints.
absl::StatusOr<std::vector<PointOfMonotonicallyIncreasingSpline>>
ConstructSplineDataPoints(const absl::Span<const double> curve_parameters,
                          const absl::Span<const double> curve_values,
                          absl::Span<const double> curve_value_derivatives) {
  INTR_RETURN_IF_ERROR(ValidateSizeConsistency(curve_parameters, curve_values,
                                               curve_value_derivatives));

  INTR_RETURN_IF_ERROR(ElementsIncreaseStrictlyMonotonically(curve_parameters))
      << "Curve parameters must be strictly monotonically increasing.";
  INTR_RETURN_IF_ERROR(ElementsIncreaseStrictlyMonotonically(curve_values))
      << "Curve values must be strictly monotonically increasing.";
  INTR_RETURN_IF_ERROR(AreAllPositive(curve_value_derivatives))
      << "Curve value derivatives must be positive.";

  const size_t num_data_points = curve_parameters.size();
  std::vector<PointOfMonotonicallyIncreasingSpline> data_points;
  data_points.reserve(num_data_points);

  for (int i = 0; i < num_data_points; ++i) {
    data_points.push_back(PointOfMonotonicallyIncreasingSpline{
        .curve_parameter = curve_parameters[i],
        .curve_value = curve_values[i],
        .curve_value_derivative = curve_value_derivatives[i]});
  }

  return data_points;
}

// Returns the index of the data point whose curve parameter is smaller than the
// given `curve_parameter`. Returns 0 (index of the first data point) if the
// first data point has a curve parameter greater or equal to the given
// `curve_parameter`.
int GetLowerDataPointIndexForCurveParameter(
    const absl::Span<const PointOfMonotonicallyIncreasingSpline> data_points,
    const double curve_parameter) {
  const auto comparator = [](double x,
                             const PointOfMonotonicallyIncreasingSpline& p) {
    return x <= p.curve_parameter;
  };

  return GetLowerIndexForValue(data_points, curve_parameter, comparator);
}

}  // namespace

/*static*/
absl::StatusOr<std::unique_ptr<MonotonicallyIncreasingSpline>>
MonotonicallyIncreasingSpline::Create(
    const absl::Span<const double> curve_parameters,
    const absl::Span<const double> curve_values,
    const absl::Span<const double> curve_value_derivatives) {
  INTR_ASSIGN_OR_RETURN(
      const std::vector<PointOfMonotonicallyIncreasingSpline> data_points,
      ConstructSplineDataPoints(curve_parameters, curve_values,
                                curve_value_derivatives));

  // WrapUnique due to private constructor.
  return absl::WrapUnique(
      new MonotonicallyIncreasingSpline(std::move(data_points)));
}

absl::StatusOr<PointOfMonotonicallyIncreasingSpline>
MonotonicallyIncreasingSpline::Evaluate(double curve_parameter) const {
  // To increase robustness, we allow the curve parameter to be slightly outside
  // the valid range, but we clamp it to the valid range aftwards.
  constexpr double kRangeTolerance = 1e-5;
  if (curve_parameter <
          data_points_.front().curve_parameter - kRangeTolerance ||
      curve_parameter > data_points_.back().curve_parameter + kRangeTolerance) {
    return absl::OutOfRangeError(absl::StrCat(
        "Curve parameter out of valid range. Got ", curve_parameter,
        ", but valid range is [", data_points_.front().curve_parameter, ",",
        data_points_.back().curve_parameter, "]."));
  }

  // Evaluate the rational linear spline according to the algorithm described in
  //
  // Hernández-Mederos V, Estrada-Sarlabous J. "Sampling points on regular
  // parametric curves with control of their distribution" Computer Aided
  // Geometric Design. Section 5. 2003 Sep 1;20(6):363-82.
  //
  // To increase readability, we use the following notation:
  //   - `x`      is the (clamped) curve parameter.
  //   - `x_l`    is the largest data point curve parameter that is < `x`. It
  //              coincides with `x` if `x` is at lower end of the valid range.
  //   - `x_u`    is the smallest data point curve parameter that is > `x`. It
  //              coindides with `x` if `x` is at upper end of the valid range.
  //   - `y`      is the curve value at `x`.
  //   - `y_l`    is the curve value at `x_l`.
  //   - `y_u`    is the curve value at `x_u`.
  //   - `dy_l`   is the curve value derivative at `x_l`.
  //   - `dy_u`   is the curve value derivative at `x_u`.
  //   - `x_m`.   is the middle point between `x_l` and `x_u`.
  //   - `y_m`    is the curve value at `x_m`, computed as (1 - c) * y_l + c *
  //              y_u, with 0 < c < 1 (refer to the implementation or the paper
  //              for more details on the computation of `c`).
  //
  // The curve value is then computed as
  //      a_0 (x - x_b) + a_1 * (x - x_m)
  // y = ---------------------------------,
  //      a_2 (x - x_b) + a_3 * (x - x_m)
  //
  // with x_b = x_l for x < x_m and x_b = x_u otherwise, and the curve value
  // derivative as:
  //        (a_1 * a_2 - a_0 * a_3) * (x_m - x_b)
  // dy = ----------------------------------------.
  //         [a_2 (x - x_b) + a_3 * (x - x_m)]^2
  // The definition of the coefficients `a_i` can be found in the
  // implementation (refer to the paper for more details).
  //
  // Note that the computation is executed in long double precision to avoid
  // numerical issues when (x_u - x_l) and/or (y_u -y_l) are close to zero.
  const long double x =
      std::clamp(curve_parameter, data_points_.front().curve_parameter,
                 data_points_.back().curve_parameter);
  const int x_l_index =
      GetLowerDataPointIndexForCurveParameter(data_points_, x);
  const int x_u_index = x_l_index + 1;

  // The curve value derivatives are limited to a max value to avoid numerical
  // issues when derivatives are close to infinity.
  constexpr long double kMaxDerivativeValue = 1e9;
  const long double dy_l = std::clamp(
      static_cast<long double>(data_points_[x_l_index].curve_value_derivative),
      0.0L, kMaxDerivativeValue);
  const long double dy_u = std::clamp(
      static_cast<long double>(data_points_[x_u_index].curve_value_derivative),
      0.0L, kMaxDerivativeValue);

  const long double x_l = data_points_[x_l_index].curve_parameter;
  const long double x_u = data_points_[x_u_index].curve_parameter;
  const long double x_m = 0.5 * (x_l + x_u);
  const long double y_l = data_points_[x_l_index].curve_value;
  const long double y_u = data_points_[x_u_index].curve_value;
  const long double c = AlmostEquals(dy_l, dy_u)
                            ? 0.5
                            : (dy_l - std::sqrt(dy_l * dy_u)) / (dy_l - dy_u);
  const long double y_m = (1.0 - c) * y_l + c * y_u;

  const long double x_b = x < x_m ? x_l : x_u;
  const long double y_b = x < x_m ? y_l : y_u;
  const long double dy_b = x < x_m ? dy_l : dy_u;

  // Early return if the curve parameter is too close to the next data point.
  if (AlmostEquals(x, x_b)) {
    return PointOfMonotonicallyIncreasingSpline{
        .curve_parameter = static_cast<double>(x_b),
        .curve_value = static_cast<double>(y_b),
        .curve_value_derivative = static_cast<double>(dy_b)};
  }

  const long double x_m_minus_x_b = x_m - x_b;
  const long double a_0 = y_m * dy_b * x_m_minus_x_b;
  const long double a_1 = y_b * (y_b - y_m);
  const long double a_2 = dy_b * x_m_minus_x_b;
  const long double a_3 = y_b - y_m;
  const long double x_minus_x_b = x - x_b;
  const long double x_minus_x_m = x - x_m;

  const long double curve_value = (a_0 * x_minus_x_b + a_1 * x_minus_x_m) /
                                  (a_2 * x_minus_x_b + a_3 * x_minus_x_m);
  const long double curve_value_derivative =
      (a_1 * a_2 - a_0 * a_3) * x_m_minus_x_b /
      ::intrinsic::IPow((a_2 * x_minus_x_b + a_3 * x_minus_x_m), 2);

  return PointOfMonotonicallyIncreasingSpline{
      .curve_parameter = static_cast<double>(x),
      .curve_value = static_cast<double>(curve_value),
      .curve_value_derivative = static_cast<double>(curve_value_derivative)};
}

}  // namespace intrinsic
