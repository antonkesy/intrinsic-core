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


#ifndef INTRINSIC_MATH_SPLINE_BSPLINE_UTILS_H_
#define INTRINSIC_MATH_SPLINE_BSPLINE_UTILS_H_

#include <algorithm>
#include <cmath>
#include <vector>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/math/almost_equals.h"
#include "intrinsic/math/ipow.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/spline/bspline.h"
#include "intrinsic/math/spline/bspline_base.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

// Enum for selecting from different methods to compute the B-spline knot
// vectors. The different methods affect the mapping between the knot curve
// parameter `u` used to evaluate a B-spline C(u), and the arc length actually
// travelled along the B-spline curve.
enum class BSplineKnotVectorSelection {
  kAverageChordLengthKnotVector,  // Knot vector according to chord length.
  kCentripetalKnotVector,  // Knot vector according to centripetal method.
  kUniformKnotVector,      // Uniformly spaced knot vector.
};

// Returns an error status if `knot_curve_parameter` is outside the `spline`
// knot vector range. The check is performed with the given `tolerance`. If
// the validation is successful, returns the input `knot_curve_parameter`,
// possibly clamped to the closest end of the valid range.
template <typename Traits>
absl::StatusOr<double> ValidateAndClampKnotCurveParameter(
    const BSplineT<Traits>& spline, const double knot_curve_parameter,
    const double tolerance) {
  if (spline.NumKnots() == 0) {
    return absl::InvalidArgumentError(
        "The spline knot vector must not be empty.");
  }

  std::vector<double> knots(spline.NumKnots());
  if (!spline.GetKnotVector(&knots)) {
    return absl::InternalError("Couldn't get spline knot vector.");
  }

  if (tolerance < 0.0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The tolerance must be non-negative. Got ", tolerance, "."));
  }

  if (knot_curve_parameter < knots.front() - tolerance ||
      knot_curve_parameter > knots.back() + tolerance) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The knot curve parameter is outside the valid range [", knots.front(),
        ", ", knots.back(), "]. Got ", knot_curve_parameter, "."));
  }

  return std::clamp(knot_curve_parameter, knots.front(), knots.back());
}

// Takes a list of points defining the `corners` of a piecewise linear curve
// (polyline) and returns waypoints for a 3rd order bspline. The spline will
// follow the polyline at distances not further than `radii` away from the
// corners. The size of `radii` must be equal to the size of `corners`, however
// the `radii` for the first and last control point will be neglected.
// Intuition: first and last corner point should not be blended and fit desired
// value exactly. This function allocates.
absl::StatusOr<std::vector<eigenmath::VectorNd>> PolyLineToBspline3Waypoints(
    const std::vector<eigenmath::VectorNd>& corners,
    const std::vector<double>& radii);

// Knot vector for approximate arc-length parameterization of a b-spline using
// the centripetal method.
//
// Returns a normalized knot-vector designed for a bspline of `spline_degree`
// around desired `control_points`. The knot vector is designed according to the
// "Average Method" in [1], in which the knot vector is chosen of a moving
// average of weighted, accumulated chord lengths computed according to the
// "Centripetal Method" [1]. For a spline C(u) constructed with this knot
// vector, the input curve parameter `u` can be interpreted as a coarse
// approximation of spline curve's arc-length.
//
// The centripetal method aims at providing an acceptable approximation of the
// spline arc-length while following the input data polygon as closely as
// possible. To achieve better overall spline stability, the centripetal method
// puts more weight on areas where the input data is sampled densely. In these
// areas, the arc-length approximation is less accurate.
//
// [1] "B-Spline Interpolation and Approximation", Hongxin Zhang and
// Jieqing Feng, Lecture notes, Zhejiang University,
// http://www.cad.zju.edu.cn/home/zhx/GM/009/00-bsia.pdf
absl::StatusOr<std::vector<double>> MakeAverageCentripetalKnotVector(
    absl::Span<const eigenmath::VectorNd> control_points, int spline_degree);

// Knot vector for approximate arc-length parameterization of a b-spline using
// the chord length method.
//
// Returns a normalized knot-vector  designed for a bspline of `spline_degree`
// around desired `control_points`. The knot vector is designed according to the
// "Average Method" in [1], in which the knot vector is chosen of a moving
// average of accumulated chord lengths (data point polygon length). For a
// spline C(u) constructed with this knot vector, the input curve parameter `u`
// can be interpreted as an approximation of spline curve's arc-length.
//
// While the chord-length method provides better approximation of the spline
// arc-length than the centripetal method (above), even in areas with dense
// `control_points`, the resulting curve may show a large deviation from the
// input path in areas where `control_points` are sparse. This can be mitigated
// to some extent by preprocessing input data points with
// `PolyLineToBspline3Waypoints()`.
//
// [1] "B-Spline Interpolation and Approximation", Hongxin Zhang and
// Jieqing Feng, Lecture notes, Zhejiang University,
// http://www.cad.zju.edu.cn/home/zhx/GM/009/00-bsia.pdf
absl::StatusOr<std::vector<double>> MakeAverageChordLengthKnotVector(
    absl::Span<const eigenmath::VectorNd> control_points, int spline_degree);

// Implementation for poses.
//   Linear quaternion interpolation is defined as a linear increase of the
//   angle around a fixed axis between the quaternions at each corner.
//   Both translation and rotational radii are required, but only the most
//   conservative value (whichever results in the smallest percentage of the
//   difference between corner poses) is used. The size of `corners`,
//   `translation_radii` and `rotation_radii` must be equal.
absl::StatusOr<std::vector<Pose3d>> PolyLineToBspline3Waypoints(
    const std::vector<Pose3d>& corners,
    const std::vector<double>& translation_radii,
    const std::vector<double>& rotation_radii);

// Implementation for poses, see documentation for VectorNd-version of
// MakeAverageCentripetalKnotVector for details. This function is equivalent,
// chord lengths are measured as logarithmic maps of difference quaternions.
absl::StatusOr<std::vector<double>> MakeAverageCentripetalKnotVector(
    absl::Span<const Pose3d> control_points, int spline_degree);

// Implementation for poses, see documentation for VectorNd-version of
// MakeAverageChordLengthKnotVector for details.  This function is equivalent,
// chord lengths are measured as logarithmic maps of difference quaternions.
absl::StatusOr<std::vector<double>> MakeAverageChordLengthKnotVector(
    absl::Span<const Pose3d> control_points, int spline_degree);

// Returns a knot vector of `knot_vector_type` for a B-spline of `spline_degree`
// constructed around the given `control_points`.
absl::StatusOr<std::vector<double>> MakeKnotVector(
    BSplineKnotVectorSelection knot_vector_type,
    absl::Span<const eigenmath::VectorNd> control_points, int spline_degree);

// Returns an OkStatus if the number of knots and control points of the given
// `spline` are non-zero, and the knot vector contains at least two distinct
// values.
absl::Status ValidateBSpline(const BSplineBase& spline,
                             double knot_equality_threshold =
                                 BSplineBase::kDefaultKnotEqualityThreshold);

// For a given `spline` with knot parameter range [knot_start, knot_end],
// returns the number of samples obtained in the range [knot_start,
// `knot_parameter`] when sampling the knot parameter range with a uniform
// `knot_step`, with the additional constraints that at least
// `min_samples_per_knot_span` samples should be taken in every non-zero knot
// span, and that a number no less than `min_samples` should be returned.
// Returns an error status if the spline has an empty knot vector, if the
// `knot_parameter` is outside the valid range, or if any of the additional
// parameters is non-positive.
absl::StatusOr<int> GetNumSamplesForKnotParameter(const BSplineBase& spline,
                                                  double knot_parameter,
                                                  double knot_step,
                                                  int min_samples_per_knot_span,
                                                  int min_samples);

// Returns the squared norm of a B-spline `point`. The norm is defined as
// Dot(`point`, `point`), with Dot() being the inner product defined by the
// spline `Traits` template parameter. Returns an error if the point is empty.
template <typename Traits>
icon::RealtimeStatusOr<double> SquaredNorm(
    const typename BSplineT<Traits>::Point& point) {
  if (Traits::Size(point) == 0) {
    return icon::InvalidArgumentError("Point is empty.");
  }
  return Traits::Dot(point, point);
}

// Returns the norm of a B-spline `point`. The norm is defined as the square
// root of Dot(`point`, `point`), with Dot() being the inner product defined by
// the spline `Traits` template parameter. Returns an error if the point is
// empty.
template <typename Traits>
icon::RealtimeStatusOr<double> Norm(
    const typename BSplineT<Traits>::Point& point) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(const double squared_norm,
                                SquaredNorm<Traits>(point));
  return std::sqrt(squared_norm);
}

// Computes the curvature at a B-spline point for which `first_path_derivative`
// and `second_path_derivative` are provided. For a B-spline point P the
// curvature is defined as
//
//       sqrt(||P'||^2 * ||P''||^2 - Dot(P', P'')^2)
// k = ----------------------------------------------
//                      ||P'||^3
//
// where P' and P'' are the first and second path derivatives at the point P,
// respectively, and ||.|| is the norm defined from the Dot() operator of the
// spline `Traits` template parameter.
template <typename Traits>
absl::StatusOr<double> BSplineCurvature(
    const typename Traits::Point& first_path_derivative,
    const typename Traits::Point& second_path_derivative) {
  if (Traits::Size(first_path_derivative) == 0) {
    return absl::InvalidArgumentError("First path derivative is empty.");
  }
  if (Traits::Size(second_path_derivative) == 0) {
    return absl::InvalidArgumentError("Second path derivative is empty.");
  }
  if (Traits::Size(first_path_derivative) !=
      Traits::Size(second_path_derivative)) {
    return absl::InvalidArgumentError(absl::StrCat(
        "First and second path derivatives must have the same size. Got ",
        Traits::Size(first_path_derivative),
        " != ", Traits::Size(second_path_derivative)));
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(const double first_derivative_norm,
                                Norm<Traits>(first_path_derivative));
  if (intrinsic::AlmostEquals(first_derivative_norm, 0.0)) {
    // Handle the case where the first derivative is zero (curvature formula
    // would return 0/0).
    return 0.0;
  }
  const double first_derivative_norm_squared =
      ::intrinsic::IPow(first_derivative_norm, 2);

  INTRINSIC_RT_ASSIGN_OR_RETURN(const double second_derivative_norm,
                                Norm<Traits>(second_path_derivative));
  const double second_derivative_norm_squared =
      ::intrinsic::IPow(second_derivative_norm, 2);
  const double dot_product =
      Traits::Dot(first_path_derivative, second_path_derivative);
  const double dot_product_squared = ::intrinsic::IPow(dot_product, 2);

  // Clamp the sqrt argument to 0 to avoid negative values due to small
  // numerical errors.
  const double sqrt_arg = std::max(
      0.0, first_derivative_norm_squared * second_derivative_norm_squared -
               dot_product_squared);

  const double curvature_numerator = std::sqrt(sqrt_arg);
  const double curvature_denominator =
      ::intrinsic::IPow(first_derivative_norm, 3);

  return curvature_numerator / curvature_denominator;
}

// Returns the curvature of the given `bspline` at the curve parameter `u`. For
// a curve C(u), the curvature is defined as
//
//        sqrt(||C'(u)||^2 * ||C''(u)||^2 - Dot(C'(u), C''(u))^2)
// k(u) = -------------------------------------------------------
//                           ||C'(u)||^3
//
// where C'(u) and C''(u) are the first and second path derivatives,
// respectively, and ||.|| is the norm defined from the Dot() operator of the
// spline `Traits` template parameter.
template <typename Traits>
absl::StatusOr<double> BSplineCurvature(const BSplineT<Traits>& bspline,
                                        double u) {
  using SplinePoint = typename Traits::Point;
  INTR_RETURN_IF_ERROR(ValidateBSpline(bspline));

  // Evaluate the B-spline and its (first and second) path derivatives.
  // TODO(b/351978904) here we exploit implementation details of B-spline
  // Traits to guarantee points are initialized with correct size also for
  // dynamic-size types.
  SplinePoint zero_point(bspline.PointDim());
  Traits::Zero(zero_point);
  std::vector<SplinePoint> spline_sample(3, zero_point);
  if (!bspline.EvalCurveAndDerivatives(u, &spline_sample)) {
    return absl::InternalError(absl::StrCat(
        "Failed to evaluate the B-spline at u=", u,
        ". This might happen if u is outside the valid knot vector range."));
  }

  return BSplineCurvature<Traits>(spline_sample[1], spline_sample[2]);
}

template <typename Traits>
absl::StatusOr<std::vector<double>>
GetBSplineNormalizedCurveParametersSampledUniformlyBetweenKnots(
    const BSplineT<Traits>& bspline, double normalized_sampling_step,
    bool force_odd_number_of_samples) {
  if (normalized_sampling_step <= 0.0) {
    return absl::InvalidArgumentError(
        absl::StrCat("Normalized sampling step must be positive. Got ",
                     normalized_sampling_step, "."));
  }

  // Get spline knots and normalize them to the range [0,1].
  const int num_knots = bspline.NumKnots();
  std::vector<double> normalized_knots(num_knots);
  if (!bspline.GetKnotVector(&normalized_knots)) {
    return absl::InternalError("Failed to get knot vector from the spline.");
  }
  const double knot_normalizer = normalized_knots.back();
  for (double& knot : normalized_knots) {
    knot /= knot_normalizer;
  }

  std::vector<double> normalized_curve_parameters;
  normalized_curve_parameters.reserve(1.0 / normalized_sampling_step);

  const int spline_degree = bspline.Degree();
  int total_num_samples_so_far = 0;
  const int final_spline_knot_index = num_knots - spline_degree - 2;
  for (int i = spline_degree; i <= final_spline_knot_index; ++i) {
    const double start_knot = normalized_knots[i];
    const double end_knot = normalized_knots[i + 1];
    // We sample at least once per knot span, namely at the start knot.
    int expected_num_samples_in_knot_span =
        std::max(1, static_cast<int>(std::ceil((end_knot - start_knot) /
                                               normalized_sampling_step)));
    if (force_odd_number_of_samples) {
      total_num_samples_so_far += expected_num_samples_in_knot_span;
      if (i == final_spline_knot_index) {
        const int expected_total_num_samples = total_num_samples_so_far + 1;
        if (expected_total_num_samples % 2 == 0) {
          // We reduce by one only if there are more than one samples
          // in the current knot span.
          expected_num_samples_in_knot_span +=
              (expected_num_samples_in_knot_span > 1) ? -1 : 1;
        }
      }
    }
    const double knot_step_in_knot_span =
        (end_knot - start_knot) / expected_num_samples_in_knot_span;
    normalized_curve_parameters.push_back(start_knot);
    for (int j = 1; j < expected_num_samples_in_knot_span; ++j) {
      normalized_curve_parameters.push_back(
          (start_knot + static_cast<double>(j) * knot_step_in_knot_span));
    }
  }
  normalized_curve_parameters.push_back(normalized_knots.back());
  return normalized_curve_parameters;
}

// Estimates the number of samples per valid knot span for a given
// `sampling_step`. Note that only valid knot spans are counted, i.e., in case
// of unclamped splines the first and last `degree` spans will be skipped.
// Each internal knot is counted twice, as terminal sample of a knot span and as
// start sample of the following one. Therefore, the returned number of samples
// for each span is at least 2. Returns an error if the `spline` has less than
// two unique knots in the valid domain, and for non-positive `sampling_step`.
absl::StatusOr<std::vector<int>> NumSamplesPerValidKnotSpan(
    const BSplineBase& spline, double sampling_step);

}  // namespace intrinsic

#endif  // INTRINSIC_MATH_SPLINE_BSPLINE_UTILS_H_
