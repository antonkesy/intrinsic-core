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


#include "intrinsic/math/spline/bspline_utils.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <vector>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/manifolds.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/spline/bspline_base.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

namespace {

// For a vector `delta` representing the difference vector between two adjacent
// corners, returns an offset vector of length `radius` having same direction as
// `delta`. However, the length of the offset vector is bounded by
// `max_offset_length_percentage` times the length of `delta`.
// `max_offset_length_percentage` is intended as a value in the range [0,1].
eigenmath::VectorNd CornerOffset(const eigenmath::VectorNd& delta,
                                 double radius,
                                 double max_offset_length_percentage) {
  // For robustness, accept values slightly outside the allowed range, but clamp
  // them afterwards.
  constexpr double kNumericTolerance = 1e-10;
  CHECK_GE(radius, -kNumericTolerance);
  CHECK_GE(max_offset_length_percentage, -kNumericTolerance);
  CHECK_LE(max_offset_length_percentage, 1.0 + kNumericTolerance);

  radius = std::clamp(radius, 0.0, std::numeric_limits<double>::max());
  max_offset_length_percentage =
      std::clamp(max_offset_length_percentage, 0.0, 1.0);

  const double delta_norm = delta.norm();
  const eigenmath::VectorNd delta_normalized = delta.normalized();

  if (radius < max_offset_length_percentage * delta_norm) {
    return delta_normalized * radius;
  } else {
    return delta_normalized * max_offset_length_percentage * delta_norm;
  }
}

}  // namespace

absl::StatusOr<std::vector<eigenmath::VectorNd>> PolyLineToBspline3Waypoints(
    const std::vector<eigenmath::VectorNd>& corners,
    const std::vector<double>& radii) {
  if (corners.size() < 2) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Vector of corners must have size >= 2. Got ", corners.size(), "."));
  }

  if (radii.size() != corners.size()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Blending radii size(), which is ", radii.size(),
        " does not fit to corner-vector size, which is ", corners.size(),
        ". Expected blending radii vector to have same size."));
  }

  eigenmath::VectorNd::Index ndof{corners.front().rows()};

  // check for consistently sized points
  for (auto& corn : corners) {
    CHECK_EQ(ndof, corn.rows());
  }

  // Construct bspline path by inserting one point before and one after each
  // corner of the piecewise-linear path. This ensures straight lines between
  // inner points and rounded corners.
  std::vector<eigenmath::VectorNd> result(3 * corners.size() - 2);
  // copy corners as spline control points
  for (size_t idx = 0; idx < corners.size(); idx++) {
    result[3 * idx] = corners[idx];
  }

  // Add remaining (inner) points at radius-offset from corners on straight
  // lines connecting corners. To make sure inner points maintain ordering along
  // the path, the radius-offset length should be bounded to half the distance
  // between the two involved corners. We reduce this further to 1/3 to avoid
  // inner points to coincide, creating duplicate spline control points that
  // would undermine the smoothness of the spline path derivatives. This choice
  // guarantees that inner points are equispaced between corners in the worst
  // case scenario, which produces the best results in terms of smoothness of
  // the spline path derivatives. See go/intrinsic-bspline-inner-points-spacing
  // for results and more details on this topic.
  // Please do not modify this value without prior consulting with mdfiore@ or
  // giftthaler@.
  constexpr double kMaxOffsetLengthPercentage = 1.0 / 3.0;

  // The for loop is skipped for the special case with two corners (no blend).
  eigenmath::VectorNd offset(ndof);
  for (size_t idx = 1; idx < corners.size() - 1; ++idx) {
    const size_t k = 3 * idx;
    const size_t knext = 3 * (idx + 1);
    const size_t klast = 3 * (idx - 1);
    const size_t kp = k + 1;
    const size_t km = k - 1;

    offset = CornerOffset(result[knext] - result[k], radii[idx],
                          kMaxOffsetLengthPercentage);
    result[kp] = result[k] + offset;

    offset = CornerOffset(result[klast] - result[k], radii[idx],
                          kMaxOffsetLengthPercentage);
    result[km] = result[k] + offset;
  }

  // Special-case first and last inner points.
  offset = CornerOffset(result[3] - result[0], radii.front(),
                        kMaxOffsetLengthPercentage);
  result[1] = result[0] + offset;

  const std::size_t sz = result.size();
  offset = CornerOffset(result[sz - 4] - result[sz - 1], radii.back(),
                        kMaxOffsetLengthPercentage);
  result[sz - 2] = result[sz - 1] + offset;

  return result;
}

namespace {

// For a pose `delta` representing the difference between two adjacent Cartesian
// corner points, returns a pose  offset having same direction as `delta` and
// length computed as the most restrictive radius between `translation_radius`,
// and `rotation_radius`. Restrictivness is computed considering the ratio
// between translational/rotational radius and the norm of the
// translational part/rotational angle of `delta`. The most conservative ratio
// is bounded by `max_offset_length_percentage`, which must be in the range
// [0,1].
Pose3d CornerOffset(const Pose3d& delta, double translation_radius,
                    double rotation_radius,
                    double max_offset_length_percentage) {
  // For robustness, accept values slightly outside the allowed range, but clamp
  // them afterwards.
  constexpr double kNumericTolerance = 1e-10;
  CHECK_GE(translation_radius, -kNumericTolerance);
  CHECK_GE(rotation_radius, -kNumericTolerance);
  CHECK_GE(max_offset_length_percentage, -kNumericTolerance);
  CHECK_LE(max_offset_length_percentage, 1.0 + kNumericTolerance);

  translation_radius =
      std::clamp(translation_radius, 0.0, std::numeric_limits<double>::max());
  rotation_radius =
      std::clamp(rotation_radius, 0.0, std::numeric_limits<double>::max());
  max_offset_length_percentage =
      std::clamp(max_offset_length_percentage, 0.0, 1.0);

  static constexpr double kMinRadius = 1e-6;
  if (translation_radius < kMinRadius || rotation_radius < kMinRadius) {
    // zero radius -> extra control point will be added on top of existing point
    return Pose3d::Identity();
  }

  // Calculate translation norm and rotation angle.
  const double translation_norm = delta.translation().norm();
  Eigen::AngleAxis<double> delta_rotation(delta.quaternion());
  const double rotation_angle = delta_rotation.angle();

  // Use most conservative offset percentage to not exceed either radius.
  double offset_pct_trans = translation_norm == 0.0
                                ? std::numeric_limits<double>::infinity()
                                : translation_radius / translation_norm;
  double offset_pct_rot = rotation_angle == 0.0
                              ? std::numeric_limits<double>::infinity()
                              : rotation_radius / rotation_angle;
  double offset_pct = std::min(offset_pct_trans, offset_pct_rot);

  if (offset_pct > max_offset_length_percentage) {
    offset_pct = max_offset_length_percentage;
  }

  // Calculate offset as percentage of delta.
  delta_rotation.angle() *= offset_pct;
  return Pose3d{eigenmath::Quaterniond(delta_rotation),
                eigenmath::Vector3d(delta.translation() * offset_pct)};
}

}  // namespace

absl::StatusOr<std::vector<Pose3d>> PolyLineToBspline3Waypoints(
    const std::vector<Pose3d>& corners,
    const std::vector<double>& translation_radii,
    const std::vector<double>& rotation_radii) {
  INTRINSIC_ASSERT_NON_REALTIME();
  if (corners.size() < 2) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Vector of corners must have size >= 2. Got ", corners.size(), "."));
  }

  if (translation_radii.size() != corners.size()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Blending translation_radii size(), which is ",
        translation_radii.size(), " does not match corners size, which is ",
        corners.size(), ". Expected blending radii vector to have same size "));
  }

  if (translation_radii.size() != rotation_radii.size()) {
    return absl::InvalidArgumentError(
        absl::StrCat("Blending translation_radii size(), which is ",
                     translation_radii.size(),
                     " does not match rotation_radii size(), which is ",
                     rotation_radii.size(),
                     ". Expected blending radii vector to both have size ",
                     translation_radii.size(), "."));
  }

  // Construct b-spline path by inserting one point before and one after each
  // corner of the piecewise-linear path. This ensures straight lines between
  // inner points and rounded corners.
  std::vector<Pose3d> result(3 * corners.size() - 2);
  // Copy corners as spline control points.
  for (size_t idx = 0; idx < corners.size(); ++idx) {
    result[3 * idx] = corners[idx];
  }

  // Add remaining points at radius-offset from corners on straight line
  // connecting corners. To make sure inner points maintain ordering along
  // the path, the radius-offset length is bounded to half of the most
  // restrictive distance (translational/rotational) between the two involved
  // corners. Note that this choice still allows inner points to coincide.
  constexpr double kMaxOffsetLengthPercentage = 0.5;

  for (size_t idx = 1; idx < corners.size() - 1; ++idx) {
    const size_t k = 3 * idx;
    const size_t knext = 3 * (idx + 1);  // next corner
    const size_t klast = 3 * (idx - 1);  // previous corner
    const size_t kp = k + 1;             // inner point after corner
    const size_t km = k - 1;             // inner point before corner

    Pose3d current_t_next_corner = result[k].inverse() * result[knext];
    Pose3d current_t_next_inner_point =
        CornerOffset(current_t_next_corner, translation_radii[idx],
                     rotation_radii[idx], kMaxOffsetLengthPercentage);
    result[kp] = result[k] * current_t_next_inner_point;

    Pose3d current_T_previous_corner = result[k].inverse() * result[klast];
    Pose3d current_T_previous_inner_point =
        CornerOffset(current_T_previous_corner, translation_radii[idx],
                     rotation_radii[idx], kMaxOffsetLengthPercentage);
    result[km] = result[k] * current_T_previous_inner_point;
  }

  // Special-case: first and last inner point.
  Pose3d first_t_next_corner = result[0].inverse() * result[3];
  Pose3d first_t_first_inner_point =
      CornerOffset(first_t_next_corner, translation_radii.front(),
                   rotation_radii.front(), kMaxOffsetLengthPercentage);
  result[1] = result[0] * first_t_first_inner_point;

  const std::size_t sz = result.size();
  Pose3d last_t_previous_corner = result[sz - 1].inverse() * result[sz - 4];
  Pose3d last_t_last_inner_point =
      CornerOffset(last_t_previous_corner, translation_radii.back(),
                   rotation_radii.back(), kMaxOffsetLengthPercentage);
  result[sz - 2] = result[sz - 1] * last_t_last_inner_point;

  return result;
}

namespace {

constexpr double kMinDataPolygonLength = 1e-6;

// Computes the accumulated, weighted "lengths" of the data polygon. Given
// control_points P_0, P_1, ..., P_n, entry k of the resulting vector v will
// contain
//
// v[k] = \sum_{i=1}^{k}(|P_i - P_{i-1}|^alpha)
//
// Select alpha = 1.0 for computing the accumulated chord lengths of the data
// polygon.
absl::StatusOr<std::vector<double>> ComputeAccumulatedLengthsOfDataPolygon(
    absl::Span<const eigenmath::VectorNd> control_points,
    const double weight_exponent_alpha) {
  if (control_points.empty()) {
    return absl::InvalidArgumentError("Data points are empty.");
  }

  // Recursively compute the accumulated length of the control polygon. The
  // first entry will (of course) be zero. The last entry corresponds to the
  // total length of the control polygon.
  std::vector<double> weighted_lengths_since_polygon_start(
      control_points.size(), 0.0);
  for (int i = 1; i < control_points.size(); ++i) {
    weighted_lengths_since_polygon_start[i] =
        weighted_lengths_since_polygon_start[i - 1] +
        std::pow((control_points[i] - control_points[i - 1]).norm(),
                 weight_exponent_alpha);
  }

  return weighted_lengths_since_polygon_start;
}

// Computes the accumulated, weighted "lengths" of the Pose3 data polygon. Given
// poses P_0, P_1, ..., P_n, entry k of the resulting vector v will
// contain
//
// v[k] = \sum_{i=1}^{k}(|P_i - P_{i-1}|^alpha)
//
// where |P_i - P_{i-1}| is realized as the l2-norm of the SE3 log-map of P_i
// w.r.t. P_{i-1}. Select alpha = 1.0 for computing the accumulated chord
// lengths of the pose data polygon.
absl::StatusOr<std::vector<double>> ComputeAccumulatedLengthsOfDataPolygon(
    absl::Span<const Pose3d> control_points,
    const double weight_exponent_alpha) {
  if (control_points.empty()) {
    return absl::InvalidArgumentError("Data points are empty.");
  }

  // Recursively compute the accumulated length of the control polygon in the
  // tangent space of SE3. The first entry will be zero. The last
  // entry corresponds to the total length of the control polygon steps in local
  // SE3 tanget spaces.
  std::vector<double> weighted_lengths_since_polygon_start(
      control_points.size(), 0.0);
  for (int i = 1; i < control_points.size(); ++i) {
    eigenmath::Vector6d pose_increment_log_map =
        eigenmath::logRiemann(control_points[i - 1], control_points[i]);
    weighted_lengths_since_polygon_start[i] =
        weighted_lengths_since_polygon_start[i - 1] +
        std::pow(pose_increment_log_map.norm(), weight_exponent_alpha);
  }

  return weighted_lengths_since_polygon_start;
}

// Returns a normalized weighted average knot vector for a B-spline of
// `spline_degree`, computed from `accumulated_weighted_chord_lengths`. If the
// accumulated chord length is zero, a uniform knot vector is returned instead.
absl::StatusOr<std::vector<double>> ComputeWeightedAverageKnotVector(
    const std::vector<double>& accumulated_weighted_chord_lengths,
    const int spline_degree) {
  const int num_knots = BSplineBase::NumKnots(
      accumulated_weighted_chord_lengths.size(), spline_degree);
  std::vector<double> knots(num_knots, 0.0);

  // Fall back to uniform knot vector in case the data polygon has length zero.
  if (accumulated_weighted_chord_lengths.back() < kMinDataPolygonLength) {
    if (!BSplineBase::MakeUniformKnotVector(
            accumulated_weighted_chord_lengths.size(), &knots, spline_degree)) {
      return absl::InternalError("Could not create uniform knot vector.");
    }
    return knots;
  }

  // Normalize the accumulated chord-lengths.
  std::vector<double> accumulated_weighted_chord_lengths_normalized =
      accumulated_weighted_chord_lengths;
  Eigen::Map<eigenmath::VectorNd>(
      accumulated_weighted_chord_lengths_normalized.data(),
      accumulated_weighted_chord_lengths_normalized.size()) /=
      accumulated_weighted_chord_lengths.back();

  // Initialize boundaries of knot vector.
  std::fill(knots.begin(), knots.begin() + spline_degree + 1, 0.0);
  std::fill(knots.end() - spline_degree - 1, knots.end(), 1.0);

  // The intermediate knot points are a moving average of the accumulated chord
  // lengths. The size of the moving average window is the spline degree.
  for (int i = spline_degree + 1; i < num_knots - spline_degree - 1; ++i) {
    knots[i] =
        std::accumulate(
            accumulated_weighted_chord_lengths_normalized.begin() + i -
                spline_degree,
            accumulated_weighted_chord_lengths_normalized.begin() + i, 0.0) /
        static_cast<double>(spline_degree);
  }

  return knots;
}

}  // namespace

absl::StatusOr<std::vector<double>> MakeAverageChordLengthKnotVector(
    absl::Span<const eigenmath::VectorNd> control_points, int spline_degree) {
  if (spline_degree < 0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Spline degree must be non-negative. Got ", spline_degree, "."));
  }
  const int num_points = control_points.size();
  if (num_points < BSplineBase::MinNumPoints(spline_degree)) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Too few points: need >= ", BSplineBase::MinNumPoints(spline_degree),
        ", but got ", num_points, "."));
  }

  INTR_ASSIGN_OR_RETURN(std::vector<double> length_since_polygon_start,
                        ComputeAccumulatedLengthsOfDataPolygon(
                            control_points, /*weight_exponent_alpha=*/1.0));

  return ComputeWeightedAverageKnotVector(length_since_polygon_start,
                                          spline_degree);
}

absl::StatusOr<std::vector<double>> MakeAverageCentripetalKnotVector(
    absl::Span<const eigenmath::VectorNd> control_points, int spline_degree) {
  if (spline_degree < 0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Spline degree must be non-negative. Got ", spline_degree, "."));
  }
  const int num_points = control_points.size();
  if (num_points < BSplineBase::MinNumPoints(spline_degree)) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Too few points: need >= ", BSplineBase::MinNumPoints(spline_degree),
        ", but got ", num_points, "."));
  }

  // For the centripetal method, we choose a weighting factor of 0.5.
  INTR_ASSIGN_OR_RETURN(std::vector<double> length_since_polygon_start,
                        ComputeAccumulatedLengthsOfDataPolygon(
                            control_points, /*weight_exponent_alpha=*/0.5));

  return ComputeWeightedAverageKnotVector(length_since_polygon_start,
                                          spline_degree);
}

absl::StatusOr<std::vector<double>> MakeAverageChordLengthKnotVector(
    absl::Span<const Pose3d> control_points, int spline_degree) {
  if (spline_degree < 0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Spline degree must be non-negative. Got ", spline_degree, "."));
  }
  const int num_points = control_points.size();
  if (num_points < BSplineBase::MinNumPoints(spline_degree)) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Too few points: need >= ", BSplineBase::MinNumPoints(spline_degree),
        ", but got ", num_points, "."));
  }

  INTR_ASSIGN_OR_RETURN(std::vector<double> length_since_polygon_start,
                        ComputeAccumulatedLengthsOfDataPolygon(
                            control_points, /*weight_exponent_alpha=*/1.0));

  return ComputeWeightedAverageKnotVector(length_since_polygon_start,
                                          spline_degree);
}

absl::StatusOr<std::vector<double>> MakeAverageCentripetalKnotVector(
    absl::Span<const Pose3d> control_points, int spline_degree) {
  if (spline_degree < 0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Spline degree must be non-negative. Got ", spline_degree, "."));
  }
  const int num_points = control_points.size();
  if (num_points < BSplineBase::MinNumPoints(spline_degree)) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Too few points: need >= ", BSplineBase::MinNumPoints(spline_degree),
        ", but got ", num_points, "."));
  }

  // For the centripetal method, we choose a weighting factor of 0.5.
  INTR_ASSIGN_OR_RETURN(std::vector<double> length_since_polygon_start,
                        ComputeAccumulatedLengthsOfDataPolygon(
                            control_points, /*weight_exponent_alpha=*/0.5));

  return ComputeWeightedAverageKnotVector(length_since_polygon_start,
                                          spline_degree);
}

absl::StatusOr<std::vector<double>> MakeKnotVector(
    const BSplineKnotVectorSelection knot_vector_type,
    absl::Span<const eigenmath::VectorNd> control_points,
    const int spline_degree) {
  if (spline_degree < 0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Spline degree must be non-negative. Got ", spline_degree, "."));
  }
  if (control_points.size() < BSplineBase::MinNumPoints(spline_degree)) {
    return absl::InvalidArgumentError(
        absl::StrCat("Too few control points: need >= ",
                     BSplineBase::MinNumPoints(spline_degree), ", but got ",
                     control_points.size(), "."));
  }

  switch (knot_vector_type) {
    case BSplineKnotVectorSelection::kUniformKnotVector: {
      std::vector<double> knots;
      knots.reserve(
          BSplineBase::NumKnots(control_points.size(), spline_degree));
      if (!BSplineBase::MakeUniformKnotVector(control_points.size(), &knots,
                                              spline_degree)) {
        return absl::InternalError("Couldn't make uniform knot vector.");
      }
      return knots;
    }
    case BSplineKnotVectorSelection::kAverageChordLengthKnotVector: {
      return MakeAverageChordLengthKnotVector(control_points, spline_degree);
    };
    case BSplineKnotVectorSelection::kCentripetalKnotVector: {
      return MakeAverageCentripetalKnotVector(control_points, spline_degree);
    }
  }
}

absl::Status ValidateBSpline(const BSplineBase& spline,
                             const double knot_equality_threshold) {
  if (spline.NumKnots() == 0) {
    return absl::InvalidArgumentError("Invalid B-spline: empty knot vector.");
  }
  if (spline.NumPoints() == 0) {
    return absl::InvalidArgumentError("Invalid B-spline: no control points.");
  }

  std::vector<double> knots(spline.NumKnots());
  if (!spline.GetKnotVector(&knots)) {
    return absl::InternalError("Failed to get the knot vector.");
  }

  if (knots.back() - knots.front() < knot_equality_threshold) {
    return absl::InvalidArgumentError("Invalid B-spline: all knots are equal.");
  }

  return absl::OkStatus();
}

absl::StatusOr<int> GetNumSamplesForKnotParameter(const BSplineBase& spline,
                                                  double knot_parameter,
                                                  double knot_step,
                                                  int min_samples_per_knot_span,
                                                  int min_samples) {
  if (spline.NumKnots() == 0) {
    return absl::InvalidArgumentError(
        "The spline knot vector must not be empty.");
  }
  if (knot_step <= 0) {
    return absl::InvalidArgumentError(
        absl::StrCat("The knot step must be positive. Got ", knot_step, "."));
  }
  if (min_samples_per_knot_span < 1) {
    return absl::InvalidArgumentError(
        absl::StrCat("The minimum number of samples per knot span must be "
                     "positive. Got ",
                     min_samples_per_knot_span, "."));
  }
  if (min_samples < 1) {
    return absl::InvalidArgumentError(
        absl::StrCat("The minimum number of samples must be positive. Got ",
                     min_samples_per_knot_span, "."));
  }

  std::vector<double> knots(spline.NumKnots());
  if (!spline.GetKnotVector(&knots)) {
    return absl::InternalError("Failed to get the knot vector.");
  }
  if (knot_parameter > knots.back() || knot_parameter < knots.front()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The knot parameter is outside the valid range. Got ", knot_parameter,
        " but valid range is [", knots.front(), ",", knots.back(), "]."));
  }

  const double knot_parameter_interval = knot_parameter - knots.front();
  const int num_samples_from_knot_interval =
      std::round(knot_parameter_interval / knot_step) + 1;

  const int knot_span = spline.KnotSpan(knot_parameter);
  const int num_samples_from_knot_spans = min_samples_per_knot_span * knot_span;

  return std::max({min_samples, num_samples_from_knot_interval,
                   num_samples_from_knot_spans});
}

absl::StatusOr<std::vector<int>> NumSamplesPerValidKnotSpan(
    const BSplineBase& spline, const double sampling_step) {
  if (!(sampling_step > 0.0)) {
    return absl::InvalidArgumentError(
        "The sampling step must be strictly positive.");
  }
  std::vector<double> unique_knots;
  if (!spline.GetValidUniqueKnots(unique_knots)) {
    return absl::InvalidArgumentError("Couldn't get the unique spline knots.");
  }
  if (unique_knots.size() < 2) {
    return absl::InvalidArgumentError(
        "The spline knot vector must have at least two unique knots.");
  }

  std::vector<int> num_samples_per_knot_span;
  num_samples_per_knot_span.reserve(unique_knots.size() - 1);

  for (size_t i = 0; i + 1 < unique_knots.size(); ++i) {
    const double start_knot = unique_knots[i];
    const double end_knot = unique_knots[i + 1];
    num_samples_per_knot_span.push_back(
        1 + std::max(1, static_cast<int>(std::ceil((end_knot - start_knot) /
                                                   sampling_step))));
  }

  return num_samples_per_knot_span;
}

}  // namespace intrinsic
