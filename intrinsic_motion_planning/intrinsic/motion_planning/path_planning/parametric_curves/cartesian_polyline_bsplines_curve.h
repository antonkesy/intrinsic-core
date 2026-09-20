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

#ifndef INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PARAMETRIC_CURVES_CARTESIAN_POLYLINE_BSPLINES_CURVE_H_
#define INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PARAMETRIC_CURVES_CARTESIAN_POLYLINE_BSPLINES_CURVE_H_

#include <memory>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/spline/bspline.h"
#include "intrinsic/math/spline/bsplineq.h"
#include "intrinsic/motion_planning/path_planning/parametric_curves/cartesian_parametric_curve.h"
#include "intrinsic/motion_planning/path_planning/parametric_curves/parametric_curve_utils.h"

namespace intrinsic {

// Represents a B-Spline fitted over a polyline defined by Cartesian waypoints.
// Two splines are internally defined, one for translation and one for rotation.
// Blending radii can be specified separately for translation and rotation.
// However, only the most conservative radius (in terms of translational/angular
// distance between consecutive waypoint poses in percentage) is applied. The
// splines will follow the polyline for those poses that present a distance
// greater than the most conservative radius from the waypoints.
class CartesianPolylineBSplinesCurve : public CartesianParametricCurve {
 public:
  static constexpr absl::string_view kParametricCurveName =
      "CartesianPolylineBSplinesCurve";

  absl::string_view Name() const override { return kParametricCurveName; }

  // Creates a `CartesianPolylineBSplinesCurve` from `blending_arc_parameters`
  // (containing the `blending_corner`, `start`, and `end` poses of the arc).
  // The `translational_blending_radius` parameter of
  // `CartesianPolylineBSplinesCurve` will automatically be determined by the
  // smaller one (or minimum) of either:
  // * the translational difference between `blending_corner` and `start`, or
  // * the translational difference between `blending_corner` and `end`.
  // Similarly, the `rotational_blending_radius` parameter of
  // `CartesianPolylineBSplinesCurve` will automatically be determined by the
  // smaller one (or minimum) of either:
  // * the rotational difference between `blending_corner` and `start`, or
  // * the rotational difference between `blending_corner` and `end`.
  static absl::StatusOr<std::unique_ptr<CartesianPolylineBSplinesCurve>> Create(
      const CartesianBlendingArcParameters& blending_arc_parameters);

  static absl::StatusOr<std::unique_ptr<CartesianPolylineBSplinesCurve>> Create(
      double rotational_blending_radius, double translational_blending_radius,
      absl::Span<const Pose3d> pose_waypoints);

  absl::StatusOr<Pose3d> Sample(
      double normalized_curve_parameter) const override;

  absl::StatusOr<std::vector<Pose3d>> Sample(
      absl::Span<const double> normalized_curve_parameters) const override;

  absl::StatusOr<PosesAtNormalizedCurveParameters> SampleUniformly(
      double normalized_sampling_step,
      bool force_odd_number_of_samples) const override;

  CartesianCurveLength GetCurveLength() const override { return length_; }

  // Sets the waypoints that define the polyline. This will recompute the
  // underlying splines.
  absl::Status SetWaypoints(absl::Span<const Pose3d> pose_waypoints);

 protected:
  // Samples the spline piecewise uniformly between each pair of consecutive and
  // distinct knots. The sampling step is normalized, i.e. the sampling step is
  // scaled by the curve length. The number of samples is determined by the
  // sampling step and the knot span, so that it produces an integer number of
  // samples. It returns the sampled poses and the normalized knot parameters.
  // If `force_odd_number_of_samples` is true, the total number of samples is
  // forced to be odd.
  absl::StatusOr<PosesAtNormalizedCurveParameters> SampleUniformlyBetweenKnots(
      double normalized_sampling_step,
      bool force_odd_number_of_samples = false) const;

 private:
  CartesianPolylineBSplinesCurve(double rotational_blending_radius,
                                 double translational_blending_radius);

  absl::Status FitSplinesToWaypoints(absl::Span<const Pose3d> waypoints);

  double rotational_blending_radius_;
  double translational_blending_radius_;
  BSpline3d translation_spline_;
  BSplineQ rotation_spline_;
  CartesianCurveLength length_ = {0.0, 0.0};
};

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_PATH_PLANNING_PARAMETRIC_CURVES_CARTESIAN_POLYLINE_BSPLINES_CURVE_H_
