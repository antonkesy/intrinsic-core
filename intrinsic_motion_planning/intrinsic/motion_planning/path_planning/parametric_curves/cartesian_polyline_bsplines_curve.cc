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

#include "intrinsic/motion_planning/path_planning/parametric_curves/cartesian_polyline_bsplines_curve.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/pose3_utils.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/spline/bspline.h"
#include "intrinsic/math/spline/bspline_base.h"
#include "intrinsic/math/spline/bspline_utils.h"
#include "intrinsic/motion_planning/path_planning/parametric_curves/cartesian_parametric_curve.h"
#include "intrinsic/motion_planning/path_planning/parametric_curves/parametric_curve_utils.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

constexpr int kDefaultSplineDegree = 4;

CartesianPolylineBSplinesCurve::CartesianPolylineBSplinesCurve(
    double rotational_blending_radius, double translational_blending_radius)
    : rotational_blending_radius_(rotational_blending_radius),
      translational_blending_radius_(translational_blending_radius) {}

/*static*/
absl::StatusOr<std::unique_ptr<CartesianPolylineBSplinesCurve>>
CartesianPolylineBSplinesCurve::Create(
    const CartesianBlendingArcParameters& blending_arc_parameters) {
  const eigenmath::PoseError blending_1st_arm_pose_difference =
      eigenmath::PoseErrorBetween(blending_arc_parameters.blending_corner,
                                  blending_arc_parameters.start);
  const eigenmath::PoseError blending_2nd_arm_pose_difference =
      eigenmath::PoseErrorBetween(blending_arc_parameters.blending_corner,
                                  blending_arc_parameters.end);
  return CartesianPolylineBSplinesCurve::Create(
      /*rotational_blending_radius=*/std::min(
          blending_1st_arm_pose_difference.rotation,
          blending_2nd_arm_pose_difference.rotation),
      /*translational_blending_radius=*/
      std::min(blending_1st_arm_pose_difference.translation,
               blending_2nd_arm_pose_difference.translation),
      /*pose_waypoints=*/
      {blending_arc_parameters.start, blending_arc_parameters.blending_corner,
       blending_arc_parameters.end});
}

/*static*/
absl::StatusOr<std::unique_ptr<CartesianPolylineBSplinesCurve>>
CartesianPolylineBSplinesCurve::Create(
    double rotational_blending_radius, double translational_blending_radius,
    absl::Span<const Pose3d> pose_waypoints) {
  if (rotational_blending_radius <= 0.0) {
    return absl::InvalidArgumentError(
        absl::StrCat("Rotational blending radius must be positive. Got ",
                     rotational_blending_radius, "."));
  }
  if (translational_blending_radius <= 0.0) {
    return absl::InvalidArgumentError(
        absl::StrCat("Translational blending radius must be positive. Got ",
                     translational_blending_radius, "."));
  }

  // WrapUnique since constructor is private.
  std::unique_ptr<CartesianPolylineBSplinesCurve>
      cartesian_polyline_bsplines_curve =
          absl::WrapUnique(new CartesianPolylineBSplinesCurve(
              rotational_blending_radius, translational_blending_radius));
  INTR_RETURN_IF_ERROR(
      cartesian_polyline_bsplines_curve->SetWaypoints(pose_waypoints));
  return std::move(cartesian_polyline_bsplines_curve);
}

absl::Status CartesianPolylineBSplinesCurve::FitSplinesToWaypoints(
    absl::Span<const Pose3d> waypoints) {
  const std::vector<Pose3d> pose_waypoints = {waypoints.begin(),
                                              waypoints.end()};

  std::vector<Pose3d> pose_control_points;
  int spline_degree = kDefaultSplineDegree;
  if (waypoints.size() > 2) {
    const std::vector<double> translation_radii(pose_waypoints.size(),
                                                translational_blending_radius_);
    const std::vector<double> rotation_radii(pose_waypoints.size(),
                                             rotational_blending_radius_);
    INTR_ASSIGN_OR_RETURN(
        pose_control_points,
        PolyLineToBspline3Waypoints(pose_waypoints, translation_radii,
                                    rotation_radii));
  } else {
    // Single-line path. We duplicate the `pose_control_points` to have enough
    // points to build a spline with degree `kDefaultSplineDegree`.
    pose_control_points = pose_waypoints;
    while (pose_control_points.size() < kDefaultSplineDegree + 1) {
      pose_control_points.insert(pose_control_points.begin(),
                                 pose_control_points.front());
      pose_control_points.push_back(pose_control_points.back());
    }
  }
  const int num_control_points = pose_control_points.size();
  const double knots_size =
      BSplineBase::NumKnots(num_control_points, spline_degree);

  // Allocate extra capacity for knots (in case it is needed later for
  // replanning) plus a minimum number.
  constexpr int kKnotCapacitySafetyFactor = 2;
  constexpr int kMinKnotCapacity = 100;
  const int knot_capacity =
      std::max(static_cast<int>(kKnotCapacitySafetyFactor * knots_size),
               kMinKnotCapacity);
  if (!translation_spline_.Init(spline_degree, knot_capacity)) {
    return absl::InternalError("Couldn't init translation spline.");
  }
  if (!rotation_spline_.Init(spline_degree, knot_capacity)) {
    return absl::InternalError("Couldn't init rotation spline.");
  }

  // To synchronize translational and rotational progress, the two curves are
  // generated from the same knot vector. We use an average chord knot vector
  // type to account for the distance between control points. This knot spacing
  // technique approximates the arc-length parameterization of the curve, which
  // is convenient when sampling the curve.
  std::vector<double> translation_knots(knots_size);
  INTR_ASSIGN_OR_RETURN(
      translation_knots,
      MakeAverageChordLengthKnotVector(pose_control_points, spline_degree));
  std::vector<double> rotation_knots = translation_knots;

  // Scale knot values to account for the total length of the translational and
  // rotational curves. We use the length of the control polygons as curve
  // length.
  length_ = {.translation_m = 0.0, .rotation_rad = 0.0};
  for (int i = 0; i + 1 < pose_control_points.size(); ++i) {
    const eigenmath::PoseError delta = eigenmath::PoseErrorBetween(
        pose_control_points[i], pose_control_points[i + 1]);
    length_.rotation_rad += delta.rotation;
    length_.translation_m += delta.translation;
  }
  // Scale knot values according to the curve length. To prevent zero knot
  // vectors and, thus, division by zero in the evaluations of the curves, we
  // add a small epsilon to the scaling factor.
  Eigen::Map<eigenmath::VectorNd>(translation_knots.data(),
                                  translation_knots.size()) *=
      (length_.translation_m + std::numeric_limits<double>::epsilon());
  Eigen::Map<eigenmath::VectorNd>(rotation_knots.data(),
                                  rotation_knots.size()) *=
      (length_.rotation_rad + std::numeric_limits<double>::epsilon());

  if (!translation_spline_.SetKnotVector(translation_knots)) {
    return absl::InternalError("Coudn't set translation knot vector");
  }
  if (!rotation_spline_.SetKnotVector(rotation_knots)) {
    return absl::InternalError("Coudn't set rotation knot vector");
  }

  std::vector<eigenmath::Vector3d> translation_control_points(
      num_control_points);
  std::vector<eigenmath::Quaterniond> rotation_control_points(
      num_control_points);
  for (int idx = 0; idx < num_control_points; idx++) {
    translation_control_points[idx] = pose_control_points[idx].translation();
    rotation_control_points[idx] = pose_control_points[idx].quaternion();
  }

  if (!translation_spline_.SetControlPoints(translation_control_points)) {
    return absl::InternalError("Couldn't set translation control points.");
  }
  if (!rotation_spline_.SetControlPoints(rotation_control_points)) {
    return absl::InternalError("Couldn't set rotation control points.");
  }

  return absl::OkStatus();
}

absl::StatusOr<Pose3d> CartesianPolylineBSplinesCurve::Sample(
    double normalized_curve_parameter) const {
  constexpr double kSmallTolerance = 1e-5;
  if (normalized_curve_parameter < -kSmallTolerance ||
      normalized_curve_parameter > 1 + kSmallTolerance) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Normalized curve parameter must be in the range [0,1]. Got",
        normalized_curve_parameter, "."));
  }

  const double translation_curve_parameter =
      std::clamp(normalized_curve_parameter * length_.translation_m, 0.0,
                 length_.translation_m);
  const double rotation_curve_parameter =
      std::clamp(normalized_curve_parameter * length_.rotation_rad, 0.0,
                 length_.rotation_rad);

  eigenmath::Vector3d translation = eigenmath::Vector3d::Zero();
  eigenmath::Quaterniond rotation = eigenmath::Quaterniond::Identity();
  if (!translation_spline_.EvalCurve(translation_curve_parameter,
                                     &translation)) {
    return absl::InternalError(absl::StrFormat(
        "Couldn't evaluate translation spline for curve parameter=%f",
        translation_curve_parameter));
  }
  if (!rotation_spline_.EvalCurve(rotation_curve_parameter, &rotation)) {
    return absl::InternalError(absl::StrFormat(
        "Couldn't evaluate rotation spline for curve parameter=%f",
        rotation_curve_parameter));
  }
  return Pose3d{rotation, translation};
}

absl::StatusOr<std::vector<Pose3d>> CartesianPolylineBSplinesCurve::Sample(
    absl::Span<const double> normalized_curve_parameters) const {
  std::vector<Pose3d> sampled_poses;
  sampled_poses.reserve(normalized_curve_parameters.size());

  for (const auto& normalized_curve_parameter : normalized_curve_parameters) {
    INTR_ASSIGN_OR_RETURN(const Pose3d pose,
                          Sample(normalized_curve_parameter));
    sampled_poses.push_back(pose);
  }

  return sampled_poses;
}

absl::StatusOr<PosesAtNormalizedCurveParameters>
CartesianPolylineBSplinesCurve::SampleUniformly(
    double normalized_sampling_step, bool force_odd_number_of_samples) const {
  return SampleUniformlyBetweenKnots(normalized_sampling_step,
                                     force_odd_number_of_samples);
}

absl::StatusOr<PosesAtNormalizedCurveParameters>
CartesianPolylineBSplinesCurve::SampleUniformlyBetweenKnots(
    double normalized_sampling_step, bool force_odd_number_of_samples) const {
  INTR_ASSIGN_OR_RETURN(
      const std::vector<double> normalized_curve_parameters,
      GetBSplineNormalizedCurveParametersSampledUniformlyBetweenKnots(
          translation_spline_, normalized_sampling_step,
          force_odd_number_of_samples));
  INTR_ASSIGN_OR_RETURN(const std::vector<Pose3d> poses,
                        Sample(normalized_curve_parameters));
  return PosesAtNormalizedCurveParameters{
      .poses = std::move(poses),
      .normalized_curve_parameters = std::move(normalized_curve_parameters)};
}

absl::Status CartesianPolylineBSplinesCurve::SetWaypoints(
    absl::Span<const Pose3d> pose_waypoints) {
  if (pose_waypoints.size() < 2) {
    return absl::InvalidArgumentError(
        absl::StrCat("At least two Cartesian waypoints are required. Got ",
                     pose_waypoints.size(), "."));
  }
  return FitSplinesToWaypoints(pose_waypoints);
}

}  // namespace intrinsic
