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

#include "intrinsic/motion_planning/trajectory_planning/topp/phase_space_target_generators.h"

#include <algorithm>
#include <cmath>

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/almost_equals.h"
#include "intrinsic/math/numopt/polytope_utils.h"

namespace intrinsic::topp {

bool IsStateAtRest(const PhaseSpacePoint& phase_state) {
  return AlmostEquals(phase_state.b, 0.0) && AlmostEquals(phase_state.bp, 0.0);
}

absl::StatusOr<ConvexHullResult2d> GetTargetPointCvxHull(
    const PhaseSpacePoint& target_point, const double tolerance) {
  if (target_point.b < 0.0) {
    return absl::InvalidArgumentError(
        "The target squared path velocity must be non-negative.");
  }
  if (tolerance <= 0.0) {
    return absl::InvalidArgumentError(
        "The tolerance must be strictly positive.");
  }

  // Compute the limits for `b` and `bp`, such that `b` is non-negative.
  const double b_min = std::max(0.0, target_point.b - tolerance);
  const double b_max = target_point.b + tolerance;

  const double bp_min = target_point.bp - tolerance;
  const double bp_max = target_point.bp + tolerance;

  return ComputeConvexHull2d(/*candidate_vertices_2d=*/{{b_min, bp_min},
                                                        {b_max, bp_min},
                                                        {b_max, bp_max},
                                                        {b_min, bp_max}},
                             GeometrySolverType::kAndrewMonotoneChain);
}

absl::StatusOr<ConvexHullResult2d> GetTargetLineCvxHull(
    const PhaseSpacePoint& p1, const PhaseSpacePoint& p2,
    const double tolerance) {
  if (p1.b < 0.0 || p2.b < 0.0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The target squared path velocity must be non-negative. Got p1: ", p1.b,
        " and p2: ", p2.b, "."));
  }
  if (tolerance <= 0.0) {
    return absl::InvalidArgumentError(
        "The tolerance must be strictly positive.");
  }

  ConvexHullResult2d result;
  result.A.resize(4, 2);
  result.b.resize(4);

  // Determine the min and max boundaries along the bp axis.
  const double bp_min = std::min(p1.bp, p2.bp);
  const double bp_max = std::max(p1.bp, p2.bp);

  const double bp_top = bp_max + tolerance;
  const double bp_bottom = bp_min - tolerance;

  // Edge case safeguard: Vertical line in phase space (constant bp).
  if (std::abs(p1.bp - p2.bp) < 1.0e-9) {
    const double b_min = std::min(p1.b, p2.b) - tolerance;
    const double b_max = std::max(p1.b, p2.b) + tolerance;

    // 1. Right bound: b <= b_max
    result.A.row(0) << 1.0, 0.0;
    result.b(0) = b_max;

    // 2. Left bound: -b <= -b_min
    result.A.row(1) << -1.0, 0.0;
    result.b(1) = -b_min;

    // 3. Top bound: bp <= bp_top
    result.A.row(2) << 0.0, 1.0;
    result.b(2) = bp_top;

    // 4. Bottom bound: -bp <= -bp_bottom
    result.A.row(3) << 0.0, -1.0;
    result.b(3) = -bp_bottom;

    result.vertices = {
        {b_max, bp_top},     // Top-Right
        {b_min, bp_top},     // Top-Left
        {b_min, bp_bottom},  // Bottom-Left
        {b_max, bp_bottom}   // Bottom-Right
    };

    return result;
  }

  // Generalized analytic construction: Calculate slope (m) and intercept (c).
  const double m = (p2.b - p1.b) / (p2.bp - p1.bp);
  const double c = p1.b - m * p1.bp;

  // H-representation (Inequalities `A * x <= b`).

  // 1. Right/Top line boundary: b - m * bp <= c + tol
  result.A.row(0) << 1.0, -m;
  result.b(0) = c + tolerance;

  // 2. Left/Bottom line boundary: -b + m * bp <= -c + tol
  result.A.row(1) << -1.0, m;
  result.b(1) = -c + tolerance;

  // 3. Upper acceleration limit: bp <= bp_top
  result.A.row(2) << 0.0, 1.0;
  result.b(2) = bp_top;

  // 4. Lower acceleration limit: -bp <= -bp_bottom
  result.A.row(3) << 0.0, -1.0;
  result.b(3) = -bp_bottom;

  // V-representation (Vertices)
  // Helper lambda to find the center line's `b` value at a given `bp`
  auto b_center = [m, c](double bp) { return m * bp + c; };

  // Stored in Counter-Clockwise (CCW) order
  result.vertices = {
      {b_center(bp_top) + tolerance, bp_top},        // Top-Right
      {b_center(bp_top) - tolerance, bp_top},        // Top-Left
      {b_center(bp_bottom) - tolerance, bp_bottom},  // Bottom-Left
      {b_center(bp_bottom) + tolerance, bp_bottom}   // Bottom-Right
  };

  return result;
}

absl::StatusOr<ConvexHullResult2d> CreateLaunchIntervalCvxHull(
    const double ds, const double bp_limit, const double tolerance) {
  // In the launch interval, the path acceleration must be positive.
  if (bp_limit <= 0.0) {
    return absl::InvalidArgumentError(
        "Launch `bp_limit` must be strictly positive.");
  }
  if (ds <= 0.0) {
    return absl::InvalidArgumentError(
        "The path distance `ds` must be strictly positive.");
  }

  const PhaseSpacePoint p1{.b = 0.0, .bp = 0.0};
  const PhaseSpacePoint p2{.b = 0.75 * ds * bp_limit, .bp = bp_limit};

  return GetTargetLineCvxHull(p1, p2, tolerance);
}

absl::StatusOr<ConvexHullResult2d> CreateLandIntervalCvxHull(
    const double ds, const double bp_limit, const double tolerance) {
  // In the land interval, the path acceleration (brake) must be negative.
  if (bp_limit >= 0.0) {
    return absl::InvalidArgumentError(
        "Land `bp_limit` must be strictly negative.");
  }
  if (ds <= 0.0) {
    return absl::InvalidArgumentError(
        "The path distance `ds` must be strictly positive.");
  }

  const PhaseSpacePoint p1{.b = 0.0, .bp = 0.0};
  // The negative sign for `b` makes sure this is always positive.
  const PhaseSpacePoint p2{.b = -0.75 * ds * bp_limit, .bp = bp_limit};

  return GetTargetLineCvxHull(p1, p2, tolerance);
}

}  // namespace intrinsic::topp
