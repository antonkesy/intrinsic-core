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

#ifndef INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_PHASE_SPACE_TARGET_GENERATORS_H_
#define INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_PHASE_SPACE_TARGET_GENERATORS_H_

#include "intrinsic/math/numopt/polytope_utils.h"

namespace intrinsic {
namespace topp {

// Constant zero tolerance used for validating non-negativity of squared path
// velocity (`b`) and evaluation in phase space operations.
constexpr double kNonNegativeSquaredPathVelocityTolerance = 1.0e-12;

// Container of a point in phase space, referring to a point composed by `b(s) =
// ds/dt^2` known as the squared path velocity and `bp(s) = db(s)/ds` known as
// the squared path velocity first derivative.
struct PhaseSpacePoint {
  // Holds the the squared path velocity `b(s) = ds/dt^2`
  double b;

  // Holds the squared path velocity first derivative `bp(s) = db(s)/ds`.
  double bp;
};

// Returns true if both components (b,bp) of the `phase_state` are zero.
bool IsStateAtRest(const PhaseSpacePoint& phase_state);

// Computes a 2D convex hull representing a tolerance box around a target state.
//
// Generates a rectangular bounding box centered at (target_b, target_bp) with
// extents defined by `tolerance`. The resulting convex hull is constrained such
// that the first coordinate (squared path velocity, b) is always strictly
// non-negative. Note that if the combination of `target_b` and `tolerance`
// leads to negative values, those values will be clamped to zero and the box
// center will be shifted.
//
// `target_b` is the target squared path velocity. Must be non-negative.
// `target_bp` is the target rate of change of the squared path velocity.
// `tolerance` is the half-width and half-height of the bounding box. Must be
// strictly positive.
//
// Returns the computed convex hull on success. Returns an error if `target_b` <
// 0.0 or if `tolerance` <= 0.0.
absl::StatusOr<ConvexHullResult2d> GetTargetPointCvxHull(
    const PhaseSpacePoint& target_point, const double tolerance = 1.0e-4);

// Computes a 2D convex hull representing a tolerance parallelogram around a
// target line in phase space.
//
// The target line is defined by two points, `p1` and `p2`, in a phase space
// where the coordinates are (squared path velocity `b`, and its rate of change
// `bp`). The function generates a bounding region by expanding the line segment
// symmetrically by `tolerance` along both dimensions. The resulting shape is a
// 4-sided parallelogram, returned with both its V-representation (4 vertices in
// Counter-Clockwise order) and H-representation (4 halfspace inequalities).
//
// `p1`: The first target point in phase space. Its `b` value must be >= 0.0.
// `p2`: The second target point in phase space. Its `b` value must be >= 0.0.
// `tolerance`: The symmetric half-width tolerance applied around the target
// line's boundaries. Must be strictly positive.
//
// Returns a ConvexHullResult2d containing the mathematically consistent
// vertices and halfspaces of the tolerance region. Returns an error if
// `tolerance` <= 0.0 or if either target point has a negative `b` component.
absl::StatusOr<ConvexHullResult2d> GetTargetLineCvxHull(
    const PhaseSpacePoint& p1, const PhaseSpacePoint& p2,
    const double tolerance);

// Creates a 2D convex hull representing the boundary constraints for a
// launch interval in phase space.
//
// This function generates a bounding parallelogram (both H-representation and
// V-representation) for a trajectory segment starting from rest. The geometric
// target line is formed between the origin (0.0, 0.0) and the boundary state
// (0.75 * ds * bp_limit, bp_limit). Refer to
// go/intrinsic-topp-trajectory-launch-and-landing-intervals for details on the
// derivation of these values.
//
// `ds` is the spatial step size of the interval. Must be strictly positive.
// `bp_limit` is the maximum acceleration limit for the launch. Must be
// strictly positive.
// `tolerance` is the geometric margin applied along the phase space axes to
// expand the bounding hull. Defaults to 1.0e-4.
//
// Returns the constructed convex hull on success, or an error if the inputs
// violate physical constraints (e.g., negative ds or bp_limit <= 0).
absl::StatusOr<ConvexHullResult2d> CreateLaunchIntervalCvxHull(
    const double ds, const double bp_limit, const double tolerance = 1.0e-4);

// Creates a 2D convex hull representing the boundary constraints for a
// landing (braking) interval in phase space.
//
// This function generates a bounding parallelogram for a trajectory segment
// that decelerates to a complete stop. The geometric target line is formed
// between the required initial state (-0.75 * ds * bp_limit, bp_limit) and the
// target origin (0.0, 0.0). Refer to
// go/intrinsic-topp-trajectory-launch-and-landing-intervals for details on the
// derivation of these values.
//
// `ds` is the spatial step size of the interval. Must be strictly positive.
// `bp_limit` is the maximum deceleration limit for the landing. Must be
// strictly negative.
// `tolerance` is the geometric margin applied along the phase space axes to
// expand the bounding hull. Defaults to 1.0e-4.
//
// Returns the constructed convex hull on success, or an error if the inputs
// violate physical constraints (e.g., negative ds or bp_limit >= 0).
absl::StatusOr<ConvexHullResult2d> CreateLandIntervalCvxHull(
    const double ds, const double bp_limit, const double tolerance = 1.0e-4);

}  // namespace topp
}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_PHASE_SPACE_TARGET_GENERATORS_H_
