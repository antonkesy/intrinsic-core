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

#ifndef INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_PATH_VELOCITY_PROPAGATOR_H_
#define INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_PATH_VELOCITY_PROPAGATOR_H_

#include "absl/status/statusor.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/topp_solver_commons.h"

namespace intrinsic {
namespace topp {

// Propagates squared path velocity bounds `b = (ds/dt)^2` forward along the
// path to compute the maximal feasible and controllable velocity profile under
// joint and Cartesian constraints.
//
// This function integrates the phase-space trajectory in the (b, bp) domain
// from the start of the path to the end. At each discrete interval `ds`, it
// solves a local linear program (LP) to maximize the downstream squared path
// velocity (`b[k+1] = b[k] + ds * bp[k]`) subject to the intersection of:
// 1. Upper-bound velocity envelopes (e.g., from joint velocity or Cartesian
//    velocity limits, also from reachable speed computed in a backward pass).
// 2. Joint acceleration or torque boundaries mapped to phase-space.
// 3. Cartesian translational/rotational 2-norm acceleration constraints modeled
//    as inner octagon approximations.
//
// The `path_increments` are the spatial grid step sizes `ds[k] = s[k+1] - s[k]`
// between consecutive path samples. Must have a size of `N-1`.
// The `squared_path_velocity_bounds` are the maximum allowable squared path
// velocity limit envelope at each waypoint. Typically represents the
// element-wise minimum of joint velocity limits/Cartesian velocity limits. Must
// have a size of `N`.
// `joint_constraints` represent the joint acceleration or joint torque
// constraints basis functions mapped per waypoint. Can be empty if no
// joint-space limits are imposed.
// `cartesian_constraints` represent the Cartesian translational and rotational
// 2-norm tracking limit wrappers mapped per waypoint. Can be empty if no
// Cartesian workspace acceleration limits are active.
// `start_squared_path_velocity` is the boundary condition specifying the
// starting squared path speed `b[0]` at the initial waypoint. Defaults to 0.0
// (starting from a stop).
// `skip_data_validation` is a flag to bypass checking the size consistency of
// the datatypes inside the input containers. Set to true only where safety and
// dimension validation have already been guaranteed upstream.
// `use_analytical_forward_pass` is a flag to select the forward pass
// integration method. When true, uses a 1D analytical method that fixes initial
// velocity per step. When false (default), solves a 2D linear program (LP).
//
// Returns a vector of size `N` containing the integrated, maximally accelerated
// valid squared path velocities `b[k]` for each waypoint. Returns an error if
// the backward transition becomes physically infeasible, or there are input
// dimensions mismatch or inconsistencies in the data when validation is active.
absl::StatusOr<std::vector<double>> PropagateSquaredPathVelocities(
    absl::Span<const double> path_increments,
    absl::Span<const double> squared_path_velocity_bounds,
    absl::Span<const JointConstraintStep> joint_constraints,
    absl::Span<const PhaseSpaceCartesianConstraint> cartesian_constraints,
    const double start_squared_path_velocity = 0.0,
    bool skip_data_validation = false,
    bool use_analytical_forward_pass = false);

}  // namespace topp
}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TOPP_PATH_VELOCITY_PROPAGATOR_H_
