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

#ifndef INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_INTERPOLATE_JOINT_TRAJECTORIES_H_
#define INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_INTERPOLATE_JOINT_TRAJECTORIES_H_

#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/kinematics/types/joint_trajectories.h"

namespace intrinsic {

// Interpolates a discretized `JointTrajectoryPVA` using the internal
// interpolation type specified within the trajectory itself.
//
// Parameters:
//  * `trajectory`: The discretized joint trajectory to interpolate.
//  * `time_since_trajectory_start`: The time at which to evaluate the
//  trajectory.
//
// Returns:
//  A `JointStatePVAJ` (Position, Velocity, Acceleration, Jerk) interpolated
//  at the requested time. Returns an `InvalidArgumentError` if
//  `time_since_trajectory_start` is outside the valid range `[0,
//  trajectory.Duration()]` or if the internal interpolation type is
//  unsupported.
icon::RealtimeStatusOr<JointStatePVAJ> InterpolateJointTrajectoryInternalType(
    const JointTrajectoryPVA& trajectory,
    absl::Duration time_since_trajectory_start);

// Performs a quadratic spline interpolation of a discretized
// `JointTrajectoryPVA`.
//
// Parameters:
//  * `joint_trajectory_pva`: The joint trajectory to interpolate.
//  * `time_since_trajectory_start`: The time at which to evaluate the
//  trajectory.
//
// Returns:
//  A `JointStatePVA` (Position, Velocity, Acceleration) interpolated at the
//  requested time. Returns an `InvalidArgumentError` if
//  `time_since_trajectory_start` is outside the valid range `[0,
//  trajectory.Duration()]`.
// TODO(bponton): Optimize by using as input only JointTrajectoryPA.
icon::RealtimeStatusOr<JointStatePVA> InterpolateQuadratic(
    const JointTrajectoryPVA& joint_trajectory_pva,
    absl::Duration time_since_trajectory_start);

// Performs a cubic spline interpolation of a discretized `JointTrajectoryPVA`,
// explicitly enforcing velocity continuity across via-points.
//
// Parameters:
//  * `joint_trajectory_pva`: The joint trajectory to interpolate.
//  * `time_since_trajectory_start`: The time at which to evaluate the
//  trajectory.
//
// Returns:
//  A `JointStatePVAJ` interpolated at the requested time. Returns an
//  `InvalidArgumentError` if `time_since_trajectory_start` is outside the valid
//  range `[0, trajectory.Duration()]`.
icon::RealtimeStatusOr<JointStatePVAJ> InterpolateCubicWithVelocityContinuity(
    const JointTrajectoryPVA& joint_trajectory_pva,
    absl::Duration time_since_trajectory_start);

// Performs a cubic spline interpolation of a discretized `JointTrajectoryPVA`,
// explicitly enforcing acceleration continuity across via-points.
//
// Parameters:
//  * `joint_trajectory_pva`: The joint trajectory to interpolate.
//  * `time_since_trajectory_start`: The time at which to evaluate the
//  trajectory.
//
// Returns:
//  A `JointStatePVA` interpolated at the requested time. Returns an
//  `InvalidArgumentError` if `time_since_trajectory_start` is outside the valid
//  range `[0, trajectory.Duration()]`.
icon::RealtimeStatusOr<JointStatePVA>
InterpolateCubicWithAccelerationContinuity(
    const JointTrajectoryPVA& joint_trajectory_pva,
    absl::Duration time_since_trajectory_start);

// Performs a quintic spline interpolation of a discretized
// `JointTrajectoryPVA`.
//
// Parameters:
//  * `joint_trajectory_pva`: The joint trajectory to interpolate.
//  * `time_since_trajectory_start`: The time at which to evaluate the
//  trajectory.
//
// Returns:
//  A `JointStatePVAJ` interpolated at the requested time. Returns an
//  `InvalidArgumentError` if `time_since_trajectory_start` is outside the valid
//  range `[0, trajectory.Duration()]`.
icon::RealtimeStatusOr<JointStatePVAJ> InterpolateQuintic(
    const JointTrajectoryPVA& joint_trajectory_pva,
    absl::Duration time_since_trajectory_start);

// Performs a simple linear interpolation of a discretized `JointTrajectoryP`.
//
// Parameters:
//  * `joint_trajectory_p`: The position-only joint trajectory to interpolate.
//  * `time_since_trajectory_start`: The time at which to evaluate the
//  trajectory.
//
// Returns:
//  A `JointStateP` (Position) representing the linear interpolation at the
//  requested time. Returns an `InvalidArgumentError` if
//  `time_since_trajectory_start` is outside the valid range `[0,
//  trajectory.Duration()]`
icon::RealtimeStatusOr<JointStateP> InterpolateLinear(
    const JointTrajectoryP& joint_trajectory_p,
    absl::Duration time_since_trajectory_start);

// Subsamples a joint trajectory to verify that all intermediate states fall
// within the specified physical limits.
//
// Note: This function strictly utilizes quintic spline interpolation internally
// to generate the sub-samples between the trajectory's discrete time steps.
//
// Parameters:
//  * `joint_trajectory`: The trajectory to check.
//  * `joint_limits`: The maximum bounds for position, velocity, and
//  acceleration.
//  * `subsampling_rate`: The number of sub-samples to evaluate per trajectory
//    interval (default: 10).
//
// Returns:
//  A `LimitCheckResult` indicating which (if any) limits were violated.
absl::StatusOr<LimitCheckResult> InterpolationIsWithinLimits(
    const JointTrajectoryPVA& joint_trajectory, const JointLimits& joint_limits,
    int subsampling_rate = 10);

// Extracts the translational Cartesian arc length from a trajectory and
// evaluates it using linear interpolation.
//
// Parameters:
//  * `trajectory`: The trajectory containing the Cartesian arc lengths.
//  * `time_since_trajectory_start`: The time at which to evaluate the arc
//  length.
//
// Returns:
//  The linearly interpolated Cartesian arc length. Returns an error if
//  `time_since_trajectory_start` is outside the valid range `[0,
//  trajectory.Duration()]`, or if the provided trajectory does not have valid
//  Cartesian arc lengths populated.
icon::RealtimeStatusOr<double> InterpolateCartesianArcLength(
    const JointTrajectoryPVA& trajectory,
    absl::Duration time_since_trajectory_start);

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_INTERPOLATE_JOINT_TRAJECTORIES_H_
