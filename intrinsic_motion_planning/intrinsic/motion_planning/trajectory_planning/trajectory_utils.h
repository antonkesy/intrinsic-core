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

#ifndef INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TRAJECTORY_UTILS_H_
#define INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TRAJECTORY_UTILS_H_

#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_trajectories.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/path_sample.h"

namespace intrinsic {

struct ZeroCrossingCount {
  // Dimension-wise number of zeros, where the left and right boundary values
  // are not evaluated and counted as zeros explicitly.
  Eigen::VectorXi num_zeros_inner;

  // Dimension-wise number of zeros, where the left and right boundary values
  // are explicitly evaluated and counted as zeros.
  Eigen::VectorXi num_zeros_including_boundaries;
};

// Computes the number of zero crossings for the `time_series`. The boundary
// values are interpreted as separate zeros if their magnitude is below
// `boundary_zero_threshold`. Only "proper" zero crossings are counted, in the
// sense that after the zero-crossing, the signal must switch its sign. If there
// is no sign switch, e.g. in case of a `plateau` at zero, this is not
// interpreted as a zero crossing. Returns `kFailedPreconditionError` in case
// `time_series` is empty, its elements have zero size, its elements have
// different size, or `boundary_zero_threshold` is negative.
absl::StatusOr<ZeroCrossingCount> CountTimeSeriesZeroCrossings(
    absl::Span<const eigenmath::VectorNd> time_series,
    double boundary_zero_threshold = 1e-8);

// Time-scales `trajectory` by `time_scaling_factor`. Returns a trajectory which
// is `time_scaling_factor` times as long as the input trajectory, where
// velocities and accelerations are re-computed to achieve differential
// consistency. `time_scaling_factor` must be greater than zero.
absl::StatusOr<JointTrajectoryPVA> TimeScale(
    const JointTrajectoryPVA& trajectory, double time_scaling_factor);

// Returns a time-scaled `trajectory` which adheres to velocity and acceleration
// `limits`. Returns original trajectory if no limit violation detected.
absl::StatusOr<JointTrajectoryPVA> DownscaleIfViolatingLimits(
    const JointTrajectoryPVA& trajectory, const JointLimits& limits);

// Returns an InvalidArgumentError if any trajectory.data().at(i) exceeds the
// Cartesian position limits at the flange of the `chain`.
absl::Status IsTrajectoryWithinCartesianPositionLimits(
    const kinematics::Chain& chain, const CartesianLimits& cartesian_limits,
    const JointTrajectoryPVA& trajectory);

// Returns a no motion trajectory consisting of two identical joint states with
// zero velocity and acceleration at the given joint configuration.
absl::StatusOr<JointTrajectoryPVA> ZeroMotionTrajectory(
    const eigenmath::VectorNd& zero_motion_joint_configuration);

// Returns a no motion trajectory consisting of two identical joint states with
// zero velocity and acceleration at the given joint configuration. Returns an
// error in case the current joint configuration has a size greater than
// VectorNd::MaxSizeAtCompileTime.
absl::StatusOr<JointTrajectoryPVA> ZeroMotionTrajectory(
    const eigenmath::VectorXd& zero_motion_joint_configuration);

// Returns a no motion path consisting of two identical joint path samples with
// zero derivatives at the given joint configuration.
absl::StatusOr<std::vector<topp::PathSample>> ZeroMotionPath(
    const eigenmath::VectorNd& zero_motion_joint_configuration);

}  // namespace intrinsic

#endif  // INTRINSIC_MOTION_PLANNING_TRAJECTORY_PLANNING_TRAJECTORY_UTILS_H_
