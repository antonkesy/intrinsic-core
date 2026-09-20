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

#include "intrinsic/motion_planning/trajectory_planning/interpolate_joint_trajectories.h"

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/time/time.h"
#include "intrinsic/eigenmath/interpolation.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/kinematics/types/joint_trajectories.h"
#include "intrinsic/math/spline/polynomial_spline.h"

namespace intrinsic {

icon::RealtimeStatusOr<JointStatePVAJ> InterpolateJointTrajectoryInternalType(
    const JointTrajectoryPVA& trajectory,
    absl::Duration time_since_trajectory_start) {
  JointStatePVAJ state;
  switch (trajectory.interpolation_type()) {
    case JointTrajectoryInterpolationType::kCubicPolynomial: {
      return InterpolateCubicWithVelocityContinuity(
          trajectory, time_since_trajectory_start);
    }
    case JointTrajectoryInterpolationType::kQuinticPolynomial: {
      return InterpolateQuintic(trajectory, time_since_trajectory_start);
    }
    default: {
      return icon::InvalidArgumentError(
          absl::StrCat("Unsupported interpolation type: ",
                       static_cast<int>(trajectory.interpolation_type())));
    }
  }
}

icon::RealtimeStatusOr<JointStatePVA> InterpolateQuadratic(
    const JointTrajectoryPVA& joint_trajectory,
    absl::Duration time_since_trajectory_start) {
  if (time_since_trajectory_start < absl::ZeroDuration()) {
    return icon::InvalidArgumentError(
        "time_since_trajectory_start < trajectory duration.");
  }

  if (time_since_trajectory_start > joint_trajectory.Duration()) {
    return icon::InvalidArgumentError(
        "time_since_trajectory_start > trajectory duration.");
  }

  // Get LHS index corresponding to the queried 'time'.
  const int lower_idx =
      joint_trajectory.GetLowerIndexForTime(time_since_trajectory_start);

  if (lower_idx >= joint_trajectory.size() - 1) {
    return JointStatePVA(joint_trajectory.data().back());
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(const absl::Duration t_lower,
                                joint_trajectory.TimeAt(lower_idx));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const absl::Duration t_upper,
                                joint_trajectory.TimeAt(lower_idx + 1));

  INTRINSIC_RT_ASSIGN_OR_RETURN(const JointStatePVA state_lower,
                                joint_trajectory.DataAt(lower_idx));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const JointStatePVA state_upper,
                                joint_trajectory.DataAt(lower_idx + 1));

  return EvaluateJointQuadraticSpline(
      /*start_joint_state=*/state_lower,
      /*target_joint_state=*/state_upper,
      /*time_horizon_seconds=*/absl::ToDoubleSeconds(t_upper - t_lower),
      /*time_eval_seconds=*/
      absl::ToDoubleSeconds(time_since_trajectory_start - t_lower));
}

icon::RealtimeStatusOr<JointStatePVAJ> InterpolateCubicWithVelocityContinuity(
    const JointTrajectoryPVA& joint_trajectory,
    absl::Duration time_since_trajectory_start) {
  if (time_since_trajectory_start < absl::ZeroDuration()) {
    return icon::InvalidArgumentError(
        "time_since_trajectory_start < trajectory duration.");
  }

  if (time_since_trajectory_start > joint_trajectory.Duration()) {
    return icon::InvalidArgumentError(
        "time_since_trajectory_start > trajectory duration.");
  }

  // Get LHS index corresponding to the queried 'time'.
  const int lower_idx =
      joint_trajectory.GetLowerIndexForTime(time_since_trajectory_start);

  if (lower_idx >= joint_trajectory.size() - 1) {
    JointStatePVAJ state;
    INTRINSIC_RT_RETURN_IF_ERROR(
        state.SetSize(joint_trajectory.data().back().size()));
    state.position = joint_trajectory.data().back().position;
    state.velocity = joint_trajectory.data().back().velocity;
    state.acceleration = joint_trajectory.data().back().acceleration;
    state.jerk.setZero();
    return state;
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(const absl::Duration t_lower,
                                joint_trajectory.TimeAt(lower_idx));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const absl::Duration t_upper,
                                joint_trajectory.TimeAt(lower_idx + 1));

  INTRINSIC_RT_ASSIGN_OR_RETURN(const JointStatePVA state_lower,
                                joint_trajectory.DataAt(lower_idx));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const JointStatePVA state_upper,
                                joint_trajectory.DataAt(lower_idx + 1));

  return EvaluateJointCubicSplineWithVelocityContinuity(
      /*start_joint_state=*/state_lower,
      /*target_joint_state=*/state_upper,
      /*time_horizon_seconds=*/absl::ToDoubleSeconds(t_upper - t_lower),
      /*time_eval_seconds=*/
      absl::ToDoubleSeconds(time_since_trajectory_start - t_lower));
}

icon::RealtimeStatusOr<JointStatePVA>
InterpolateCubicWithAccelerationContinuity(
    const JointTrajectoryPVA& joint_trajectory,
    absl::Duration time_since_trajectory_start) {
  if (time_since_trajectory_start < absl::ZeroDuration()) {
    return icon::InvalidArgumentError(
        "time_since_trajectory_start < trajectory duration.");
  }

  if (time_since_trajectory_start > joint_trajectory.Duration()) {
    return icon::InvalidArgumentError(
        "time_since_trajectory_start > trajectory duration.");
  }

  // Get LHS index corresponding to the queried 'time'.
  const int lower_idx =
      joint_trajectory.GetLowerIndexForTime(time_since_trajectory_start);

  if (lower_idx >= joint_trajectory.size() - 1) {
    return JointStatePVA(joint_trajectory.data().back());
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(const absl::Duration t_lower,
                                joint_trajectory.TimeAt(lower_idx));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const absl::Duration t_upper,
                                joint_trajectory.TimeAt(lower_idx + 1));

  INTRINSIC_RT_ASSIGN_OR_RETURN(const JointStatePVA state_lower,
                                joint_trajectory.DataAt(lower_idx));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const JointStatePVA state_upper,
                                joint_trajectory.DataAt(lower_idx + 1));

  return EvaluateJointCubicSplineWithAccelerationContinuity(
      /*start_joint_state=*/state_lower,
      /*target_joint_state=*/state_upper,
      /*time_horizon_seconds=*/absl::ToDoubleSeconds(t_upper - t_lower),
      /*time_eval_seconds=*/
      absl::ToDoubleSeconds(time_since_trajectory_start - t_lower));
}

icon::RealtimeStatusOr<JointStatePVAJ> InterpolateQuintic(
    const JointTrajectoryPVA& joint_trajectory,
    absl::Duration time_since_trajectory_start) {
  if (time_since_trajectory_start < absl::ZeroDuration()) {
    return icon::InvalidArgumentError(
        "time_since_trajectory_start < trajectory duration.");
  }

  if (time_since_trajectory_start > joint_trajectory.Duration()) {
    return icon::InvalidArgumentError(
        "time_since_trajectory_start > trajectory duration.");
  }

  // Get LHS index corresponding to the queried 'time'.
  const int lower_idx =
      joint_trajectory.GetLowerIndexForTime(time_since_trajectory_start);

  if (lower_idx >= joint_trajectory.size() - 1) {
    JointStatePVAJ last_state;
    const int ndofs = joint_trajectory.data().back().size();
    INTRINSIC_RT_RETURN_IF_ERROR(last_state.SetSize(ndofs));
    last_state.position = joint_trajectory.data().back().position;
    last_state.velocity = joint_trajectory.data().back().velocity;
    last_state.acceleration = joint_trajectory.data().back().acceleration;
    last_state.jerk.setZero();
    return last_state;
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(const absl::Duration t_lower,
                                joint_trajectory.TimeAt(lower_idx));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const absl::Duration t_upper,
                                joint_trajectory.TimeAt(lower_idx + 1));

  INTRINSIC_RT_ASSIGN_OR_RETURN(const JointStatePVA state_lower,
                                joint_trajectory.DataAt(lower_idx));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const JointStatePVA state_upper,
                                joint_trajectory.DataAt(lower_idx + 1));

  return EvaluateJointQuinticSpline(
      /*start_joint_state=*/state_lower,
      /*target_joint_state=*/state_upper,
      /*time_horizon_seconds=*/absl::ToDoubleSeconds(t_upper - t_lower),
      /*time_eval_seconds=*/
      absl::ToDoubleSeconds(time_since_trajectory_start - t_lower));
}

icon::RealtimeStatusOr<JointStateP> InterpolateLinear(
    const JointTrajectoryP& joint_trajectory,
    absl::Duration time_since_trajectory_start) {
  if (time_since_trajectory_start < absl::ZeroDuration()) {
    return icon::InvalidArgumentError(
        "time_since_trajectory_start < trajectory duration.");
  }

  if (time_since_trajectory_start > joint_trajectory.Duration()) {
    return icon::InvalidArgumentError(
        "time_since_trajectory_start > trajectory duration.");
  }

  // Get LHS index corresponding to the queried 'time_sec'.
  const int lower_idx =
      joint_trajectory.GetLowerIndexForTime(time_since_trajectory_start);

  if (lower_idx >= joint_trajectory.size() - 1) {
    return JointStateP(joint_trajectory.data().back());
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(const absl::Duration t_lower,
                                joint_trajectory.TimeAt(lower_idx));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const absl::Duration t_upper,
                                joint_trajectory.TimeAt(lower_idx + 1));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const JointStateP position_lower,
                                joint_trajectory.DataAt(lower_idx));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const JointStateP position_upper,
                                joint_trajectory.DataAt(lower_idx + 1));

  const double t_percentage = absl::FDivDuration(
      (time_since_trajectory_start - t_lower), (t_upper - t_lower));

  JointStateP result;
  INTRINSIC_RT_RETURN_IF_ERROR(result.SetSize(position_lower.size()));

  result.position = eigenmath::InterpolateLinear(
      t_percentage, position_lower.position, position_upper.position);

  return result;
}

absl::StatusOr<LimitCheckResult> InterpolationIsWithinLimits(
    const JointTrajectoryPVA& joint_trajectory, const JointLimits& joint_limits,
    int subsampling_rate) {
  const int num_intervals = joint_trajectory.size();

  LimitCheckResult limit_check_result;
  for (int i = 0; i < num_intervals - 1; ++i) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(auto t_lower, joint_trajectory.TimeAt(i));
    INTRINSIC_RT_ASSIGN_OR_RETURN(auto t_upper, joint_trajectory.TimeAt(i + 1));
    for (absl::Duration eval_time = t_lower; eval_time <= t_upper;
         eval_time += (t_upper - t_lower) / subsampling_rate) {
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          JointStatePVA subsample,
          InterpolateQuintic(joint_trajectory, eval_time));
      INTRINSIC_RT_ASSIGN_OR_RETURN(limit_check_result,
                                    IsWithinLimits(subsample, joint_limits));
      if (limit_check_result == false) {
        return limit_check_result;
      }
    }
  }

  return limit_check_result;
}

icon::RealtimeStatusOr<double> InterpolateCartesianArcLength(
    const JointTrajectoryPVA& trajectory,
    absl::Duration time_since_trajectory_start) {
  if (time_since_trajectory_start < absl::ZeroDuration()) {
    return icon::InvalidArgumentError(
        "time_since_trajectory_start < trajectory duration.");
  }

  if (time_since_trajectory_start > trajectory.Duration()) {
    return icon::InvalidArgumentError(
        "time_since_trajectory_start > trajectory duration.");
  }

  if (!trajectory.HasCartesianArcLength()) {
    return icon::InvalidArgumentError(
        "trajectory does not have Cartesian arc lengths.");
  }

  // Get LHS index corresponding to the queried 'time_sec'.
  const int lower_idx =
      trajectory.GetLowerIndexForTime(time_since_trajectory_start);

  if (lower_idx >= trajectory.size() - 1) {
    return trajectory.cartesian_arc_lengths().value().back();
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(const absl::Duration t_lower,
                                trajectory.TimeAt(lower_idx));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const absl::Duration t_upper,
                                trajectory.TimeAt(lower_idx + 1));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const double cartesian_arc_length_lower,
                                trajectory.CartesianArcLengthAt(lower_idx));
  INTRINSIC_RT_ASSIGN_OR_RETURN(const double cartesian_arc_length_upper,
                                trajectory.CartesianArcLengthAt(lower_idx + 1));

  const double t_percentage = absl::FDivDuration(
      (time_since_trajectory_start - t_lower), (t_upper - t_lower));

  return eigenmath::InterpolateLinear(t_percentage, cartesian_arc_length_lower,
                                      cartesian_arc_length_upper);
}

}  // namespace intrinsic
