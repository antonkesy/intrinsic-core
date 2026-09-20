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

#include "intrinsic/icon/control/algorithms/speed_override_factor_limits_computation.h"

#include <algorithm>
#include <memory>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/time/time.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/algorithms/speed_override_factor_limits_utils.h"
#include "intrinsic/icon/control/algorithms/speed_override_factor_types.h"
#include "intrinsic/icon/dynamics/rigid_body_interface.h"
#include "intrinsic/icon/dynamics/validate_rigid_body_interface.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/dynamic_limits_check_mode.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/kinematics/types/joint_trajectories.h"
#include "intrinsic/motion_planning/trajectory_planning/interpolate_joint_trajectories.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

namespace {

// Valid phase range.
static constexpr double kMinimumPhase = 0.0;
static constexpr double kMaximumPhase = 1.0;

// The allowed epsilon used to check if the phase is within the range
// [`kMinimumPhase`, `kMaximumPhase`].
static constexpr double kAllowedPhaseEpsilon = 1.0e-6;

// Validates that
// - the input `trajectory` is non-empty.
// - the input `trajectory` has a valid interpolation type.
// - the `joint_limits` match in size to the input `trajectory`.
// - if the trajectory is torque limited, that the dynamics model is valid.
// Returns an error if any of the above conditions is not met.
absl::Status ValidateInputs(const JointTrajectoryPVA& trajectory,
                            const JointLimits& joint_limits,
                            RigidBodyInterface* dynamics) {
  if (trajectory.data().empty()) {
    return absl::InvalidArgumentError("The input trajectory cannot be empty.");
  }

  if (trajectory.data().front().position.size() != joint_limits.size()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The input trajectory and joint limits have different dimensions. Got ",
        trajectory.data().front().position.size(), " vs. ", joint_limits.size(),
        "."));
  }

  if (trajectory.interpolation_type() ==
      JointTrajectoryInterpolationType::kUnspecified) {
    return absl::InvalidArgumentError("Unspecified interpolation type.");
  }

  if (trajectory.joint_dynamic_limits_check_mode() ==
      DynamicLimitsCheckMode::kCheckNone) {
    INTR_RETURN_IF_ERROR(
        ValidateRigidBodyInterface(dynamics, trajectory.data().front().position,
                                   trajectory.data().front().velocity));
  }
  return absl::OkStatus();
}

// Samples the trajectory at the desired `phase` in [0.0, 1.0].
RealtimeStatusOr<JointStatePVAJ> SampleTrajectoryAtPhase(
    const JointTrajectoryPVA& trajectory, const double phase) {
  if (phase < kMinimumPhase - kAllowedPhaseEpsilon ||
      phase > kMaximumPhase + kAllowedPhaseEpsilon) {
    return OutOfRangeError(absl::StrCat("Phase is out of range [",
                                        kMinimumPhase, ", ", kMaximumPhase,
                                        "]. Got ", phase, "."));
  }
  const double clamped_phase = std::clamp(phase, kMinimumPhase, kMaximumPhase);
  const absl::Duration time_since_trajectory_start =
      clamped_phase * trajectory.Duration();
  return InterpolateJointTrajectoryInternalType(trajectory,
                                                time_since_trajectory_start);
}

}  // namespace

SpeedOverrideFactorLimitsComputation::SpeedOverrideFactorLimitsComputation(
    const JointTrajectoryPVA& trajectory, const JointLimits& joint_limits,
    std::unique_ptr<RigidBodyInterface> dynamics)
    : trajectory_(std::move(trajectory)),
      joint_limits_(std::move(joint_limits)),
      dynamics_(std::move(dynamics)) {}

absl::StatusOr<SpeedOverrideFactorLimitsComputation>
SpeedOverrideFactorLimitsComputation::Create(
    const JointTrajectoryPVA& trajectory, const JointLimits& joint_limits,
    std::unique_ptr<RigidBodyInterface> dynamics) {
  INTR_RETURN_IF_ERROR(
      ValidateInputs(trajectory, joint_limits, dynamics.get()));
  return SpeedOverrideFactorLimitsComputation(trajectory, joint_limits,
                                              std::move(dynamics));
}

RealtimeStatusOr<SpeedOverrideFactorLimits>
SpeedOverrideFactorLimitsComputation::ComputeAllSpeedOverrideFactorLimits(
    const double phase,
    const SpeedOverrideFactorStateWithDerivative& speed_override_factor_state,
    bool clamp_sof_to_zero_one_range) const {
  INTRINSIC_RT_ASSIGN_OR_RETURN(const JointStatePVAJ state,
                                SampleTrajectoryAtPhase(trajectory_, phase));

  SpeedOverrideFactorLimits state_limits;
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      state_limits.sof_limits,
      ComputeSpeedOverrideFactorLimits(state, joint_limits_,
                                       clamp_sof_to_zero_one_range));
  if (trajectory_.joint_dynamic_limits_check_mode() ==
      DynamicLimitsCheckMode::kCheckJointAcceleration) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        state_limits.dsof_dt_limits,
        ComputeTimeDerivativeOfSpeedOverrideFactorLimitsForAccLimitedTraj(
            state, joint_limits_, speed_override_factor_state.sof));
  } else {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        state_limits.dsof_dt_limits,
        ComputeTimeDerivativeOfSpeedOverrideFactorLimitsForTorqueLimitedTraj(
            state, joint_limits_, speed_override_factor_state.sof, *dynamics_));
  }
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      state_limits.d2sof_dt2_limits,
      ComputeSecondTimeDerivativeOfSpeedOverrideFactorLimits(
          state, joint_limits_, speed_override_factor_state.sof,
          speed_override_factor_state.dsof_dt));

  return state_limits;
}

}  // namespace intrinsic::icon
