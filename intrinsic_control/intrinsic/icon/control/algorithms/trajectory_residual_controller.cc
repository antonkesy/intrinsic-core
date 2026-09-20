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

#include "intrinsic/icon/control/algorithms/trajectory_residual_controller.h"

#include <memory>

#include "absl/log/check.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"

namespace intrinsic::icon {

// static
absl::StatusOr<std::unique_ptr<TrajectoryResidualController>>
TrajectoryResidualController::Create(double control_frequency_hz,
                                     const JointLimits& planning_limits,
                                     const JointLimits& system_limits) {
  if (control_frequency_hz <= 0.0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "TrajectoryResidualController: control_frequency_hz must be > "
        "0.0, but got ",
        control_frequency_hz));
  }

  // Check if there is a margin between planning and control limits for
  // velocity, acceleration and jerk.
  if ((system_limits.max_velocity.array() <=
       planning_limits.max_velocity.array())
          .any()) {
    // TODO(b/370695712): formalize how much larger they need to be.
    return absl::InvalidArgumentError(
        "Planning and system velocity limits seem to be conflicting, "
        "control velocity limits must be strictly larger than planning "
        "limits.");
  }
  if ((system_limits.max_acceleration.array() <=
       planning_limits.max_acceleration.array())
          .any()) {
    // TODO(b/370695712)): formalize how much larger they need to be.
    return absl::InvalidArgumentError(
        "Planning and system acceleration limits seem to be conflicting, "
        "control acceleration limits must be strictly larger than planning "
        "limits.");
  }
  if ((system_limits.max_jerk.array() <= planning_limits.max_jerk.array())
          .any()) {
    // TODO(b/370695712)): formalize how much larger they need to be.
    return absl::InvalidArgumentError(
        "Planning and system jerk limits seem to be conflicting, "
        "control jerk limits must be strictly larger than planning "
        "limits.");
  }

  // Using WrapUnique due to private constructor.
  return absl::WrapUnique(new TrajectoryResidualController(
      control_frequency_hz, planning_limits, system_limits));
}

TrajectoryResidualController::TrajectoryResidualController(
    double control_frequency_hz, const JointLimits& planning_limits,
    const JointLimits& system_limits)
    : planning_limits_(planning_limits),
      reflexxes_(system_limits.size(), control_frequency_hz) {
  CHECK_OK(limits_margin_.SetSize(system_limits.size()));
  limits_margin_
      .SetUnlimited();  // position limits are set to infinity, since Reflexxes
                        // will not actively prevent their violation anyway.
  limits_margin_.max_velocity =
      system_limits.max_velocity - planning_limits.max_velocity;
  limits_margin_.max_acceleration =
      system_limits.max_acceleration - planning_limits.max_acceleration;
  limits_margin_.max_jerk = system_limits.max_jerk - planning_limits.max_jerk;
  QCHECK(reflexxes_.SetLimits(limits_margin_));

  // The residual is supposed to be brought to zero, define residual target
  // state accordingly.
  JointStatePV residual_target;
  CHECK_OK(residual_target.SetSize(planning_limits.size()));
  residual_target.position.setZero();
  residual_target.velocity.setZero();
  QCHECK(reflexxes_.SetTarget(residual_target));
}

icon::RealtimeStatusOr<JointStatePVA>
TrajectoryResidualController::ComputeResidualUsingLimitMargin(
    const JointStatePVA& previous_residual) {
  if (!reflexxes_.SetLimits(limits_margin_)) {
    return icon::FailedPreconditionError("Failed to set limits_margin_.");
  }

  JointStatePVA new_residual = previous_residual;
  if (!reflexxes_.SetPrevious(previous_residual)) {
    return icon::InvalidArgumentError("Setting previous_residual failed.");
  }
  if (!reflexxes_.ComputeSetpoint(&new_residual)) {
    return icon::InternalError("ComputeSetpoint() failed.");
  }

  return new_residual;
}

icon::RealtimeStatusOr<JointStatePVA>
TrajectoryResidualController::ComputeResidualUsingPlanningLimits(
    const JointStatePVA& previous_residual) {
  if (!reflexxes_.SetLimits(planning_limits_)) {
    return icon::FailedPreconditionError("Failed to set planning_limits_.");
  }

  JointStatePVA new_residual = previous_residual;
  if (!reflexxes_.SetPrevious(previous_residual)) {
    return icon::InvalidArgumentError("Setting previous_residual failed.");
  }
  if (!reflexxes_.ComputeSetpoint(&new_residual)) {
    return icon::InternalError("ComputeSetpoint() failed.");
  }

  return new_residual;
}

}  // namespace intrinsic::icon
