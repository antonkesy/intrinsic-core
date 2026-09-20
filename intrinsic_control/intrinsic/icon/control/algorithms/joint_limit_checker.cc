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

#include "intrinsic/icon/control/algorithms/joint_limit_checker.h"

#include <optional>

#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/icon/utils/fixed_str_cat.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/kinematics/types/to_fixed_string.h"

namespace intrinsic::icon {

JointLimitChecker::JointLimitChecker(absl::Duration cycle_duration,
                                     int num_joints,
                                     std::optional<JointStatePVA> initial_state)
    : previous_state_(JointStatePVAJ::Zero(num_joints)),
      last_state_(JointStatePVAJ::Zero(num_joints)),
      cycle_duration_seconds_(absl::ToDoubleSeconds(cycle_duration)) {
  Reset(initial_state);
}

RealtimeStatus JointLimitChecker::Check(absl::Span<const double> position,
                                        const JointLimits& joint_limits) {
  if (position.size() != joint_limits.size()) {
    return FailedPreconditionError("position has the wrong size.");
  }
  JointStatePVAJ state;
  INTRINSIC_RT_RETURN_IF_ERROR(state.SetSize(joint_limits.size()));
  for (int i = 0; i < position.size(); ++i) {
    state.position(i) = position[i];
  }

  state.velocity =
      (state.position - previous_state_.position) / cycle_duration_seconds_;
  state.acceleration =
      (state.velocity - previous_state_.velocity) / cycle_duration_seconds_;
  state.jerk = (state.acceleration - previous_state_.acceleration) /
               cycle_duration_seconds_;

  previous_state_ = state;

  switch (num_cycles_) {
    case 0:
      // Check only the position in the first cycle.
      state.velocity.setZero();
      [[fallthrough]];
    case 1:
      // Check only the position and velocity in the second cycle.
      state.acceleration.setZero();
      [[fallthrough]];
    case 2:
      // Check only the position, velocity and acceleration in the third cycle.
      state.jerk.setZero();
      [[fallthrough]];
    default:
      break;
  }
  ++num_cycles_;
  last_state_ = state;

  INTRINSIC_RT_ASSIGN_OR_RETURN(auto check_result,
                                IsWithinLimits(state, joint_limits));
  if (!check_result) {
    const auto limit_check_result = ToFixedString(check_result);
    INTRINSIC_RT_LOG(INFO) << "Checked state is not within limits. "
                           << limit_check_result << ".\nCurrent state:\n"
                           << ToFixedString(state) << "\nchecked limits:\n"
                           << ToFixedString(joint_limits);
    return InternalError(FixedStrCat<RealtimeStatus::kMaxMessageLength>(
        "Trajectory is not within limits.", ToFixedString(check_result)));
  }

  return OkStatus();
}

void JointLimitChecker::Reset(std::optional<JointStatePVA> initial_state) {
  num_cycles_ = 0;
  previous_state_ = JointStatePVAJ::Zero(previous_state_.size());
  if (initial_state.has_value()) {
    previous_state_ = *initial_state;
    // Skip the initialization cycles.
    num_cycles_ = 3;
  }
}

}  // namespace intrinsic::icon
