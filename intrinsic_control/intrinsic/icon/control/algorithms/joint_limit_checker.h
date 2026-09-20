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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_JOINT_LIMIT_CHECKER_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_JOINT_LIMIT_CHECKER_H_

#include <optional>

#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"

namespace intrinsic::icon {

// Checks that a sequence of joint position setpoints is within joint limits.
// The velocity, acceleration and jerk are computed from the position using
// finite differences.
class JointLimitChecker {
 public:
  // `cycle_duration` is the cycle time of the control loop. `num_joints` is the
  // number of joints in the robot. `initial_state` is used to initialize the
  // internal state, which is needed to compute the position derivatives. If
  // `initial_state` is not set, the first 3 cycles are used for the
  // initialization. In the first cycle only the position is checked, in the
  // second cycle the position and velocity are checked and in the third cycle
  // the position, velocity and acceleration are checked.
  JointLimitChecker(absl::Duration cycle_duration, int num_joints,
                    std::optional<JointStatePVA> initial_state = std::nullopt);

  // Call `Check` once every cycle to check that the `position` command is
  // within limits.
  RealtimeStatus Check(absl::Span<const double> position,
                       const JointLimits& joint_limits);

  // Resets the internal state. If `initial_state` is not set, the first 3
  // cycles after reset are used for the initialization.
  void Reset(std::optional<JointStatePVA> initial_state = std::nullopt);

  // Returns the last checked joint state (including calculated jerk).
  const JointStatePVAJ& last_state() const { return last_state_; }

 private:
  JointStatePVA previous_state_;
  JointStatePVAJ last_state_;
  const double cycle_duration_seconds_;
  int num_cycles_ = 0;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_JOINT_LIMIT_CHECKER_H_
