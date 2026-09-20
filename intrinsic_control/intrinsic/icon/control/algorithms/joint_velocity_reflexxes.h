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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_JOINT_VELOCITY_REFLEXXES_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_JOINT_VELOCITY_REFLEXXES_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/reflexxes/flags.h"
#include "intrinsic/icon/reflexxes/inputs.h"
#include "intrinsic/icon/reflexxes/reflexxes_api.h"
#include "intrinsic/icon/reflexxes/status.h"
#include "intrinsic/icon/reflexxes/velocity_flags.h"
#include "intrinsic/icon/reflexxes/velocity_inputs.h"
#include "intrinsic/icon/reflexxes/velocity_outputs.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"

namespace intrinsic {
namespace icon {

// JointVelocityReflexxes wraps Reflexxes to compute joint set points at each
// control cycle. This implementation is specific in that it uses structs
// defined in intrinsic/kinematics/types.h, rather than the
// reflexxes-specific types. Position limits are not avoided and, by default,
// the computed trajectory can exceed these limits without reporting error.
class JointVelocityReflexxes {
 public:
  // Constructor initializes Reflexxes
  // ndof the number of degrees of freedom
  // frequency The rate in Hz that the set point generator will be called.
  JointVelocityReflexxes(std::size_t ndof, double frequency);

  // SetSelection
  // selection the dofs which are active
  // Returns true if inputs were successfully set.
  bool SetSelection(const eigenmath::VectorNb& selection);

  // SetLimits for trajectory calculation
  // Note: Position limits will be checked for violation, but the implementation
  //       will not attempt to find a solution within the limits.
  //       Position Limits get clamped to the maximum that reflexxes supports.
  //       Clamping should be fine, as those limits are bigger than our
  //       universe.
  // Expected: limits_.min_position < limits_.max_position
  // Returns true if inputs were successfully set.
  bool SetLimits(const JointLimits& limits);

  // SetTarget set targets for trajectory
  // target contains target velocity & velocity
  // Returns true if inputs were successfully set.
  bool SetTarget(const JointStateV& target);

  // SetPrevious set previous state.
  // previous contains the last state to connect the trajectory to
  // Returns true if inputs were successfully set.
  bool SetPrevious(const JointStatePVA& previous);

  // ComputeSetpoint() calls reflexxes::ComputeVelocity().
  // output variable to store new setpoint in.
  // Returns true for reflexxes states of kWorking or
  // kFinalStateReached.
  bool ComputeSetpoint(JointStatePVA* output);

  /// Get reflexxes status for last call to ComputeSetpoint.
  /// Returns the result of reflexxes::ComputeVelocity().
  reflexxes::Status GetReflexxesStatus() const { return reflexxes_status_; }

  // Returns whether or not the new state from the last call to ComputeSetpoint
  // breaches positional limits before coming to a stop. Note that this call is
  // meaningful with any positional flag for `PositionalLimitsBehavior` such as
  // `kActivelyPrevent` or `kIgnore`.
  icon::RealtimeStatusOr<bool>
  DoesStopMotionFromNewStateBreachPositionalLimits() const;

  icon::RealtimeStatusOr<JointStateP> PositionAtTargetVelocity() const;

  // Set the positional limits behavior. Defaults to kIgnore.
  void SetPositionalLimitsBehavior(
      reflexxes::Flags::PositionalLimitsBehavior behavior);

  // SetMinimumSynchronizationTime() accesses reflexxes::VelocityInputs.
  // Sets min_sync_time_ in reflexxes.
  // See reflexxes/examples/example_08_reflexxes_velocity.cc for usage.
  void SetMinimumSynchronizationTime(const double sync_time) {
    input_params_.SetMinimumSynchronizationTime(sync_time);
  }

  // GetSyncTime() accesses reflexxes::VelocityOutputs.
  // Returns sync_time_ from reflexxes.
  double GetSyncTime() const { return output_params_.GetSyncTime(); }

  // GetTimeCounter() accesses reflexxes::VelocityOutputs.
  // Returns time_counter_ from reflexxes.
  uint64_t GetTimeCounter() const { return output_params_.GetTimeCounter(); }

  // SetSyncBehavior sets the reflexxes synchronization behavior.
  void SetSyncBehavior(reflexxes::Flags::SyncBehavior sync_behavior) {
    reflexxes_velocity_flags_.synchronization_behavior = sync_behavior;
  }

 private:
  std::size_t ndof_ = 0;
  reflexxes::State reflexxes_state_;
  reflexxes::VelocityInputs input_params_;
  reflexxes::VelocityOutputs output_params_;
  reflexxes::VelocityFlags reflexxes_velocity_flags_;
  reflexxes::Status reflexxes_status_ = reflexxes::Status::kErrorUndefined;
  std::optional<bool> trajectory_breaches_limits_ = std::nullopt;
  std::optional<reflexxes::MaxDOFFixedVector<double>>
      position_at_velocity_target_ = std::nullopt;
  JointLimits limits_;
  JointStatePVA previous_;
  JointStateV target_;
};

}  // namespace icon
}  // namespace intrinsic
#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_JOINT_VELOCITY_REFLEXXES_H_
