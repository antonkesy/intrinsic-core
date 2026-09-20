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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_JOINT_POSITION_REFLEXXES_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_JOINT_POSITION_REFLEXXES_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/reflexxes/flags.h"
#include "intrinsic/icon/reflexxes/position_flags.h"
#include "intrinsic/icon/reflexxes/position_inputs.h"
#include "intrinsic/icon/reflexxes/position_outputs.h"
#include "intrinsic/icon/reflexxes/reflexxes_api.h"
#include "intrinsic/icon/reflexxes/status.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"

namespace intrinsic {
namespace icon {

// JointPositionReflexxes wraps Reflexxes to compute joint set points at each
// control cycle. This implementation is specific in that it uses types
// defined in intrinsic/kinematics/types/joint_state.h, rather than the
// reflexxes-specific types. Position limits are not avoided and, by default,
// the computed trajectory can exceed these limits without reporting error.

class JointPositionReflexxes {
 public:
  // Constructor initializes Reflexxes
  // ndof the number of degrees of freedom
  // frequency The rate in Hz that the set point generator will be called.
  JointPositionReflexxes(std::size_t ndof, double frequency);

  // SetSelection
  // selection the dofs which are active
  // Returns true if inputs were successfully set.
  bool SetSelection(const eigenmath::VectorNb& selection);

  // SetLimits for trajectory calculation
  // Note: Position limits get clamped to the maximum that reflexxes supports.
  //       Clamping should be fine, as those limits are bigger than our
  //       universe.
  // Expected: limits_.min_position < limits_.max_position
  // Returns true if inputs were successfully set.
  bool SetLimits(const JointLimits& limits);

  // SetTarget set targets for trajectory
  // target contains target position & velocity
  // Returns true if inputs were successfully set.
  bool SetTarget(const JointStatePV& target);

  // SetPrevious set previous state.
  // previous contains the last state to connect the trajectory to
  // Returns true if inputs were successfully set.
  bool SetPrevious(const JointStatePVA& previous);

  // ComputeSetpoint() calls reflexxes::ComputePosition().
  // output variable to store new setpoint in.
  // Returns true for reflexxes states of kWorking or
  // kFinalStateReached.
  bool ComputeSetpoint(JointStatePVA* output);

  // Get reflexxes status for last call to computSetpoint.
  // Returns the result of reflexxes::ComputePosition().
  reflexxes::Status GetReflexxesStatus() const { return reflexxes_status_; }

  // Set the positional limits behavior. Defaults to kIgnore.
  void SetPositionalLimitsBehavior(
      reflexxes::Flags::PositionalLimitsBehavior behavior);

  // Set the behaviour when the initial state breaches the constraints. The two
  // options are kGetIntoBoundariesAtZeroAcceleration or kGetIntoBoundariesFast.
  // Defaults to kGetIntoBoundariesAtZeroAcceleration.
  void SetBehaviorIfInitialStateBreachesConstraints(
      reflexxes::PositionFlags::BehaviorIfInitialStateBreachesConstraints
          behavior);

  // SetMinimumSynchronizationTime() accesses reflexxes::PositionInputs.
  // Sets min_sync_time_ in reflexxes.
  // See reflexxes/examples/example_07_reflexxes_velocity.cc for usage.
  void SetMinimumSynchronizationTime(const double sync_time) {
    input_params_.SetMinimumSynchronizationTime(sync_time);
  }

  // GetSyncTime() accesses reflexxes::PositionOutputs.
  // Returns sync_time_ from reflexxes.
  double GetSyncTime() const { return output_params_.GetSyncTime(); }

  // GetTimeCounter() accesses reflexxes::PositionOutputs.
  // Returns time_counter_ from reflexxes.
  uint64_t GetTimeCounter() const { return output_params_.GetTimeCounter(); }

  // SetSyncBehavior sets the reflexxes synchronization behavior.
  void SetSyncBehavior(reflexxes::Flags::SyncBehavior sync_behavior) {
    reflexxes_position_flags_.synchronization_behavior = sync_behavior;
  }

 private:
  std::size_t ndof_ = 0;
  reflexxes::State reflexxes_state_;
  reflexxes::PositionInputs input_params_;
  reflexxes::PositionOutputs output_params_;
  reflexxes::PositionFlags reflexxes_position_flags_;
  reflexxes::Status reflexxes_status_ = reflexxes::Status::kErrorUndefined;
  JointLimits limits_;
  JointStatePVA previous_;
  JointStatePV target_;
};

}  // namespace icon
}  // namespace intrinsic
#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_JOINT_POSITION_REFLEXXES_H_
