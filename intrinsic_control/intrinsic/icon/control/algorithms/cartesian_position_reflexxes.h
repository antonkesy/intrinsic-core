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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_CARTESIAN_POSITION_REFLEXXES_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_CARTESIAN_POSITION_REFLEXXES_H_

#include <cstdint>

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/algorithms/axis_angle_kinematics.h"
#include "intrinsic/icon/reflexxes/flags.h"
#include "intrinsic/icon/reflexxes/position_flags.h"
#include "intrinsic/icon/reflexxes/position_inputs.h"
#include "intrinsic/icon/reflexxes/position_outputs.h"
#include "intrinsic/icon/reflexxes/reflexxes_api.h"
#include "intrinsic/icon/reflexxes/status.h"
#include "intrinsic/kinematics/types/cart_state.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"

namespace intrinsic {
namespace icon {

// CartesianPositionReflexxes wraps Reflexxes to compute Cartsian position
// setpoints at each control cycle. This implementation is specific in that it
// uses structs defined in common/interfaces_types.h, rather
// than the reflexxes-specific types.
//
// For rest-to-rest trajectories, this class will generate straigh-line
// trajectories. Caveat: Reflexxes plans minimum-time trajectories for a curve
// in R^n with box constraints on the derivatives. So rotations must be planned
// via a non-redundant position parametrization and constraints are applied to
// the derivative of the position parameters instead of angular velocity and
// angular acceleration. This implementation uses rotation axis*rotation angle
// as position parametrization. Re-initializing reflexxes requires computing
// the position parameters and its derivatives from the last Cartesian setpoint
// (quaternion, angular velocity and angular acceleration), which has
// singularities.
class CartesianPositionReflexxes {
  static constexpr double kMaxPositionalLimit = 1e50;

 public:
  // Constructor initializes Reflexxes
  // frequency The rate in Hz that the set point generator will be called.
  explicit CartesianPositionReflexxes(double frequency);

  // SetSelection
  // translation_selection the translational dofs which are active
  // rotation_selection the rotational dofs which are active
  void SetSelection(const eigenmath::Vector3b& translation_selection,
                    bool rotation_selection);

  // SetLimits for trajectory calculation
  // Note: Position limits get clamped to the maximum that reflexxes supports.
  //       Clamping should be fine, as those limits are bigger than our
  //       universe.
  // Expected: limits_.min_translational_position <
  // limits_.max_translational_position
  // Returns true if inputs were successfully set.
  bool SetLimits(const CartesianLimits& limits);

  // setTarget set targets for trajectory
  // target contains target position & velocity
  void SetTarget(const CartStatePV& target);

  // setPrevious set previous state.
  // previous contains the last state to connect the trajectory to
  void SetPrevious(const CartStatePVA& previous);

  // ComputeSetpoint() calls reflexxes::ComputePosition().
  // output variable to store new setpoint in.
  // Returns true for reflexxes states of kWorking or
  // kFinalStateReached.
  bool ComputeSetpoint(CartStatePVA* output);

  // Get reflexxes status for last call to computSetpoint
  // Returns the result of reflexxes::ComputePosition().
  reflexxes::Status GetReflexxesStatus() const { return reflexxes_status_; }

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

  // SetPositionalLimitsBehavior sets the reflexxes behavior when position
  // limits are violated.
  void SetPositionalLimitsBehavior(
      reflexxes::Flags::PositionalLimitsBehavior positional_limits_behavior) {
    reflexxes_position_flags_.positional_limits_behavior =
        positional_limits_behavior;
  }

 private:
  reflexxes::State reflexxes_state_;
  reflexxes::PositionInputs input_params_;
  reflexxes::PositionOutputs output_params_;
  reflexxes::PositionFlags reflexxes_position_flags_;
  reflexxes::Status reflexxes_status_ = reflexxes::Status::kErrorUndefined;
  control::AxisAngleKinematics aa_kinematics_;

  CartesianLimits limits_;
  CartStatePV target_;
  CartStatePVA previous_;
};

}  // namespace icon
}  // namespace intrinsic
#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_CARTESIAN_POSITION_REFLEXXES_H_
