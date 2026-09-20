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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_CARTESIAN_VELOCITY_REFLEXXES_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_CARTESIAN_VELOCITY_REFLEXXES_H_

#include <cstdint>

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/reflexxes/flags.h"
#include "intrinsic/icon/reflexxes/reflexxes_api.h"
#include "intrinsic/icon/reflexxes/status.h"
#include "intrinsic/icon/reflexxes/velocity_flags.h"
#include "intrinsic/icon/reflexxes/velocity_inputs.h"
#include "intrinsic/icon/reflexxes/velocity_outputs.h"
#include "intrinsic/kinematics/types/cart_state.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"

namespace intrinsic {
namespace icon {

// CartesianVelocityReflexxes computes trajectories in Cartesian space with
// velocity targets.
//
// The implementation uses reflexxes to compute velocities and accelerations for
// a trajectory to a given velocity target. Translational components also use
// the position computed by reflexxes. The 3D rotation is computed by numerical
// integration and positions computed by reflexxes are discarded.
class CartesianVelocityReflexxes {
 public:
  // Constructor initializes Reflexxes
  // frequency The rate in Hz that the set point generator will be called.
  explicit CartesianVelocityReflexxes(double frequency);

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

  // SetTarget set targets for trajectory
  // target contains target velocity
  void SetTarget(const CartStateV& target);

  // SetPrevious set previous state.
  // previous contains the last state to connect the trajectory to
  void SetPrevious(const CartStatePVA& previous);

  // ComputeSetpoint() calls reflexxes::ComputeVelocity().
  // output variable to store new setpoint in.
  // Returns true for reflexxes states of kWorking or
  // kFinalStateReached.
  bool ComputeSetpoint(CartStatePVA* output);

  // Get reflexxes status for last call to computSetpoint
  // Returns the result of reflexxes::ComputeVelocity().
  reflexxes::Status GetReflexxesStatus() const { return reflexxes_status_; }

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

  // SetPositionalLimitsBehavior sets the reflexxes behavior when position
  // limits are violated.
  void SetPositionalLimitsBehavior(
      reflexxes::Flags::PositionalLimitsBehavior positional_limits_behavior) {
    reflexxes_velocity_flags_.positional_limits_behavior =
        positional_limits_behavior;
  }

 private:
  bool CheckRMLStatus(const reflexxes::Status& stat,
                      const reflexxes::VelocityOutputs& out);

  double dt_ = 0;
  reflexxes::State reflexxes_state_;
  reflexxes::VelocityInputs input_params_;
  reflexxes::VelocityOutputs output_params_;
  reflexxes::VelocityFlags reflexxes_velocity_flags_;
  reflexxes::Status reflexxes_status_ = reflexxes::Status::kErrorUndefined;
  eigenmath::Vector6d reflexxes_position_ = eigenmath::Vector6d::Zero();
  CartesianLimits limits_;
  CartStateV target_;
  CartStatePVA previous_;
};

}  // namespace icon
}  // namespace intrinsic
#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_CARTESIAN_VELOCITY_REFLEXXES_H_
