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


#include "intrinsic/icon/control/algorithms/position_reflexxes_cartesian_impedance_reference_generator.h"

#include <algorithm>

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/algorithms/cartesian_impedance_commons.h"
#include "intrinsic/icon/control/algorithms/cartesian_position_reflexxes.h"
#include "intrinsic/icon/control/algorithms/cartesian_velocity_reflexxes.h"
#include "intrinsic/icon/reflexxes/status.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/cart_state.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/kinematics/types/check_cartesian_limits.h"
#include "intrinsic/math/ipow.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/twist.h"

namespace intrinsic::icon {

PositionReflexxesCartesianImpedanceReferenceGenerator::
    PositionReflexxesCartesianImpedanceReferenceGenerator(double frequency_hz)
    : cartesian_position_reflexxes_(frequency_hz),
      cartesian_velocity_reflexxes_(frequency_hz) {
  // Set all degrees of freedom to be active for planning.
  cartesian_position_reflexxes_.SetSelection({true, true, true}, true);
  cartesian_velocity_reflexxes_.SetSelection({true, true, true}, true);
}

RealtimeStatus
PositionReflexxesCartesianImpedanceReferenceGenerator::SetReference(
    const cartesian_impedance::RealTimeCartesianTarget& reference,
    const CartesianLimits& cartesian_limits) {
  CartStatePV base_t_target_desired;
  base_t_target_desired.pose =
      reference.robot_base_t_task *
      Pose3d(reference.task_t_tool_reference_orientation,
             reference.task_t_tool_reference_position);
  base_t_target_desired.velocity = reference.tool_reference_twist;
  if (!IsWithinLimits(base_t_target_desired, cartesian_limits)) {
    return FailedPreconditionError("Target state is not within limits.");
  }

  reference_ = reference;
  nominal_cartesian_limits_ = cartesian_limits;

  return OkStatus();
}

RealtimeStatusOr<cartesian_impedance::RealTimeCartesianTarget>
PositionReflexxesCartesianImpedanceReferenceGenerator::Evaluate(
    const cartesian_impedance::RealTimeCartesianTarget& current,
    const CartStatePVA& /*cart_state_sensed*/, double speed_override) {
  if (speed_override < kMinimumSpeedOverride ||
      speed_override > kMaximumSpeedOverride) {
    return InvalidArgumentError(RealtimeStatus::StrCat(
        "speed_override should be in range [", kMinimumSpeedOverride, ", ",
        kMaximumSpeedOverride, "], but instead has value ", speed_override,
        "."));
  }

  CartesianLimits scaled_cartesian_limits = nominal_cartesian_limits_;
  const double clamped_speed_override =
      std::max(speed_override, kSwitchSpeedOverride);
  scaled_cartesian_limits.min_translational_velocity *= clamped_speed_override;
  scaled_cartesian_limits.min_translational_acceleration *=
      ::intrinsic::IPow(clamped_speed_override, 2);
  scaled_cartesian_limits.min_translational_jerk *=
      ::intrinsic::IPow(clamped_speed_override, 3);
  scaled_cartesian_limits.max_translational_velocity *= clamped_speed_override;
  scaled_cartesian_limits.max_translational_acceleration *=
      ::intrinsic::IPow(clamped_speed_override, 2);
  scaled_cartesian_limits.max_translational_jerk *=
      ::intrinsic::IPow(clamped_speed_override, 3);
  scaled_cartesian_limits.max_rotational_velocity *= clamped_speed_override;
  scaled_cartesian_limits.max_rotational_acceleration *=
      ::intrinsic::IPow(clamped_speed_override, 2);
  scaled_cartesian_limits.max_rotational_jerk *=
      ::intrinsic::IPow(clamped_speed_override, 3);
  if (!cartesian_position_reflexxes_.SetLimits(scaled_cartesian_limits)) {
    return InternalError(RealtimeStatus::StrCat(
        "Reflexxes SetLimits() failed with: ",
        reflexxes::GetStatusString(
            cartesian_position_reflexxes_.GetReflexxesStatus())));
  }
  if (!cartesian_velocity_reflexxes_.SetLimits(scaled_cartesian_limits)) {
    return InternalError(RealtimeStatus::StrCat(
        "Reflexxes SetLimits() failed with: ",
        reflexxes::GetStatusString(
            cartesian_velocity_reflexxes_.GetReflexxesStatus())));
  }

  CartStatePV scaled_base_t_target_desired;
  scaled_base_t_target_desired.pose =
      Pose3d(reference_.task_t_tool_reference_orientation,
             reference_.task_t_tool_reference_position);
  scaled_base_t_target_desired.velocity =
      speed_override * reference_.tool_reference_twist;

  cartesian_position_reflexxes_.SetTarget(scaled_base_t_target_desired);

  CartStateV zero_velocity_target_state;
  zero_velocity_target_state.velocity.setZero();
  cartesian_velocity_reflexxes_.SetTarget(zero_velocity_target_state);

  // Transcribe all reference parameters into the new target first, modify
  // selected ones further below.
  cartesian_impedance::RealTimeCartesianTarget new_target = reference_;
  CartStatePVA previous_base_t_target;
  previous_base_t_target.pose =
      Pose3d(current.task_t_tool_reference_orientation,
             current.task_t_tool_reference_position);
  previous_base_t_target.velocity = current.tool_reference_twist;
  previous_base_t_target.acceleration = current.tool_reference_acceleration;

  cartesian_position_reflexxes_.SetPrevious(previous_base_t_target);
  cartesian_velocity_reflexxes_.SetPrevious(previous_base_t_target);

  CartStatePVA base_t_target_new;
  if (speed_override > kSwitchSpeedOverride) {
    if (!cartesian_position_reflexxes_.ComputeSetpoint(&base_t_target_new)) {
      return InternalError(RealtimeStatus::StrCat(
          "Reflexxes ComputeSetpoint() failed with: ",
          reflexxes::GetStatusString(
              cartesian_position_reflexxes_.GetReflexxesStatus())));
    }
  } else {
    if (!cartesian_velocity_reflexxes_.ComputeSetpoint(&base_t_target_new)) {
      return InternalError(RealtimeStatus::StrCat(
          "Reflexxes ComputeSetpoint() failed with: ",
          reflexxes::GetStatusString(
              cartesian_velocity_reflexxes_.GetReflexxesStatus())));
    }
  }

  new_target.task_t_tool_reference_orientation =
      base_t_target_new.pose.quaternion();
  new_target.task_t_tool_reference_position =
      base_t_target_new.pose.translation();
  new_target.tool_reference_twist = base_t_target_new.velocity;
  new_target.tool_reference_acceleration = base_t_target_new.acceleration;

  return new_target;
}

const cartesian_impedance::RealTimeCartesianTarget&
PositionReflexxesCartesianImpedanceReferenceGenerator::GetReference() const {
  return reference_;
}

}  // namespace intrinsic::icon
