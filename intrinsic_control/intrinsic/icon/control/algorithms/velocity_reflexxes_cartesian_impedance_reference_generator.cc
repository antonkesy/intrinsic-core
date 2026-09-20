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


#include "intrinsic/icon/control/algorithms/velocity_reflexxes_cartesian_impedance_reference_generator.h"

#include <algorithm>

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/algorithms/cartesian_impedance_commons.h"
#include "intrinsic/icon/control/algorithms/cartesian_target_scaling.h"
#include "intrinsic/icon/control/algorithms/cartesian_velocity_reflexxes.h"
#include "intrinsic/icon/control/algorithms/reference_limit_settings.h"
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

VelocityReflexxesCartesianImpedanceReferenceGenerator::
    VelocityReflexxesCartesianImpedanceReferenceGenerator(double frequency_hz)
    : cartesian_velocity_reflexxes_(frequency_hz) {
  // Set all degrees of freedom to be active for planning.
  cartesian_velocity_reflexxes_.SetSelection({true, true, true}, true);

  // Start activating at 75 % of the maximum, in this case at 3 cm distance
  // between reference and sensed pose, or 15 degrees rotation.
  // TODO(b/194790661): those parameters are hard-coded for now. Hand over those
  // parameters via action parameters.
  reference_limit_settings_.activation_ratio = 0.75;
  reference_limit_settings_.max_rotation = 0.15;
  reference_limit_settings_.max_translation = 0.03;
}

RealtimeStatus
VelocityReflexxesCartesianImpedanceReferenceGenerator::SetReference(
    const cartesian_impedance::RealTimeCartesianTarget& reference,
    const CartesianLimits& cartesian_limits) {
  CartStateV target_state;
  target_state.velocity = reference.tool_reference_twist;
  if (!IsWithinLimits(target_state, cartesian_limits)) {
    return FailedPreconditionError("Target state is not within limits.");
  }

  nominal_cartesian_limits_ = cartesian_limits;
  nominal_reference_ = reference;
  if (!cartesian_velocity_reflexxes_.SetLimits(nominal_cartesian_limits_)) {
    return InternalError(RealtimeStatus::StrCat(
        "Reflexxes SetLimits() failed with: ",
        reflexxes::GetStatusString(
            cartesian_velocity_reflexxes_.GetReflexxesStatus())));
  }
  cartesian_velocity_reflexxes_.SetTarget(target_state);
  return OkStatus();
}

RealtimeStatusOr<cartesian_impedance::RealTimeCartesianTarget>
VelocityReflexxesCartesianImpedanceReferenceGenerator::Evaluate(
    const cartesian_impedance::RealTimeCartesianTarget& current,
    const CartStatePVA& cart_state_sensed, double speed_override) {
  if (speed_override < kMinimumSpeedOverride ||
      speed_override > kMaximumSpeedOverride) {
    return InvalidArgumentError(RealtimeStatus::StrCat(
        "speed_override should be in range [", kMinimumSpeedOverride, ", ",
        kMaximumSpeedOverride, "], but instead has value ", speed_override,
        "."));
  }

  // Variable naming: reflexxes uses two key variables:
  // 1) a target state for the movement
  // 2) a reference state, that forms the reference trajectory to the target.
  // Within this method, naming is used consistently. Unfortunately, other
  // components of this class use a bit more of a confusing notation, e.g.,
  // SetReference() actually means "SetTarget" in reflexxes (and calls this
  // function in reflexxes).
  // Copy current reference state to previous reference state to keep the
  // input/output structures in reflexxes consistent.
  CartStatePVA current_cart_reference_state_pva;
  current_cart_reference_state_pva.pose =
      Pose3d(current.task_t_tool_reference_orientation,
             current.task_t_tool_reference_position);
  current_cart_reference_state_pva.velocity = current.tool_reference_twist;
  current_cart_reference_state_pva.acceleration =
      current.tool_reference_acceleration;
  cartesian_velocity_reflexxes_.SetPrevious(current_cart_reference_state_pva);

  // Scale the limits appropriately.
  double clamped_speed_override =
      std::max(speed_override, kMinimumSpeedOverrideClamping);

  CartesianLimits scaled_cartesian_limits = nominal_cartesian_limits_;
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
  if (!cartesian_velocity_reflexxes_.SetLimits(scaled_cartesian_limits)) {
    return InternalError(RealtimeStatus::StrCat(
        "Reflexxes SetLimits() failed with: ",
        reflexxes::GetStatusString(
            cartesian_velocity_reflexxes_.GetReflexxesStatus())));
  }

  // Scale the target of the reflexxes trajectory due to speed_override.
  cartesian_impedance::RealTimeCartesianTarget scaled_target =
      nominal_reference_;
  scaled_target.tool_reference_twist *= speed_override;
  scaled_target.tool_reference_acceleration *=
      ::intrinsic::IPow(speed_override, 2);
  CartStateV scaled_target_state_v;
  scaled_target_state_v.velocity = scaled_target.tool_reference_twist;
  cartesian_velocity_reflexxes_.SetTarget(scaled_target_state_v);

  // Scale the target twist to avoid violating limits in Reflexxes.
  CartStatePV scaled_cart_target_state_pv;
  scaled_cart_target_state_pv.pose =
      Pose3d(scaled_target.task_t_tool_reference_orientation,
             scaled_target.task_t_tool_reference_position);
  scaled_cart_target_state_pv.velocity = scaled_target.tool_reference_twist;
  ScalePoseTwistToLimits(scaled_cartesian_limits,
                         &scaled_cart_target_state_pv.velocity);

  // First compute the state with the twist as commanded.
  // ScaleTargetCommand() uses this to decide whether the command drives us
  // to far away from the sensed pose.
  CartStatePVA new_cart_reference_state_candidate_pva;
  if (!cartesian_velocity_reflexxes_.ComputeSetpoint(
          &new_cart_reference_state_candidate_pva)) {
    return InternalError(RealtimeStatus::StrCat(
        "Reflexxes ComputeSetpoint() failed with: ",
        reflexxes::GetStatusString(
            cartesian_velocity_reflexxes_.GetReflexxesStatus())));
  }

  CartStatePV doubly_scaled_cart_target_state_pv;
  CartStatePVA new_cart_reference_state_pva;

  if (ScaleTargetCommand(
          TrajectoryGenerationMode::VELOCITY, scaled_cart_target_state_pv,
          cart_state_sensed, current_cart_reference_state_pva,
          new_cart_reference_state_candidate_pva, reference_limit_settings_,
          doubly_scaled_cart_target_state_pv)) {
    cartesian_velocity_reflexxes_.SetTarget(
        doubly_scaled_cart_target_state_pv.velocity);

    if (!cartesian_velocity_reflexxes_.ComputeSetpoint(
            &new_cart_reference_state_pva)) {
      return InternalError(RealtimeStatus::StrCat(
          "Reflexxes ComputeSetpoint() failed with: ",
          reflexxes::GetStatusString(
              cartesian_velocity_reflexxes_.GetReflexxesStatus())));
    }
  } else {
    new_cart_reference_state_pva = new_cart_reference_state_candidate_pva;
  }

  cartesian_impedance::RealTimeCartesianTarget new_cart_reference =
      nominal_reference_;
  new_cart_reference.task_t_tool_reference_orientation =
      new_cart_reference_state_pva.pose.quaternion();
  new_cart_reference.task_t_tool_reference_position =
      new_cart_reference_state_pva.pose.translation();
  new_cart_reference.tool_reference_twist =
      new_cart_reference_state_pva.velocity;
  new_cart_reference.tool_reference_acceleration =
      new_cart_reference_state_pva.acceleration;

  return new_cart_reference;
}

const cartesian_impedance::RealTimeCartesianTarget&
VelocityReflexxesCartesianImpedanceReferenceGenerator::GetReference() const {
  return nominal_reference_;
}

}  // namespace intrinsic::icon
