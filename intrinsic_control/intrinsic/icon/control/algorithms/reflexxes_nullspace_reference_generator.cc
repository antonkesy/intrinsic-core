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

#include "intrinsic/icon/control/algorithms/reflexxes_nullspace_reference_generator.h"

#include <algorithm>

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/algorithms/cartesian_impedance_commons.h"
#include "intrinsic/icon/control/algorithms/joint_position_reflexxes.h"
#include "intrinsic/icon/reflexxes/status.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/ipow.h"

namespace intrinsic::icon {

ReflexxesNullspaceReferenceGenerator::ReflexxesNullspaceReferenceGenerator(
    int njoints, double frequency_hz)
    : joint_position_reflexxes_(njoints, frequency_hz),
      joint_velocity_reflexxes_(njoints, frequency_hz) {
  joint_position_reflexxes_.SetSelection(
      eigenmath::VectorNb::Constant(njoints, 1, true));
  joint_velocity_reflexxes_.SetSelection(
      eigenmath::VectorNb::Constant(njoints, 1, true));
}

RealtimeStatus ReflexxesNullspaceReferenceGenerator::SetReference(
    const cartesian_impedance::RealTimeNullspaceTarget& reference,
    const JointLimits& joint_limits) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      bool within_limits, IsWithinLimits(reference.joint_state, joint_limits));
  if (!within_limits) {
    return icon::FailedPreconditionError("Joint state is not within limits.");
  }

  nominal_reference_ = reference;
  nominal_joint_limits_ = joint_limits;
  if (!joint_position_reflexxes_.SetLimits(nominal_joint_limits_)) {
    return InternalError(RealtimeStatus::StrCat(
        "Reflexxes SetLimits() failed with: ",
        reflexxes::GetStatusString(
            joint_position_reflexxes_.GetReflexxesStatus())));
  }
  if (!joint_velocity_reflexxes_.SetLimits(nominal_joint_limits_)) {
    return InternalError(RealtimeStatus::StrCat(
        "Reflexxes SetLimits() failed with: ",
        reflexxes::GetStatusString(
            joint_velocity_reflexxes_.GetReflexxesStatus())));
  }

  if (!joint_position_reflexxes_.SetTarget(nominal_reference_.joint_state)) {
    return InternalError(RealtimeStatus::StrCat(
        "Reflexxes SetTarget() failed with: ",
        reflexxes::GetStatusString(
            joint_position_reflexxes_.GetReflexxesStatus())));
  }
  JointStateV zero_velocity_joint_state;
  INTRINSIC_RT_RETURN_IF_ERROR(
      zero_velocity_joint_state.SetSize(nominal_joint_limits_.size()));
  zero_velocity_joint_state.velocity.setZero();
  if (!joint_velocity_reflexxes_.SetTarget(zero_velocity_joint_state)) {
    return InternalError(RealtimeStatus::StrCat(
        "Reflexxes SetTarget() failed with: ",
        reflexxes::GetStatusString(
            joint_velocity_reflexxes_.GetReflexxesStatus())));
  }
  return OkStatus();
}

RealtimeStatusOr<cartesian_impedance::RealTimeNullspaceTarget>
ReflexxesNullspaceReferenceGenerator::Evaluate(
    const cartesian_impedance::RealTimeNullspaceTarget& current,
    double speed_override) {
  if (speed_override < kMinimumSpeedOverride ||
      speed_override > kMaximumSpeedOverride) {
    return InvalidArgumentError(RealtimeStatus::StrCat(
        "speed_override should be in range [", kMinimumSpeedOverride, ", ",
        kMaximumSpeedOverride, "], but instead has value ", speed_override,
        "."));
  }

  const double clamped_speed_override =
      std::max(speed_override, kSwitchSpeedOverride);

  JointLimits scaled_joint_limits = nominal_joint_limits_;
  scaled_joint_limits.max_velocity *= clamped_speed_override;
  scaled_joint_limits.max_acceleration *=
      ::intrinsic::IPow(clamped_speed_override, 2);
  scaled_joint_limits.max_jerk *= ::intrinsic::IPow(clamped_speed_override, 3);
  if (!joint_position_reflexxes_.SetLimits(scaled_joint_limits)) {
    return InternalError(RealtimeStatus::StrCat(
        "Reflexxes SetLimits() failed with: ",
        reflexxes::GetStatusString(
            joint_position_reflexxes_.GetReflexxesStatus())));
  }
  if (!joint_velocity_reflexxes_.SetLimits(scaled_joint_limits)) {
    return InternalError(RealtimeStatus::StrCat(
        "Reflexxes SetLimits() failed with: ",
        reflexxes::GetStatusString(
            joint_velocity_reflexxes_.GetReflexxesStatus())));
  }

  cartesian_impedance::RealTimeNullspaceTarget scaled_reference =
      nominal_reference_;
  scaled_reference.joint_state.velocity *= speed_override;
  scaled_reference.joint_state.acceleration *=
      ::intrinsic::IPow(speed_override, 2);
  if (!joint_position_reflexxes_.SetTarget(scaled_reference.joint_state)) {
    return InternalError(RealtimeStatus::StrCat(
        "Reflexxes SetTarget() failed with: ",
        reflexxes::GetStatusString(
            joint_position_reflexxes_.GetReflexxesStatus())));
  }
  JointStateV zero_velocity_joint_state;
  INTRINSIC_RT_RETURN_IF_ERROR(
      zero_velocity_joint_state.SetSize(nominal_joint_limits_.size()));
  zero_velocity_joint_state.velocity.setZero();
  if (!joint_velocity_reflexxes_.SetTarget(zero_velocity_joint_state)) {
    return InternalError(RealtimeStatus::StrCat(
        "Reflexxes SetTarget() failed with: ",
        reflexxes::GetStatusString(
            joint_velocity_reflexxes_.GetReflexxesStatus())));
  }

  if (!joint_position_reflexxes_.SetPrevious(current.joint_state)) {
    return InternalError(RealtimeStatus::StrCat(
        "Reflexxes SetPrevious() failed with: ",
        reflexxes::GetStatusString(
            joint_position_reflexxes_.GetReflexxesStatus())));
  }
  if (!joint_velocity_reflexxes_.SetPrevious(current.joint_state)) {
    return InternalError(RealtimeStatus::StrCat(
        "Reflexxes SetPrevious() failed with: ",
        reflexxes::GetStatusString(
            joint_velocity_reflexxes_.GetReflexxesStatus())));
  }

  cartesian_impedance::RealTimeNullspaceTarget new_target = nominal_reference_;
  if (speed_override > kSwitchSpeedOverride) {
    if (!joint_position_reflexxes_.ComputeSetpoint(&new_target.joint_state)) {
      return InternalError(RealtimeStatus::StrCat(
          "Reflexxes ComputeSetpoint() failed with: ",
          reflexxes::GetStatusString(
              joint_position_reflexxes_.GetReflexxesStatus())));
    }
  } else {
    if (!joint_velocity_reflexxes_.ComputeSetpoint(&new_target.joint_state)) {
      return InternalError(RealtimeStatus::StrCat(
          "Reflexxes ComputeSetpoint() failed with: ",
          reflexxes::GetStatusString(
              joint_velocity_reflexxes_.GetReflexxesStatus())));
    }
  }

  return new_target;
}

}  // namespace intrinsic::icon
