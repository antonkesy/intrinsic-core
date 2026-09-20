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

#include "intrinsic/icon/control/algorithms/cartesian_position_reflexxes.h"

#include <algorithm>
#include <cstddef>
#include <limits>

#include "intrinsic/eigenmath/manifolds.h"
#include "intrinsic/eigenmath/scalar_utils.h"
#include "intrinsic/eigenmath/so3.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/reflexxes/constants_external.h"
#include "intrinsic/icon/reflexxes/flags.h"
#include "intrinsic/icon/reflexxes/inputs.h"
#include "intrinsic/icon/reflexxes/outputs.h"
#include "intrinsic/icon/reflexxes/reflexxes_api.h"
#include "intrinsic/icon/reflexxes/status.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/kinematics/types/cart_state.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"

namespace intrinsic {
namespace icon {

CartesianPositionReflexxes::CartesianPositionReflexxes(const double frequency)
    : reflexxes_state_(static_cast<unsigned int>(6), 1 / frequency),
      input_params_(static_cast<unsigned int>(6), 1 / frequency),
      output_params_(static_cast<unsigned int>(6), 1 / frequency) {
  reflexxes_position_flags_.synchronization_behavior =
      reflexxes::Flags::SyncBehavior::kPhaseSynchronizationWhenCollinear;

  // Return error on position limit violation.
  reflexxes_position_flags_.positional_limits_behavior =
      reflexxes::Flags::PositionalLimitsBehavior::kErrorMsgOnly;
}

void CartesianPositionReflexxes::SetSelection(
    const eigenmath::Vector3b& translation_selection,
    const bool rotation_selection) {
  for (size_t ii = 0; ii < 3; ++ii) {
    input_params_.GetDOFs()[ii].selected = translation_selection[ii];
    input_params_.GetDOFs()[ii + 3].selected = rotation_selection;
  }
}

bool CartesianPositionReflexxes::SetLimits(const CartesianLimits& limits) {
#define CHECK_AGAINST(lhs, op, rhs, i)                                   \
  do {                                                                   \
    if (lhs op rhs) {                                                    \
      INTRINSIC_RT_LOG_THROTTLED(ERROR)                                  \
          << "did not expect " #lhs " " #op " " #rhs ", but " #lhs " = " \
          << lhs << " and " #rhs " = " << rhs << " for i = " << i;       \
      return false;                                                      \
    }                                                                    \
  } while (0)

#define CLAMP_AND_ASSIGN(in, out)                             \
  out = std::clamp(in, std::numeric_limits<double>::lowest(), \
                   std::numeric_limits<double>::max());

  for (int i = 0; i < 3; i++) {
    CHECK_AGAINST(limits.max_translational_position[i], <=,
                  limits.min_translational_position[i], i);
    limits_.min_translational_position[i] = eigenmath::Saturate(
        limits.min_translational_position[i], reflexxes::kMaxPositionalLimit);
    limits_.max_translational_position[i] = eigenmath::Saturate(
        limits.max_translational_position[i], reflexxes::kMaxPositionalLimit);

    CHECK_AGAINST(limits.min_translational_velocity[i], >=, 0.0, i);
    CLAMP_AND_ASSIGN(limits.min_translational_velocity[i],
                     limits_.min_translational_velocity[i]);
    CHECK_AGAINST(limits.max_translational_velocity[i], <=, 0.0, i);
    CLAMP_AND_ASSIGN(limits.max_translational_velocity[i],
                     limits_.max_translational_velocity[i]);

    CHECK_AGAINST(limits.min_translational_acceleration[i], >=, 0.0, i);
    CLAMP_AND_ASSIGN(limits.min_translational_acceleration[i],
                     limits_.min_translational_acceleration[i]);
    CHECK_AGAINST(limits.max_translational_acceleration[i], <=, 0.0, i);
    CLAMP_AND_ASSIGN(limits.max_translational_acceleration[i],
                     limits_.max_translational_acceleration[i]);

    CHECK_AGAINST(limits.min_translational_jerk[i], >=, 0.0, i);
    CHECK_AGAINST(limits.max_translational_jerk[i], <=, 0.0, i);
    limits_.min_translational_jerk[i] = eigenmath::Saturate(
        limits.min_translational_jerk[i], reflexxes::kMaxJerkLimit);
    limits_.max_translational_jerk[i] = eigenmath::Saturate(
        limits.max_translational_jerk[i], reflexxes::kMaxJerkLimit);
  }

#undef CHECK_AGAINST

#define CHECK_AGAINST(lhs, op, rhs)                                      \
  do {                                                                   \
    if (lhs op rhs) {                                                    \
      INTRINSIC_RT_LOG_THROTTLED(ERROR)                                  \
          << "did not expect " #lhs " " #op " " #rhs ", but " #lhs " = " \
          << lhs << " and " #rhs " = " << rhs;                           \
      return false;                                                      \
    }                                                                    \
  } while (0)

  CHECK_AGAINST(limits.max_rotational_velocity, <=, 0.0);
  CLAMP_AND_ASSIGN(limits.max_rotational_velocity,
                   limits_.max_rotational_velocity);
  CHECK_AGAINST(limits.max_rotational_acceleration, <=, 0.0);
  CLAMP_AND_ASSIGN(limits.max_rotational_acceleration,
                   limits_.max_rotational_acceleration);
  CHECK_AGAINST(limits.max_rotational_jerk, <=, 0.0);
  limits_.max_rotational_jerk =
      eigenmath::Saturate(limits.max_rotational_jerk, reflexxes::kMaxJerkLimit);

#undef CHECK_AGAINST
#undef CLAMP_AND_ASSIGN
  return true;
}

void CartesianPositionReflexxes::SetTarget(const CartStatePV& target) {
  target_ = target;
}

void CartesianPositionReflexxes::SetPrevious(const CartStatePVA& previous) {
  previous_ = previous;
}

bool CartesianPositionReflexxes::ComputeSetpoint(CartStatePVA* output) {
  // update previous & target positions.
  // translational components are directly used.
  // Rotation is described relative to target, so that linear interpolation
  // of axis*angle leads to planar rotation.

  eigenmath::Vector6d target_position;        // [translation;axis*angle]
  eigenmath::Vector6d target_velocity;        // [transl. vel; d(axis*angle)/dt]
  eigenmath::Vector6d previous_position;      // [translation;axis*angle]
  eigenmath::Vector6d previous_velocity;      // [translation;axis*angle]
  eigenmath::Vector6d previous_acceleration;  // [translation;axis*angle]

  // directly use translational components
  target_position.head<3>() = target_.pose.translation();
  target_velocity.head<3>() = target_.velocity.head<3>();

  previous_position.head<3>() = previous_.pose.translation();
  previous_velocity.head<3>() = previous_.velocity.head<3>();
  previous_acceleration.head<3>() = previous_.acceleration.head<3>();

  eigenmath::Matrix3d target_R = target_.pose.rotationMatrix().transpose();
  target_position.tail<3>().setZero();  // target position always zero
  target_velocity.tail<3>() = target_R * target_.velocity.tail<3>();
  // transform to axis*angle representation
  eigenmath::Vector3d dot_axis_angle;
  aa_kinematics_.ToAxisAngleDerivative(
      target_position.tail<3>(), target_velocity.tail<3>(), &dot_axis_angle);

  target_velocity.tail<3>() = dot_axis_angle;
  eigenmath::Matrix3d Rstart = target_R * previous_.pose.rotationMatrix();
  previous_position.tail<3>() =
      intrinsic::eigenmath::logSO3(eigenmath::SO3d(Rstart));

  // Verify that we are not close to an unhandled singularity
  if (aa_kinematics_.CloseToUnhandledSingularity(previous_position.tail<3>())) {
    INTRINSIC_RT_LOG(ERROR)
        << "Current angle axis*angle too close to unhandled "
           "singularity, value= "
        << previous_position[3] << " " << previous_position[4] << " "
        << previous_position[5];
    return false;
  }

  previous_velocity.tail<3>() = target_R * previous_.velocity.tail<3>();
  previous_acceleration.tail<3>() = target_R * previous_.acceleration.tail<3>();
  // transform angular velocity & angular acceleration to axis*angle
  // representation
  eigenmath::Vector3d ddot_axis_angle;
  aa_kinematics_.ToAxisAngleDerivatives(
      previous_position.tail<3>(), previous_velocity.tail<3>(),
      previous_acceleration.tail<3>(), &dot_axis_angle, &ddot_axis_angle);

  previous_velocity.tail<3>() = dot_axis_angle;
  previous_acceleration.tail<3>() = ddot_axis_angle;

  // copy data to reflexxes
  // For limits, first three translational elements are directly from the
  // limits.
  for (unsigned int i = 0; i < 3; ++i) {
    reflexxes::Inputs::DOF& dof = input_params_.GetDOFs()[i];
    dof.max_position = limits_.max_translational_position[i];
    dof.max_velocity = limits_.max_translational_velocity[i];
    dof.max_acceleration = limits_.max_translational_acceleration[i];
    dof.max_jerk = limits_.max_translational_jerk[i];

    dof.min_position = limits_.min_translational_position[i];
    dof.min_velocity = limits_.min_translational_velocity[i];
    dof.min_acceleration = limits_.min_translational_acceleration[i];
    dof.min_jerk = limits_.min_translational_jerk[i];
  }

  // TODO (buschmann, rphilipp) we run Reflexxes in axis-times-angle
  // space, but would ideally want to apply limits in angular
  // velocity and acceleration. However, that mapping is non-linear
  // and configuration-dependent, and if we keep changing the limits
  // in Reflexxes it will continuously replan. So, for now just copy
  // the angular limits over into the representation space (at the
  // target frame, the mapping is identity so at least there the
  // limits are the correct ones). However, this actually leads to
  // issues when we have non-zero angular target velocities (see
  // b/27473089).

  for (unsigned int i = 3; i < 6; ++i) {
    reflexxes::Inputs::DOF& dof = input_params_.GetDOFs()[i];

    dof.max_position = kMaxPositionalLimit;
    dof.max_velocity = limits_.max_rotational_velocity;
    dof.max_acceleration = limits_.max_rotational_acceleration;
    dof.max_jerk = limits_.max_rotational_jerk;

    dof.min_position = -kMaxPositionalLimit;
    dof.min_velocity = -limits_.max_rotational_velocity;
    dof.min_acceleration = -limits_.max_rotational_acceleration;
    dof.min_jerk = -limits_.max_rotational_jerk;
  }

  for (reflexxes::Inputs::DOF& dof : input_params_.GetDOFs()) {
    dof.position = previous_position[dof.index];
    dof.velocity = previous_velocity[dof.index];
    dof.acceleration = previous_acceleration[dof.index];

    dof.target_position = target_position[dof.index];
    dof.target_velocity = target_velocity[dof.index];
  }

  if (!input_params_.IsValid()) {
    auto [error_code, degree_of_freedom] = input_params_.CheckForValidity();
    INTRINSIC_RT_LOG(ERROR)
        << "CheckForValidity returned false: dof " << degree_of_freedom
        << " code " << static_cast<int>(error_code);
    return false;
  }

  reflexxes_status_ =
      reflexxes::ComputePosition(input_params_, reflexxes_position_flags_,
                                 output_params_, reflexxes_state_);

#define MAKE_ERROR_CASE(x)                                                     \
  case x:                                                                      \
    INTRINSIC_RT_LOG(ERROR)                                                    \
        << "got error from reflexxes call: " << reflexxes::GetStatusString(x); \
    for (reflexxes::Inputs::DOF& dof : input_params_.GetDOFs()) {              \
      INTRINSIC_RT_LOG(ERROR)                                                  \
          << "current_position[" << dof.index << "] = " << dof.position;       \
      INTRINSIC_RT_LOG(ERROR)                                                  \
          << "current_velocity[" << dof.index << "] = " << dof.velocity;       \
      INTRINSIC_RT_LOG(ERROR) << "current_acceleration[" << dof.index          \
                              << "] = " << dof.acceleration;                   \
      INTRINSIC_RT_LOG(ERROR)                                                  \
          << "max_velocity[" << dof.index << "] = " << dof.max_velocity;       \
      INTRINSIC_RT_LOG(ERROR) << "max_acceleration[" << dof.index              \
                              << "] = " << dof.max_acceleration;               \
      INTRINSIC_RT_LOG(ERROR)                                                  \
          << "max_jerk[" << dof.index << "] = " << dof.max_jerk;               \
      INTRINSIC_RT_LOG(ERROR)                                                  \
          << "min_velocity[" << dof.index << "] = " << dof.min_velocity;       \
      INTRINSIC_RT_LOG(ERROR) << "min_acceleration[" << dof.index              \
                              << "] = " << dof.min_acceleration;               \
      INTRINSIC_RT_LOG(ERROR)                                                  \
          << "min_jerk[" << dof.index << "] = " << dof.min_jerk;               \
      INTRINSIC_RT_LOG(ERROR)                                                  \
          << "target_position[" << dof.index << "] = " << dof.target_position; \
      INTRINSIC_RT_LOG(ERROR)                                                  \
          << "target_velocity[" << dof.index << "] = " << dof.target_velocity; \
    }                                                                          \
    return false;
  switch (reflexxes_status_) {
    case reflexxes::Status::kWorking:
    case reflexxes::Status::kFinalStateReached:
      break;
      MAKE_ERROR_CASE(reflexxes::Status::kErrorNumberOfDofs);
      MAKE_ERROR_CASE(reflexxes::Status::kErrorCycleTime);
      MAKE_ERROR_CASE(reflexxes::Status::kErrorInvalidInputValues);
      MAKE_ERROR_CASE(reflexxes::Status::kErrorInvalidTargetState);
      MAKE_ERROR_CASE(reflexxes::Status::kErrorInvalidConstraints);
      MAKE_ERROR_CASE(reflexxes::Status::kErrorInvalidScaleOfInputValues);
      MAKE_ERROR_CASE(reflexxes::Status::kErrorExecutionTimeCalculation);
      MAKE_ERROR_CASE(reflexxes::Status::kErrorSynchronization);
      MAKE_ERROR_CASE(reflexxes::Status::kErrorNoPhaseSynchronization);
      MAKE_ERROR_CASE(reflexxes::Status::kErrorExecutionTimeTooBig);
      MAKE_ERROR_CASE(reflexxes::Status::kErrorUserTimeOutOfRange);
      MAKE_ERROR_CASE(reflexxes::Status::kErrorPositionalLimits);
      MAKE_ERROR_CASE(reflexxes::Status::kErrorUndefined);
    default:
      INTRINSIC_RT_LOG(ERROR) << "Unexpected Reflexxes status "
                              << static_cast<int>(reflexxes_status_);
      return false;
  }
#undef MAKE_ERROR_CASE

  eigenmath::Vector6d new_position, new_velocity, new_acceleration;
  for (const reflexxes::Outputs::DOF& dof : output_params_.GetDOFs()) {
    new_position[dof.index] = dof.new_position;
    new_velocity[dof.index] = dof.new_velocity;
    new_acceleration[dof.index] = dof.new_acceleration;
  }

  // copy new translations
  output->pose.setTranslation(new_position.head<3>());
  output->velocity.head<3>() = new_velocity.head<3>();
  output->acceleration.head<3>() = new_acceleration.head<3>();

  // transform rotations
  // to to angular velocity & acceleration
  eigenmath::Vector3d omega;
  eigenmath::Vector3d dot_omega;
  aa_kinematics_.ToAngularVelocityAcceleration(
      new_position.tail<3>(), new_velocity.tail<3>(),
      new_acceleration.tail<3>(), &omega, &dot_omega);

  eigenmath::Matrix3d new_R =
      intrinsic::eigenmath::expSO3(new_position.tail<3>().eval()).matrix();

  const eigenmath::Matrix3d target_R_inv(target_.pose.rotationMatrix());

  output->pose.setRotationMatrix(eigenmath::Matrix3d{target_R_inv * new_R});
  output->velocity.tail<3>() = target_R_inv * omega;
  output->acceleration.tail<3>() = target_R_inv * dot_omega;

  return true;
}

}  // namespace icon
}  // namespace intrinsic
