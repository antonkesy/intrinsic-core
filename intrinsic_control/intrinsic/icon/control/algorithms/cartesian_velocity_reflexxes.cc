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

#include "intrinsic/icon/control/algorithms/cartesian_velocity_reflexxes.h"

#include <algorithm>
#include <cstddef>
#include <limits>

#include "intrinsic/eigenmath/scalar_utils.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/reflexxes/constants_external.h"
#include "intrinsic/icon/reflexxes/flags.h"
#include "intrinsic/icon/reflexxes/inputs.h"
#include "intrinsic/icon/reflexxes/outputs.h"
#include "intrinsic/icon/reflexxes/reflexxes_api.h"
#include "intrinsic/icon/reflexxes/status.h"
#include "intrinsic/icon/reflexxes/velocity_outputs.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/kinematics/types/cart_state.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"

namespace intrinsic {
namespace icon {

using Vector4d = Eigen::Matrix<double, 4, 1>;

CartesianVelocityReflexxes::CartesianVelocityReflexxes(const double frequency)
    : dt_(0.5 / frequency),
      reflexxes_state_(static_cast<unsigned int>(6), dt_),
      input_params_(static_cast<unsigned int>(6), dt_),
      output_params_(static_cast<unsigned int>(6), dt_) {
  reflexxes_velocity_flags_.synchronization_behavior =
      reflexxes::Flags::SyncBehavior::kPhaseSynchronizationIfPossible;
  // Return error on position limit violation
  reflexxes_velocity_flags_.positional_limits_behavior =
      reflexxes::Flags::PositionalLimitsBehavior::kErrorMsgOnly;
}

void CartesianVelocityReflexxes::SetSelection(
    const eigenmath::Vector3b& translation_selection,
    const bool rotation_selection) {
  for (size_t ii = 0; ii < 3; ++ii) {
    input_params_.GetDOFs()[ii].selected = translation_selection[ii];
    input_params_.GetDOFs()[ii + 3].selected = rotation_selection;
  }
}

bool CartesianVelocityReflexxes::SetLimits(const CartesianLimits& limits) {
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
    CLAMP_AND_ASSIGN(limits.min_translational_jerk[i],
                     limits_.min_translational_jerk[i]);
    CHECK_AGAINST(limits.max_translational_jerk[i], <=, 0.0, i);
    CLAMP_AND_ASSIGN(limits.max_translational_jerk[i],
                     limits_.max_translational_jerk[i]);
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
  CLAMP_AND_ASSIGN(limits.max_rotational_jerk, limits_.max_rotational_jerk);

#undef CHECK_AGAINST
#undef CLAMP_AND_ASSIGN

  return true;
}

void CartesianVelocityReflexxes::SetTarget(const CartStateV& target) {
  target_ = target;
}

void CartesianVelocityReflexxes::SetPrevious(const CartStatePVA& previous) {
  previous_ = previous;
}

bool CartesianVelocityReflexxes::CheckRMLStatus(
    const reflexxes::Status& stat, const reflexxes::VelocityOutputs& out) {
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
      INTRINSIC_RT_LOG(ERROR) << "max_acceleration[" << dof.index              \
                              << "] = " << dof.max_acceleration;               \
      INTRINSIC_RT_LOG(ERROR)                                                  \
          << "max_jerk[" << dof.index << "] = " << dof.max_jerk;               \
      INTRINSIC_RT_LOG(ERROR) << "min_acceleration[" << dof.index              \
                              << "] = " << dof.min_acceleration;               \
      INTRINSIC_RT_LOG(ERROR)                                                  \
          << "min_jerk[" << dof.index << "] = " << dof.min_jerk;               \
      INTRINSIC_RT_LOG(ERROR)                                                  \
          << "target_velocity[" << dof.index << "] = " << dof.target_velocity; \
    }                                                                          \
    return false;
  switch (stat) {
    case reflexxes::Status::kWorking:
    case reflexxes::Status::kFinalStateReached:
      return true;
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
  }
  INTRINSIC_RT_LOG(ERROR) << "Unexpected Reflexxes status "
                          << static_cast<int>(stat);
  return false;
#undef MAKE_ERROR_CASE
}

// quaternion multiplication on vector4d
inline Vector4d Qmul(const Vector4d& a, const Vector4d& b) {
  return Vector4d(a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],   // x
                  a[3] * b[1] + a[1] * b[3] + a[2] * b[0] - a[0] * b[2],   // y
                  a[3] * b[2] + a[2] * b[3] + a[0] * b[1] - a[1] * b[0],   // z
                  a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]);  // w
}

inline Vector4d MakeVector4(const eigenmath::Quaterniond& q) {
  return Vector4d(q.x(), q.y(), q.z(), q.w());
}

bool CartesianVelocityReflexxes::ComputeSetpoint(CartStatePVA* output) {
  // What this function does:
  // 1) Copy target and limits to reflexxes
  // 2) Get reflexxes output for two dt_ timesteps, which are 1/2 the size of
  // the user specified timestep.
  // 3) Copy solution for translation, angular velocity and angular acceleration
  // at last timestep to output
  // 4) Numerically integrate angular velocity to get new quaternion.
  //    Uses RK4 with previous, middle and last timestep.
  //    Normalize quaternion.
  // Caveat: the integration scheme is not well suited for large timesteps and
  // is not structure preserving for the rotations. Geometric integration
  // schemes like Crouch-Grossman (CG3) might be better suited, but require
  // evaluating reflexxes output at odd fractions of the timestep.

  constexpr double kDeltaAngleWarning = 0.1;

  // 1)
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

  // maximum angular velocity and acceleration (no limits on rotations)
  for (unsigned int i = 3; i < 6; ++i) {
    reflexxes::Inputs::DOF& dof = input_params_.GetDOFs()[i];

    dof.max_position = reflexxes::kMaxPositionalLimit;
    dof.max_velocity = limits_.max_rotational_velocity;
    dof.max_acceleration = limits_.max_rotational_acceleration;
    dof.max_jerk = limits_.max_rotational_jerk;

    dof.min_position = -reflexxes::kMaxPositionalLimit;
    dof.min_velocity = -limits_.max_rotational_velocity;
    dof.min_acceleration = -limits_.max_rotational_acceleration;
    dof.min_jerk = -limits_.max_rotational_jerk;
  }

  // 2)
  // get velocities and accelerations for next tick (and subtick) from reflexxes
  eigenmath::Vector6d new_position[2], new_velocity[2], new_acceleration[2];
  eigenmath::Vector6d previous_position;
  eigenmath::Vector6d previous_velocity = previous_.velocity;
  eigenmath::Vector6d previous_acceleration = previous_.acceleration;
  previous_position.head<3>() = previous_.pose.translation();
  previous_position.tail<3>() = reflexxes_position_.tail<3>();

  for (reflexxes::Inputs::DOF& dof : input_params_.GetDOFs()) {
    dof.target_velocity = target_.velocity[dof.index];

    dof.position = previous_position[dof.index];
    dof.velocity = previous_velocity[dof.index];
    dof.acceleration = previous_acceleration[dof.index];
  }

  reflexxes_status_ =
      reflexxes::ComputeVelocity(input_params_, reflexxes_velocity_flags_,
                                 output_params_, reflexxes_state_);

  if (!CheckRMLStatus(reflexxes_status_, output_params_)) {
    return false;
  }

  for (int i = 0; i < 6; i++) {
    reflexxes::Inputs::DOF& input_dof = input_params_.GetDOFs()[i];
    reflexxes::Outputs::DOF& output_dof = output_params_.GetDOFs()[i];
    new_position[0][i] = output_dof.new_position;
    new_velocity[0][i] = output_dof.new_velocity;
    new_acceleration[0][i] = output_dof.new_acceleration;

    input_dof.position = new_position[0][i];
    input_dof.velocity = new_velocity[0][i];
    input_dof.acceleration = new_acceleration[0][i];
  }

  // If minimum_sync_time is used, it needs to be decremented after a call
  // to reflexxes to avoid reflexxes replanning.
  double minimum_sync_time = input_params_.GetMinimumSynchronizationTime();
  if (minimum_sync_time > 0) {
    minimum_sync_time -= input_params_.GetCycleTime();
    if (minimum_sync_time < 0) {
      minimum_sync_time = 0;
    }
    input_params_.SetMinimumSynchronizationTime(minimum_sync_time);
  }

  // reflexxes call 2
  reflexxes_status_ =
      reflexxes::ComputeVelocity(input_params_, reflexxes_velocity_flags_,
                                 output_params_, reflexxes_state_);

  if (!CheckRMLStatus(reflexxes_status_, output_params_)) {
    return false;
  }

  for (const reflexxes::Outputs::DOF& dof : output_params_.GetDOFs()) {
    new_position[1][dof.index] = dof.new_position;
    new_velocity[1][dof.index] = dof.new_velocity;
    new_acceleration[1][dof.index] = dof.new_acceleration;
  }

  // 3)
  // translation, directly use reflexxes solution
  output->pose.setTranslation(new_position[1].head<3>());
  output->velocity.head<3>() = new_velocity[1].head<3>();
  output->acceleration.head<3>() = new_acceleration[1].head<3>();
  // rotations: use velocity and acceleration solution
  output->velocity.tail<3>() = new_velocity[1].tail<3>();
  output->acceleration.tail<3>() = new_acceleration[1].tail<3>();

  // 4)
  // rotation: use RK4 to integrate kinematic equation:
  // dq/dt = 1/2 [omega_x;omega_y;omega_z;0] * [q_x;q_y;q_z;q_w]
  // with: omega: angular velocity, q:quaternion, "*" the quaternion product
  Vector4d omega_quat_0 = Vector4d::Zero();
  Vector4d omega_quat_dt = Vector4d::Zero();
  Vector4d omega_quat_2dt = Vector4d::Zero();
  Vector4d k1, k2, k3, k4;
  omega_quat_0.head<3>() = previous_velocity.tail<3>();
  omega_quat_dt.head<3>() = new_velocity[0].tail<3>();
  omega_quat_2dt.head<3>() = new_velocity[1].tail<3>();

  Vector4d q0 = MakeVector4(previous_.pose.quaternion());

  k1 = 0.5 * Qmul(omega_quat_0, q0);
  k2 = 0.5 * Qmul(omega_quat_dt, q0 + dt_ * k1);
  k3 = 0.5 * Qmul(omega_quat_dt, q0 + dt_ * k2);
  k4 = 0.5 * Qmul(omega_quat_2dt, q0 + 2.0 * dt_ * k3);

  Vector4d qnew = q0 + 2.0 * dt_ / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

  eigenmath::Quaterniond new_quat;
  new_quat.x() = qnew[0];
  new_quat.y() = qnew[1];
  new_quat.z() = qnew[2];
  new_quat.w() = qnew[3];

  new_quat.normalize();

  // check if we rotated very far, which would mean that the integration error
  // is probably larger..
  Eigen::AngleAxisd aa(new_quat * previous_.pose.quaternion().inverse());
  if (aa.angle() > kDeltaAngleWarning) {
    INTRINSIC_RT_LOG(ERROR)
        << "Large angle increment, integration might be inaccurate.";
    return false;
  }

  output->pose.setQuaternion(new_quat);
  reflexxes_position_ = new_position[1];

  return true;
}

}  // namespace icon
}  // namespace intrinsic
