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

#include "intrinsic/icon/control/algorithms/joint_velocity_reflexxes.h"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <optional>

#include "absl/log/check.h"
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
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"

namespace intrinsic {
namespace icon {

JointVelocityReflexxes::JointVelocityReflexxes(const std::size_t ndof,
                                               const double frequency)
    : ndof_(ndof),
      reflexxes_state_(static_cast<unsigned int>(ndof), 1 / frequency),
      input_params_(static_cast<unsigned int>(ndof), 1 / frequency),
      output_params_(static_cast<unsigned int>(ndof), 1 / frequency) {
  reflexxes_velocity_flags_.synchronization_behavior =
      reflexxes::Flags::SyncBehavior::kPhaseSynchronizationWhenCollinear;
  // Ignore position limit violations by default
  reflexxes_velocity_flags_.positional_limits_behavior =
      reflexxes::Flags::PositionalLimitsBehavior::kIgnore;

  CHECK_OK(limits_.SetSize(ndof));
  CHECK_OK(previous_.SetSize(ndof));
  CHECK_OK(target_.SetSize(ndof));
}

bool JointVelocityReflexxes::SetSelection(
    const eigenmath::VectorNb& selection) {
  if (selection.size() != ndof_) {
    INTRINSIC_RT_LOG(ERROR)
        << "size mismatch: selection.size()= " << selection.size()
        << ", but ndof= " << ndof_;
    return false;
  }

  for (reflexxes::Inputs::DOF& dof : input_params_.GetDOFs()) {
    dof.selected = selection[dof.index];
  }
  return true;
}

bool JointVelocityReflexxes::SetLimits(const JointLimits& limits) {
  if (limits.size() != ndof_) {
    INTRINSIC_RT_LOG(ERROR) << "size mismatch: limits.size()= " << limits.size()
                            << ", but ndof= " << ndof_;
    return false;
  }
  if (!limits.IsSizeConsistent()) {
    INTRINSIC_RT_LOG(ERROR) << "inconsistent sizes in limits";
    return false;
  }

#define CHECK_AGAINST(lhs, op, rhs, dof)                                 \
  do {                                                                   \
    if (lhs op rhs) {                                                    \
      INTRINSIC_RT_LOG(ERROR)                                            \
          << "did not expect " #lhs " " #op " " #rhs ", but " #lhs " = " \
          << lhs << " and " #rhs "= " << rhs << " for dof = " << dof;    \
      return false;                                                      \
    }                                                                    \
  } while (0)

#define CLAMP_AND_ASSIGN(in, out)                             \
  out = std::clamp(in, std::numeric_limits<double>::lowest(), \
                   std::numeric_limits<double>::max());

  for (eigenmath::VectorNd::Index dof = 0; dof < limits.size(); ++dof) {
    CHECK_AGAINST(limits.max_position[dof], <=, limits.min_position[dof], dof);
    limits_.min_position[dof] = eigenmath::Saturate(
        limits.min_position[dof], reflexxes::kMaxPositionalLimit);
    limits_.max_position[dof] = eigenmath::Saturate(
        limits.max_position[dof], reflexxes::kMaxPositionalLimit);

    CHECK_AGAINST(limits.max_acceleration[dof], <=, 0.0, dof);
    CLAMP_AND_ASSIGN(limits.max_acceleration[dof],
                     limits_.max_acceleration[dof]);

    CHECK_AGAINST(limits.max_jerk[dof], <=, 0.0, dof);
    limits_.max_jerk[dof] =
        eigenmath::Saturate(limits.max_jerk[dof], reflexxes::kMaxJerkLimit);
  }

#undef CLAMP_AND_ASSIGN
#undef CHECK_AGAINST
  // Resets the cached value that stores if in the last call of ComputeSetpoint
  // the trajectory breached the positional limits.
  trajectory_breaches_limits_ = std::nullopt;
  // Resets the cached position at the target velocity.
  position_at_velocity_target_ = std::nullopt;

  return true;
}

bool JointVelocityReflexxes::SetTarget(const JointStateV& target) {
  if (target.size() != ndof_) {
    INTRINSIC_RT_LOG(ERROR) << "size mismatch: target.size()= " << target.size()
                            << ", but ndof= " << ndof_;
    return false;
  }
  if (!target.IsSizeConsistent()) {
    INTRINSIC_RT_LOG(ERROR) << "inconsistent sizes in target";
    return false;
  }
  target_ = target;

  // Resets the cached value that stores if in the last call of ComputeSetpoint
  // the trajectory breached the positional limits.
  trajectory_breaches_limits_ = std::nullopt;
  // Resets the cached position at the target velocity.
  position_at_velocity_target_ = std::nullopt;

  return true;
}

bool JointVelocityReflexxes::SetPrevious(const JointStatePVA& previous) {
  if (previous.size() != ndof_) {
    INTRINSIC_RT_LOG(ERROR)
        << "size mismatch: previous.size()= " << previous.size()
        << ", but ndof= " << ndof_;
    return false;
  }
  if (!previous.IsSizeConsistent()) {
    INTRINSIC_RT_LOG(ERROR) << "inconsistent sizes in previous";
    return false;
  }
  previous_ = previous;

  // Resets the cached value that stores if in the last call of ComputeSetpoint
  // the trajectory breached the positional limits.
  trajectory_breaches_limits_ = std::nullopt;
  // Resets the cached position at the target velocity.
  position_at_velocity_target_ = std::nullopt;
  return true;
}

bool JointVelocityReflexxes::ComputeSetpoint(JointStatePVA* output) {
  CHECK(nullptr != output) << "Output is null";
  if (output->size() != ndof_) {
    INTRINSIC_RT_LOG(ERROR)
        << "size mismatch: output.size()= " << output->size()
        << ", but ndof= " << ndof_;
    return false;
  }
  if (!output->IsSizeConsistent()) {
    INTRINSIC_RT_LOG(ERROR) << "inconsistent sizes in output";
    return false;
  }

  for (reflexxes::Inputs::DOF& dof : input_params_.GetDOFs()) {
    dof.max_position = limits_.max_position[dof.index];
    dof.max_acceleration = limits_.max_acceleration[dof.index];
    dof.max_jerk = limits_.max_jerk[dof.index];

    dof.min_position = limits_.min_position[dof.index];
    dof.min_acceleration = -limits_.max_acceleration[dof.index];
    dof.min_jerk = -limits_.max_jerk[dof.index];

    dof.target_velocity = target_.velocity[dof.index];

    dof.position = previous_.position[dof.index];
    dof.velocity = previous_.velocity[dof.index];
    dof.acceleration = previous_.acceleration[dof.index];
  }

  reflexxes_status_ =
      reflexxes::ComputeVelocity(input_params_, reflexxes_velocity_flags_,
                                 output_params_, reflexxes_state_);
  position_at_velocity_target_ =
      output_params_.GetPositionValuesAtTargetVelocity();

  trajectory_breaches_limits_ =
      reflexxes::DoesStopMotionFromNewStateBreachPositionalLimits(
          input_params_, reflexxes_velocity_flags_, output_params_);

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
      MAKE_ERROR_CASE(reflexxes::Status::kErrorUndefined);
    case reflexxes::Status::kErrorPositionalLimits:
      // not an error, violation checked below
      break;
    default:
      INTRINSIC_RT_LOG(ERROR) << "Unexpected Reflexxes status "
                              << static_cast<int>(reflexxes_status_);
      return false;
  }
#undef MAKE_ERROR_CASE

  // Check for joint limit violations, but only fault if the violations are
  // getting worse.
  if (reflexxes_status_ == reflexxes::Status::kErrorPositionalLimits) {
    bool limit_fault = false;
    eigenmath::VectorNd position(ndof_);
    eigenmath::VectorNd velocity(ndof_);
    for (const reflexxes::Outputs::DOF& dof : output_params_.GetDOFs()) {
      position[dof.index] = dof.new_position;
      velocity[dof.index] = dof.new_velocity;
      const double upper = limits_.max_position[dof.index];
      const double lower = limits_.min_position[dof.index];
      if ((position[dof.index] > upper) && (velocity[dof.index] > 0)) {
        INTRINSIC_RT_LOG_THROTTLED(ERROR)
            << "Upper position limit violation for dof= " << dof.index
            << ", and velocity " << velocity[dof.index] << " > 0 "
            << "[position= " << position[dof.index] << "; upper= " << upper
            << "]";
        limit_fault = true;
      } else if ((position[dof.index] < lower) && (velocity[dof.index] < 0)) {
        INTRINSIC_RT_LOG_THROTTLED(ERROR)
            << "Lower position limit violation for dof= " << dof.index
            << ", and velocity " << velocity[dof.index] << " > 0 "
            << "[position= " << position[dof.index] << "; lower= " << lower
            << "]";
        limit_fault = true;
      }
    }

    if (limit_fault) {
      return false;
    } else {
      // Threshold below which the target is considered reached.
      // Reflexxes does not always hit the target with < DBL_EPSILON precision,
      // but 100 x DBL_EPSILON was determined to work experimentally.
      constexpr double kTargetEpsilon =
          std::numeric_limits<double>::epsilon() * 100;
      const double velocity_delta =
          (velocity - target_.velocity).lpNorm<Eigen::Infinity>();
      if (velocity_delta < kTargetEpsilon) {
        reflexxes_status_ = reflexxes::Status::kFinalStateReached;
      } else {
        reflexxes_status_ = reflexxes::Status::kWorking;
      }
    }
  }

  for (const reflexxes::Outputs::DOF& dof : output_params_.GetDOFs()) {
    output->position[dof.index] = dof.new_position;
    output->velocity[dof.index] = dof.new_velocity;
    output->acceleration[dof.index] = dof.new_acceleration;
  }

  return true;
}

void JointVelocityReflexxes::SetPositionalLimitsBehavior(
    reflexxes::Flags::PositionalLimitsBehavior behavior) {
  reflexxes_velocity_flags_.positional_limits_behavior = behavior;
}

icon::RealtimeStatusOr<bool>
JointVelocityReflexxes::DoesStopMotionFromNewStateBreachPositionalLimits()
    const {
  if (!trajectory_breaches_limits_.has_value()) {
    return icon::FailedPreconditionError(
        "You must call ComputeSetpoint() before calling this function.");
  }
  return trajectory_breaches_limits_.value();
}

icon::RealtimeStatusOr<JointStateP>
JointVelocityReflexxes::PositionAtTargetVelocity() const {
  if (!position_at_velocity_target_.has_value()) {
    return icon::FailedPreconditionError(
        "You must call ComputeSetpoint() before calling this function.");
  }
  JointStateP position_at_target;
  INTRINSIC_RT_RETURN_IF_ERROR(position_at_target.SetSize(ndof_));
  for (size_t i = 0; i < ndof_; ++i) {
    position_at_target.position[i] = position_at_velocity_target_.value()[i];
  }
  return position_at_target;
}

}  // namespace icon
}  // namespace intrinsic
