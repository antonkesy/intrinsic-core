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

#include "intrinsic/icon/control/algorithms/joint_position_reflexxes.h"

#include <algorithm>
#include <cstddef>
#include <limits>

#include "absl/log/check.h"
#include "intrinsic/eigenmath/scalar_utils.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/reflexxes/constants_external.h"
#include "intrinsic/icon/reflexxes/flags.h"
#include "intrinsic/icon/reflexxes/inputs.h"
#include "intrinsic/icon/reflexxes/outputs.h"
#include "intrinsic/icon/reflexxes/position_flags.h"
#include "intrinsic/icon/reflexxes/reflexxes_api.h"
#include "intrinsic/icon/reflexxes/status.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"

namespace intrinsic {
namespace icon {

JointPositionReflexxes::JointPositionReflexxes(const std::size_t ndof,
                                               const double frequency)
    : ndof_(ndof),
      reflexxes_state_(static_cast<unsigned int>(ndof), 1 / frequency),
      input_params_(static_cast<unsigned int>(ndof), 1 / frequency),
      output_params_(static_cast<unsigned int>(ndof), 1 / frequency) {
  reflexxes_position_flags_.synchronization_behavior =
      reflexxes::Flags::SyncBehavior::kPhaseSynchronizationWhenCollinear;
  // Only log error on position limit violation.
  reflexxes_position_flags_.positional_limits_behavior =
      reflexxes::Flags::PositionalLimitsBehavior::kIgnore;

  CHECK_OK(limits_.SetSize(ndof));
  CHECK_OK(previous_.SetSize(ndof));
  CHECK_OK(target_.SetSize(ndof));
}

bool JointPositionReflexxes::SetSelection(
    const eigenmath::VectorNb& selection) {
  if (selection.size() != ndof_) {
    INTRINSIC_RT_LOG(ERROR)
        << "size missmatch: selection.size()= " << selection.size()
        << ", but ndof= " << ndof_;
    return false;
  }

  for (reflexxes::Inputs::DOF& dof : input_params_.GetDOFs()) {
    dof.selected = selection[dof.index];
  }

  return true;
}

bool JointPositionReflexxes::SetLimits(const JointLimits& limits) {
  if (limits.size() != ndof_) {
    INTRINSIC_RT_LOG(ERROR)
        << "size missmatch: limits.size()= " << limits.size()
        << ", but ndof= " << ndof_;
    return false;
  }
  if (!limits.IsSizeConsistent()) {
    INTRINSIC_RT_LOG(ERROR) << "inconsistent sizes in limits\n";
    return false;
  }

#define CHECK_AGAINST(lhs, op, rhs, dof)                                 \
  do {                                                                   \
    if (lhs op rhs) {                                                    \
      INTRINSIC_RT_LOG_THROTTLED(ERROR)                                  \
          << "did not expect " #lhs " " #op " " #rhs ", but " #lhs " = " \
          << lhs << " and " #rhs " = " << rhs << " for dof = " << dof;   \
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

    CHECK_AGAINST(limits.max_velocity[dof], <=, 0.0, dof);
    CLAMP_AND_ASSIGN(limits.max_velocity[dof], limits_.max_velocity[dof]);

    CHECK_AGAINST(limits.max_acceleration[dof], <=, 0.0, dof);
    CLAMP_AND_ASSIGN(limits.max_acceleration[dof],
                     limits_.max_acceleration[dof]);

    CHECK_AGAINST(limits.max_jerk[dof], <=, 0.0, dof);
    limits_.max_jerk[dof] =
        eigenmath::Saturate(limits.max_jerk[dof], reflexxes::kMaxJerkLimit);
  }
#undef CLAMP_AND_ASSIGN
#undef CHECK_AGAINST

  return true;
}

bool JointPositionReflexxes::SetTarget(const JointStatePV& target) {
  if (target.size() != ndof_) {
    INTRINSIC_RT_LOG(ERROR)
        << "size missmatch: target.size()= " << target.size()
        << ", but ndof= " << ndof_;
    return false;
  }
  if (!target.IsSizeConsistent()) {
    INTRINSIC_RT_LOG(ERROR) << "inconsistent sizes in target";
    return false;
  }
  target_ = target;

  return true;
}

bool JointPositionReflexxes::SetPrevious(const JointStatePVA& previous) {
  if (previous.size() != ndof_) {
    INTRINSIC_RT_LOG(ERROR)
        << "size missmatch: previous.size()= " << previous.size()
        << ", but ndof= " << ndof_;
    return false;
  }
  if (!previous.IsSizeConsistent()) {
    INTRINSIC_RT_LOG(ERROR) << "inconsistent sizes in previous";
    return false;
  }
  previous_ = previous;

  return true;
}

bool JointPositionReflexxes::ComputeSetpoint(JointStatePVA* output) {
  CHECK(nullptr != output) << "Output is null";
  if (output->size() != ndof_) {
    INTRINSIC_RT_LOG(ERROR)
        << "size missmatch: output.size()= " << output->size()
        << " but ndof= " << ndof_;
    return false;
  }
  if (!output->IsSizeConsistent()) {
    INTRINSIC_RT_LOG(ERROR) << "inconsistent sizes in output";
    return false;
  }

  for (reflexxes::Inputs::DOF& dof : input_params_.GetDOFs()) {
    dof.max_position = limits_.max_position[dof.index];
    dof.max_velocity = limits_.max_velocity[dof.index];
    dof.max_acceleration = limits_.max_acceleration[dof.index];
    dof.max_jerk = limits_.max_jerk[dof.index];

    dof.min_position = limits_.min_position[dof.index];
    dof.min_velocity = -limits_.max_velocity[dof.index];
    dof.min_acceleration = -limits_.max_acceleration[dof.index];
    dof.min_jerk = -limits_.max_jerk[dof.index];

    dof.target_position = target_.position[dof.index];
    dof.target_velocity = target_.velocity[dof.index];

    dof.position = previous_.position[dof.index];
    dof.velocity = previous_.velocity[dof.index];
    dof.acceleration = previous_.acceleration[dof.index];
  }

  reflexxes_status_ =
      reflexxes::ComputePosition(input_params_, reflexxes_position_flags_,
                                 output_params_, reflexxes_state_);

#define MAKE_ERROR_CASE(x)                                                     \
  case x:                                                                      \
    INTRINSIC_RT_LOG(ERROR)                                                    \
        << "Got error from reflexxes call: " << reflexxes::GetStatusString(x); \
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
    case reflexxes::Status::kWorking:
    case reflexxes::Status::kFinalStateReached:
      break;
    case reflexxes::Status::kErrorPositionalLimits:
      // not an error, violation checked below
      break;
    default:
      INTRINSIC_RT_LOG(ERROR) << "Unexpected Reflexxes status "
                              << static_cast<int>(reflexxes_status_);
      return false;
  }
#undef MAKE_ERROR_CASE

  // Check for joint limit violations, but only fault if violations are
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
      // when moving exactly along a joint constraint, reflexxes can cause
      // small violations,
      if ((position[dof.index] > upper) &&
          (velocity[dof.index] > std::numeric_limits<double>::epsilon())) {
        INTRINSIC_RT_LOG(ERROR)
            << "Upper position limit violation for dof= " << dof.index
            << ", and velocity > 0; position= " << position[dof.index]
            << "; upper= " << upper
            << "; (upper-position)= " << upper - position[dof.index]
            << "; velocity= " << velocity[dof.index];
        limit_fault = true;
      } else if ((position[dof.index] < lower) &&
                 (velocity[dof.index] <
                  -std::numeric_limits<double>::epsilon())) {
        INTRINSIC_RT_LOG(ERROR)
            << "Lower position limit violation for dof= " << dof.index
            << ", and velocity < 0; position= " << position[dof.index]
            << "; lower = " << lower
            << "; (lower-position)= " << lower - position[dof.index]
            << "; velocity= " << velocity[dof.index];

        limit_fault = true;
      }
    }

    if (limit_fault) {
      return false;
    } else {
      // Threshold below which the target is considered reached.
      // Reflexxes does not always hit the target with < DBL_EPSILON
      // precision, but the below value was determined to work
      // experimentally.
      constexpr double kTargetEpsilon = 1e-6;
      const double position_delta =
          (position - target_.position).lpNorm<Eigen::Infinity>();
      const double velocity_delta =
          (velocity - target_.velocity).lpNorm<Eigen::Infinity>();
      if ((position_delta < kTargetEpsilon) &&
          (velocity_delta < kTargetEpsilon)) {
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

void JointPositionReflexxes::SetPositionalLimitsBehavior(
    reflexxes::Flags::PositionalLimitsBehavior behavior) {
  reflexxes_position_flags_.positional_limits_behavior = behavior;
}

void JointPositionReflexxes::SetBehaviorIfInitialStateBreachesConstraints(
    reflexxes::PositionFlags::BehaviorIfInitialStateBreachesConstraints
        behavior) {
  reflexxes_position_flags_.behavior_if_initial_state_breaches_constraints =
      behavior;
}

}  // namespace icon
}  // namespace intrinsic
