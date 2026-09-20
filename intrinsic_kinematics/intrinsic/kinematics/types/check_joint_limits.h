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

#ifndef INTRINSIC_KINEMATICS_TYPES_CHECK_JOINT_LIMITS_H_
#define INTRINSIC_KINEMATICS_TYPES_CHECK_JOINT_LIMITS_H_

#include <math.h>

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/utils/fixed_string.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {

// Result type for limit check functions.
struct LimitCheckResult {
  bool p_ok = true;
  bool v_ok = true;
  bool a_ok = true;
  bool j_ok = true;
  bool t_ok = true;

  // Mark operator bool() explicit to allow incremental
  // migration of code that expects IsWithinLimits functions to return
  // StatusOr<bool>.
  // NOLINTNEXTLINE
  operator bool() const { return p_ok && v_ok && a_ok && j_ok && t_ok; }

  bool operator==(const LimitCheckResult& other) const {
    return other.p_ok == this->p_ok && other.v_ok == this->v_ok &&
           other.a_ok == this->a_ok && other.j_ok == this->j_ok &&
           other.t_ok == this->t_ok;
  }
};

// Returns a (terse) fixed string listing violated joint limits.
icon::FixedString<16> ToFixedString(const LimitCheckResult& limit_check_result);

// These functions support either JointLimits or JointLimitsXd, but
// they should be realtime safe when JointLimits is used.
template <typename T>
icon::RealtimeStatusOr<LimitCheckResult> IsWithinLimits(
    const JointStateP& joint_state, const T& joint_limits) {
  if (joint_state.size() != joint_limits.size()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Size of provided joint state of ", joint_state.size(),
        " is different from the limit size ", joint_limits.size()));
  }

  if ((joint_state.position.array() < joint_limits.min_position.array())
          .any() ||
      (joint_state.position.array() > joint_limits.max_position.array())
          .any()) {
    return LimitCheckResult{.p_ok = false};
  }
  return LimitCheckResult{.p_ok = true};
}

template <typename T>
icon::RealtimeStatusOr<LimitCheckResult> IsWithinLimits(
    const JointStateV& joint_state, const T& joint_limits) {
  if (joint_state.size() != joint_limits.size()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Size of provided joint state of ", joint_state.size(),
        " is different from the limit size ", joint_limits.size()));
  }

  if ((joint_state.velocity.array().abs() > joint_limits.max_velocity.array())
          .any()) {
    return LimitCheckResult{.v_ok = false};
  }

  return LimitCheckResult{.v_ok = true};
}

template <typename T>
icon::RealtimeStatusOr<LimitCheckResult> IsWithinLimits(
    const JointStateT& joint_state, const T& joint_limits) {
  if (joint_state.size() != joint_limits.size()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Size of provided joint state of ", joint_state.size(),
        " is different from the limit size ", joint_limits.size()));
  }

  if ((joint_state.torque.array().abs() > joint_limits.max_torque.array())
          .any()) {
    return LimitCheckResult{.t_ok = false};
  }

  return LimitCheckResult{.t_ok = true};
}

template <typename T>
icon::RealtimeStatusOr<LimitCheckResult> IsWithinLimits(
    const JointStatePV& joint_state, const T& joint_limits) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto in_p_limits,
      IsWithinLimits(static_cast<const JointStateP&>(joint_state),
                     joint_limits));
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto in_v_limits,
      IsWithinLimits(static_cast<const JointStateV&>(joint_state),
                     joint_limits));

  return LimitCheckResult{.p_ok = in_p_limits.p_ok, .v_ok = in_v_limits.v_ok};
}

template <typename T>
icon::RealtimeStatusOr<LimitCheckResult> IsWithinLimits(
    const JointStatePVA& joint_state, const T& joint_limits) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto in_p_limits,
      IsWithinLimits(static_cast<const JointStateP&>(joint_state),
                     joint_limits));
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto in_v_limits,
      IsWithinLimits(static_cast<const JointStateV&>(joint_state),
                     joint_limits));

  bool a_ok = true;
  if ((joint_state.acceleration.array().abs() >
       joint_limits.max_acceleration.array())
          .any()) {
    a_ok = false;
  }
  return LimitCheckResult{
      .p_ok = in_p_limits.p_ok, .v_ok = in_v_limits.v_ok, .a_ok = a_ok};
}

template <typename T>
icon::RealtimeStatusOr<LimitCheckResult> IsWithinLimits(
    const JointStatePVAJ& joint_state, const T& joint_limits) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto in_pva_limits,
      IsWithinLimits(static_cast<const JointStatePVA&>(joint_state),
                     joint_limits));

  bool j_ok = true;
  if ((joint_state.jerk.array().abs() > joint_limits.max_jerk.array()).any()) {
    j_ok = false;
  }
  return LimitCheckResult{.p_ok = in_pva_limits.p_ok,
                          .v_ok = in_pva_limits.v_ok,
                          .a_ok = in_pva_limits.a_ok,
                          .j_ok = j_ok};
}

template <typename T>
icon::RealtimeStatusOr<LimitCheckResult> IsWithinLimits(
    const eigenmath::VectorNd& joint_position, const T& joint_limits) {
  if (joint_position.size() != joint_limits.size()) {
    return icon::FailedPreconditionError(icon::RealtimeStatus::StrCat(
        "Size of joint vector does not match limits size: ",
        joint_position.size(), " vs ", joint_limits.size()));
  }
  if ((joint_position.array() < joint_limits.min_position.array()).any() ||
      (joint_position.array() > joint_limits.max_position.array()).any()) {
    return LimitCheckResult{.p_ok = false};
  }
  return LimitCheckResult{.p_ok = true};
}

template <typename T>
icon::RealtimeStatusOr<LimitCheckResult> IsWithinLimits(
    const eigenmath::VectorXd& joint_position, const T& joint_limits,
    double epsilon) {
  const eigenmath::VectorXd epsilon_vec =
      eigenmath::VectorXd::Constant(joint_limits.size(), epsilon);

  if (joint_position.size() != joint_limits.size()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Size of joint vector does not match limits size: ",
        joint_position.size(), " vs ", joint_limits.size()));
  }
  if ((joint_position.array() >
       (joint_limits.max_position + epsilon_vec).array())
          .any() ||
      (joint_position.array() <
       (joint_limits.min_position - epsilon_vec).array())
          .any()) {
    return LimitCheckResult{.p_ok = false};
  }
  return LimitCheckResult{.p_ok = true};
}

template <typename T>
icon::RealtimeStatusOr<LimitCheckResult> IsWithinLimits(
    const eigenmath::VectorXd& joint_position, const T& joint_limits) {
  return IsWithinLimits(joint_position, joint_limits, 0.0);
}

template <typename T>
icon::RealtimeStatusOr<LimitCheckResult> IsWithinLimits(
    const T& limits_to_check, const T& joint_limits) {
  if (limits_to_check.size() != joint_limits.size()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Inconsistent limit size ", limits_to_check.size(), " vs ",
        joint_limits.size()));
  }

  LimitCheckResult result{
      .p_ok = !((limits_to_check.min_position.array() <
                 joint_limits.min_position.array())
                    .any() ||
                (limits_to_check.max_position.array() >
                 joint_limits.max_position.array())
                    .any()),
      .v_ok = !((limits_to_check.max_velocity.array() >
                 joint_limits.max_velocity.array())
                    .any()),
      .a_ok = !((limits_to_check.max_acceleration.array() >
                 joint_limits.max_acceleration.array())
                    .any()),
      .j_ok =
          !((limits_to_check.max_jerk.array() > joint_limits.max_jerk.array())
                .any()),
      .t_ok = !(
          (limits_to_check.max_torque.array() > joint_limits.max_torque.array())
              .any())};
  return result;
}

template <typename T>
icon::RealtimeStatusOr<LimitCheckResult> IsWithinLimits(
    const T& limits_to_check, const T& joint_limits, double epsilon) {
  if (limits_to_check.size() != joint_limits.size()) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "Inconsistent limit size ", limits_to_check.size(), " vs ",
        joint_limits.size()));
  }

  eigenmath::VectorNd epsilon_vec =
      eigenmath::VectorNd::Constant(joint_limits.size(), epsilon);

  LimitCheckResult result{
      .p_ok = !((limits_to_check.min_position.array() <
                 (joint_limits.min_position - epsilon_vec).array())
                    .any() ||
                (limits_to_check.max_position.array() >
                 (joint_limits.max_position + epsilon_vec).array())
                    .any()),
      .v_ok = !((limits_to_check.max_velocity.array() >
                 (joint_limits.max_velocity + epsilon_vec).array())
                    .any()),
      .a_ok = !((limits_to_check.max_acceleration.array() >
                 (joint_limits.max_acceleration + epsilon_vec).array())
                    .any()),
      .j_ok = !((limits_to_check.max_jerk.array() >
                 (joint_limits.max_jerk + epsilon_vec).array())
                    .any()),
      .t_ok = !((limits_to_check.max_torque.array() >
                 (joint_limits.max_torque + epsilon_vec).array())
                    .any())};

  return result;
}

}  // namespace intrinsic

#endif  // INTRINSIC_KINEMATICS_TYPES_CHECK_JOINT_LIMITS_H_
