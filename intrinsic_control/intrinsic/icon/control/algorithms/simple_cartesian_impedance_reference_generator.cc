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


#include "intrinsic/icon/control/algorithms/simple_cartesian_impedance_reference_generator.h"

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/actions/cartesian_impedance_parameters.h"
#include "intrinsic/icon/control/algorithms/cartesian_impedance_commons.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/cart_state.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/kinematics/types/check_cartesian_limits.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/ipow.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/twist.h"

namespace intrinsic::icon {

namespace {

// A first-order filter for reference smoothing in Euclidean space.
template <typename T>
T filter(const T& target, const T& current, const double alpha) {
  return T(alpha * target + (1.0 - alpha) * current);
}

}  // namespace

SimpleCartesianImpedanceReferenceGenerator::
    SimpleCartesianImpedanceReferenceGenerator(double alpha)
    : alpha_(alpha) {}

RealtimeStatus SimpleCartesianImpedanceReferenceGenerator::SetReference(
    const cartesian_impedance::RealTimeCartesianTarget& reference,
    const CartesianLimits& cartesian_limits) {
  CartStatePV target_state;
  target_state.pose = reference.robot_base_t_task *
                      Pose3d(reference.task_t_tool_reference_orientation,
                             reference.task_t_tool_reference_position);
  target_state.velocity = reference.tool_reference_twist;
  if (!IsWithinLimits(target_state, cartesian_limits)) {
    return icon::FailedPreconditionError("Target state is not within limits.");
  }

  nominal_reference_ = reference;
  return OkStatus();
}

RealtimeStatusOr<cartesian_impedance::RealTimeCartesianTarget>
SimpleCartesianImpedanceReferenceGenerator::Evaluate(
    const cartesian_impedance::RealTimeCartesianTarget& current,
    const CartStatePVA& cart_state_sensed, double speed_override) {
  if (speed_override < kMinimumSpeedOverride ||
      speed_override > kMaximumSpeedOverride) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "speed_override should be in range [", kMinimumSpeedOverride, ", ",
        kMaximumSpeedOverride, "], but instead has value ", speed_override,
        "."));
  }

  cartesian_impedance::RealTimeCartesianTarget new_target;
  const double speed_overridden_alpha = speed_override * alpha_;
  cartesian_impedance::RealTimeCartesianTarget scaled_reference =
      nominal_reference_;
  // We only scale twist and acceleration here, not wrench and other quantities.
  scaled_reference.tool_reference_twist *= speed_override;
  scaled_reference.tool_reference_acceleration *=
      ::intrinsic::IPow(speed_override, 2);
  new_target.task_t_tool_reference_position =
      filter(scaled_reference.task_t_tool_reference_position,
             current.task_t_tool_reference_position, speed_overridden_alpha);
  new_target.tool_reference_twist =
      filter(scaled_reference.tool_reference_twist,
             current.tool_reference_twist, speed_overridden_alpha);
  new_target.tool_reference_acceleration =
      filter(scaled_reference.tool_reference_acceleration,
             current.tool_reference_acceleration, speed_overridden_alpha);
  new_target.tool_reference_wrench =
      filter(scaled_reference.tool_reference_wrench,
             current.tool_reference_wrench, speed_overridden_alpha);
  new_target.virtual_cartesian_inertia_inverse =
      filter(scaled_reference.virtual_cartesian_inertia_inverse,
             current.virtual_cartesian_inertia_inverse, speed_overridden_alpha);
  new_target.cartesian_stiffness =
      filter(scaled_reference.cartesian_stiffness, current.cartesian_stiffness,
             speed_overridden_alpha);
  new_target.cartesian_damping =
      filter(scaled_reference.cartesian_damping, current.cartesian_damping,
             speed_overridden_alpha);

  new_target.wrench_selection_matrix =
      filter(scaled_reference.wrench_selection_matrix,
             current.wrench_selection_matrix, speed_overridden_alpha);

  new_target.task_t_tool_reference_orientation =
      current.task_t_tool_reference_orientation.slerp(
          speed_overridden_alpha,
          scaled_reference.task_t_tool_reference_orientation);

  new_target.robot_tip_t_robot_tool.setQuaternion(
      current.robot_tip_t_robot_tool.quaternion().slerp(
          speed_overridden_alpha,
          scaled_reference.robot_tip_t_robot_tool.quaternion()));
  new_target.robot_tip_t_robot_tool.translation() = filter(
      scaled_reference.robot_tip_t_robot_tool.translation(),
      current.robot_tip_t_robot_tool.translation(), speed_overridden_alpha);

  new_target.robot_base_t_task.setQuaternion(
      current.robot_base_t_task.quaternion().slerp(
          speed_overridden_alpha,
          scaled_reference.robot_base_t_task.quaternion()));
  new_target.robot_base_t_task.translation() =
      filter(scaled_reference.robot_base_t_task.translation(),
             current.robot_base_t_task.translation(), speed_overridden_alpha);

  return new_target;
}

const cartesian_impedance::RealTimeCartesianTarget&
SimpleCartesianImpedanceReferenceGenerator::GetReference() const {
  return nominal_reference_;
}

RealtimeStatus SimpleCartesianImpedanceReferenceGenerator::Configure(
    const cartesian_impedance::SimpleImpedanceGeneratorParameters& params) {
  if (params.lowpass_filter_constant > 1.0 ||
      params.lowpass_filter_constant <= 0.0) {
    return FailedPreconditionError(
        "Lowpass filter constant out of bound ]0 1].");
  }
  alpha_ = params.lowpass_filter_constant;
  return OkStatus();
}

SimpleNullspaceReferenceGenerator::SimpleNullspaceReferenceGenerator(
    double alpha)
    : alpha_(alpha) {}

RealtimeStatusOr<cartesian_impedance::RealTimeNullspaceTarget>
SimpleNullspaceReferenceGenerator::Evaluate(
    const cartesian_impedance::RealTimeNullspaceTarget& current,
    double speed_override) {
  if (speed_override < kMinimumSpeedOverride ||
      speed_override > kMaximumSpeedOverride) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "speed_override should be in range [", kMinimumSpeedOverride, ", ",
        kMaximumSpeedOverride, "], but instead has value ", speed_override,
        "."));
  }

  cartesian_impedance::RealTimeNullspaceTarget new_target;
  const double speed_overridden_alpha = speed_override * alpha_;
  cartesian_impedance::RealTimeNullspaceTarget scaled_reference =
      nominal_reference_;
  scaled_reference.joint_state.velocity *= speed_override;
  scaled_reference.joint_state.acceleration *=
      ::intrinsic::IPow(speed_override, 2);
  new_target.joint_state.position =
      filter(scaled_reference.joint_state.position,
             current.joint_state.position, speed_overridden_alpha);
  new_target.joint_state.velocity =
      filter(scaled_reference.joint_state.velocity,
             current.joint_state.velocity, speed_overridden_alpha);
  new_target.joint_state.acceleration =
      filter(scaled_reference.joint_state.acceleration,
             current.joint_state.acceleration, speed_overridden_alpha);
  new_target.nullspace_stiffness =
      filter(scaled_reference.nullspace_stiffness, current.nullspace_stiffness,
             speed_overridden_alpha);
  new_target.nullspace_damping =
      filter(scaled_reference.nullspace_damping, current.nullspace_damping,
             speed_overridden_alpha);

  return new_target;
}

RealtimeStatus SimpleNullspaceReferenceGenerator::SetReference(
    const cartesian_impedance::RealTimeNullspaceTarget& reference,
    const JointLimits& joint_limits) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      bool within_limits, IsWithinLimits(reference.joint_state, joint_limits));
  if (!within_limits) {
    return icon::FailedPreconditionError("Joint state is not within limits.");
  }
  nominal_reference_ = reference;
  return OkStatus();
}

RealtimeStatus SimpleNullspaceReferenceGenerator::Configure(
    const cartesian_impedance::SimpleNullspaceReferenceGeneratorParameters&
        params) {
  if (params.lowpass_filter_constant > 1.0 ||
      params.lowpass_filter_constant <= 0.0) {
    return FailedPreconditionError(
        "Lowpass filter constant out of bound ]0 1].");
  }
  alpha_ = params.lowpass_filter_constant;
  return OkStatus();
}

}  // namespace intrinsic::icon
