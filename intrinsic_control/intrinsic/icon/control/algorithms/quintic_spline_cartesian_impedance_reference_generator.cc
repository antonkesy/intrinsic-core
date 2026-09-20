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

#include "intrinsic/icon/control/algorithms/quintic_spline_cartesian_impedance_reference_generator.h"

#include <algorithm>
#include <limits>

#include "absl/log/check.h"
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
#include "intrinsic/math/spline/polynomial_spline.h"
#include "intrinsic/math/twist.h"

namespace intrinsic::icon {

namespace {

const double kMinusculeRegularizer = std::numeric_limits<double>::epsilon();
const double kMaximumMovementDurationSeconds =
    3.154e+12;  // equal to 100,000 years

CartStatePVA MakeCartStatePva(
    const cartesian_impedance::RealTimeCartesianTarget& rt_cartesian_target) {
  CartStatePVA cart_state_pva;
  cart_state_pva.pose =
      Pose3d(rt_cartesian_target.task_t_tool_reference_orientation,
             rt_cartesian_target.task_t_tool_reference_position);
  cart_state_pva.velocity = rt_cartesian_target.tool_reference_twist;
  cart_state_pva.acceleration = rt_cartesian_target.tool_reference_acceleration;
  return cart_state_pva;
}

}  // namespace

QuinticSplineCartesianImpedanceReferenceGenerator::
    QuinticSplineCartesianImpedanceReferenceGenerator(double dt_seconds)
    : dt_seconds_([&]() {
        CHECK(dt_seconds >=
              intrinsic::polynomial_spline_internal::kMinimumDtSeconds)
            << "dt_seconds must be >= "
            << intrinsic::polynomial_spline_internal::kMinimumDtSeconds;
        return dt_seconds;
      }()) {}

RealtimeStatus QuinticSplineCartesianImpedanceReferenceGenerator::SetReference(
    const cartesian_impedance::RealTimeCartesianTarget& reference,
    const CartesianLimits& cartesian_limits) {
  if (nominal_movement_duration_seconds_ < 0.0) {
    return FailedPreconditionError(
        "Movement duration not set. Did you call Configure()?");
  }
  // Reset internal states:
  phase_to_go_ = 1.0;

  CartStatePVA target_state;
  target_state.pose = reference.robot_base_t_task *
                      Pose3d(reference.task_t_tool_reference_orientation,
                             reference.task_t_tool_reference_position);
  target_state.velocity = reference.tool_reference_twist;
  target_state.acceleration = reference.tool_reference_acceleration;
  if (!IsWithinLimits(target_state, cartesian_limits)) {
    return FailedPreconditionError("Target state is not within limits.");
  }
  reference_ = reference;
  return OkStatus();
}

RealtimeStatusOr<cartesian_impedance::RealTimeCartesianTarget>
QuinticSplineCartesianImpedanceReferenceGenerator::Evaluate(
    const cartesian_impedance::RealTimeCartesianTarget& current,
    const CartStatePVA& cart_state_sensed, double speed_override) {
  if (speed_override < kMinimumSpeedOverride ||
      speed_override > kMaximumSpeedOverride) {
    return InvalidArgumentError(RealtimeStatus::StrCat(
        "speed_override should be in range [", kMinimumSpeedOverride, ", ",
        kMaximumSpeedOverride, "], but instead has value ", speed_override,
        "."));
  }
  if (phase_to_go_ < 0) {
    return OutOfRangeError(
        "phase_to_go_ must be >= 0, probably missing a call to "
        "SetReference()?");
  }
  const double movement_duration_seconds =
      std::min(nominal_movement_duration_seconds_ /
                   (speed_override + kMinusculeRegularizer),
               kMaximumMovementDurationSeconds);
  double time_to_go_seconds = phase_to_go_ * movement_duration_seconds;

  cartesian_impedance::RealTimeCartesianTarget scaled_reference = reference_;
  scaled_reference.tool_reference_twist *= speed_override;
  scaled_reference.tool_reference_acceleration *=
      ::intrinsic::IPow(speed_override, 2);
  cartesian_impedance::RealTimeCartesianTarget next_target = scaled_reference;
  if (time_to_go_seconds < dt_seconds_) {
    time_to_go_seconds = 0.0;
  } else {
    CartStatePVA current_state = MakeCartStatePva(current);
    CartStatePVA reference_state = MakeCartStatePva(scaled_reference);

    CartStatePVA next_state;
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        next_state,
        GetNextStateCartesianPose3QuinticSpline(
            current_state, reference_state, time_to_go_seconds, dt_seconds_));

    time_to_go_seconds -= dt_seconds_;

    next_target.task_t_tool_reference_orientation =
        next_state.pose.quaternion();
    next_target.task_t_tool_reference_position = next_state.pose.translation();
    next_target.tool_reference_twist = next_state.velocity;
    next_target.tool_reference_acceleration = next_state.acceleration;
  }
  phase_to_go_ = time_to_go_seconds / movement_duration_seconds;
  return next_target;
}

const cartesian_impedance::RealTimeCartesianTarget&
QuinticSplineCartesianImpedanceReferenceGenerator::GetReference() const {
  return reference_;
}

RealtimeStatus QuinticSplineCartesianImpedanceReferenceGenerator::Configure(
    const cartesian_impedance::QuinticSplineCartesianGeneratorParameters&
        params) {
  if (params.movement_duration_seconds < 0.0) {
    return FailedPreconditionError("Movement duration must be >= 0.0");
  }
  nominal_movement_duration_seconds_ = params.movement_duration_seconds;
  return OkStatus();
}

QuinticSplineNullspaceReferenceGenerator::
    QuinticSplineNullspaceReferenceGenerator(double dt_seconds)
    : dt_seconds_([&]() {
        CHECK(dt_seconds >=
              intrinsic::polynomial_spline_internal::kMinimumDtSeconds)
            << "dt_seconds must be >= "
            << intrinsic::polynomial_spline_internal::kMinimumDtSeconds;
        return dt_seconds;
      }()) {}

RealtimeStatus QuinticSplineNullspaceReferenceGenerator::SetReference(
    const cartesian_impedance::RealTimeNullspaceTarget& reference,
    const JointLimits& joint_limits) {
  if (nominal_movement_duration_seconds_ < 0.0) {
    return FailedPreconditionError(
        "Movement duration not set. Did you call Configure()?");
  }
  // Reset internal states:
  phase_to_go_ = 1.0;

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      bool within_limits, IsWithinLimits(reference.joint_state, joint_limits));
  if (!within_limits) {
    return FailedPreconditionError("Joint state is not within limits.");
  }

  reference_ = reference;
  return OkStatus();
}

RealtimeStatusOr<cartesian_impedance::RealTimeNullspaceTarget>
QuinticSplineNullspaceReferenceGenerator::Evaluate(
    const cartesian_impedance::RealTimeNullspaceTarget& current,
    double speed_override) {
  if (speed_override < kMinimumSpeedOverride ||
      speed_override > kMaximumSpeedOverride) {
    return InvalidArgumentError(RealtimeStatus::StrCat(
        "speed_override should be in range [", kMinimumSpeedOverride, ", ",
        kMaximumSpeedOverride, "], but instead has value ", speed_override,
        "."));
  }
  if (phase_to_go_ < 0) {
    return OutOfRangeError(
        "phase_to_go_ must be >= 0, probably missing a call to "
        "SetReference()?");
  }
  const double movement_duration_seconds =
      std::min(nominal_movement_duration_seconds_ /
                   (speed_override + kMinusculeRegularizer),
               kMaximumMovementDurationSeconds);
  double time_to_go_seconds = phase_to_go_ * movement_duration_seconds;

  cartesian_impedance::RealTimeNullspaceTarget scaled_reference = reference_;
  scaled_reference.joint_state.velocity *= speed_override;
  scaled_reference.joint_state.acceleration *=
      ::intrinsic::IPow(speed_override, 2);
  cartesian_impedance::RealTimeNullspaceTarget next_target = scaled_reference;
  if (time_to_go_seconds < dt_seconds_) {
    time_to_go_seconds = 0.0;
  } else {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        next_target.joint_state,
        GetNextStateJointQuinticSpline(current.joint_state,
                                       scaled_reference.joint_state,
                                       time_to_go_seconds, dt_seconds_));

    time_to_go_seconds -= dt_seconds_;
  }
  phase_to_go_ = time_to_go_seconds / movement_duration_seconds;
  return next_target;
}

RealtimeStatus QuinticSplineNullspaceReferenceGenerator::Configure(
    const cartesian_impedance::
        QuinticSplineNullspaceReferenceGeneratorParameters& params) {
  if (params.movement_duration_seconds < 0.0) {
    return FailedPreconditionError("Movement duration must be >= 0.0");
  }
  nominal_movement_duration_seconds_ = params.movement_duration_seconds;
  return OkStatus();
}

}  // namespace intrinsic::icon
