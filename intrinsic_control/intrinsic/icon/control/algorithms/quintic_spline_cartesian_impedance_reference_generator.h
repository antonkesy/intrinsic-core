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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_QUINTIC_SPLINE_CARTESIAN_IMPEDANCE_REFERENCE_GENERATOR_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_QUINTIC_SPLINE_CARTESIAN_IMPEDANCE_REFERENCE_GENERATOR_H_

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/actions/cartesian_impedance_parameters.h"
#include "intrinsic/icon/control/algorithms/cartesian_impedance_commons.h"
#include "intrinsic/icon/control/algorithms/cartesian_impedance_reference_generator_interface.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/cart_state.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

// Uses Quintic Spline to interpolate the pose, twist and acceleration, while
// considering the `speed_override` value. All other quantities (wrench,
// Cartesian stiffness, etc.) are directly set to target values. WARNING: this
// reference generator is intended to connect different poses, and not to plan
// complex force/impedance trajectories.
class QuinticSplineCartesianImpedanceReferenceGenerator
    : public CartesianImpedanceReferenceGeneratorInterface {
 public:
  static constexpr double kMinimumSpeedOverride = 0.0;
  static constexpr double kMaximumSpeedOverride = 1.0;

  QuinticSplineCartesianImpedanceReferenceGenerator() = delete;

  // `dt_seconds` is the real-time control period, to be set as
  // `dt_seconds` = 1.0 / `freq_hz`, with `freq_hz` is
  // the real-time control frequency in Hertz (Hz).
  explicit QuinticSplineCartesianImpedanceReferenceGenerator(double dt_seconds);

  // Reset internal state (if any) and sets a `reference` as well as
  // `cartesian_limits`. `reference` will be checked against `cartesian_limits`
  // to make sure that there is no violation to these specified limits, and if
  // there is a violation the method will return an error.
  RealtimeStatus SetReference(
      const cartesian_impedance::RealTimeCartesianTarget& reference,
      const CartesianLimits& cartesian_limits) override;

  // Steps the reference generator using `current` and interpolates towards
  // the reference using a quintic spline, taking into account the
  // `speed_override` value. Note that this Evaluate() method does not check for
  // limit violations. Fails when stepping the quintic spline returns an error.
  // Returns a `kInvalidArgument` if the specified `speed_override` is outside
  // the range [`kMinimumSpeedOverride`, `kMaximumSpeedOverride`].
  RealtimeStatusOr<cartesian_impedance::RealTimeCartesianTarget> Evaluate(
      const cartesian_impedance::RealTimeCartesianTarget& current,
      const CartStatePVA& cart_state_sensed, double speed_override) override;

  const cartesian_impedance::RealTimeCartesianTarget& GetReference()
      const override;

  RealtimeStatus Configure(
      const cartesian_impedance::QuinticSplineCartesianGeneratorParameters&
          params);

  double nominal_movement_duration_seconds() const {
    return nominal_movement_duration_seconds_;
  }

 private:
  // the goal/reference state
  cartesian_impedance::RealTimeCartesianTarget reference_;

  // `nominal_movement_duration_seconds_` is the time length in seconds, to
  // complete the movement starting from the initial/start state to the
  // goal/reference state when `speed_override` value is 1.0. Initialized to an
  // invalid negative value to enforce failure in case of a missing call to
  // Configure().
  double nominal_movement_duration_seconds_ = -1.0;

  // `dt_seconds_` is the real-time control period, to be set as
  // `dt_seconds_` = 1.0 / `freq_hz`, with `freq_hz` is
  // the real-time control frequency in Hertz (Hz).
  double dt_seconds_;

  // Before the movement starts, `phase_to_go_` is set equal to 1.0. At each
  // call to Evaluate() method, `phase_to_go_` is decreased by considering
  // `dt_seconds_`, `nominal_movement_duration_seconds_`, and the
  // `speed_override` value (`phase_to_go_` reaches zero, at which time the
  // generator should arrive at the `reference_`). Initialized to an invalid
  // negative value to enforce failure in case of a missing call to
  // SetReference().
  double phase_to_go_ = -1.0;
};

// Uses Quintic Spline to interpolate the joint position, velocity and
// acceleration, while considering the `speed_override` value. All other
// quantities (nullspace stiffness and damping) are directly set to target
// values.
class QuinticSplineNullspaceReferenceGenerator
    : public NullspaceReferenceGeneratorInterface {
 public:
  static constexpr double kMinimumSpeedOverride = 0.0;
  static constexpr double kMaximumSpeedOverride = 1.0;

  QuinticSplineNullspaceReferenceGenerator() = delete;

  // `dt_seconds` is the real-time control period, to be set as `dt_seconds`
  // = 1.0 / `freq_hz`, with `freq_hz` is the real-time control frequency in
  // Hertz (Hz).
  explicit QuinticSplineNullspaceReferenceGenerator(double dt_seconds);

  // Reset internal state (if any) and sets a `reference` as well as
  // `joint_limits`. `reference` will be checked against `joint_limits`
  // to make sure that there is no violation to these specified limits, and if
  // there is a violation the method will return an error.
  RealtimeStatus SetReference(
      const cartesian_impedance::RealTimeNullspaceTarget& reference,
      const JointLimits& joint_limits) override;

  // Steps the reference generator using `current` and interpolates towards
  // the reference using a quintic spline, taking into account the
  // `speed_override` value. Note that this Evaluate() method
  // does not check for limit violations. Fails when stepping
  // the quintic spline returns an error. Returns a `kInvalidArgument` if the
  // specified `speed_override` is outside the range [`kMinimumSpeedOverride`,
  // `kMaximumSpeedOverride`].
  RealtimeStatusOr<cartesian_impedance::RealTimeNullspaceTarget> Evaluate(
      const cartesian_impedance::RealTimeNullspaceTarget& current,
      double speed_override) override;

  RealtimeStatus Configure(
      const cartesian_impedance::
          QuinticSplineNullspaceReferenceGeneratorParameters& params);

  double nominal_movement_duration_seconds() const {
    return nominal_movement_duration_seconds_;
  }

 private:
  // the goal/reference state
  cartesian_impedance::RealTimeNullspaceTarget reference_;

  // `nominal_movement_duration_seconds_` is the time length in seconds, to
  // complete the movement starting from the initial/start state to the
  // goal/reference state when `speed_override` value is 1.0. Initialized to an
  // invalid negative value to enforce failure in case of a missing call to
  // Configure().
  double nominal_movement_duration_seconds_ = -1.0;

  // `dt_seconds_` is the real-time control period, to be set as
  // `dt_seconds_` = 1.0 / `freq_hz`, with `freq_hz` is
  // the real-time control frequency in Hertz (Hz).
  double dt_seconds_;

  // Before the movement starts, `phase_to_go_` is set equal to 1.0. At each
  // call to Evaluate() method, `phase_to_go_` is decreased by considering
  // `dt_seconds_`, `nominal_movement_duration_seconds_`, and the
  // `speed_override` value (`phase_to_go_` reaches zero, at which time the
  // generator should arrive at the `reference_`). Initialized to an invalid
  // negative value to enforce failure in case of a missing call to
  // SetReference().
  double phase_to_go_ = -1.0;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_QUINTIC_SPLINE_CARTESIAN_IMPEDANCE_REFERENCE_GENERATOR_H_
