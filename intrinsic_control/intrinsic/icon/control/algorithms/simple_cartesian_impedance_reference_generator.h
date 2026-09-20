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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_SIMPLE_CARTESIAN_IMPEDANCE_REFERENCE_GENERATOR_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_SIMPLE_CARTESIAN_IMPEDANCE_REFERENCE_GENERATOR_H_

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

// Implements an impedance reference generator acting as discrete-time PT1
// filter element. Employs the discrete-time update equation
//
// x_{n+1} = x_n + speed_override * alpha * (reference - x_n)
//
// with current state `x_n`, a `reference` target, (scalar) filter constant
// alpha, and the `speed_override` value.
class SimpleCartesianImpedanceReferenceGenerator
    : public CartesianImpedanceReferenceGeneratorInterface {
 public:
  static constexpr double kDefaultAlpha = 5e-3;
  static constexpr double kMinimumSpeedOverride = 0.0;
  static constexpr double kMaximumSpeedOverride = 1.0;

  explicit SimpleCartesianImpedanceReferenceGenerator(
      double alpha = kDefaultAlpha);

  RealtimeStatus SetReference(
      const cartesian_impedance::RealTimeCartesianTarget& reference,
      const CartesianLimits& cartesian_limits) override;

  // Evaluates the reference generator, taking into account the `speed_override`
  // value. Note that Evaluate() does not check for limit violations.
  // Returns a `kInvalidArgument` if the specified `speed_override` is outside
  // the range [`kMinimumSpeedOverride`, `kMaximumSpeedOverride`].
  RealtimeStatusOr<cartesian_impedance::RealTimeCartesianTarget> Evaluate(
      const cartesian_impedance::RealTimeCartesianTarget& current,
      const CartStatePVA& cart_state_sensed, double speed_override) override;

  const cartesian_impedance::RealTimeCartesianTarget& GetReference()
      const override;

  RealtimeStatus Configure(
      const cartesian_impedance::SimpleImpedanceGeneratorParameters& params);

  double alpha() const { return alpha_; }

 private:
  cartesian_impedance::RealTimeCartesianTarget nominal_reference_;
  double alpha_ = kDefaultAlpha;
};

// Implements a nullspace joint reference generator acting as discrete-time PT1
// filter element. Employs the discrete-time update equation
//
// x_{n+1} = x_n + speed_override * alpha * (reference - x_n)
//
// with current state `x_n`, a `reference` target, (scalar) filter constant
// alpha, and the `speed_override` value.
class SimpleNullspaceReferenceGenerator
    : public NullspaceReferenceGeneratorInterface {
 public:
  static constexpr double kDefaultAlpha = 5e-3;
  static constexpr double kMinimumSpeedOverride = 0.0;
  static constexpr double kMaximumSpeedOverride = 1.0;

  explicit SimpleNullspaceReferenceGenerator(double alpha = kDefaultAlpha);

  RealtimeStatus SetReference(
      const cartesian_impedance::RealTimeNullspaceTarget& reference,
      const JointLimits& joint_limits) override;

  // Evaluates the reference generator, taking into account the `speed_override`
  // value. Note that Evaluate() does not check for limit violations.
  // Returns a `kInvalidArgument` if the specified `speed_override` is outside
  // the range [`kMinimumSpeedOverride`, `kMaximumSpeedOverride`].
  RealtimeStatusOr<cartesian_impedance::RealTimeNullspaceTarget> Evaluate(
      const cartesian_impedance::RealTimeNullspaceTarget& current,
      double speed_override) override;

  RealtimeStatus Configure(
      const cartesian_impedance::SimpleNullspaceReferenceGeneratorParameters&
          params);

  double alpha() const { return alpha_; }

 private:
  cartesian_impedance::RealTimeNullspaceTarget nominal_reference_;
  double alpha_ = kDefaultAlpha;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_SIMPLE_CARTESIAN_IMPEDANCE_REFERENCE_GENERATOR_H_
