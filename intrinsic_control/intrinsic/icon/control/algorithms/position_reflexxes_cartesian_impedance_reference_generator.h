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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_POSITION_REFLEXXES_CARTESIAN_IMPEDANCE_REFERENCE_GENERATOR_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_POSITION_REFLEXXES_CARTESIAN_IMPEDANCE_REFERENCE_GENERATOR_H_

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/algorithms/cartesian_impedance_commons.h"
#include "intrinsic/icon/control/algorithms/cartesian_impedance_reference_generator_interface.h"
#include "intrinsic/icon/control/algorithms/cartesian_position_reflexxes.h"
#include "intrinsic/icon/control/algorithms/cartesian_velocity_reflexxes.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/cart_state.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

// Uses Cartesian Position Reflexxes to interpolate the pose, twist and
// acceleration subject to user-provided Cartesian limits and speed override.
// All other quantities (wrench, Cartesian stiffness, etc.) are directly set to
// target values. WARNING: this reference generator is intended to connect
// different poses, and not to plan complex force/impedance trajectories.
class PositionReflexxesCartesianImpedanceReferenceGenerator
    : public CartesianImpedanceReferenceGeneratorInterface {
 public:
  static constexpr double kMinimumSpeedOverride = 0.0;
  static constexpr double kMaximumSpeedOverride = 1.0;

  // This threshold for the speed override value defines where the
  // reflexxes-based trajectory generator switches from a position to a velocity
  // based. Between 0 and the threshold a velocity-based controller is used and
  // between the threshold and 1 a position-based controller is used.
  static constexpr double kSwitchSpeedOverride = 0.001;

  PositionReflexxesCartesianImpedanceReferenceGenerator() = delete;

  explicit PositionReflexxesCartesianImpedanceReferenceGenerator(
      double frequency_hz);

  RealtimeStatus SetReference(
      const cartesian_impedance::RealTimeCartesianTarget& reference,
      const CartesianLimits& cartesian_limits) override;

  // Evaluates Cartesian Reflexxes, guarantees to be within user-provided and
  // speed-override-adjusted Cartesian limits. Returns a `kInvalidArgument` if
  // the specified `speed_override` is outside the range
  // [`kMinimumSpeedOverride`, `kMaximumSpeedOverride`].
  RealtimeStatusOr<cartesian_impedance::RealTimeCartesianTarget> Evaluate(
      const cartesian_impedance::RealTimeCartesianTarget& current,
      const CartStatePVA& cart_state_sensed, double speed_override) override;

  const cartesian_impedance::RealTimeCartesianTarget& GetReference()
      const override;

 private:
  CartesianPositionReflexxes cartesian_position_reflexxes_;
  CartesianVelocityReflexxes cartesian_velocity_reflexxes_;
  cartesian_impedance::RealTimeCartesianTarget reference_;

  // Variable caches which will be scaled down within the Evaluate() function
  // according to the speed_override value.
  CartesianLimits nominal_cartesian_limits_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_POSITION_REFLEXXES_CARTESIAN_IMPEDANCE_REFERENCE_GENERATOR_H_
