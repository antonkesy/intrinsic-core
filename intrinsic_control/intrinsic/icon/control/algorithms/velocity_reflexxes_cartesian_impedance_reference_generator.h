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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_VELOCITY_REFLEXXES_CARTESIAN_IMPEDANCE_REFERENCE_GENERATOR_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_VELOCITY_REFLEXXES_CARTESIAN_IMPEDANCE_REFERENCE_GENERATOR_H_

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/algorithms/cartesian_impedance_commons.h"
#include "intrinsic/icon/control/algorithms/cartesian_impedance_reference_generator_interface.h"
#include "intrinsic/icon/control/algorithms/cartesian_velocity_reflexxes.h"
#include "intrinsic/icon/control/algorithms/reference_limit_settings.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/cart_state.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

// Uses Cartesian Velocity Reflexxes to interpolate the pose, twist and
// acceleration, based on the current target twist only, subject to
// user-provided speed override.
//
// The generated Cartesian trajectory obeys the user-provided Cartesian limits.
// All other quantities (wrench, Cartesian stiffness, etc.) are directly set to
// target values.
class VelocityReflexxesCartesianImpedanceReferenceGenerator
    : public CartesianImpedanceReferenceGeneratorInterface {
 public:
  static constexpr double kMinimumSpeedOverride = 0.0;
  static constexpr double kMaximumSpeedOverride = 1.0;

  // Minimum value of speed override value allowed by reflexxes
  static constexpr double kMinimumSpeedOverrideClamping = 0.001;

  VelocityReflexxesCartesianImpedanceReferenceGenerator() = delete;

  explicit VelocityReflexxesCartesianImpedanceReferenceGenerator(
      double frequency_hz);

  RealtimeStatus SetReference(
      const cartesian_impedance::RealTimeCartesianTarget& reference,
      const CartesianLimits& cartesian_limits) override;

  // Evaluates Cartesian Velocity Reflexxes, guarantees to be within the
  // user-provided and speed-override-adjusted Cartesian limits. Uses difference
  // between `cart_state_sensed` and reference pose to scale the reference
  // velocity to avoid divergence between current and reference state. Returns a
  // `kInvalidArgument` if the specified `speed_override` is outside the range
  // [`kMinimumSpeedOverride`, `kMaximumSpeedOverride`].
  RealtimeStatusOr<cartesian_impedance::RealTimeCartesianTarget> Evaluate(
      const cartesian_impedance::RealTimeCartesianTarget& current,
      const CartStatePVA& cart_state_sensed, double speed_override) override;

  const cartesian_impedance::RealTimeCartesianTarget& GetReference()
      const override;

 private:
  CartesianVelocityReflexxes cartesian_velocity_reflexxes_;

  cartesian_impedance::RealTimeCartesianTarget nominal_reference_;
  CartesianLimits nominal_cartesian_limits_;
  icon::ReferenceLimitSettings reference_limit_settings_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_VELOCITY_REFLEXXES_CARTESIAN_IMPEDANCE_REFERENCE_GENERATOR_H_
