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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_CARTESIAN_IMPEDANCE_REFERENCE_GENERATOR_INTERFACE_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_CARTESIAN_IMPEDANCE_REFERENCE_GENERATOR_INTERFACE_H_

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/algorithms/cartesian_impedance_commons.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/cart_state.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"

namespace intrinsic::icon {

// A common interface class for Cartesian impedance reference generators.
class CartesianImpedanceReferenceGeneratorInterface {
 public:
  virtual ~CartesianImpedanceReferenceGeneratorInterface() = default;

  // Steps the Cartesian impedance reference generator using `current` and
  // `cart_state_sensed` and interpolates towards the reference with parameter
  // `speed_override`. `speed_override` has the valid value range of [0, 1],
  // with `speed_override` == 1 means execution at the nominal speed,
  // `speed_override` == 0.25 means execution at 0.25 x nominal speed, and
  // `speed_override` == 0 means execution at 0 x nominal speed, i.e. staying
  // still. It is assumed that no explicit notion of time is required,
  // implementation details are up to the derived classes.
  virtual RealtimeStatusOr<cartesian_impedance::RealTimeCartesianTarget>
  Evaluate(const cartesian_impedance::RealTimeCartesianTarget& current,
           const CartStatePVA& cart_state_sensed, double speed_override) = 0;

  // Reset internal state (if any) and sets a `reference` as well as
  // new `cartesian_limits`.
  virtual RealtimeStatus SetReference(
      const cartesian_impedance::RealTimeCartesianTarget& reference,
      const CartesianLimits& cartesian_limits) = 0;

  virtual const cartesian_impedance::RealTimeCartesianTarget& GetReference()
      const = 0;
};

// A common interface class for nullspace control task reference generators.
class NullspaceReferenceGeneratorInterface {
 public:
  virtual ~NullspaceReferenceGeneratorInterface() = default;

  // Steps the nullspace impedance reference generator using `current` with
  // parameter `speed_override`. `speed_override` has the valid value range of
  // [0, 1], with `speed_override` == 1 means execution at the nominal speed,
  // `speed_override` == 0.25 means execution at 0.25 x nominal speed, and
  // `speed_override` == 0 means execution at 0 x nominal speed, i.e. staying
  // still.
  virtual RealtimeStatusOr<cartesian_impedance::RealTimeNullspaceTarget>
  Evaluate(const cartesian_impedance::RealTimeNullspaceTarget& current,
           double speed_override) = 0;

  // Reset internal state (if any) and sets a new nullspace `reference`
  // as well as new `joint_limits`.
  virtual RealtimeStatus SetReference(
      const cartesian_impedance::RealTimeNullspaceTarget& reference,
      const JointLimits& joint_limits) = 0;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_CARTESIAN_IMPEDANCE_REFERENCE_GENERATOR_INTERFACE_H_
