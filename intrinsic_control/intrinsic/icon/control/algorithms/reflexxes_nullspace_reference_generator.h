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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_REFLEXXES_NULLSPACE_REFERENCE_GENERATOR_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_REFLEXXES_NULLSPACE_REFERENCE_GENERATOR_H_

#include "intrinsic/icon/control/algorithms/cartesian_impedance_commons.h"
#include "intrinsic/icon/control/algorithms/cartesian_impedance_reference_generator_interface.h"
#include "intrinsic/icon/control/algorithms/joint_position_reflexxes.h"
#include "intrinsic/icon/control/algorithms/joint_velocity_reflexxes.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/joint_limits.h"

namespace intrinsic::icon {

// Uses Joint Position Reflexxes to interpolate the joint position, velocity and
// acceleration subject to user-provided joint limits and speed override. All
// other quantities (nullspace stiffness and damping) are directly set to target
// values.
class ReflexxesNullspaceReferenceGenerator
    : public NullspaceReferenceGeneratorInterface {
 public:
  static constexpr double kMinimumSpeedOverride = 0.0;
  static constexpr double kMaximumSpeedOverride = 1.0;

  // This threshold for the speed override value defines where the
  // reflexxes-based trajectory generator switches from a position to a velocity
  // based. Between 0 and the threshold a velocity-based controller is used and
  // between the threshold and 1 a position-based controller is used.
  static constexpr double kSwitchSpeedOverride = 0.001;

  ReflexxesNullspaceReferenceGenerator() = delete;

  ReflexxesNullspaceReferenceGenerator(int njoints, double frequency_hz);

  RealtimeStatus SetReference(
      const cartesian_impedance::RealTimeNullspaceTarget& reference,
      const JointLimits& joint_limits) override;

  // Evaluates Reflexxes, guarantees to respect user-provided and
  // speed-override-adjusted limits. Returns a `kInvalidArgument` if the
  // specified `speed_override` is outside the range [`kMinimumSpeedOverride`,
  // `kMaximumSpeedOverride`].
  RealtimeStatusOr<cartesian_impedance::RealTimeNullspaceTarget> Evaluate(
      const cartesian_impedance::RealTimeNullspaceTarget& current,
      double speed_override) override;

 private:
  JointPositionReflexxes joint_position_reflexxes_;
  JointVelocityReflexxes joint_velocity_reflexxes_;

  // Variable caches which will be scaled down within the Evaluate() function
  // according to the speed_override value.
  cartesian_impedance::RealTimeNullspaceTarget nominal_reference_;
  JointLimits nominal_joint_limits_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_REFLEXXES_NULLSPACE_REFERENCE_GENERATOR_H_
