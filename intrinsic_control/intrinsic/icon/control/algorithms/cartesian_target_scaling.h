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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_CARTESIAN_TARGET_SCALING_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_CARTESIAN_TARGET_SCALING_H_

#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/algorithms/reference_limit_settings.h"
#include "intrinsic/kinematics/types/cart_state.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"

namespace intrinsic::icon {
enum class TrajectoryGenerationMode {
  TRAJECTORY_GENERATION_MODE_UNSPECIFIED = 0,
  POSITION = 1,
  VELOCITY = 2,
  POSITION_AND_VELOCITY = 3,
};
// Checks whether the target needs to be scaled and returns the updated target
// in scaled_target_state, and a boolean whether the target was updated.
bool ScaleTargetCommand(const TrajectoryGenerationMode& mode,
                        const CartStatePV& target_state,
                        const CartStatePVA& current_state,
                        const CartStatePVA& current_reference_state,
                        const CartStatePVA& next_reference_state,
                        const ReferenceLimitSettings& settings,
                        CartStatePV& scaled_target_state);

// Mutates pose_twist such that it abides to the velocity limits. Treats
// translational and rotational velocity separately, and will preserve direction
// of motion, i.e. all translational dofs are scaled evenly if only one of the
// dofs actually violates its limit. For the rotational part the limit is
// scalar, and we simply scale the norm of the rotational velocity if necessary.
// Returns true if the twist was modified.
bool ScalePoseTwistToLimits(const CartesianLimits& limits,
                            eigenmath::Vector6d* pose_twist);

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_CARTESIAN_TARGET_SCALING_H_
