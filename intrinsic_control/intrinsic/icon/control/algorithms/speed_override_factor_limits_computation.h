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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_SPEED_OVERRIDE_FACTOR_LIMITS_COMPUTATION_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_SPEED_OVERRIDE_FACTOR_LIMITS_COMPUTATION_H_

#include <memory>

#include "absl/status/statusor.h"
#include "intrinsic/icon/control/algorithms/speed_override_factor_types.h"
#include "intrinsic/icon/dynamics/rigid_body_interface.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_trajectories.h"

namespace intrinsic::icon {

class SpeedOverrideFactorLimitsComputation {
 public:
  // Validates the input and creates an object.
  static absl::StatusOr<SpeedOverrideFactorLimitsComputation> Create(
      const JointTrajectoryPVA& trajectory, const JointLimits& joint_limits,
      std::unique_ptr<RigidBodyInterface> dynamics = nullptr);

  // Computes the limits of the speed override factor (sof) and its time
  // derivatives. They depend on two components:
  // 1) the trajectory derivatives at the given `phase` in [0.0, 1.0], and
  // 2) the `speed_override_factor_state` which defines the current state of the
  // speed override factor transition.
  //
  // If `clamp_sof_to_zero_one_range` is true, the limits for the speed override
  // factor zero-th derivative are clamped to the range [0.0, 1.0].
  // This implies that the limits do not allow making progress on the trajectory
  // faster than the original trajectory speed, they allow to stop, but not to
  // go backwards. If the previous conditions want to be relaxed, then the flag
  // can be set to false.
  RealtimeStatusOr<SpeedOverrideFactorLimits>
  ComputeAllSpeedOverrideFactorLimits(
      double phase,
      const SpeedOverrideFactorStateWithDerivative& speed_override_factor_state,
      bool clamp_sof_to_zero_one_range) const;

  // Getter for the trajectory.
  const JointTrajectoryPVA& GetTrajectory() const { return trajectory_; }

  // Getter for the joint limits.
  const JointLimits& GetJointLimits() const { return joint_limits_; }

 private:
  SpeedOverrideFactorLimitsComputation(
      const JointTrajectoryPVA& trajectory, const JointLimits& joint_limits,
      std::unique_ptr<RigidBodyInterface> dynamics);

  const JointTrajectoryPVA trajectory_;
  const JointLimits joint_limits_;
  std::unique_ptr<RigidBodyInterface> dynamics_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_SPEED_OVERRIDE_FACTOR_LIMITS_COMPUTATION_H_
