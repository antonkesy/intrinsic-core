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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_TRAJECTORY_RESIDUAL_CONTROLLER_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_TRAJECTORY_RESIDUAL_CONTROLLER_H_

#include <memory>

#include "absl/status/statusor.h"
#include "intrinsic/icon/control/algorithms/joint_position_reflexxes.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"

namespace intrinsic::icon {

// Residual controller which drives a given JointStatePVA to zero. Can be used
// to correct a given JointStatePVA trajectory for deviating initial states by
// recursively computing offsets to the reference trajectory. Exploits the
// margin between `system_limits` and `planning_limits` limits to compute this
// offset. Since summation/accumulation is a linear operation in discrete time,
// this residual can be simply added to the reference trajectory.
//
// The combination of the residual plus the reference trajectory is guaranteed
// to satisfy `system_limits'` velocity, acceleration and jerk, if the original
// trajectory has been planned with `planning_limits` limits. Position limit
// satisfaction cannot be guaranteed.
class TrajectoryResidualController {
 public:
  TrajectoryResidualController() = delete;

  // Creates a `TrajectoryResidualController` where `control_frequency_hz` is
  // the sampling time of the digital control loop in which the
  // TrajectoryResidualController is called. `planning_limits` are the limits at
  // which the trajectory has been planned, and `system_limits` are the maximum
  // feedback limits of the control system. Returns kInvalidArgument in case the
  // planning_limits exceed the system_limits.
  static absl::StatusOr<std::unique_ptr<TrajectoryResidualController>> Create(
      double control_frequency_hz, const JointLimits& planning_limits,
      const JointLimits& system_limits);

  // Compute state trajectory residual from a `previous_residual` using the
  // margin between control and planning limits. The residual will be zero after
  // convergence.
  icon::RealtimeStatusOr<JointStatePVA> ComputeResidualUsingLimitMargin(
      const JointStatePVA& previous_residual);

  // Compute state trajectory residual from a `previous_residual` using the
  // planning limits only. The residual will be zero after convergence.
  icon::RealtimeStatusOr<JointStatePVA> ComputeResidualUsingPlanningLimits(
      const JointStatePVA& previous_residual);

 private:
  TrajectoryResidualController(double control_frequency_hz,
                               const JointLimits& planning_limits,
                               const JointLimits& system_limits);

  JointLimits planning_limits_;
  // Difference between planning and control limits.
  JointLimits limits_margin_;
  JointPositionReflexxes reflexxes_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_TRAJECTORY_RESIDUAL_CONTROLLER_H_
