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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_SPEED_OVERRIDE_FACTOR_DYNAMICS_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_SPEED_OVERRIDE_FACTOR_DYNAMICS_H_

#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/algorithms/speed_override_factor_dynamics_utils.h"
#include "intrinsic/icon/control/algorithms/speed_override_factor_limits_computation.h"
#include "intrinsic/icon/control/algorithms/speed_override_factor_reflexxes_utils.h"
#include "intrinsic/icon/control/algorithms/speed_override_factor_types.h"
#include "intrinsic/icon/dynamics/rigid_body_interface.h"
#include "intrinsic/icon/reflexxes/motion_polynomials.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/kinematics/types/joint_trajectories.h"

namespace intrinsic::icon {

// Parameters for the `SpeedOverrideFactorDynamics` class.
struct SpeedOverrideFactorDynamicsParams {
  // The control sampling time.
  double control_dt_seconds;

  // The SpeedOverrideFactorDynamics class looks `stop_check_lookahead_cycles`
  // into the future for any given state, to see if it's possible to reach a
  // full stop (i.e. `dsof_dt` and `d2sof_dt2` reach zero up to
  // `reached_target_velocity_tolerance`) until then. A state is deemed
  // controllable if it is possible to reach a full stop in
  // `stop_check_lookahead_cycles`.
  int stop_check_lookahead_cycles = 5;
  double reached_target_velocity_tolerance = 1.0e-8;

  // Scaling factors are unique, in descending order values in the range
  // (0.0, 1.0] excluding zero. The scaling factors are applied in order, and
  // the first one that yields a transition from a current state to a next
  // state (such that it is possible to reach a full stop in
  // `stop_check_lookahead_cycles` from the next state) is used.
  std::vector<double> control_scaling_factors = {1.0, 0.5, 0.25, 0.125};

  // If true, the backward differentiation of the position is used to check
  // whether the next state is within joint acceleration and jerk limits. This
  // happens in addition to the continuous limit checking of the interpolated
  // trajectory.
  bool enable_backward_differentiation_limit_checking = false;
};

// Implements a Reflexxes-based algorithm that allows to transition the speed
// override factor from its current state to a target state subject to
// time-varying limits. The Dynamics part of the name refers to the fact that
// the speed override factor limits are time-varying and the state is a
// second-order system.
class SpeedOverrideFactorDynamics {
 public:
  // Creates a `SpeedOverrideFactorDynamics` instance with the speed override
  // factor dynamics parameters `params` and the values for `trajectory`,
  // `joint_limits`, and `dynamics` which are used to construct the speed
  // override factor limits. A nullptr `dynamics` is only valid for
  // acceleration-limited trajectories. A torque-limited trajectory requires a
  // non-null `dynamics`. Returns a `kInvalidArgument` error if any of the
  // parameters is invalid.
  static absl::StatusOr<SpeedOverrideFactorDynamics> Create(
      const SpeedOverrideFactorDynamicsParams& params,
      const JointTrajectoryPVA& trajectory, const JointLimits& joint_limits,
      std::unique_ptr<RigidBodyInterface> dynamics = nullptr);

  // Returns the next speed override factor state and next trajectory phase
  // given the current speed override factor state `start_state`, the target
  // speed override factor state `target_state` and the current value of the
  // `trajectory_phase`, which is used to update the limits.
  RealtimeStatusOr<SpeedOverrideFactorDynamicsResult> GetNextState(
      const SpeedOverrideFactorStateWithSecondDerivative& start_state,
      const SpeedOverrideFactorStateWithDerivative& target_state,
      double trajectory_phase);

  // Updates the previous state with the commanded position. Calling this method
  // to perform this update is only required when limit checking based on
  // backward differences of the position is enabled.
  RealtimeStatus UpdatePreviousState(const eigenmath::VectorNd& next_position);

 private:
  SpeedOverrideFactorDynamics(
      bool check_jerk_limits, double trajectory_duration_seconds,
      SpeedOverrideFactorDynamicsParams params,
      JointLimits joint_limits_with_jerk_limits_tolerance,
      PathToPositionTarget path_to_position_target,
      PathToVelocityTarget path_to_velocity_target,
      SpeedOverrideFactorLimitsComputation limits_computation,
      JointStatePVAJ previous_state);

  // Whether the trajectory is jerk-limited and thus jerk-limits should be
  // checked or not.
  bool check_jerk_limits_;

  // The duration of the trajectory whose playback speed is controlled. This is
  // used to compute the next trajectory phase.
  const double trajectory_duration_seconds_;

  const SpeedOverrideFactorDynamicsParams params_;

  // The joint limits with jerk limits tolerance are used for checking
  // the satisfaction of joint acceleration and jerk limits of the next states,
  // either based on the continuous limit checking of the interpolated
  // trajectory or the backward differentiation of the position.
  const JointLimits joint_limits_with_jerk_limits_tolerance_;

  // Reflexxes-based one-dimensional limit-aware position-trajectory-generator
  // that plans a path from the current to the target speed override factor.
  PathToPositionTarget path_to_position_target_;

  // Reflexxes-based one-dimensional limit-aware velocity-trajectory-generator
  // that plans a path from the current first derivative of the speed override
  // factor with respect to time `dsof_dt` to zero. This is used to check
  // whether a a state is controllable (`dsof_dt` and `d2sof_dt2` values can
  // reach zero within limits in `stop_check_lookahead_cycles`).
  PathToVelocityTarget path_to_velocity_target_;

  // Computes the speed override factor limits (for `sof`, `dsof_dt` and
  // `d2sof_dt2`) for a given trajectory phase and speed override factor state.
  SpeedOverrideFactorLimitsComputation limits_computation_;

  // Memory of the previous commanded joint state. This is used for checking the
  // satisfaction of joint acceleration and jerk limits of the next command
  // based on backward differentiation with respect to time. Updating this state
  // is only required when limit checking based on backward differences of the
  // position is enabled. Its initial value is the first trajectory state.
  JointStatePVAJ previous_state_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_SPEED_OVERRIDE_FACTOR_DYNAMICS_H_
