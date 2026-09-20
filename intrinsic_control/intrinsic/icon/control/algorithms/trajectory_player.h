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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_TRAJECTORY_PLAYER_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_TRAJECTORY_PLAYER_H_

#include <memory>

#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "intrinsic/icon/control/algorithms/speed_override_factor_dynamics.h"
#include "intrinsic/icon/control/algorithms/speed_override_factor_types.h"
#include "intrinsic/icon/dynamics/rigid_body_interface.h"
#include "intrinsic/icon/reflexxes/motion_polynomials.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/kinematics/types/joint_trajectories.h"

namespace intrinsic::icon {

// A class that takes a trajectory as an input, and produces reference states
// for real-time trajectory tracking controller, while allowing a Limit-Aware
// Online Speed Overriding.
class TrajectoryPlayer {
 public:
  static constexpr double kMinimumSpeedOverrideFactor = 0.0;
  static constexpr double kMaximumSpeedOverrideFactor = 1.0;

  // Creates a `TrajectoryPlayer` given a joint `trajectory`, `control_delta_t`,
  // and `joint_limits`.
  // Returns a `kInvalidArgument` if:
  // * the `trajectory` is empty.
  // * the dimensions of `trajectory` and `joint_limits` are mis-matched.
  // * there is a state in `trajectory` which violates the `joint_limits`.
  static absl::StatusOr<std::unique_ptr<TrajectoryPlayer>> Create(
      JointTrajectoryPVA trajectory, absl::Duration control_delta_t,
      JointLimits joint_limits,
      std::unique_ptr<RigidBodyInterface> dynamics = nullptr);

  // Get the first/initial state of the trajectory.
  RealtimeStatusOr<JointStatePVA> GetInitialState() {
    return GetStateAtPhase(0.0);
  }

  // Advances the state of the trajectory planner. This process has several
  // steps:
  // [1] advance the speed override factor state following the limit-aware
  //     time-optimal dynamics,
  // [2] compute the joint state to be returned, and
  // [3] update the phase variable.
  RealtimeStatusOr<JointStatePVA> GetNextState();

  // Sets the desired speed override factor. It may take a number of steps for
  // the actual speed override factor to converge to the desired one.
  // Returns a `kInvalidArgument` if the specified
  // `desired_speed_override_factor` is outside the range
  // [`kMinimumSpeedOverrideFactor`, `kMaximumSpeedOverrideFactor`].
  RealtimeStatus SetDesiredSpeedOverrideFactor(
      double desired_speed_override_factor);

  // Immediately sets the speed override factor to the provided value. WARNING:
  // This skips the gradual convergence that `SetDesiredSpeedOverrideFactor()`
  // includes, thus there is a risk of motion discontinuity. Use with care, for
  // example when (re-)initializing a TrajectoryPlayer in an ICON Action's
  // "OnEnter()" method.
  // Returns a `kInvalidArgument` if the specified `speed_override_factor` is
  // outside the range [`kMinimumSpeedOverrideFactor`,
  // `kMaximumSpeedOverrideFactor`].
  RealtimeStatus ForceSpeedOverrideFactor(double speed_override_factor);

  // Returns the current (actual) speed override factor, which may be
  // different from (but tracks) the user-set desired speed override factor
  // value, i.e. the one being commanded via SetDesiredSpeedOverrideFactor().
  double GetSpeedOverrideFactor() const;

  // Resets the phase to zero.
  void ResetPhase() { phase_ = 0.0; }

  double GetPhase() { return phase_; }

  // Returns the current translational Cartesian arc length progressed along the
  // trajectory. If the trajectory does not have Cartesian arc lengths, returns
  // an error.
  RealtimeStatusOr<double> GetCartesianArcLength();

  // As mentioned in "SetDesiredSpeedOverrideFactor()", there may be delay
  // between the setting of the desired speed override factor and when the
  // `TrajectoryPlayer` actually reaching that value. This function checks
  // whether the actual speed override factor in `TrajectoryPlayer` has actually
  // reached the set desired value.
  bool SpeedOverrideStateHasConverged() const;

  // Gets the state at the specified phase, assuming a steady-state speed
  // override factor.
  // Returns a `kInvalidArgument` if the `phase` is outside the range [0, 1].
  // Returns a `kInternal` if the speed override factor has not converged.
  RealtimeStatusOr<JointStatePVA>
  GetStateAtPhaseIfSpeedOverrideConvergedTestOnly(double phase);

  // Returns the control sampling time used for trajectory interpolation.
  absl::Duration GetControlSamplingTime() const { return control_delta_t_; }

  // Returns the tracked trajectory.
  const JointTrajectoryPVA& GetTrajectory() const { return trajectory_; }

  // Computes the speed override adjusted time-to-go to the target phase.
  RealtimeStatusOr<double> ComputeTimeToTargetPhase(
      double target_trajectory_phase) const;

 private:
  TrajectoryPlayer(JointTrajectoryPVA trajectory,
                   absl::Duration control_delta_t, JointLimits joint_limits,
                   SpeedOverrideFactorDynamics sof_dynamics,
                   reflexxes::MotionPolynomials initial_sof_motion_polynomials);

  // Get the state at the specified phase. The phase has to be in range [0, 1],
  // otherwise the function will return a `kInvalidArgument`.
  RealtimeStatusOr<JointStatePVA> GetStateAtPhase(double phase);

  // Get the translational Cartesian arc length at the specified `phase`. The
  // `phase` has to be in range [0, 1], otherwise the function will return a
  // `kInvalidArgument` error.
  RealtimeStatusOr<double> GetCartesianArcLengthAtPhase(double phase);

  // Class responsible for the limit-aware time-optimal speed override factor
  // dynamics that transitions from the current speed override factor state to
  // the target speed override factor state.
  SpeedOverrideFactorDynamics sof_dynamics_;

  const JointTrajectoryPVA trajectory_;
  const absl::Duration control_delta_t_;  // 1.0 / controller_frequency
  const JointLimits joint_limits_;

  // The phase of the trajectory, whose value is in the range [0, 1].
  double phase_ = 0.0;
  double next_phase_ = 0.0;

  // The user-commanded desired value of the speed override factor.
  double target_sof_ = 1.0;

  // The speed override factor state varies with time (has dynamics). `sof`
  // tracks the `target_sof_`, while `dsof_dt` and `d2sof_dt2` both track 0.
  SpeedOverrideFactorStateWithSecondDerivative curr_state_ = {
      .sof = 1.0, .dsof_dt = 0.0, .d2sof_dt2 = 0.0};

  // If `curr_state_` contains the speed override factor at the (discretized)
  // time index i, then `next_state_` contains the speed override factor at the
  // (discretized) time index (i + 1).
  SpeedOverrideFactorStateWithSecondDerivative next_state_ = {
      .sof = 1.0, .dsof_dt = 0.0, .d2sof_dt2 = 0.0};

  // The variables required to construct the time-to-go to a future event. We
  // need to keep track of the `motion_polynomial_phase_` which represents the
  // phase for which the motion polynomials were constructed. It corresponds to
  // one timestep before the `phase_` variable that stores the next reachable
  // phase. We also need to keep track of the `speed_override_trajectory_` that
  // describe the continuous evolution of the speed override factor state and
  // will allow us to make a prediction about an event.
  double motion_polynomial_phase_ = 0.0;
  reflexxes::MotionPolynomials speed_override_trajectory_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_TRAJECTORY_PLAYER_H_
