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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_SPEED_OVERRIDE_FACTOR_REFLEXXES_UTILS_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_SPEED_OVERRIDE_FACTOR_REFLEXXES_UTILS_H_

#include <optional>

#include "absl/status/statusor.h"
#include "intrinsic/icon/control/algorithms/speed_override_factor_types.h"
#include "intrinsic/icon/reflexxes/motion_polynomials.h"
#include "intrinsic/icon/reflexxes/position_flags.h"
#include "intrinsic/icon/reflexxes/position_inputs.h"
#include "intrinsic/icon/reflexxes/position_outputs.h"
#include "intrinsic/icon/reflexxes/reflexxes_api.h"
#include "intrinsic/icon/reflexxes/velocity_flags.h"
#include "intrinsic/icon/reflexxes/velocity_inputs.h"
#include "intrinsic/icon/reflexxes/velocity_outputs.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

// Maximum absolute value for all speed override factor values (states, limits).
constexpr double kSpeedOverrideFactorReflexxesInfinity = 1.0e14;

// Maximum value for the time-to-go computation.
constexpr double kSpeedOverrideAdjustedMaximumTimeToGo = 1.0e14;

// Integrates the position in the given `motion_polynomials` from 0.0 to
// `time_horizon_seconds`. Returns the integral of the position polynomial or an
// error if the integration fails.
RealtimeStatusOr<double> IntegratePositionInMotionPolynomials(
    const reflexxes::MotionPolynomials& motion_polynomials,
    double time_horizon_seconds);

// Checks whether a steady-state `target_velocity` is reached by the given
// `motion_polynomials` within a time horizon of `evaluation_time_sec`. Returns
// true if the target velocity is reached up to the desired threshold and the
// acceleration is zero at `evaluation_time_sec`.
RealtimeStatusOr<double> ReachesTargetVelocity(
    const reflexxes::MotionPolynomials& motion_polynomials,
    double target_velocity, double reached_velocity_threshold,
    double evaluation_time_sec);

// Reflexxes is used to compute a trajectory that goes from an initial speed
// override factor to a final speed override factor. The result are motion
// polynomial segments that represent a trajectory for this transition subject
// to constraints. For that reason we use the position field of a motion
// polynomial segment to represent the speed override factor. This function
// creates a motion polynomial with one segment with a constant `speed_override`
// factor that is valid from time 0.0 to
// `kSpeedOverrideAdjustedMaximumTimeToGo`.
reflexxes::MotionPolynomials CreateMotionPolynomialsWithConstantSpeedOverride(
    double speed_override);

// Returns the time to go from the `current_trajectory_phase` to the
// `target_trajectory_phase`, given the `speed_override_trajectory` of
// the trajectory that express how the derivative of the trajectory phase
// evolves over time. The `trajectory_duration_seconds` is a normalization
// constant used to relate time and phase domain. A normalized phase [0, 1]
// corresponds to the time domain [0, `trajectory_duration_seconds`].
// Returns an error if the `target_trajectory_phase` is lower than the
// `current_trajectory_phase`, as it implies that the target is in the past and
// the time to go would be negative. This information however cannot be computed
// as the motion polynomials only predict the future evolution of the phase.
RealtimeStatusOr<double> GetSpeedOverrideAdjustedTimeToGo(
    double current_trajectory_phase, double target_trajectory_phase,
    const reflexxes::MotionPolynomials& speed_override_trajectory,
    double trajectory_duration_seconds);

// A wrapper around the next state in a path to a target state. The next state
// is optional because a path that satisfies the limits may not exist, in which
// case the next state is not set.
struct PathToTargetState {
  struct NextState {
    // The immediate state that can be reached within one control cycle.
    SpeedOverrideFactorStateWithSecondDerivative state;
    // The motion polynomials that describe the path to the target state.
    reflexxes::MotionPolynomials motion_polynomials;
  };

  bool HasNextState() const { return next_state.has_value(); }
  std::optional<NextState> next_state = std::nullopt;
};

// Computes a path from a start state to a target state subject to the given
// limits using Reflexxes position-based Online Trajectory Generation.
class PathToPositionTarget {
 public:
  static absl::StatusOr<PathToPositionTarget> Create(double control_dt_seconds);

  // Computes a path from `start_state` to `target_state` subject to the given
  // `limits`. Note that the inputs (`start_state`, `target_state`, `limits`)
  // may vary from cycle to cycle. Returns the state in the path reachable at
  // the next control cycle. `check_path_limits_are_satisfied` allows to
  // enable/disable checking the satisfaction of path `limits`:
  // - If it is false, returns the next state, even if the path violates the
  //   `limits`.
  // - If it is true, returns a PathToTargetState without next state if the path
  //   violates the `limits`. Otherwise, returns the next state.
  // The flag is used to early discard paths that violate the limits when set to
  // true. But it also allows to always compute a best-effort command when set
  // to false.
  PathToTargetState GetNextState(
      const SpeedOverrideFactorStateWithSecondDerivative& start_state,
      const SpeedOverrideFactorStateWithDerivative& target_state,
      const SpeedOverrideFactorLimits& limits,
      bool check_path_limits_are_satisfied);

 private:
  explicit PathToPositionTarget(double control_dt_seconds);

  reflexxes::State state_;
  reflexxes::PositionFlags flags_;
  reflexxes::PositionInputs inputs_;
  reflexxes::PositionOutputs outputs_;
};

// Computes a path from a start state to a target state subject to the given
// limits using Reflexxes velocity-based Online Trajectory Generation.
class PathToVelocityTarget {
 public:
  static absl::StatusOr<PathToVelocityTarget> Create(double control_dt_seconds);

  // Computes a path from `start_state` to `target_state` subject to the given
  // `limits`. Note that the inputs (`start_state`, `target_state`, `limits`)
  // may vary from cycle to cycle. Returns the state in the path reachable at
  // the next control cycle. `check_path_limits_are_satisfied` allows to
  // enable/disable checking the satisfaction of path `limits`:
  // - If it is false, returns the next state, even if the path violates the
  //   `limits`.
  // - If it is true, returns a PathToTargetState without next state if the path
  //   violates the `limits`. Otherwise, returns the next state.
  // The flag is used to early discard paths that violate the limits when set to
  // true. But it also allows to always compute a best-effort command when set
  // to false.
  PathToTargetState GetNextState(
      const SpeedOverrideFactorStateWithSecondDerivative& start_state,
      const SpeedOverrideFactorStateWithDerivative& target_state,
      const SpeedOverrideFactorLimits& limits,
      bool check_path_limits_are_satisfied);

 private:
  explicit PathToVelocityTarget(double control_dt_seconds);

  reflexxes::State state_;
  reflexxes::VelocityFlags flags_;
  reflexxes::VelocityInputs inputs_;
  reflexxes::VelocityOutputs outputs_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_SPEED_OVERRIDE_FACTOR_REFLEXXES_UTILS_H_
