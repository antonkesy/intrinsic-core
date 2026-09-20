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

#ifndef INTRINSIC_ICON_CONTROL_ALGORITHMS_SPEED_OVERRIDE_FACTOR_DYNAMICS_UTILS_H_
#define INTRINSIC_ICON_CONTROL_ALGORITHMS_SPEED_OVERRIDE_FACTOR_DYNAMICS_UTILS_H_

#include "intrinsic/icon/control/algorithms/speed_override_factor_reflexxes_utils.h"
#include "intrinsic/icon/control/algorithms/speed_override_factor_types.h"
#include "intrinsic/icon/reflexxes/motion_polynomials.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

struct SpeedOverrideFactorDynamicsResult {
  // The next speed override factor state.
  SpeedOverrideFactorStateWithSecondDerivative next_state;

  // The motion polynomials that describe the path from the current speed
  // override factor state to the target speed override factor state.
  reflexxes::MotionPolynomials motion_polynomials;

  // The `next_trajectory_phase` is the phase in the trajectory that can be
  // reached in a single control cycle and is implied by the control
  // `SpeedOverrideFactorStateWithSecondDerivative.d2sof_dt2` at the current
  // state and phase.
  double next_trajectory_phase;
};

// Returns true if the input `state`'s first and second derivatives are strictly
// in a steady state (numerically zero).
bool IsSteadyState(const SpeedOverrideFactorStateWithSecondDerivative& state);

// Returns the next trajectory phase value. This is computed by adding the
// current `trajectory_phase`, and the trajectory phase change implied by the
// `motion_polynomials` integrated over the control sampling time
// `control_dt_seconds` and normalized by the trajectory duration
// `trajectory_duration_seconds`. The result is clamped to the range [0.0, 1.0].
RealtimeStatusOr<double> GetNextTrajectoryPhase(
    const double trajectory_phase,
    const reflexxes::MotionPolynomials& motion_polynomials,
    const double control_dt_seconds, const double trajectory_duration_seconds);

// For the `steady_state` of the speed override factor dynamics (state where the
// first derivative `dsof/dt` and second derivative `d2sof/dt2` equal zero) at
// the current `trajectory_phase`, this function:
// 1) computes the motion polynomial that represents the constant speed override
//    factor evolution over time.
// 2) computes the next trajectory phase, by integrating the motion polynomial
//    and adding it to the current `trajectory_phase`. The integral is computed
//    over the control sampling time `control_dt_seconds` and normalized by the
//    trajectory duration `trajectory_duration_seconds`.
RealtimeStatusOr<SpeedOverrideFactorDynamicsResult>
GetNextStateResultForSteadyStateSpeedOverrideFactor(
    const double trajectory_phase, const double control_dt_seconds,
    const double trajectory_duration_seconds,
    const SpeedOverrideFactorStateWithSecondDerivative& steady_state);

// Same as above, but makes use of the motion polynomial within the
// `non_steady_path_to_target_state`.
RealtimeStatusOr<SpeedOverrideFactorDynamicsResult>
GetNextStateResultForNonSteadyStateSpeedOverrideFactor(
    const double trajectory_phase, const double control_dt_seconds,
    const double trajectory_duration_seconds,
    PathToTargetState&& non_steady_path_to_target_state);

// Scales the first derivative of the speed override factor `limits` by
// `dsof_dt_factor` and the second derivative of the speed override factor
// `limits` by `d2sof_dt2_factor`.
RealtimeStatusOr<SpeedOverrideFactorLimits> ScaleSpeedOverrideFactorLimits(
    const SpeedOverrideFactorLimits& limits, const double dsof_dt_factor,
    const double d2sof_dt2_factor);

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ALGORITHMS_SPEED_OVERRIDE_FACTOR_DYNAMICS_UTILS_H_
