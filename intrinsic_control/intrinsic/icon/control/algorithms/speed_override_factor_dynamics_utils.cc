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

#include "intrinsic/icon/control/algorithms/speed_override_factor_dynamics_utils.h"

#include "intrinsic/icon/control/algorithms/speed_override_factor_reflexxes_utils.h"
#include "intrinsic/icon/reflexxes/motion_polynomials.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/math/almost_equals.h"

namespace intrinsic::icon {

namespace {

// Minimum and maximum valid phase values.
constexpr double kMinimumPhase = 0.0;
constexpr double kMaximumPhase = 1.0;

// Returns a scaled version of the `limits` with the desired `factor`. The
// resulting limits are centered around the middle of the original limits. In
// that way, they remain feasible (within the original limits), as long as
// factor <= `1.0`. As an example, zero scaling would yield the center of the
// original limit range. A boundary condition is that the scaled limits are
// clamped to the range [-`kSpeedOverrideFactorReflexxesInfinity`,
// `kSpeedOverrideFactorReflexxesInfinity`] to avoid numerical issues.
inline SpeedOverrideFactorLimits::LimitPair ScaleLimitPair(
    const SpeedOverrideFactorLimits::LimitPair& limits, const double factor) {
  if (AlmostEquals(factor, 1.0)) {
    return limits;
  }

  const double lower =
      std::max(-kSpeedOverrideFactorReflexxesInfinity, limits.lower);
  const double upper =
      std::min(kSpeedOverrideFactorReflexxesInfinity, limits.upper);
  const double mid = 0.5 * (lower + upper);
  const double half_range = upper - mid;
  return {.lower = std::max(-kSpeedOverrideFactorReflexxesInfinity,
                            mid - factor * half_range),
          .upper = std::min(kSpeedOverrideFactorReflexxesInfinity,
                            mid + factor * half_range)};
}

}  // namespace

bool IsSteadyState(const SpeedOverrideFactorStateWithSecondDerivative& state) {
  return (AlmostEquals(state.dsof_dt, 0.0) &&
          AlmostEquals(state.d2sof_dt2, 0.0));
}

RealtimeStatusOr<double> GetNextTrajectoryPhase(
    const double trajectory_phase,
    const reflexxes::MotionPolynomials& motion_polynomials,
    const double control_dt_seconds, const double trajectory_duration_seconds) {
  if (trajectory_duration_seconds <= 0.0) {
    return InvalidArgumentError(
        "The `trajectory_duration_seconds` must be strictly positive.");
  }
  INTRINSIC_RT_ASSIGN_OR_RETURN(const double motion_polynomials_integral,
                                IntegratePositionInMotionPolynomials(
                                    motion_polynomials, control_dt_seconds));
  const double trajectory_phase_change =
      motion_polynomials_integral / trajectory_duration_seconds;
  const double next_trajectory_phase =
      trajectory_phase + trajectory_phase_change;

  // The trajectory phase must always be in the range [0.0, 1.0].
  return std::clamp(next_trajectory_phase, kMinimumPhase, kMaximumPhase);
}

RealtimeStatusOr<SpeedOverrideFactorDynamicsResult>
GetNextStateResultForSteadyStateSpeedOverrideFactor(
    const double trajectory_phase, const double control_dt_seconds,
    const double trajectory_duration_seconds,
    const SpeedOverrideFactorStateWithSecondDerivative& steady_state) {
  const reflexxes::MotionPolynomials motion_polynomials =
      CreateMotionPolynomialsWithConstantSpeedOverride(steady_state.sof);
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const double next_trajectory_phase,
      GetNextTrajectoryPhase(trajectory_phase, motion_polynomials,
                             control_dt_seconds, trajectory_duration_seconds));
  return SpeedOverrideFactorDynamicsResult{
      .next_state = steady_state,
      .motion_polynomials = std::move(motion_polynomials),
      .next_trajectory_phase = next_trajectory_phase};
}

RealtimeStatusOr<SpeedOverrideFactorDynamicsResult>
GetNextStateResultForNonSteadyStateSpeedOverrideFactor(
    const double trajectory_phase, const double control_dt_seconds,
    const double trajectory_duration_seconds,
    PathToTargetState&& non_steady_path_to_target_state) {
  if (!non_steady_path_to_target_state.HasNextState()) {
    return InvalidArgumentError(
        "The `non_steady_path_to_target_state` must have a next state.");
  }
  PathToTargetState::NextState& next_state =
      *non_steady_path_to_target_state.next_state;
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const double next_trajectory_phase,
      GetNextTrajectoryPhase(trajectory_phase, next_state.motion_polynomials,
                             control_dt_seconds, trajectory_duration_seconds));
  return SpeedOverrideFactorDynamicsResult{
      .next_state = std::move(next_state.state),
      .motion_polynomials = std::move(next_state.motion_polynomials),
      .next_trajectory_phase = next_trajectory_phase};
}

RealtimeStatusOr<SpeedOverrideFactorLimits> ScaleSpeedOverrideFactorLimits(
    const SpeedOverrideFactorLimits& limits, const double dsof_dt_factor,
    const double d2sof_dt2_factor) {
  if (dsof_dt_factor <= 0.0 || d2sof_dt2_factor <= 0.0) {
    return InvalidArgumentError(
        "The `dsof_dt_factor` and `d2sof_dt2_factor` must be > 0.0.");
  }

  return SpeedOverrideFactorLimits{
      .sof_limits = limits.sof_limits,
      .dsof_dt_limits = ScaleLimitPair(limits.dsof_dt_limits, dsof_dt_factor),
      .d2sof_dt2_limits =
          ScaleLimitPair(limits.d2sof_dt2_limits, d2sof_dt2_factor)};
}

}  // namespace intrinsic::icon
