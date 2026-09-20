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

#include "intrinsic/icon/control/algorithms/speed_override_factor_reflexxes_utils.h"

#include <algorithm>
#include <cstdlib>
#include <limits>
#include <optional>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/icon/control/algorithms/speed_override_factor_types.h"
#include "intrinsic/icon/reflexxes/inputs.h"
#include "intrinsic/icon/reflexxes/motion_polynomials.h"
#include "intrinsic/icon/reflexxes/position_flags.h"
#include "intrinsic/icon/reflexxes/position_inputs.h"
#include "intrinsic/icon/reflexxes/position_outputs.h"
#include "intrinsic/icon/reflexxes/reflexxes_api.h"
#include "intrinsic/icon/reflexxes/status.h"
#include "intrinsic/icon/reflexxes/velocity_flags.h"
#include "intrinsic/icon/reflexxes/velocity_inputs.h"
#include "intrinsic/icon/reflexxes/velocity_outputs.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/math/almost_equals.h"
#include "intrinsic/math/ipow.h"

namespace intrinsic::icon {

namespace {

constexpr int kOneDof = 1;

// Tolerance for the phase value to be considered as a valid phase.
constexpr double kPhaseTolerance = 1.0e-4;
constexpr double kMinimumTrajectoryPhase = 0.0;
constexpr double kMaximumTrajectoryPhase = 1.0;

inline bool IsOk(reflexxes::Status status) {
  return (status == reflexxes::Status::kWorking) ||
         (status == reflexxes::Status::kFinalStateReached);
}

// Clamps the given input `value` to the range [`min_value`, `max_value`] which
// defaults to [-`kSpeedOverrideFactorReflexxesInfinity`,
// `kSpeedOverrideFactorReflexxesInfinity`].
inline double Clamp(
    double value,
    const double min_value = -kSpeedOverrideFactorReflexxesInfinity,
    const double max_value = kSpeedOverrideFactorReflexxesInfinity) {
  return std::clamp(value, min_value, max_value);
}

// Clamps the given input `phase` to the range [`min_value`, `max_value`] which
// defaults to [`kMinimumTrajectoryPhase`, `kMaximumTrajectoryPhase`].
inline double ClampPhase(double value,
                         const double min_value = kMinimumTrajectoryPhase,
                         const double max_value = kMaximumTrajectoryPhase) {
  return std::clamp(value, min_value, max_value);
}

inline void SetInputsToReflexxes(
    const SpeedOverrideFactorStateWithSecondDerivative& start_state,
    const SpeedOverrideFactorStateWithDerivative& target_state,
    const SpeedOverrideFactorLimits& limits, reflexxes::Inputs& inputs) {
  // Define start state:
  inputs.GetDOFs()[0].position = start_state.sof;
  inputs.GetDOFs()[0].velocity = start_state.dsof_dt;
  inputs.GetDOFs()[0].acceleration = start_state.d2sof_dt2;

  // Define target state:
  inputs.GetDOFs()[0].target_position = target_state.sof;
  inputs.GetDOFs()[0].target_velocity = target_state.dsof_dt;

  // Define state limits: Note that internally, Reflexxes performs a finiteness
  // check on the inputs. Thus, these limits are clamped to the range
  // [-`kSpeedOverrideFactorReflexxesInfinity`,
  // `kSpeedOverrideFactorReflexxesInfinity`] to satisfy this check.
  inputs.GetDOFs()[0].min_position = Clamp(limits.sof_limits.lower);
  inputs.GetDOFs()[0].max_position = Clamp(limits.sof_limits.upper);
  inputs.GetDOFs()[0].min_velocity = Clamp(limits.dsof_dt_limits.lower);
  inputs.GetDOFs()[0].max_velocity = Clamp(limits.dsof_dt_limits.upper);
  inputs.GetDOFs()[0].min_acceleration = Clamp(limits.d2sof_dt2_limits.lower);
  inputs.GetDOFs()[0].max_acceleration = Clamp(limits.d2sof_dt2_limits.upper);
  inputs.GetDOFs()[0].min_jerk = -kSpeedOverrideFactorReflexxesInfinity;
  inputs.GetDOFs()[0].max_jerk = kSpeedOverrideFactorReflexxesInfinity;
}

// Evaluates the integral of a polynomial of degree 3 with coefficients [p, v,
// a, j] from t0 to tf .
double EvalIntegral(const double t0, const double tf, const double p,
                    const double v, const double a, const double j) {
  const double t02 = t0 * t0;
  const double t03 = t02 * t0;
  const double t04 = t03 * t0;
  const double eval0 =
      p * t0 + 1.0 / 2.0 * v * t02 + 1.0 / 6.0 * a * t03 + 1.0 / 24.0 * j * t04;

  const double tf2 = tf * tf;
  const double tf3 = tf2 * tf;
  const double tf4 = tf3 * tf;
  const double evalf =
      p * tf + 1.0 / 2.0 * v * tf2 + 1.0 / 6.0 * a * tf3 + 1.0 / 24.0 * j * tf4;

  return evalf - eval0;
}

// Searches in the input `segment` polynomial for the time
// `segment_time_to_target` such that the polynomial evaluated at this point in
// time reaches the `target_phase` starting from `start_phase`. The time
// `trajectory_duration_seconds` is a normalization factor for the phase values.
// In short, it is called `D`. The `segment` stores the following polynomial:
//   p(t) = [p, v, a/2, j/6] * [1, t, t^2, t^3]
// where t is the time in seconds, valid in the range [0.0,
// `segment.end_time - segment.start_time`].
// The target phase can be computed as follows:
//                                          time_to_target
//   target_phase = start_phase + (1/D) * Integral  p(t) dt
//                                          t = 0.0
// It can be rewritten in the form:
//   (target_phase-start_phase)*D = (p*t + (v/2)*t^2 + (a/6)*t^3 + (j/24)*t^4) |
//   evaluated from t=0.0 to t=time_to_target
// This leads to the following polynomial
//   0 = (j/24)*T^4 + (a/6)*T^3 + (v/2)*T^2 + p*T - (target_phase-start_phase)*D
// where T is the time to target. Solving this equation for T and the segment
// start time, we obtain the total time to target.
icon::RealtimeStatusOr<double> GetSegmentTimeToTarget(
    const double segment_start_phase, const double target_phase,
    const reflexxes::MotionPolynomials::Segment& segment,
    const double trajectory_duration_seconds) {
  const double a0 = segment.jerk / 24.0;
  const double a1 = segment.acceleration / 6.0;
  const double a2 = segment.velocity / 2.0;
  const double a3 = segment.position;
  const double a4 =
      -(target_phase - segment_start_phase) * trajectory_duration_seconds;

  auto polynomial_at_time = [&](double T) {
    return a0 * ::intrinsic::IPow(T, 4) + a1 * ::intrinsic::IPow(T, 3) +
           a2 * ::intrinsic::IPow(T, 2) + a3 * T + a4;
  };
  auto polynomial_derivative_at_time = [&](double T) {
    return 4.0 * a0 * ::intrinsic::IPow(T, 3) +
           3.0 * a1 * ::intrinsic::IPow(T, 2) + 2.0 * a2 * T + a3;
  };

  // The `time_to_target` initial guess has to be in the range [0.0,
  // `segment.end_time - segment.start_time`]. For simplicity, we set the
  // initial guess to 0.0.
  double time_to_target = 0.0;
  constexpr int kMaxNewtonSteps = 20;
  for (int i = 0; i < kMaxNewtonSteps; ++i) {
    const double f_xn = polynomial_at_time(time_to_target);
    const double fp_xn = polynomial_derivative_at_time(time_to_target);
    // If either `f_xn` or `fp_xn` are zero, then no more improvement is
    // possible and we have converged.
    const double kEpsilon = std::numeric_limits<double>::epsilon();
    if (::intrinsic::AlmostEquals(std::abs(f_xn), 0.0, kEpsilon) ||
        ::intrinsic::AlmostEquals(std::abs(fp_xn), 0.0, kEpsilon))
      break;

    const double update = f_xn / fp_xn;
    if (::intrinsic::AlmostEquals(std::abs(update), 0.0, kEpsilon)) break;
    time_to_target -= update;
  }
  return std::clamp(time_to_target, 0.0, segment.end_time - segment.start_time);
}

}  // namespace

RealtimeStatusOr<double> IntegratePositionInMotionPolynomials(
    const reflexxes::MotionPolynomials& motion_polynomials,
    double time_horizon_seconds) {
  if (time_horizon_seconds < 0.0) {
    return InvalidArgumentError(
        absl::StrCat("The time horizon must be non-negative. Got ",
                     time_horizon_seconds, "."));
  }

  double position_integral = 0.0;
  for (const reflexxes::MotionPolynomials::Segment& segment :
       motion_polynomials.GetSegments()) {
    const bool is_last_segment = (segment.end_time >= time_horizon_seconds);
    const double eval_time =
        (is_last_segment ? time_horizon_seconds : segment.end_time) -
        segment.start_time;
    position_integral +=
        EvalIntegral(0.0, eval_time, segment.position, segment.velocity,
                     segment.acceleration, segment.jerk);
    if (is_last_segment) {
      break;
    }
  }
  return position_integral;
}

RealtimeStatusOr<double> ReachesTargetVelocity(
    const reflexxes::MotionPolynomials& motion_polynomials,
    const double target_velocity, const double reached_velocity_threshold,
    const double evaluation_time_sec) {
  if (evaluation_time_sec < 0.0) {
    return InvalidArgumentError(absl::StrCat(
        "The evaluation time must be non-negative. Got ", evaluation_time_sec));
  }
  if (reached_velocity_threshold <= 0.0) {
    return InvalidArgumentError(
        absl::StrCat("The reached_velocity_threshold must be positive. Got ",
                     reached_velocity_threshold));
  }

  // Evaluate the state at `evaluation_time_sec` given by the motion polynomials
  // and check whether the target velocity is reached up to the desired
  // threshold.
  reflexxes::MotionPolynomials::MotionState state_at_evaluation_time =
      motion_polynomials.GetStateOfMotionAtTime(evaluation_time_sec);
  return AlmostEquals(state_at_evaluation_time.velocity, target_velocity,
                      reached_velocity_threshold) &&
         AlmostEquals(state_at_evaluation_time.acceleration, 0.0,
                      reached_velocity_threshold);
}

reflexxes::MotionPolynomials CreateMotionPolynomialsWithConstantSpeedOverride(
    const double speed_override) {
  reflexxes::MotionPolynomials::Segment segment;
  segment.position = speed_override;
  segment.end_time = kSpeedOverrideAdjustedMaximumTimeToGo;
  reflexxes::MotionPolynomials motion_polynomials;
  motion_polynomials.AddSegment(segment);
  return motion_polynomials;
}

RealtimeStatusOr<double> GetSpeedOverrideAdjustedTimeToGo(
    const double current_trajectory_phase, const double target_trajectory_phase,
    const reflexxes::MotionPolynomials& speed_override_trajectory,
    const double trajectory_duration_seconds) {
  if ((current_trajectory_phase < kMinimumTrajectoryPhase - kPhaseTolerance) ||
      (current_trajectory_phase > kMaximumTrajectoryPhase + kPhaseTolerance)) {
    return icon::InvalidArgumentError(
        "The `current_trajectory_phase` must be in the range [0.0, 1.0].");
  }
  if ((target_trajectory_phase < kMinimumTrajectoryPhase - kPhaseTolerance) ||
      (target_trajectory_phase > kMaximumTrajectoryPhase + kPhaseTolerance)) {
    return icon::InvalidArgumentError(
        "The `target_trajectory_phase` must be in the range [0.0, 1.0].");
  }
  if (target_trajectory_phase < current_trajectory_phase) {
    return icon::InvalidArgumentError(
        "The `target_trajectory_phase` must be >= `current_trajectory_phase`.");
  }
  if (trajectory_duration_seconds <= 0.0) {
    return icon::InvalidArgumentError(
        "The `trajectory_duration_seconds` must be positive.");
  }
  if (speed_override_trajectory.GetSegments().empty()) {
    return icon::InvalidArgumentError(
        "The `speed_override_trajectory` must have at least one segment.");
  }

  double current_phase = ClampPhase(current_trajectory_phase);
  const double target_phase = ClampPhase(target_trajectory_phase);

  // Loop over all motion segments and search in which segment the target
  // trajectory phase is reached.
  for (int id = 0; id < speed_override_trajectory.GetSegmentCount(); ++id) {
    const bool is_last_segment =
        (id == speed_override_trajectory.GetSegmentCount() - 1);
    const reflexxes::MotionPolynomials::Segment& segment =
        speed_override_trajectory.GetSegments()[id];

    double next_phase;
    if (!is_last_segment) {
      const double phase_change =
          EvalIntegral(0.0, segment.end_time - segment.start_time,
                       segment.position, segment.velocity, segment.acceleration,
                       segment.jerk) /
          trajectory_duration_seconds;
      next_phase = ClampPhase(current_phase + phase_change);
    } else {
      // We need to treat the last segment separately, because the end time of
      // the last segment is infinity. Since the last segment of the
      // `speed_override_trajectory` has zero velocity, acceleration and jerk,
      // the maximum phase `kMaximumTrajectoryPhase` can be reached if the
      // position value (speed override factor) is non-zero.
      next_phase = ::intrinsic::AlmostEquals(segment.position, 0.0)
                       ? current_phase
                       : kMaximumTrajectoryPhase;
    }

    // Find time to target in this segment.
    if (current_phase <= target_phase && target_phase <= next_phase) {
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          double segment_time_to_target,
          GetSegmentTimeToTarget(current_phase, target_phase, segment,
                                 trajectory_duration_seconds));
      return segment.start_time + segment_time_to_target;
    }
    current_phase = next_phase;
  }

  // If we have reached this point, we have found that the target trajectory
  // phase is not reachable in finite time. Thus, we return infinity.
  return kSpeedOverrideAdjustedMaximumTimeToGo;
}

/* static */
absl::StatusOr<PathToPositionTarget> PathToPositionTarget::Create(
    double control_dt_seconds) {
  if (control_dt_seconds <= 0.0) {
    return absl::InvalidArgumentError(
        absl::StrCat("The control sampling time must be positive. Got ",
                     control_dt_seconds));
  }
  return PathToPositionTarget(control_dt_seconds);
}

PathToPositionTarget::PathToPositionTarget(double control_dt_seconds)
    : state_(/*num_dofs=*/kOneDof, /*cycle_time=*/control_dt_seconds),
      inputs_(/*num_dofs=*/kOneDof, /*cycle_time=*/control_dt_seconds),
      outputs_(/*num_dofs=*/kOneDof, /*cycle_time=*/control_dt_seconds) {}

PathToTargetState PathToPositionTarget::GetNextState(
    const SpeedOverrideFactorStateWithSecondDerivative& start_state,
    const SpeedOverrideFactorStateWithDerivative& target_state,
    const SpeedOverrideFactorLimits& limits,
    bool check_path_limits_are_satisfied) {
  flags_.positional_limits_behavior =
      check_path_limits_are_satisfied
          ? reflexxes::PositionFlags::PositionalLimitsBehavior::kActivelyPrevent
          : reflexxes::PositionFlags::PositionalLimitsBehavior::kIgnore;
  SetInputsToReflexxes(start_state, target_state, limits, inputs_);
  if (!IsOk(reflexxes::ComputePosition(inputs_, flags_, outputs_, state_))) {
    return PathToTargetState{.next_state = std::nullopt};
  }

  return PathToTargetState{
      .next_state = PathToTargetState::NextState{
          .state =
              SpeedOverrideFactorStateWithSecondDerivative{
                  .sof = outputs_.GetDOFs()[0].new_position,
                  .dsof_dt = outputs_.GetDOFs()[0].new_velocity,
                  .d2sof_dt2 = outputs_.GetDOFs()[0].new_acceleration},
          .motion_polynomials = outputs_.GetDOFs()[0].motion_polynomials}};
}

/* static */
absl::StatusOr<PathToVelocityTarget> PathToVelocityTarget::Create(
    double control_dt_seconds) {
  if (control_dt_seconds <= 0.0) {
    return absl::InvalidArgumentError(
        absl::StrCat("The control sampling time must be positive. Got ",
                     control_dt_seconds));
  }
  return PathToVelocityTarget(control_dt_seconds);
}

PathToVelocityTarget::PathToVelocityTarget(double control_dt_seconds)
    : state_(/*num_dofs=*/kOneDof, /*cycle_time=*/control_dt_seconds),
      inputs_(/*num_dofs=*/kOneDof, /*cycle_time=*/control_dt_seconds),
      outputs_(/*num_dofs=*/kOneDof, /*cycle_time=*/control_dt_seconds) {}

PathToTargetState PathToVelocityTarget::GetNextState(
    const SpeedOverrideFactorStateWithSecondDerivative& start_state,
    const SpeedOverrideFactorStateWithDerivative& target_state,
    const SpeedOverrideFactorLimits& limits,
    const bool check_path_limits_are_satisfied) {
  flags_.positional_limits_behavior =
      check_path_limits_are_satisfied
          ? reflexxes::VelocityFlags::PositionalLimitsBehavior::kErrorMsgOnly
          : reflexxes::VelocityFlags::PositionalLimitsBehavior::kIgnore;
  SetInputsToReflexxes(start_state, target_state, limits, inputs_);
  if (!IsOk(reflexxes::ComputeVelocity(inputs_, flags_, outputs_, state_))) {
    return PathToTargetState{.next_state = std::nullopt};
  }

  return PathToTargetState{
      .next_state = PathToTargetState::NextState{
          .state =
              SpeedOverrideFactorStateWithSecondDerivative{
                  .sof = outputs_.GetDOFs()[0].new_position,
                  .dsof_dt = outputs_.GetDOFs()[0].new_velocity,
                  .d2sof_dt2 = outputs_.GetDOFs()[0].new_acceleration},
          .motion_polynomials = outputs_.GetDOFs()[0].motion_polynomials}};
}

}  // namespace intrinsic::icon
