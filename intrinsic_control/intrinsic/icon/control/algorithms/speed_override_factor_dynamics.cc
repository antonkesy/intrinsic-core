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

#include "intrinsic/icon/control/algorithms/speed_override_factor_dynamics.h"

#include <algorithm>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/time/time.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/algorithms/speed_override_factor_dynamics_utils.h"
#include "intrinsic/icon/control/algorithms/speed_override_factor_limits_computation.h"
#include "intrinsic/icon/control/algorithms/speed_override_factor_reflexxes_utils.h"
#include "intrinsic/icon/control/algorithms/speed_override_factor_types.h"
#include "intrinsic/icon/dynamics/rigid_body_interface.h"
#include "intrinsic/icon/reflexxes/motion_polynomials.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/kinematics/types/dynamic_limits_check_mode.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/kinematics/types/joint_trajectories.h"
#include "intrinsic/math/ipow.h"
#include "intrinsic/motion_planning/trajectory_planning/interpolate_joint_trajectories.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

namespace {

// The scaling factor for the `dsof_dt` limits. This is used to scale down the
// `dsof_dt` limits to avoid numerical violations of joint acceleration limits.
constexpr double kDsofDtLimitFactor = 1.0;

// The scaling factor for the `d2sof_dt2` limits. This is used to scale down
// the `d2sof_dt2` limits to avoid numerical violations of joint jerk limits.
constexpr double kD2sofDt2LimitFactor = 0.95;

// The scaling factor for the joint jerk limits when checking if the
// interpolated trajectory is within the joint limits.
constexpr double kMaxJerkLimitOvershootAtInterpolatedSample = 1.0;

absl::Status ValidateParams(const SpeedOverrideFactorDynamicsParams& params) {
  if (params.control_dt_seconds <= 0.0) {
    return absl::InvalidArgumentError(
        absl::StrCat("The control sampling time must be positive. Got ",
                     params.control_dt_seconds));
  }
  if (params.stop_check_lookahead_cycles <= 1) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The number of exact check-to-stop cycles must be larger than 1. Got ",
        params.stop_check_lookahead_cycles));
  }
  if (params.reached_target_velocity_tolerance <= 0.0) {
    return absl::InvalidArgumentError(absl::StrCat(
        "The reached target velocity tolerance must be positive. Got ",
        params.reached_target_velocity_tolerance));
  }

  if (params.control_scaling_factors.empty()) {
    return absl::InvalidArgumentError(
        "The control scaling factors must not be empty.");
  }
  for (const double scaling : params.control_scaling_factors) {
    if (scaling <= 0.0 || scaling > 1.0) {
      return absl::InvalidArgumentError(
          absl::StrCat("The control scaling factors must be in the range (0.0, "
                       "1.0]. Got ",
                       scaling, "."));
    }
  }
  absl::flat_hash_set<double> unique_factors(
      params.control_scaling_factors.begin(),
      params.control_scaling_factors.end());
  if (unique_factors.size() != params.control_scaling_factors.size()) {
    return absl::InvalidArgumentError(
        "The control scaling factors must be unique.");
  }
  if (!std::is_sorted(params.control_scaling_factors.begin(),
                      params.control_scaling_factors.end(),
                      std::greater<double>())) {
    return absl::InvalidArgumentError(
        "The control scaling factors must be sorted in descending order.");
  }

  return absl::OkStatus();
}

// Returns a scaled version of the `limits` with a safety margin. The safety
// margin is applied to the `dsof_dt` and `d2sof_dt2` limits.
RealtimeStatusOr<SpeedOverrideFactorLimits> SofLimitsWithSafetyMargin(
    const double trajectory_phase,
    const SpeedOverrideFactorStateWithSecondDerivative& sof_state,
    const SpeedOverrideFactorLimitsComputation& limits_computation) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const SpeedOverrideFactorLimits limits,
      limits_computation.ComputeAllSpeedOverrideFactorLimits(
          trajectory_phase,
          {.sof = sof_state.sof, .dsof_dt = sof_state.dsof_dt},
          /*clamp_sof_to_zero_one_range=*/true));
  return ScaleSpeedOverrideFactorLimits(limits, kDsofDtLimitFactor,
                                        kD2sofDt2LimitFactor);
}

// Returns true if the `state` (sof, dsof_dt, d2sof_dt2) is within the `limits`.
inline bool IsStateWithinLimits(
    const SpeedOverrideFactorStateWithSecondDerivative& state,
    const SpeedOverrideFactorLimits& limits) {
  const double kEpsilon = 1e-13;
  return state.sof >= limits.sof_limits.lower - kEpsilon &&
         state.sof <= limits.sof_limits.upper + kEpsilon &&
         state.dsof_dt >= limits.dsof_dt_limits.lower - kEpsilon &&
         state.dsof_dt <= limits.dsof_dt_limits.upper + kEpsilon &&
         state.d2sof_dt2 >= limits.d2sof_dt2_limits.lower - kEpsilon &&
         state.d2sof_dt2 <= limits.d2sof_dt2_limits.upper + kEpsilon;
}

// Takes the `joint_limits` and returns a version of them that adds a safety
// margin to the joint jerk limits.
JointLimits JointLimitsWithJerkLimitsTolerance(
    const JointLimits& joint_limits) {
  JointLimits joint_limits_with_tolerance = joint_limits;
  joint_limits_with_tolerance.max_jerk *=
      kMaxJerkLimitOvershootAtInterpolatedSample;
  return joint_limits_with_tolerance;
}

// Constructs the speed overridden acceleration limited trajectory (from the
// basis functions given by `trajectory` and the speed override factor
// trajectory given by `motion_polynomials`), and checks if it is within the
// acceleration and jerk limits given by `joint_limits`. The checks are
// performed at substeps of the control sampling time
// `params.control_dt_seconds`, the speed override factor dynamics described by
// the `motion_polynomials` is integrated to get the corresponding trajectory
// phase. Returns true if the interpolated trajectory is within `joint_limits`
// and false otherwise.
RealtimeStatusOr<bool> IsInterpolatedAccelerationLimitedTrajectoryWithinLimits(
    const double trajectory_phase,
    const reflexxes::MotionPolynomials& motion_polynomials,
    const double trajectory_duration_seconds,
    const JointTrajectoryPVA& trajectory,
    const JointLimits& joint_limits_with_jerk_limits_tolerance,
    const SpeedOverrideFactorDynamicsParams& params,
    const bool check_jerk_limits) {
  // Prepare the interpolated joint state for the middle points that will be
  // used for checking satisfaction of acceleration and jerk limits.
  const int num_dofs = joint_limits_with_jerk_limits_tolerance.size();
  JointStatePVAJ interpolated_joint_state;
  INTRINSIC_RT_RETURN_IF_ERROR(interpolated_joint_state.SetSize(num_dofs));

  // We evaluate that several interpolated points along the current step given
  // by `params.control_dt_seconds` satisfy the acceleration and jerk limits. At
  // these points, we integrate the speed override factor trajectory to get the
  // corresponding trajectory phases. At these trajectory phases, we interpolate
  // the trajectory to perform the limits check. Fine substep checks are
  // performed to get a better overview of the behaviour of the speed-overridden
  // trajectory.
  for (const double factor : {0.2, 0.4, 0.6, 0.8, 1.0}) {
    const double test_sof_time = factor * params.control_dt_seconds;
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        const double test_traj_phase,
        GetNextTrajectoryPhase(trajectory_phase, motion_polynomials,
                               test_sof_time, trajectory_duration_seconds));

    const absl::Duration test_traj_time =
        test_traj_phase * trajectory.Duration();
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        JointStatePVAJ test_state_basis_functions,
        InterpolateJointTrajectoryInternalType(trajectory, test_traj_time));
    reflexxes::MotionPolynomials::MotionState test_sof_state =
        motion_polynomials.GetStateOfMotionAtTime(test_sof_time);

    const eigenmath::VectorNd& qp = test_state_basis_functions.velocity;
    const eigenmath::VectorNd& qpp = test_state_basis_functions.acceleration;
    const eigenmath::VectorNd& qppp = test_state_basis_functions.jerk;
    const double sof = test_sof_state.position;
    const double sof2 = ::intrinsic::IPow(test_sof_state.position, 2);
    const double sof3 = ::intrinsic::IPow(test_sof_state.position, 3);
    const double dsof_dt = test_sof_state.velocity;
    const double d2sof_dt2 = test_sof_state.acceleration;
    // Positions and velocities are not set, as only accelerations and jerks
    // constraints are approximated. Other constraints are exact.
    // Parameterizing the trajectory in terms of the speed override factor
    // `sof`, leads to the following equations:
    // - Joint velocity: `qd` = `qp` * `sof`.
    // - Joint acceleration: `qdd` = `qpp` * `sof^2` + `qp` * `dsof/dt`.
    // - Joint jerk: `qddd` = `qppp` * `sof^3` + 3 * `qpp` * `sof` * `dsof/dt`
    //   + `qp` * `d2sof/dt2`.
    interpolated_joint_state.acceleration = qpp * sof2 + qp * dsof_dt;
    interpolated_joint_state.jerk =
        qppp * sof3 + 3.0 * qpp * sof * dsof_dt + qp * d2sof_dt2;

    INTRINSIC_RT_ASSIGN_OR_RETURN(
        LimitCheckResult limit_check_result,
        IsWithinLimits(interpolated_joint_state,
                       joint_limits_with_jerk_limits_tolerance));
    if ((!limit_check_result.a_ok) ||
        (check_jerk_limits && !limit_check_result.j_ok)) {
      return false;
    }
  }
  return true;
}

// Computes an updated previous state composed by adding the position in
// `next_position` to the history of positions captured in the `previous_state`.
// The derivatives of the updated previous state are computed using the finite
// differences with respect to time `dt_sec`. Returns an error if the size of
// the previous and the next state do not match.
RealtimeStatusOr<JointStatePVAJ> UpdatePreviousStateFromPosition(
    const JointStatePVA& previous_state,
    const eigenmath::VectorNd& next_position, const double dt_sec) {
  if (previous_state.size() != next_position.size()) {
    return InternalError(
        "The size of the previous and the next state do not match.");
  }

  JointStatePVAJ state;
  INTRINSIC_RT_RETURN_IF_ERROR(state.SetSize(previous_state.size()));
  state.position = next_position;
  state.velocity = (state.position - previous_state.position) / dt_sec;
  state.acceleration = (state.velocity - previous_state.velocity) / dt_sec;
  state.jerk = (state.acceleration - previous_state.acceleration) / dt_sec;
  return state;
}

// Same as the above function, but takes the `trajectory_phase` and interpolates
// the trajectory to get the next position before updating the previous state.
RealtimeStatusOr<JointStatePVAJ> UpdatePreviousStateFromTrajectoryPhase(
    const JointStatePVA& previous_state, const double trajectory_phase,
    const double dt_sec, const JointTrajectoryPVA& trajectory) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      JointStatePVAJ next_state,
      InterpolateJointTrajectoryInternalType(
          /*trajectory=*/trajectory,
          /*time_since_trajectory_start=*/trajectory_phase *
              trajectory.Duration()));
  return UpdatePreviousStateFromPosition(previous_state, next_state.position,
                                         dt_sec);
}

// Computes the updated version of the `previous_state` obtained with the next
// position sampled from the trajectory at phase `trajectory_phase` and
// differentiated with respect to time `dt_sec`. Then checks whether this
// `next_state` is within the joint limits with a given tolerance. Returns true
// if the acceleration and jerk limits are satisfied and false otherwise.
RealtimeStatusOr<bool> IsBackwardDifferentiatedKinematicsStateWithinLimits(
    const JointStatePVA& previous_state, const double trajectory_phase,
    const double dt_sec, const JointTrajectoryPVA& trajectory,
    const JointLimits& joint_limits_with_jerk_limits_tolerance,
    const bool check_jerk_limits) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      JointStatePVAJ next_state,
      UpdatePreviousStateFromTrajectoryPhase(previous_state, trajectory_phase,
                                             dt_sec, trajectory));

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      LimitCheckResult limit_check_result,
      IsWithinLimits(next_state, joint_limits_with_jerk_limits_tolerance));
  return !check_jerk_limits
             ? limit_check_result.a_ok
             : limit_check_result.a_ok && limit_check_result.j_ok;
}

// Checks whether an acceleration-limited or torque-limited interpolated
// trajectory is within the joint limits
// `joint_limits_with_jerk_limits_tolerance`. Checks for acceleration-limited
// trajectories include interpolated accelerations and jerks. Checks for
// torque-limited trajectories are not implemented yet.
RealtimeStatusOr<bool> IsInterpolatedTrajectoryWithinLimits(
    const double trajectory_phase, const double next_trajectory_phase,
    const reflexxes::MotionPolynomials& motion_polynomials,
    const JointStatePVA& previous_state,
    const double trajectory_duration_seconds,
    const JointTrajectoryPVA& trajectory,
    const JointLimits& joint_limits_with_jerk_limits_tolerance,
    const SpeedOverrideFactorDynamicsParams& params,
    const bool check_jerk_limits) {
  if (trajectory.joint_dynamic_limits_check_mode() ==
      DynamicLimitsCheckMode::kCheckJointAcceleration) {
    bool discrete_within_limits_check = true;
    if (params.enable_backward_differentiation_limit_checking) {
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          discrete_within_limits_check,
          IsBackwardDifferentiatedKinematicsStateWithinLimits(
              previous_state, next_trajectory_phase, params.control_dt_seconds,
              trajectory, joint_limits_with_jerk_limits_tolerance,
              check_jerk_limits));
    }
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        const bool continuous_within_limits_check,
        IsInterpolatedAccelerationLimitedTrajectoryWithinLimits(
            trajectory_phase, motion_polynomials, trajectory_duration_seconds,
            trajectory, joint_limits_with_jerk_limits_tolerance, params,
            check_jerk_limits));
    return discrete_within_limits_check && continuous_within_limits_check;
  }

  // TODO(b/400611509): Implement limits checking for torque-limited trajectory.
  return true;
}

// Returns true if the speed override factor state can be controlled to zero
// velocity in a finite maximum number of cycles (see
// `params.stop_check_lookahead_cycles`) up to a current tolerance (see
// `params.reached_target_velocity_tolerance`) starting from `start_state` at
// phase `trajectory_phase` and having as sof target `target_state`.
// Compute the speed override factor limits with `limits_computation` at the
// current `trajectory_phase`, compute the best possible stop command using
// `path_to_velocity_target`, update the speed override factor state and
// trajectory phase (using the `GetNextTrajectoryPhase()` function) based on
// the stop command. Run this update for `stop_check_lookahead_cycles`. To
// check whether a stop state can be reached, at each update, the velocity of
// speed override factor state is checked against the target velocity with a
// tolerance of `reached_target_velocity_tolerance`.
RealtimeStatusOr<bool> IsDsofDtControllableToZero(
    const double trajectory_phase,
    const SpeedOverrideFactorDynamicsParams& params,
    const SpeedOverrideFactorStateWithSecondDerivative& start_state,
    const SpeedOverrideFactorStateWithDerivative& target_state,
    const SpeedOverrideFactorLimitsComputation& limits_computation,
    const double trajectory_duration_seconds,
    const JointTrajectoryPVA& trajectory,
    const JointLimits& joint_limits_with_jerk_limits_tolerance,
    const JointStatePVA& previous_state, const bool check_jerk_limits,
    PathToVelocityTarget& path_to_velocity_target) {
  // Initialize current phase and state.
  double curr_phase = trajectory_phase;
  SpeedOverrideFactorStateWithSecondDerivative curr_state = start_state;
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      JointStatePVAJ curr_previous_state,
      UpdatePreviousStateFromTrajectoryPhase(
          previous_state, trajectory_phase, params.control_dt_seconds,
          limits_computation.GetTrajectory()));

  // Check if the speed override factor state can be controlled to zero velocity
  // in a finite number of control cycles.
  for (int i = 0; i < params.stop_check_lookahead_cycles; ++i) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        const SpeedOverrideFactorLimits curr_limits,
        SofLimitsWithSafetyMargin(curr_phase, curr_state, limits_computation));
    PathToTargetState path_to_stop_state = path_to_velocity_target.GetNextState(
        curr_state, {.sof = target_state.sof, .dsof_dt = 0.0}, curr_limits,
        /*check_path_limits_are_satisfied=*/true);
    if (!path_to_stop_state.HasNextState() ||
        !IsStateWithinLimits(path_to_stop_state.next_state->state,
                             curr_limits)) {
      return false;
    }
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        SpeedOverrideFactorDynamicsResult path_to_stop_state_with_next_phase,
        GetNextStateResultForNonSteadyStateSpeedOverrideFactor(
            curr_phase, params.control_dt_seconds, trajectory_duration_seconds,
            std::move(path_to_stop_state)));
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        const bool is_interpolated_trajectory_within_limits,
        IsInterpolatedTrajectoryWithinLimits(
            curr_phase,
            path_to_stop_state_with_next_phase.next_trajectory_phase,
            path_to_stop_state_with_next_phase.motion_polynomials,
            curr_previous_state, trajectory_duration_seconds, trajectory,
            joint_limits_with_jerk_limits_tolerance, params,
            check_jerk_limits));
    if (!is_interpolated_trajectory_within_limits) {
      return false;
    }
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        curr_previous_state,
        UpdatePreviousStateFromTrajectoryPhase(
            curr_previous_state,
            path_to_stop_state_with_next_phase.next_trajectory_phase,
            params.control_dt_seconds, limits_computation.GetTrajectory()));
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        bool can_stop,
        ReachesTargetVelocity(
            path_to_stop_state_with_next_phase.motion_polynomials,
            /*target_velocity=*/0.0, params.reached_target_velocity_tolerance,
            params.control_dt_seconds));
    if (can_stop) {
      return true;
    }

    curr_phase = path_to_stop_state_with_next_phase.next_trajectory_phase;
    curr_state = path_to_stop_state_with_next_phase.next_state;
  }

  return false;
}

}  // namespace

/* static */
absl::StatusOr<SpeedOverrideFactorDynamics> SpeedOverrideFactorDynamics::Create(
    const SpeedOverrideFactorDynamicsParams& params,
    const JointTrajectoryPVA& trajectory, const JointLimits& joint_limits,
    std::unique_ptr<RigidBodyInterface> dynamics) {
  INTR_RETURN_IF_ERROR(ValidateParams(params));

  INTR_ASSIGN_OR_RETURN(
      PathToPositionTarget path_to_position_target,
      PathToPositionTarget::Create(params.control_dt_seconds));
  INTR_ASSIGN_OR_RETURN(
      PathToVelocityTarget path_to_velocity_target,
      PathToVelocityTarget::Create(params.control_dt_seconds));
  INTR_ASSIGN_OR_RETURN(SpeedOverrideFactorLimitsComputation limits_computation,
                        SpeedOverrideFactorLimitsComputation::Create(
                            trajectory, joint_limits, std::move(dynamics)));

  // Only trajectories that enforce third-order constraints (i.e. joint jerk or
  // joint torque rate limits) should use jerk limit satisfaction within the
  // dynamics of the speed override factor trajectory.
  const bool check_jerk_limits =
      trajectory.interpolation_type() ==
      JointTrajectoryInterpolationType::kQuinticPolynomial;

  const JointLimits joint_limits_with_jerk_limits_tolerance =
      JointLimitsWithJerkLimitsTolerance(joint_limits);

  JointStatePVAJ previous_state;
  INTRINSIC_RT_RETURN_IF_ERROR(previous_state.SetSize(joint_limits.size()));
  previous_state.position = trajectory.data().front().position;
  previous_state.velocity = trajectory.data().front().velocity;
  previous_state.acceleration = trajectory.data().front().acceleration;

  return SpeedOverrideFactorDynamics(
      check_jerk_limits, absl::ToDoubleSeconds(trajectory.Duration()), params,
      std::move(joint_limits_with_jerk_limits_tolerance),
      std::move(path_to_position_target), std::move(path_to_velocity_target),
      std::move(limits_computation), std::move(previous_state));
}

SpeedOverrideFactorDynamics::SpeedOverrideFactorDynamics(
    bool check_jerk_limits, double trajectory_duration_seconds,
    SpeedOverrideFactorDynamicsParams params,
    JointLimits joint_limits_with_jerk_limits_tolerance,
    PathToPositionTarget path_to_position_target,
    PathToVelocityTarget path_to_velocity_target,
    SpeedOverrideFactorLimitsComputation limits_computation,
    JointStatePVAJ previous_state)
    : check_jerk_limits_(check_jerk_limits),
      trajectory_duration_seconds_(trajectory_duration_seconds),
      params_(params),
      joint_limits_with_jerk_limits_tolerance_(
          std::move(joint_limits_with_jerk_limits_tolerance)),
      path_to_position_target_(std::move(path_to_position_target)),
      path_to_velocity_target_(std::move(path_to_velocity_target)),
      limits_computation_(std::move(limits_computation)),
      previous_state_(std::move(previous_state)) {}

RealtimeStatusOr<SpeedOverrideFactorDynamicsResult>
SpeedOverrideFactorDynamics::GetNextState(
    const SpeedOverrideFactorStateWithSecondDerivative& start_state,
    const SpeedOverrideFactorStateWithDerivative& target_state,
    double trajectory_phase) {
  // Compute limits at current phase.
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const SpeedOverrideFactorLimits limits,
      SofLimitsWithSafetyMargin(trajectory_phase, start_state,
                                limits_computation_));

  // Evaluate if there exists a scaling of control limits d2sof_dt2 that leads
  // to a next state, that is controllable to zero velocity. This happens by
  // iteratively reducing the scaling factor in a two step approach:
  // 1. Scale limits and attempt to find the next state along a limit-aware
  //    speed override factor trajectory. Also compute the trajectory phase
  //    reached by the next state.
  // 2. Check whether the new state is controllable to zero velocity `dsof_dt`
  //    and acceleration `d2sof_dt2`. If yes, return the next state and
  //    trajectory phase. This step uses the full non-scaled down limits.
  // The scaling factor is reduced iteratively until a feasible and controllable
  // command is found or no scaling factor leads to a next state, in which case
  // a stop command is returned. Note that dure to the recursive nature of the
  // algorithm, the current state is always controllable to zero velocity.
  for (const double scaling_factor : params_.control_scaling_factors) {
    // Scale limits and attempt to find the next state along a limit-aware speed
    // override factor trajectory.
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        const SpeedOverrideFactorLimits scaled_limits,
        ScaleSpeedOverrideFactorLimits(limits, /*dsof_dt_factor=*/1.0,
                                       /*d2sof_dt2_factor=*/scaling_factor));
    PathToTargetState path_to_position_target_state =
        path_to_position_target_.GetNextState(
            start_state, target_state, scaled_limits,
            /*check_path_limits_are_satisfied=*/true);
    if (!path_to_position_target_state.HasNextState() ||
        !IsStateWithinLimits(path_to_position_target_state.next_state->state,
                             scaled_limits)) {
      // Attempt using the next scaling factor, as this one is not feasible.
      continue;
    }
    // Compute the trajectory phase reached by the next state and check
    // whether the new state is controllable to zero velocity.
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        SpeedOverrideFactorDynamicsResult
            path_to_position_target_state_with_next_phase,
        GetNextStateResultForNonSteadyStateSpeedOverrideFactor(
            trajectory_phase, params_.control_dt_seconds,
            trajectory_duration_seconds_,
            std::move(path_to_position_target_state)));

    INTRINSIC_RT_ASSIGN_OR_RETURN(
        const bool is_interpolated_trajectory_within_limits,
        IsInterpolatedTrajectoryWithinLimits(
            trajectory_phase,
            path_to_position_target_state_with_next_phase.next_trajectory_phase,
            path_to_position_target_state_with_next_phase.motion_polynomials,
            previous_state_, trajectory_duration_seconds_,
            limits_computation_.GetTrajectory(),
            joint_limits_with_jerk_limits_tolerance_, params_,
            check_jerk_limits_));
    if (!is_interpolated_trajectory_within_limits) {
      // Attempt using the next scaling factor, as this one is not feasible.
      continue;
    }
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        const bool can_stop,
        IsDsofDtControllableToZero(
            path_to_position_target_state_with_next_phase.next_trajectory_phase,
            params_, path_to_position_target_state_with_next_phase.next_state,
            target_state, limits_computation_, trajectory_duration_seconds_,
            limits_computation_.GetTrajectory(),
            joint_limits_with_jerk_limits_tolerance_, previous_state_,
            check_jerk_limits_, path_to_velocity_target_));
    if (can_stop) {
      return path_to_position_target_state_with_next_phase;
    }
  }

  // If the speed override factor state is already in a stable state (dsof_dt
  // and d2sof_dt2 are almost zero), we can return the `start_state`.
  if (IsSteadyState(start_state)) {
    return GetNextStateResultForSteadyStateSpeedOverrideFactor(
        trajectory_phase, params_.control_dt_seconds,
        trajectory_duration_seconds_, start_state);
  }

  // If no feasible and controllable command is available, then command a stop.
  PathToTargetState path_to_stop_state = path_to_velocity_target_.GetNextState(
      start_state, {.sof = target_state.sof, .dsof_dt = 0.0}, limits,
      /*check_path_limits_are_satisfied=*/false);
  return GetNextStateResultForNonSteadyStateSpeedOverrideFactor(
      trajectory_phase, params_.control_dt_seconds,
      trajectory_duration_seconds_, std::move(path_to_stop_state));
}

RealtimeStatus SpeedOverrideFactorDynamics::UpdatePreviousState(
    const eigenmath::VectorNd& next_position) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      previous_state_,
      UpdatePreviousStateFromPosition(previous_state_, next_position,
                                      params_.control_dt_seconds));
  return OkStatus();
}

}  // namespace intrinsic::icon
