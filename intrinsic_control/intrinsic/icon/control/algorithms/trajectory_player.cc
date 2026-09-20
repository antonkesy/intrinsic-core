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

#include "intrinsic/icon/control/algorithms/trajectory_player.h"

#include <algorithm>
#include <cstdlib>
#include <memory>
#include <utility>

#include "absl/log/check.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "intrinsic/eigenmath/clamp.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/algorithms/speed_override_factor_dynamics.h"
#include "intrinsic/icon/control/algorithms/speed_override_factor_reflexxes_utils.h"
#include "intrinsic/icon/control/algorithms/speed_override_factor_types.h"
#include "intrinsic/icon/dynamics/rigid_body_interface.h"
#include "intrinsic/icon/dynamics/validate_rigid_body_interface.h"
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

inline bool HasConvergedToTargetSpeedOverrideFactor(
    const SpeedOverrideFactorStateWithSecondDerivative& state,
    double target_sof, double threshold = 1.0e-29) {
  return ((std::abs(state.sof - target_sof) <= threshold) &&
          (std::abs(state.dsof_dt) <= threshold) &&
          (std::abs(state.d2sof_dt2) <= threshold));
}

}  // namespace

/* static */
absl::StatusOr<std::unique_ptr<TrajectoryPlayer>> TrajectoryPlayer::Create(
    JointTrajectoryPVA trajectory, absl::Duration control_delta_t,
    JointLimits joint_limits, std::unique_ptr<RigidBodyInterface> dynamics) {
  if (trajectory.data().empty()) {
    return absl::InvalidArgumentError(
        "TrajectoryPlayer::Create(): The given trajectory is empty!");
  }

  if (trajectory.data().front().position.size() != joint_limits.size()) {
    return absl::InvalidArgumentError(
        absl::StrCat("TrajectoryPlayer::Create(): ",
                     "Dimension mismatch between trajectory, which is ",
                     trajectory.data().front().position.size(),
                     " and joint_limits, which is ", joint_limits.size()));
  }

  // If the trajectory uses as interpolation type quintic polynomials, then the
  // trajectory has third order constraints (i.e. it has jerk-limits or
  // torque-rate-limits) and we can include a simple check based on
  // time-differences of consecutive states.
  const bool has_third_order_constraints =
      (trajectory.interpolation_type() ==
       JointTrajectoryInterpolationType::kQuinticPolynomial);
  // Flag to mark which limits should be checked, either kinematic limits (such
  // as joint accelerations and joint jerk limits), or dynamic limits (such as
  // joint torque or joint torque rate limits).
  const bool check_kinematic_limits =
      (trajectory.joint_dynamic_limits_check_mode() ==
       DynamicLimitsCheckMode::kCheckJointAcceleration);

  // Loop through discretized trajectory and check it against limits.
  for (size_t i = 0; i < trajectory.size(); ++i) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        auto limit_check_result,
        IsWithinLimits(trajectory.data()[i], joint_limits));
    if (!limit_check_result.p_ok) {
      return absl::InvalidArgumentError(absl::StrCat(
          "TrajectoryPlayer::Create(): ",
          "Provided joint trajectory violates joint position limits at index ",
          i));
    } else if (!limit_check_result.v_ok) {
      return absl::InvalidArgumentError(absl::StrCat(
          "TrajectoryPlayer::Create(): ",
          "Provided joint trajectory violates joint velocity limits at index ",
          i));
    } else if (!limit_check_result.a_ok && check_kinematic_limits) {
      return absl::InvalidArgumentError(absl::StrCat(
          "TrajectoryPlayer::Create(): ",
          "Provided joint trajectory violates joint acceleration limits at "
          "index ",
          i));
    }
  }

  // If the trajectory is torque-limited, we validate the dynamics model and
  // check that the input trajectory satisfies torque limits.
  if (!check_kinematic_limits) {
    INTR_RETURN_IF_ERROR(icon::ValidateRigidBodyInterface(
        dynamics.get(), trajectory.data().front().position,
        trajectory.data().front().velocity));
    // Safety percentage margin from the maximum system torque limits which is
    // unused in torque-limited speed override motions in order to have a small
    // room for numerical errors. This avoids adding an external numerical
    // margin to check the validity of the commands.
    const double kSafetyMarginFromTorqueLimits = 0.01;
    joint_limits.max_torque *= (1.0 - kSafetyMarginFromTorqueLimits);

    // Loop along the trajectory and check torque limits for each state.
    for (size_t i = 0; i < trajectory.size(); ++i) {
      const JointStatePVA& state = trajectory.data().at(i);

      JointStateT generalized_force;
      INTRINSIC_RT_RETURN_IF_ERROR(generalized_force.SetSize(state.size()));
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          generalized_force.torque,
          dynamics->ComputeInverseDynamics(state.position, state.velocity,
                                           state.acceleration));
      INTRINSIC_RT_ASSIGN_OR_RETURN(
          intrinsic::LimitCheckResult limit_check_dynamics,
          IsWithinLimits(generalized_force, joint_limits));
      if (!limit_check_dynamics.t_ok) {
        return absl::InvalidArgumentError(
            absl::StrCat("TrajectoryPlayer::Create(): Provided joint "
                         "trajectory violates joint torque limits at index ",
                         i, "."));
      }
    }
    if (has_third_order_constraints) {
      // TODO(b/402169795): Add a check for torque rate limits.
    }
  }

  if (trajectory.interpolation_type() ==
      JointTrajectoryInterpolationType::kUnspecified) {
    return absl::InvalidArgumentError(
        "Trajectory fine interpolation type is set to kUnspecified.");
  }

  // The speed override factor dynamics algorithm looks
  // `kStopCheckLookaheadCycles` into the future for any given state, to see if
  // it's possible to reach a full stop (for the speed override factor) until
  // then. A state is deemed controllable if it is possible to reach a full stop
  // in `kStopCheckLookaheadCycles`. A larger value allows faster transitions of
  // the speed override value, but also implies more computational effort.
  constexpr int kStopCheckLookaheadCycles = 18;
  const SpeedOverrideFactorDynamicsParams sof_dynamics_params =
      SpeedOverrideFactorDynamicsParams{
          .control_dt_seconds = absl::ToDoubleSeconds(control_delta_t),
          .stop_check_lookahead_cycles = kStopCheckLookaheadCycles,
          .control_scaling_factors = {1.0, 0.1},
          .enable_backward_differentiation_limit_checking = true};
  INTR_ASSIGN_OR_RETURN(
      SpeedOverrideFactorDynamics sof_dynamics,
      SpeedOverrideFactorDynamics::Create(sof_dynamics_params, trajectory,
                                          joint_limits, std::move(dynamics)));

  // Using absl::WrapUnique() below because of the private constructor of
  // TrajectoryPlayer.
  return absl::WrapUnique(new TrajectoryPlayer(
      std::move(trajectory), control_delta_t, std::move(joint_limits),
      std::move(sof_dynamics),
      CreateMotionPolynomialsWithConstantSpeedOverride(1.0)));
}

TrajectoryPlayer::TrajectoryPlayer(
    JointTrajectoryPVA trajectory, absl::Duration control_delta_t,
    JointLimits joint_limits, SpeedOverrideFactorDynamics sof_dynamics,
    reflexxes::MotionPolynomials initial_sof_motion_polynomials)
    : sof_dynamics_(std::move(sof_dynamics)),
      trajectory_(std::move(trajectory)),
      control_delta_t_(control_delta_t),
      joint_limits_(std::move(joint_limits)),
      speed_override_trajectory_(std::move(initial_sof_motion_polynomials)) {}

RealtimeStatus TrajectoryPlayer::SetDesiredSpeedOverrideFactor(
    double desired_speed_override_factor) {
  if (desired_speed_override_factor < kMinimumSpeedOverrideFactor ||
      desired_speed_override_factor > kMaximumSpeedOverrideFactor) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "desired_speed_override_factor should be in value range [",
        kMinimumSpeedOverrideFactor, ", ", kMaximumSpeedOverrideFactor,
        "], but ", "instead has value ", desired_speed_override_factor, "."));
  }

  target_sof_ = desired_speed_override_factor;
  return icon::OkStatus();
}

RealtimeStatus TrajectoryPlayer::ForceSpeedOverrideFactor(
    double speed_override_factor) {
  if (speed_override_factor < kMinimumSpeedOverrideFactor ||
      speed_override_factor > kMaximumSpeedOverrideFactor) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "speed_override_factor should be in value range [",
        kMinimumSpeedOverrideFactor, ", ", kMaximumSpeedOverrideFactor,
        "], but ", "instead has value ", speed_override_factor, "."));
  }

  target_sof_ = speed_override_factor;
  curr_state_ = {.sof = target_sof_, .dsof_dt = 0.0, .d2sof_dt2 = 0.0};
  next_state_ = curr_state_;
  speed_override_trajectory_ =
      CreateMotionPolynomialsWithConstantSpeedOverride(target_sof_);

  return icon::OkStatus();
}

double TrajectoryPlayer::GetSpeedOverrideFactor() const {
  return curr_state_.sof;
}

bool TrajectoryPlayer::SpeedOverrideStateHasConverged() const {
  return HasConvergedToTargetSpeedOverrideFactor(curr_state_, target_sof_);
}

RealtimeStatusOr<double> TrajectoryPlayer::GetCartesianArcLengthAtPhase(
    double phase) {
  if (phase < 0.0 || phase > 1.0) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "phase should be in value range [0, 1], but instead has value ", phase,
        "."));
  }

  if (!trajectory_.HasCartesianArcLength()) {
    return icon::FailedPreconditionError(
        "Trajectory does not have Cartesian arc length.");
  }

  const absl::Duration time_since_trajectory_start =
      phase * trajectory_.Duration();

  return InterpolateCartesianArcLength(trajectory_,
                                       time_since_trajectory_start);
}

RealtimeStatusOr<JointStatePVA> TrajectoryPlayer::GetStateAtPhase(
    double phase) {
  if (phase < 0.0 || phase > 1.0) {
    return icon::InvalidArgumentError(icon::RealtimeStatus::StrCat(
        "phase should be in value range [0, 1], but instead has value ", phase,
        "."));
  }

  const absl::Duration time = phase * trajectory_.Duration();
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      JointStatePVAJ state_without_speed_override,
      InterpolateJointTrajectoryInternalType(trajectory_, time));
  INTRINSIC_RT_RETURN_IF_ERROR(sof_dynamics_.UpdatePreviousState(
      /*next_position=*/state_without_speed_override.position));

  motion_polynomial_phase_ = phase;
  if (!SpeedOverrideStateHasConverged()) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        SpeedOverrideFactorDynamicsResult sof_dynamics_result,
        sof_dynamics_.GetNextState(
            curr_state_, {.sof = target_sof_, .dsof_dt = 0.0}, phase));
    next_state_ = std::move(sof_dynamics_result.next_state);
    next_phase_ = sof_dynamics_result.next_trajectory_phase;
    speed_override_trajectory_ =
        std::move(sof_dynamics_result.motion_polynomials);
  } else {
    next_phase_ = std::clamp(
        phase + curr_state_.sof * absl::FDivDuration(control_delta_t_,
                                                     trajectory_.Duration()),
        0.0, 1.0);
    speed_override_trajectory_ =
        CreateMotionPolynomialsWithConstantSpeedOverride(target_sof_);
  }

  // compute the speed-overridden next_state:
  const eigenmath::VectorNd& qp = state_without_speed_override.velocity;
  const eigenmath::VectorNd& qpp = state_without_speed_override.acceleration;

  JointStatePVA state;
  state.position = state_without_speed_override.position;
  // Compute the joint state's velocity and acceleration.
  state.velocity = curr_state_.sof * qp;
  // Please note that for computing state.acceleration, we use curr_state_.sof
  // and next_state_.dsof_dt. This is because next_state_.dsof_dt's bounds were
  // computed using curr_state_.sof. If we use next_state_.sof and
  // next_state_.dsof_dt instead, the computed state.acceleration result may
  // violate the joint limits, especially when the motion is nearby the limits.
  const double curr_sof_squared = ::intrinsic::IPow(curr_state_.sof, 2);
  state.acceleration = (curr_sof_squared * qpp) + (next_state_.dsof_dt * qp);

  if (trajectory_.joint_dynamic_limits_check_mode() ==
      DynamicLimitsCheckMode::kCheckJointAcceleration) {
    // TODO(b/251328525): accelerations are clamped due to overshoot introduced
    // by trajectory postprocessing.
    QCHECK(eigenmath::ClampVector(-joint_limits_.max_acceleration,
                                  joint_limits_.max_acceleration,
                                  state.acceleration));
  }

  // Finally, update the speed override factor state:
  curr_state_ = next_state_;
  // The speed override factor is clamped to its limits as it avoids small
  // numerical violations that would lead to failures.
  curr_state_.sof = std::clamp(curr_state_.sof, kMinimumSpeedOverrideFactor,
                               kMaximumSpeedOverrideFactor);

  return state;
}

RealtimeStatusOr<JointStatePVA>
TrajectoryPlayer::GetStateAtPhaseIfSpeedOverrideConvergedTestOnly(
    double phase) {
  if (!SpeedOverrideStateHasConverged()) {
    return icon::InternalError("Speed override factor has not converged yet!");
  }

  return GetStateAtPhase(phase);
}

RealtimeStatusOr<JointStatePVA> TrajectoryPlayer::GetNextState() {
  INTRINSIC_RT_ASSIGN_OR_RETURN(JointStatePVA next_state,
                                GetStateAtPhase(phase_));
  phase_ = next_phase_;
  return next_state;
}

RealtimeStatusOr<double> TrajectoryPlayer::GetCartesianArcLength() {
  return GetCartesianArcLengthAtPhase(phase_);
}

RealtimeStatusOr<double> TrajectoryPlayer::ComputeTimeToTargetPhase(
    double target_trajectory_phase) const {
  return GetSpeedOverrideAdjustedTimeToGo(
      /*current_trajectory_phase=*/motion_polynomial_phase_,
      target_trajectory_phase, speed_override_trajectory_,
      absl::ToDoubleSeconds(trajectory_.Duration()));
}

}  // namespace intrinsic::icon
