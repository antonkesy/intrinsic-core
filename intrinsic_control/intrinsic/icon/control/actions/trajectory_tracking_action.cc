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

#include "intrinsic/icon/control/actions/trajectory_tracking_action.h"

#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <utility>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "intrinsic/eigenmath/clamp.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/actions/trajectory_tracking_action.pb.h"
#include "intrinsic/icon/actions/trajectory_tracking_action_info.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/common/builtins.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/algorithms/is_settled_criterion.h"
#include "intrinsic/icon/control/algorithms/trajectory_player.h"
#include "intrinsic/icon/control/algorithms/trajectory_residual_controller.h"
#include "intrinsic/icon/control/joint_position_command.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/realtime_signal_types.h"
#include "intrinsic/icon/control/slot_types.h"
#include "intrinsic/icon/dynamics/rigid_body_interface.h"
#include "intrinsic/icon/dynamics/robotics_library_dynamics_creator.h"
#include "intrinsic/icon/proto/cart_space_conversion.h"
#include "intrinsic/icon/proto/generic_part_config.pb.h"
#include "intrinsic/icon/proto/joint_trajectory_conversion.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/proto/kinematics_conversion.h"
#include "intrinsic/kinematics/skeleton.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/kinematics/types/dynamic_limits_check_mode.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/kinematics/types/joint_trajectories.h"
#include "intrinsic/motion_planning/trajectory_planning/trajectory_utils.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

namespace {

using ::intrinsic_proto::icon::v1::BehaviorOverrideRequest;

// Returns `true` if the trajectory is jerk-limited and false otherwise. The
// interpolation type is used to identify if the trajectory is
// acceleration-limited (kCubicPolynomial) or jerk-limited (kQuinticPolynomial).
bool IsTrajectoryJerkLimited(const JointTrajectoryPVA& trajectory) {
  return trajectory.interpolation_type() ==
         JointTrajectoryInterpolationType::kQuinticPolynomial;
}

// Designs the initial state residual for the `trajectory` and
// `previous_joint_setpoint` such that it guarantees continuity of the state. In
// general, this is computed as the difference between the
// `previous_joint_setpoint` and the `trajectory` initial state.
// - If the trajectory is jerk-limited, then the initial state residual is set
//   to the difference between the previous setpoint and the initial state
//   of the trajectory such that the acceleration is continuous.
// - If the trajectory is acceleration-limited, we consider continuity of the PV
// state only. The initial acceleration residual is set to zero.
icon::RealtimeStatusOr<JointStatePVA> DesignInitialStateResidual(
    const JointTrajectoryPVA& trajectory,
    const JointStatePVA& previous_joint_setpoint) {
  if (trajectory.size() == 0) {
    return icon::InvalidArgumentError(
        "TrajectoryTrackingAction requires a non-empty trajectory.");
  }
  if (previous_joint_setpoint.size() != trajectory.data().front().size()) {
    return icon::InvalidArgumentError(absl::StrCat(
        "Dimension mismatch for trajectory (", trajectory.data().front().size(),
        ") and setpoint (", previous_joint_setpoint.size(), ")."));
  }

  JointStatePVA initial_state_residual;
  INTRINSIC_RT_RETURN_IF_ERROR(
      initial_state_residual.SetSize(previous_joint_setpoint.size()));
  INTRINSIC_RT_ASSIGN_OR_RETURN(JointStatePVA trajectory_initial_state,
                                trajectory.DataAt(0));

  initial_state_residual.position =
      previous_joint_setpoint.position - trajectory_initial_state.position;
  initial_state_residual.velocity =
      previous_joint_setpoint.velocity - trajectory_initial_state.velocity;
  if (IsTrajectoryJerkLimited(trajectory)) {
    initial_state_residual.acceleration = previous_joint_setpoint.acceleration -
                                          trajectory_initial_state.acceleration;
  }
  return initial_state_residual;
}

}  // namespace

// static
absl::StatusOr<TrajectoryTrackingAction::Params>
TrajectoryTrackingAction::Params::FromProto(
    const intrinsic_proto::icon::actions::TrajectoryTrackingActionFixedParams&
        proto_params) {
  INTRINSIC_ASSERT_NON_REALTIME();
  TrajectoryTrackingAction::Params params;
  INTR_ASSIGN_OR_RETURN(params.trajectory,
                        intrinsic::FromProto(proto_params.trajectory()));

  return params;
}

TrajectoryTrackingAction::TrajectoryTrackingAction(
    RealtimeSlotId slot_id, std::unique_ptr<TrajectoryPlayer> trajectory_player,
    std::unique_ptr<TrajectoryResidualController>
        trajectory_residual_controller,
    std::unique_ptr<IsSettledCriterion> is_settled_criterion,
    RealtimeSignalId path_accurate_stop_signal_id,
    const JointLimits& planning_limits, const JointLimits& system_limits)
    : slot_id_(slot_id),
      trajectory_player_(std::move(trajectory_player)),
      residual_controller_(std::move(trajectory_residual_controller)),
      planning_limits_(planning_limits),
      system_limits_(system_limits),
      is_settled_criterion_(std::move(is_settled_criterion)),
      path_accurate_stop_signal_id_(path_accurate_stop_signal_id) {}

// static
absl::StatusOr<std::unique_ptr<TrajectoryTrackingAction>>
TrajectoryTrackingAction::Create(
    const TrajectoryTrackingActionInfo::FixedParams& params_proto,
    ActionFactoryContext& context) {
  INTRINSIC_ASSERT_NON_REALTIME();
  INTR_ASSIGN_OR_RETURN(
      SlotInfo arm_info,
      context.GetSlotInfo(TrajectoryTrackingActionInfo::kSlotName));
  const ::intrinsic_proto::icon::GenericJointLimitsConfig& joint_limits_config =
      arm_info.config.generic_config().joint_limits_config();

  if (!arm_info.config.generic_config().has_joint_limits_config()) {
    return absl::InvalidArgumentError(
        "TrajectoryTrackingAction requires joint limits.");
  }

  // TODO(b/234426024): there will be no headroom for controlling towards the
  // desired trajectory if the provided trajectory already saturates ICON system
  // limits. Decide on b/234426024 and use application limits as planning
  // limits here.
  INTR_ASSIGN_OR_RETURN(
      JointLimits planning_limits,
      ::intrinsic::FromProto(joint_limits_config.application_limits()));
  INTR_ASSIGN_OR_RETURN(
      JointLimits system_limits,
      ::intrinsic::FromProto(joint_limits_config.system_limits()));

  if (!params_proto.has_trajectory()) {
    return absl::InvalidArgumentError(
        "TrajectoryTrackingAction requires a trajectory.");
  }
  INTR_ASSIGN_OR_RETURN(Params command, Params::FromProto(params_proto));
  if (command.trajectory.data().front().position.size() !=
      system_limits.size()) {
    return absl::InvalidArgumentError(
        absl::StrCat("Dimension mismatch between trajectory, which is ",
                     command.trajectory.data().front().position.size(),
                     " and limits, which is ", system_limits.size()));
  }

  // Loop through discretized trajectory and check it against limits.
  for (size_t i = 0; i < command.trajectory.size(); ++i) {
    const JointStatePVA& state = command.trajectory.data().at(i);
    INTRINSIC_RT_ASSIGN_OR_RETURN(auto limit_check_result,
                                  IsWithinLimits(state, system_limits));
    if (!limit_check_result.p_ok) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Provided joint trajectory violates joint position limits at index ",
          i));
    } else if (!limit_check_result.v_ok) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Provided joint trajectory violates joint velocity limits at index ",
          i));
    } else if (!limit_check_result.a_ok &&
               command.trajectory.joint_dynamic_limits_check_mode() ==
                   DynamicLimitsCheckMode::kCheckJointAcceleration) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Provided joint trajectory violates joint acceleration limits at "
          "index ",
          i));
    }
  }

  // Check the trajectory against Cartesian limits if possible.
  if (arm_info.config.generic_config().has_cartesian_limits_config() &&
      arm_info.config.generic_config().has_manipulator_kinematics_config()) {
    INTR_ASSIGN_OR_RETURN(CartesianLimits cartesian_limits,
                          icon::FromProto(arm_info.config.generic_config()
                                              .cartesian_limits_config()
                                              .default_cartesian_limits()));
    INTR_ASSIGN_OR_RETURN(
        std::unique_ptr<intrinsic::kinematics::Skeleton> skeleton,
        intrinsic::kinematics::FromProto(arm_info.config.generic_config()
                                             .manipulator_kinematics_config()
                                             .skeleton()));
    INTR_ASSIGN_OR_RETURN(kinematics::Chain chain,
                          kinematics::ExtractNonBranchingChain(*skeleton));
    INTR_RETURN_IF_ERROR(intrinsic::IsTrajectoryWithinCartesianPositionLimits(
        chain, cartesian_limits, command.trajectory));
  }

  // If the commanded trajectory is torque-limited, we construct a rigid body
  // interface for the trajectory player to be able to play it while respecting
  // torque-limits.
  std::unique_ptr<icon::RigidBodyInterface> rigid_body_interface = nullptr;
  if (command.trajectory.joint_dynamic_limits_check_mode() ==
          DynamicLimitsCheckMode::kCheckNone &&
      arm_info.config.generic_config().has_manipulator_kinematics_config()) {
    INTR_ASSIGN_OR_RETURN(
        std::unique_ptr<intrinsic::kinematics::Skeleton> skeleton,
        intrinsic::kinematics::FromProto(arm_info.config.generic_config()
                                             .manipulator_kinematics_config()
                                             .skeleton()));
    INTR_ASSIGN_OR_RETURN(
        rigid_body_interface,
        icon::CreateRoboticsLibraryDynamics(std::move(skeleton)));
  }

  // Store the commanded trajectory in the ActionFactoryContext. This allows an
  // ICON client to retrieve it via the ICON client API using an
  // ActionInstanceId.
  context.StoreTrajectory(command.trajectory);

  INTR_ASSIGN_OR_RETURN(std::unique_ptr<TrajectoryResidualController>
                            trajectory_residual_controller,
                        TrajectoryResidualController::Create(
                            context.ServerConfig().frequency_hz(),
                            planning_limits, system_limits));

  INTR_ASSIGN_OR_RETURN(
      std::unique_ptr<TrajectoryPlayer> trajectory_player,
      TrajectoryPlayer::Create(
          std::move(command.trajectory),
          absl::Seconds(1. / context.ServerConfig().frequency_hz()),
          system_limits, std::move(rigid_body_interface)));

  INTR_ASSIGN_OR_RETURN(
      auto is_settled_criterion,
      IsSettledCriterion::Create(context.ServerConfig().frequency_hz()));

  INTR_ASSIGN_OR_RETURN(
      RealtimeSignalId path_accurate_stop_signal_id,
      context.GetRealtimeSignalId(
          TrajectoryTrackingActionInfo::kSignalPathAccurateStop));

  return std::make_unique<TrajectoryTrackingAction>(
      arm_info.slot_id, std::move(trajectory_player),
      std::move(trajectory_residual_controller),
      std::move(is_settled_criterion), path_accurate_stop_signal_id,
      planning_limits, system_limits);
}

RealtimeStatus TrajectoryTrackingAction::OnEnter(OnEnterParameters params) {
  // The final state residual will be computed and used after the trajectory has
  // been finished.
  final_state_residual_ = std::nullopt;

  trajectory_player_->ResetPhase();
  // Initialize the trajectory player to currently active speed override
  // factors. This allows consistent Action switching despite speed overriding
  // being active.
  INTRINSIC_RT_RETURN_IF_ERROR(
      trajectory_player_->ForceSpeedOverrideFactor(params.speed_override));

  const JointPosition* const joint_position_interface =
      params.slot_map.GetInterfaceForSlot<JointPosition>(slot_id_);
  if (joint_position_interface == nullptr) {
    return InternalError("Slot doesn't have JointPosition.");
  }

  const JointPositionCommand& previous_position_setpoints =
      joint_position_interface->PreviousPositionSetpoints();
  const int ndof = previous_position_setpoints.position().size();

  JointStatePVA previous_joint_setpoint;
  INTRINSIC_RT_RETURN_IF_ERROR(previous_joint_setpoint.SetSize(ndof));
  previous_joint_setpoint.position = previous_position_setpoints.position();

  // If there is no velocity feedforward available in the previous command, we
  // set the starting velocity to the sensed velocity. This should only happen
  // once, in the very first control cycle of this action.
  const auto* sensed_velocity =
      params.slot_map.GetInterfaceForSlot<JointVelocityEstimator>(slot_id_);
  if (sensed_velocity == nullptr) {
    return InternalError("Slot does not have JointVelocityEstimator");
  }
  previous_joint_setpoint.velocity =
      previous_position_setpoints.velocity_feedforward().value_or(
          sensed_velocity->GetVelocityEstimate().velocity);

  // If there is no acceleration feedforward available in the previous command,
  // we set the starting acceleration to zero. This should only happen once, in
  // the very first control cycle of this action.
  // TODO(b/370698109): this can lead to very high jerk values and potentially
  // jerk limit violation.
  previous_joint_setpoint.acceleration =
      previous_position_setpoints.acceleration_feedforward().value_or(
          eigenmath::VectorNd::Zero(ndof));

  // Set the initial state residual.
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      initial_state_residual_,
      DesignInitialStateResidual(trajectory_player_->GetTrajectory(),
                                 previous_joint_setpoint));

  if (initial_state_residual_.position.cwiseAbs().maxCoeff() >
      TrajectoryTrackingActionInfo::kMaxInitialJointPositionDeviation) {
    return FailedPreconditionError(RealtimeStatus::StrCat(
        "Difference between previous setpoint and position command (",
        initial_state_residual_.position.cwiseAbs().maxCoeff(), ") exceeds ",
        TrajectoryTrackingActionInfo::kMaxInitialJointPositionDeviation));
  }

  if (initial_state_residual_.velocity.cwiseAbs().maxCoeff() >
      TrajectoryTrackingActionInfo::kMaxInitialJointVelocityDeviation) {
    return FailedPreconditionError(RealtimeStatus::StrCat(
        "Difference between previous setpoint and velocity command (",
        initial_state_residual_.velocity.cwiseAbs().maxCoeff(), ") exceeds ",
        TrajectoryTrackingActionInfo::kMaxInitialJointVelocityDeviation));
  }

  // Reset state variables.
  distance_to_final_setpoint_ = std::numeric_limits<double>::max();
  trajectory_done_for_seconds_ = 0.0;
  wall_time_passed_since_trajectory_start_ = 0.0;
  is_settled_ = false;
  INTRINSIC_RT_RETURN_IF_ERROR(is_settled_criterion_->Initialize());

  return OkStatus();
}

RealtimeStatus TrajectoryTrackingAction::Sense(SenseParameters params) {
  // Update distance to final setpoint.
  const auto* sensed_joint_position =
      params.slot_map.GetInterfaceForSlot<JointPositionSensor>(slot_id_);
  if (sensed_joint_position == nullptr) {
    return InternalError("Slot does not have JointPositionSensor");
  }
  distance_to_final_setpoint_ =
      (sensed_joint_position->GetSensedPosition().position -
       trajectory_player_->GetTrajectory().data().back().position)
          .norm();
  if (IsDone()) {
    INTRINSIC_RT_LOG_THROTTLED(INFO)
        << "Trajectory done for " << trajectory_done_for_seconds_
        << " seconds.";
    trajectory_done_for_seconds_ +=
        absl::ToDoubleSeconds(trajectory_player_->GetControlSamplingTime());
  }
  wall_time_passed_since_trajectory_start_ +=
      absl::ToDoubleSeconds(trajectory_player_->GetControlSamplingTime());
  // Update the settled state estimator with new measurements.
  const JointPosition* position_command =
      params.slot_map.GetInterfaceForSlot<JointPosition>(slot_id_);
  if (position_command == nullptr) {
    return FailedPreconditionError(
        "Slot does not have JointPosition feature interface");
  }

  CHECK(position_command->PreviousPositionSetpoints()
            .velocity_feedforward()
            .has_value());
  const auto* sensed_velocity =
      params.slot_map.GetInterfaceForSlot<JointVelocityEstimator>(slot_id_);
  if (sensed_velocity == nullptr) {
    return InternalError("Slot does not have JointVelocityEstimator");
  }
  INTRINSIC_RT_ASSIGN_OR_RETURN(auto isDoneVariant, GetStateVariable(kIsDone));
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      is_settled_, is_settled_criterion_->Update(
                       sensed_velocity->GetVelocityEstimate().velocity,
                       position_command->PreviousPositionSetpoints()
                           .velocity_feedforward()
                           .value(),
                       /*has_trajectory_ended=*/std::get<bool>(isDoneVariant)));
  if (is_settled_) {
    INTRINSIC_RT_LOG_THROTTLED(INFO) << "Trajectory settled.";
  }
  // Listen for path accurate stop request and store signal. Signal will only be
  // evaluated in Control() to avoid race conditions between Sense() and the
  // evaluation of state variables.
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      path_accurate_stop_signal_value_,
      params.signal_access.ReadSignal(path_accurate_stop_signal_id_));

  return icon::OkStatus();
}

RealtimeStatus TrajectoryTrackingAction::Control(ControlParameters params) {
  JointPosition* const joint_position_interface =
      params.slot_map.GetMutableInterfaceForSlot<JointPosition>(slot_id_);
  if (joint_position_interface == nullptr) {
    return InternalError("Slot doesn't have JointPosition.");
  }

  // Evaluate path accurate stop signal. This needs to happen in Control() to
  // avoid race conditions between Sense() and state variable evaluation. The
  // path accurate stop can only be triggered once, currently it is impossible
  // to recover from it (and resume a motion).
  const bool path_accurate_stop_requested =
      path_accurate_stop_requested_ ||
      path_accurate_stop_signal_value_.current_value;
  // Logs once exactly when the path accurate stop is requested.
  if (path_accurate_stop_requested_ != path_accurate_stop_requested) {
    const double trajectory_phase = trajectory_player_->GetPhase();
    INTRINSIC_RT_LOG(INFO)
        << "Path accurate stop requested at trajectory phase "
        << trajectory_phase;
  }
  path_accurate_stop_requested_ = path_accurate_stop_requested;

  // Pause can be requested and cleared.
  bool pause_requested_ = false;
  if (params.requested_behavior_override ==
      BehaviorOverrideRequest::BEHAVIOR_OVERRIDE_REQUEST_PAUSE) {
    INTRINSIC_RT_LOG_THROTTLED(INFO) << "PAUSE requested at trajectory phase "
                                     << trajectory_player_->GetPhase();
    pause_requested_ = true;
  }

  // If a path accurate stop is requested, the speed override is overwritten to
  // be zero.
  const double combined_speed_override_factor =
      params.speed_override *
      static_cast<double>(!path_accurate_stop_requested_) *
      static_cast<double>(!pause_requested_);

  INTRINSIC_RT_RETURN_IF_ERROR(
      trajectory_player_->SetDesiredSpeedOverrideFactor(
          combined_speed_override_factor));

  // TODO(b/370891053): kIsDone should also account for the internal state of
  // the residual controller.
  const bool is_done = IsDone();

  // Evaluate the new reference state according to the trajectory.
  INTRINSIC_RT_ASSIGN_OR_RETURN(JointStatePVA new_trajectory_reference_state,
                                trajectory_player_->GetNextState());
  if (is_done) {
    // We need to account for the fact that the commanded trajectory may not
    // exactly end with zero velocity or acceleration. Since we should
    // effectively reach a steady-state if there is no follow-up action, we need
    // to actively enforce zero velocity and acceleration once the full
    // trajectory has been played back and is therefore "done". The residual
    // controller is then used to eliminate a "final" residual, e.g. a non-zero
    // terminal velocity/acceleration in the trajectory. We initialize the final
    // residual controller once.
    if (!final_state_residual_.has_value()) {
      final_state_residual_ = JointStatePVA();
      INTRINSIC_RT_RETURN_IF_ERROR(final_state_residual_->SetSize(
          new_trajectory_reference_state.size()));
      // We would like to converge to the position of the last trajectory
      // datapoint, so the final residual position just needs to take over the
      // remaining "initial" position residual.
      final_state_residual_->position = initial_state_residual_.position;
      // Start from the final velocity and acceleration setpoint interpolated
      // from the trajectory, and add the remaining residual from the initial
      // state. This total residual will subsequently be brought to zero using
      // the full planning limits.
      final_state_residual_->velocity =
          new_trajectory_reference_state.velocity +
          initial_state_residual_.velocity;
      if (IsTrajectoryJerkLimited(trajectory_player_->GetTrajectory())) {
        final_state_residual_->acceleration =
            new_trajectory_reference_state.acceleration +
            initial_state_residual_.acceleration;
      } else {
        final_state_residual_->acceleration.setZero();
      }
    }
    // New reference velocity and acceleration get set to zero. This needs to
    // happen after the initialization of the final_state_residual_.
    new_trajectory_reference_state.velocity.setZero();
    new_trajectory_reference_state.acceleration.setZero();
  }

  JointStatePVA filtered_reference = new_trajectory_reference_state;
  // The actual command is a superposition of reference state and residual.
  // Update and include initial state residual as long as the trajectory has not
  // been played back to its end.
  if (!final_state_residual_.has_value()) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        initial_state_residual_,
        residual_controller_->ComputeResidualUsingLimitMargin(
            initial_state_residual_));
    filtered_reference.position += initial_state_residual_.position;
    filtered_reference.velocity += initial_state_residual_.velocity;
    filtered_reference.acceleration += initial_state_residual_.acceleration;
  } else {
    // Final state of the trajectory has been reached, update and include final
    // state residual. Since the residual compensation remains the only motion,
    // use planning limits.
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        final_state_residual_,
        residual_controller_->ComputeResidualUsingPlanningLimits(
            *final_state_residual_));
    filtered_reference.position += final_state_residual_->position;
    filtered_reference.velocity += final_state_residual_->velocity;
    filtered_reference.acceleration += final_state_residual_->acceleration;
  }

  // Polynomial splining (as used by the trajectory player) may lead to slight
  // oscillations at the acceleration level, which might sometimes surpass the
  // maximum acceleration limits. This is not a critical failure, can be
  // tolerated, and trajectory execution may proceed as intended. To avoid a
  // failure at the HAL level (which strictly checks the acceleration), we
  // saturate the joint acceleration command ONLY for acceleration-limited
  // trajectories. In jerk-limited trajectories, this is not enforced as it
  // could induce an actual jerk-limit violation.
  // TODO(b/371952663): Remove once fine interpolation for acceleration limited
  // trajectories is limit-aware at the acceleration level.
  if (!IsTrajectoryJerkLimited(trajectory_player_->GetTrajectory())) {
    if (!eigenmath::ClampVector(-system_limits_.max_acceleration,
                                system_limits_.max_acceleration,
                                filtered_reference.acceleration)) {
      return InternalError("Clamping joint accelerations to limits failed.");
    }
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      JointPositionCommand new_joint_position_command,
      JointPositionCommand::Create(filtered_reference.position,
                                   filtered_reference.velocity,
                                   filtered_reference.acceleration,
                                   trajectory_player_->GetTrajectory()
                                       .joint_dynamic_limits_check_mode()));

  return joint_position_interface->SetPositionSetpoints(
      new_joint_position_command);
}

bool TrajectoryTrackingAction::IsDone() const {
  if (path_accurate_stop_requested_ &&
      trajectory_player_->SpeedOverrideStateHasConverged()) {
    return true;
  }

  constexpr double kDoubleEqThreshold = 1.0e-14;
  const double phase = trajectory_player_->GetPhase();
  return 1.0 - phase < kDoubleEqThreshold;
}

RealtimeStatusOr<StateVariableValue> TrajectoryTrackingAction::GetStateVariable(
    absl::string_view name) const {
  if (name == kIsDone) {
    return StateVariableValue(IsDone());
  }
  if (name == TrajectoryTrackingActionInfo::kIsSettled) {
    return StateVariableValue(is_settled_);
  }
  if (name == TrajectoryTrackingActionInfo::kIsSettledUncertainty) {
    return StateVariableValue(is_settled_criterion_->GetUncertainty());
  }
  if (name == TrajectoryTrackingActionInfo::kTrajectoryProgress) {
    return StateVariableValue(trajectory_player_->GetPhase());
  }
  if (name == TrajectoryTrackingActionInfo::kTrajectoryDoneForSeconds) {
    return StateVariableValue(trajectory_done_for_seconds_);
  }
  if (name == TrajectoryTrackingActionInfo::kDistanceToFinalSetpoint) {
    return StateVariableValue(distance_to_final_setpoint_);
  }
  if (name == TrajectoryTrackingActionInfo::kTimeSinceTrajectoryStartSeconds) {
    return StateVariableValue(wall_time_passed_since_trajectory_start_);
  }
  if (name ==
      TrajectoryTrackingActionInfo::kCartesianArcLengthAlongTrajectoryMeters) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(double cart_arc_length,
                                  trajectory_player_->GetCartesianArcLength());
    return StateVariableValue(cart_arc_length);
  }
  return NotFoundError(RealtimeStatus::StrCat(
      "TrajectoryTrackingAction, state variable not found ", name));
}

}  // namespace intrinsic::icon
