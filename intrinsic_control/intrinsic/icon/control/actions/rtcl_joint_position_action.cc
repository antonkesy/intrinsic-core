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

#include "intrinsic/icon/control/actions/rtcl_joint_position_action.h"

#include <cstddef>
#include <memory>
#include <optional>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/actions/point_to_point_move_info.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/common/builtins.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/algorithms/is_settled_criterion.h"
#include "intrinsic/icon/control/joint_position_command.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/slot_types.h"
#include "intrinsic/icon/proto/cart_space_conversion.h"
#include "intrinsic/icon/proto/eigen_conversion.h"
#include "intrinsic/icon/proto/generic_part_config.pb.h"
#include "intrinsic/icon/proto/joint_space.pb.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/reflexxes/flags.h"
#include "intrinsic/icon/reflexxes/position_flags.h"
#include "intrinsic/icon/reflexxes/status.h"
#include "intrinsic/icon/utils/fixed_str_cat.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/chain.h"
#include "intrinsic/kinematics/proto/kinematics_conversion.h"
#include "intrinsic/kinematics/skeleton.h"
#include "intrinsic/kinematics/state.h"
#include "intrinsic/kinematics/types/cartesian_limits.h"
#include "intrinsic/kinematics/types/check_cartesian_limits.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {
JointPositionAction::JointPositionAction(
    RealtimeSlotId slot_id, size_t ndof, double frequency_hz, Params params,
    std::unique_ptr<IsSettledCriterion> is_settled_criterion)
    : slot_id_(slot_id),
      ndof_(ndof),
      params_(std::move(params)),
      trajectory_generator_position_(ndof, frequency_hz),
      trajectory_generator_velocity_(ndof, frequency_hz),
      frequency_hz_(frequency_hz),
      is_settled_criterion_(std::move(is_settled_criterion)) {
  trajectory_generator_position_.SetSelection(
      eigenmath::VectorNb::Constant(ndof, 1, true));
  trajectory_generator_velocity_.SetSelection(
      eigenmath::VectorNb::Constant(ndof, 1, true));
  trajectory_generator_position_.SetBehaviorIfInitialStateBreachesConstraints(
      reflexxes::PositionFlags::BehaviorIfInitialStateBreachesConstraints::
          kGetIntoBoundariesAtZeroAcceleration);
}

absl::StatusOr<JointPositionAction::Params> FromProto(
    const PointToPointMoveInfo::FixedParams& param_proto,
    const ::intrinsic_proto::icon::GenericPartConfig& part_config) {
  if (!param_proto.has_goal_position()) {
    return absl::InvalidArgumentError(
        "The PointToPointMoveFixedParams must contain the `goal_position`.");
  }
  if (!param_proto.has_goal_velocity()) {
    return absl::InvalidArgumentError(
        "The PointToPointMoveFixedParams must contain the `goal_velocity`.");
  }

  size_t ndof = part_config.joint_position_config().num_joints();
  if (ndof != param_proto.goal_position().joints_size() ||
      ndof != param_proto.goal_velocity().joints_size()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Mismatch with configured ndof: ", ndof,
        " goal_position size: ", param_proto.goal_position().joints_size(),
        " goal_velocity size: ", param_proto.goal_velocity().joints_size()));
  }
  JointPositionAction::Params params;
  INTR_ASSIGN_OR_RETURN(params.joint_goal.position,
                        FromProto(param_proto.goal_position()));
  INTR_ASSIGN_OR_RETURN(params.joint_goal.velocity,
                        FromProto(param_proto.goal_velocity()));

  if (param_proto.has_joint_limits()) {
    INTR_ASSIGN_OR_RETURN(params.joint_limits,
                          intrinsic::FromProto(param_proto.joint_limits()));

    INTR_ASSIGN_OR_RETURN(
        JointLimits application_limits,
        ::intrinsic::FromProto(
            part_config.joint_limits_config().application_limits()));
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        LimitCheckResult is_within_limits,
        IsWithinLimits(params.joint_limits, application_limits));
    if (!is_within_limits) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Provided joint_limits (", ::intrinsic::ToProto(params.joint_limits),
          ") violate application limits (",
          ::intrinsic::ToProto(application_limits),
          "): ", ToFixedString(is_within_limits)));
    }
  } else {
    INTR_ASSIGN_OR_RETURN(
        params.joint_limits,
        ::intrinsic::FromProto(
            part_config.joint_limits_config().application_limits()));
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      bool is_within_limits,
      IsWithinLimits(params.joint_goal, params.joint_limits));
  if (!is_within_limits) {
    return absl::InvalidArgumentError("Goal violates Part limits.");
  }

  return params;
}

// static
absl::StatusOr<std::unique_ptr<JointPositionAction>>
JointPositionAction::Create(
    const PointToPointMoveInfo::FixedParams& param_proto,
    ActionFactoryContext& context) {
  INTR_ASSIGN_OR_RETURN(SlotInfo slot_info,
                        context.GetSlotInfo(PointToPointMoveInfo::kSlotName));
  const ::intrinsic_proto::icon::GenericPartConfig& part_config =
      slot_info.config.generic_config();

  INTR_ASSIGN_OR_RETURN(Params params, FromProto(param_proto, part_config));
  size_t ndof = part_config.joint_position_config().num_joints();

  // Check the goal against Cartesian limits if possible.
  if (slot_info.config.generic_config().has_cartesian_limits_config() &&
      slot_info.config.generic_config().has_manipulator_kinematics_config()) {
    INTR_ASSIGN_OR_RETURN(CartesianLimits cartesian_limits,
                          FromProto(slot_info.config.generic_config()
                                        .cartesian_limits_config()
                                        .default_cartesian_limits()));
    INTR_ASSIGN_OR_RETURN(
        std::unique_ptr<intrinsic::kinematics::Skeleton> skeleton,
        intrinsic::kinematics::FromProto(slot_info.config.generic_config()
                                             .manipulator_kinematics_config()
                                             .skeleton()));
    INTR_ASSIGN_OR_RETURN(kinematics::Chain chain,
                          kinematics::ExtractNonBranchingChain(*skeleton));

    kinematics::State state(&chain);
    INTRINSIC_RT_RETURN_IF_ERROR(
        state.SetDofPositions(params.joint_goal.position));
    INTRINSIC_RT_ASSIGN_OR_RETURN(Pose3d tip_pose,
                                  state.GetTransform(chain.GetTipId()));

    if (!IsWithinLimits(tip_pose, cartesian_limits)) {
      return absl::InvalidArgumentError(
          "Provided joint target violates Cartesian position limits.");
    }
  }

  INTR_ASSIGN_OR_RETURN(
      auto is_settled_criterion,
      IsSettledCriterion::Create(context.ServerConfig().frequency_hz()));

  return std::make_unique<JointPositionAction>(
      slot_info.slot_id, ndof, context.ServerConfig().frequency_hz(),
      std::move(params), std::move(is_settled_criterion));
}

RealtimeStatus JointPositionAction::OnEnter(OnEnterParameters params) {
  // Unset previous_setpoint_ and distance_to_target_ – they will be populated
  // in Sense().
  previous_setpoint_ = std::nullopt;
  done_buffer_ = false;
  state_variables_ = std::nullopt;

  trajectory_generator_position_.SetPositionalLimitsBehavior(
      reflexxes::Flags::PositionalLimitsBehavior::kIgnore);
  if (!trajectory_generator_position_.SetLimits(params_.joint_limits)) {
    return InternalError("Failed to set position reflexxes limits.");
  }
  if (!trajectory_generator_velocity_.SetLimits(params_.joint_limits)) {
    return InternalError("Failed to set velocity reflexxes limits.");
  }

  current_goal_ = params_.joint_goal;
  if (!trajectory_generator_position_.SetTarget(current_goal_.value())) {
    return InternalError("Failed to set reflexxes position target.");
  }
  JointStateV zero_velocity;
  INTRINSIC_RT_RETURN_IF_ERROR(zero_velocity.SetSize(ndof_));
  zero_velocity.velocity.setConstant(0.);
  if (!trajectory_generator_velocity_.SetTarget(zero_velocity)) {
    return InternalError("Failed to set reflexxes velocity target.");
  }

  // Ignoring joint position limits so that stopping is attempted when starting
  // the action outside of the limits, or if the stop trajectory ends outside of
  // the limits. Such a violation needs to be handled by the part or safety
  // implementation.
  trajectory_generator_velocity_.SetPositionalLimitsBehavior(
      reflexxes::Flags::PositionalLimitsBehavior::kIgnore);

  // Initialize the criterion to determine the settled state.
  INTRINSIC_RT_RETURN_IF_ERROR(is_settled_criterion_->Initialize());

  return OkStatus();
}

RealtimeStatus JointPositionAction::Sense(SenseParameters params) {
  if (!current_goal_.has_value()) {
    return FailedPreconditionError(
        "Called Sense() when Action has no goal. SetParameters() "
        "must be called once before Sense() or Control() to set a goal.");
  }

  const JointPosition* const position_interface =
      params.slot_map.GetInterfaceForSlot<JointPosition>(slot_id_);
  if (position_interface == nullptr) {
    return InternalError("Slot doesn't have JointPosition.");
  }
  const JointPositionSensor* const position_sensor =
      params.slot_map.GetInterfaceForSlot<JointPositionSensor>(slot_id_);
  if (position_sensor == nullptr) {
    return InternalError("Slot doesn't have JointPositionSensor.");
  }
  const JointVelocityEstimator* const velocity_estimator =
      params.slot_map.GetInterfaceForSlot<JointVelocityEstimator>(slot_id_);
  if (velocity_estimator == nullptr) {
    return InternalError("Slot doesn't have JointVelocityEstimator.");
  }
  const JointAccelerationEstimator* const acceleration_estimator =
      params.slot_map.GetInterfaceForSlot<JointAccelerationEstimator>(slot_id_);

  // If previous_setpoint_ is not set yet, assume the current sensed position as
  // the last target.
  if (!previous_setpoint_.has_value()) {
    // Initialize previous_setpoint_ with the previous setpoint as reported by
    // the part.
    //
    // We don't read this every cycle, because we know what command we sent last
    // cycle.
    JointPositionCommand previous_command =
        position_interface->PreviousPositionSetpoints();
    JointStatePVA previous_setpoint;
    INTRINSIC_RT_RETURN_IF_ERROR(previous_setpoint.SetSize(ndof_));
    previous_setpoint.position = previous_command.position();
    previous_setpoint.velocity =
        previous_command.velocity_feedforward().value_or(
            velocity_estimator->GetVelocityEstimate().velocity);
    previous_setpoint.acceleration.setConstant(0.0);
    if (previous_command.velocity_feedforward().has_value()) {
      previous_setpoint.acceleration =
          previous_command.acceleration_feedforward().value();
    } else if (acceleration_estimator != nullptr) {
      previous_setpoint.acceleration =
          acceleration_estimator->GetAccelerationEstimate().acceleration;
    }
    previous_setpoint_ = previous_setpoint;
  }

  // If state_variables_ is not set yet, initialize it
  if (!state_variables_.has_value()) {
    state_variables_ = StateVariables();
  } else {
    // If the state variables are already initialized, update the time. We don't
    // do this unconditionally to ensure there is one cycle where
    // seconds_since_start is 0.
    state_variables_->seconds_since_start += 1 / frequency_hz_;
  }

  // Update remaining state variables regardless of whether they were just
  // initialized or not.
  if (!state_variables_->done && done_buffer_) {
    state_variables_->seconds_when_done = state_variables_->seconds_since_start;
    state_variables_->done = true;
  }

  if (current_goal_.has_value()) {
    state_variables_->distance_to_target =
        (position_sensor->GetSensedPosition().position -
         current_goal_->position)
            .norm();
  }

  // Update the settled state estimator with new measurements.
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      state_variables_->settled,
      is_settled_criterion_->Update(
          velocity_estimator->GetVelocityEstimate().velocity,
          previous_setpoint_->velocity,
          /*has_trajectory_ended=*/state_variables_->done));

  return OkStatus();
}

RealtimeStatus JointPositionAction::Control(ControlParameters params) {
  if (!current_goal_.has_value()) {
    return FailedPreconditionError(
        "JointPositionAction::Control() called when no goal is set. "
        "SetParameters() must be called once before Sense() or "
        "Control() to set a goal.");
  }
  if (!previous_setpoint_.has_value()) {
    return InternalError(
        "Previous target is unknown. Did you forget to call Sense() before "
        "Control()?");
  }
  if (!trajectory_generator_position_.SetPrevious(*previous_setpoint_)) {
    return InternalError("Failed to set previous reflexxes position target.");
  }
  if (!trajectory_generator_velocity_.SetPrevious(*previous_setpoint_)) {
    return InternalError("Failed to set previous reflexxes velocity target.");
  }

  MoveOk* const move_ok_interface =
      params.slot_map.GetMutableInterfaceForSlot<MoveOk>(slot_id_);
  if (move_ok_interface == nullptr) {
    return InternalError("Slot doesn't have MoveOk.");
  }
  JointStatePVA new_target;
  INTRINSIC_RT_RETURN_IF_ERROR(new_target.SetSize(ndof_));
  bool fall_back_to_stop_trajectory = true;
  const bool is_paused = (params.requested_behavior_override ==
                          intrinsic_proto::icon::v1::BehaviorOverrideRequest::
                              BEHAVIOR_OVERRIDE_REQUEST_PAUSE);
  if (is_paused) {
    INTRINSIC_RT_LOG_THROTTLED(INFO) << "PAUSE requested for PointToPointMove.";
  }
  const double effective_speed_override =
      is_paused ? 0.0 : params.speed_override;

  if (effective_speed_override > kSwitchSpeedOverride) {
    JointLimits scaled_limits = params_.joint_limits;
    scaled_limits.max_velocity *= effective_speed_override;
    // TODO(b/254432130): scaling acceleration and jerk limits results in
    // infeasible scenarios.
    // scaled_limits.max_acceleration *=
    // ::intrinsic::IPow(params.speed_override, 2); scaled_limits.max_jerk
    // *= ::intrinsic::IPow(params.speed_override, 3);
    if (!trajectory_generator_position_.SetLimits(scaled_limits)) {
      return InternalError("Failed to set reflexxes limits.");
    }

    INTRINSIC_RT_ASSIGN_OR_RETURN(
        const LimitCheckResult limit_result,
        IsWithinLimits(*previous_setpoint_, params_.joint_limits));
    // When still within limits use the kActivelyPrevent flag to plan a
    // trajectory which tries to respect limits. If the position limits are
    // already breached then the limits must be ignored to allow recovery.
    // TODO(b/262706464) Switch to only using kIgnore if MoveChecker and
    // l1_controller use jerk limited stop trajectory.
    if (!limit_result.p_ok) {
      trajectory_generator_position_.SetPositionalLimitsBehavior(
          reflexxes::Flags::PositionalLimitsBehavior::kIgnore);
    } else {
      trajectory_generator_position_.SetPositionalLimitsBehavior(
          reflexxes::Flags::PositionalLimitsBehavior::kActivelyPrevent);
    }

    if (!trajectory_generator_position_.ComputeSetpoint(&new_target)) {
      return InternalError(
          "Failed to compute setpoint for new reflexxes position target.");
    }
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        const auto setpoints_to_check,
        JointPositionCommand::Create(new_target.position, new_target.velocity,
                                     new_target.acceleration));
    fall_back_to_stop_trajectory = !move_ok_interface->IsMoveOkWithUserLimits(
        setpoints_to_check, scaled_limits);
  }
  if (fall_back_to_stop_trajectory) {
    if (!trajectory_generator_velocity_.ComputeSetpoint(&new_target)) {
      return InternalError(
          "Failed to compute setpoint for new reflexxes velocity target.");
    }
  }

  JointPosition* const position_interface =
      params.slot_map.GetMutableInterfaceForSlot<JointPosition>(slot_id_);
  if (position_interface == nullptr) {
    return InternalError("Slot doesn't have JointPosition.");
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const auto setpoints,
      JointPositionCommand::Create(new_target.position, new_target.velocity,
                                   new_target.acceleration));
  INTRINSIC_RT_RETURN_IF_ERROR(
      position_interface->SetPositionSetpoints(setpoints));

  // Only update previous_setpoint if everything went well.
  previous_setpoint_ = new_target;
  done_buffer_ = trajectory_generator_position_.GetReflexxesStatus() ==
                 reflexxes::Status::kFinalStateReached;
  return OkStatus();
}

RealtimeStatusOr<StateVariableValue> JointPositionAction::GetStateVariable(
    absl::string_view name) const {
  if (!state_variables_.has_value()) {
    return icon::FailedPreconditionError(
        "State variables are uninitialized (did you call "
        "SetParameters() and Sense() at least once?");
  }
  if (name == kIsDone) {
    return StateVariableValue(state_variables_->done);
  }
  if (name == PointToPointMoveInfo::kIsSettled) {
    return StateVariableValue(state_variables_->settled);
  }
  if (name == PointToPointMoveInfo::kIsSettledUncertainty) {
    return StateVariableValue(is_settled_criterion_->GetUncertainty());
  }
  if (name == kActionElapsedTime) {
    return StateVariableValue(state_variables_->seconds_since_start);
  }
  if (name == PointToPointMoveInfo::kSetpointDoneForSeconds) {
    if (!state_variables_->seconds_when_done.has_value()) {
      return icon::UnavailableError("Not done executing trajectory yet.");
    }
    return StateVariableValue(state_variables_->seconds_since_start -
                              *(state_variables_->seconds_when_done));
  }
  if (name == PointToPointMoveInfo::kDistanceToSensed) {
    if (!state_variables_->distance_to_target.has_value()) {
      return icon::UnavailableError(
          "Distance to target is unknown (did you set call "
          "SetParameters() and Sense() at least once?");
    }
    return StateVariableValue(*(state_variables_->distance_to_target));
  }
  return NotFoundError(FixedStrCat<RealtimeStatus::kMaxMessageLength>(
      "JointPositionAction, state variable not found ", name));
}

}  // namespace intrinsic::icon
