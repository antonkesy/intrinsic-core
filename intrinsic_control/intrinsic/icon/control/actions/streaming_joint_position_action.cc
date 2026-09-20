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

#include "intrinsic/icon/control/actions/streaming_joint_position_action.h"

#include <stddef.h>

#include <memory>
#include <optional>
#include <utility>

#include "absl/functional/bind_front.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/actions/streaming_joint_position.pb.h"
#include "intrinsic/icon/actions/streaming_joint_position_info.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/common/builtins.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/algorithms/joint_position_reflexxes.h"
#include "intrinsic/icon/control/joint_position_command.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/realtime_slot_map.h"
#include "intrinsic/icon/control/slot_types.h"
#include "intrinsic/icon/control/streaming_io_realtime.h"
#include "intrinsic/icon/control/streaming_io_types.h"
#include "intrinsic/icon/proto/eigen_conversion.h"
#include "intrinsic/icon/proto/generic_part_config.pb.h"
#include "intrinsic/icon/proto/joint_space.pb.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/reflexxes/flags.h"
#include "intrinsic/icon/reflexxes/position_flags.h"
#include "intrinsic/icon/reflexxes/status.h"
#include "intrinsic/icon/utils/fixed_str_cat.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

StreamingJointPositionAction::StreamingJointPositionAction(
    RealtimeSlotId slot_id, StreamingInputId streaming_input_id, size_t ndof,
    double frequency_hz, Params initial_params)
    : slot_id_(slot_id),
      streaming_input_id_(streaming_input_id),
      ndof_(ndof),
      trajectory_generator_position_(ndof, frequency_hz),
      trajectory_generator_velocity_(ndof, frequency_hz),
      initial_params_(std::move(initial_params)) {
  current_params_ = initial_params_;
  trajectory_generator_position_.SetSelection(
      eigenmath::VectorNb::Constant(ndof, 1, true));
  trajectory_generator_velocity_.SetSelection(
      eigenmath::VectorNb::Constant(ndof, 1, true));
  trajectory_generator_position_.SetBehaviorIfInitialStateBreachesConstraints(
      reflexxes::PositionFlags::BehaviorIfInitialStateBreachesConstraints::
          kGetIntoBoundariesAtZeroAcceleration);
}

// static
absl::StatusOr<std::unique_ptr<StreamingJointPositionAction>>
StreamingJointPositionAction::Create(
    StreamingJointPositionInfo::FixedParams params_proto,
    ActionFactoryContext& context) {
  INTRINSIC_ASSERT_NON_REALTIME();
  INTR_ASSIGN_OR_RETURN(
      SlotInfo arm_info,
      context.GetSlotInfo(StreamingJointPositionInfo::kSlotName));
  const ::intrinsic_proto::icon::GenericJointPositionConfig&
      joint_position_config =
          arm_info.config.generic_config().joint_position_config();

  size_t ndof = joint_position_config.num_joints();
  if (!arm_info.config.generic_config().has_joint_limits_config()) {
    return absl::InvalidArgumentError(
        "StreamingJointPositionAction requires joint limits.");
  }
  const ::intrinsic_proto::icon::GenericJointLimitsConfig& joint_limits_config =
      arm_info.config.generic_config().joint_limits_config();
  INTR_ASSIGN_OR_RETURN(
      JointLimits application_limits,
      ::intrinsic::FromProto(joint_limits_config.application_limits()));

  // Use ParseStreamingInput to check the limits and goal.
  INTR_ASSIGN_OR_RETURN(Params initial_params,
                        ParseStreamingInput(application_limits, params_proto));

  INTR_ASSIGN_OR_RETURN(
      StreamingInputId streaming_input_id,
      (context.AddStreamingInputParser<StreamingJointPositionInfo::FixedParams,
                                       Params>(
          StreamingJointPositionInfo::kStreamingInputName,
          absl::bind_front(&ParseStreamingInput, application_limits))));

  return std::make_unique<StreamingJointPositionAction>(
      arm_info.slot_id, streaming_input_id, ndof,
      context.ServerConfig().frequency_hz(), std::move(initial_params));
}

RealtimeStatus StreamingJointPositionAction::OnEnter(OnEnterParameters params) {
  // Unset previous_setpoint_ and distance_to_target_ – they will be populated
  // in Sense().
  previous_setpoint_ = std::nullopt;
  done_buffer_ = false;
  return UpdateGoal(current_params_.goal, current_params_.joint_limits);
}

RealtimeStatus StreamingJointPositionAction::UpdateGoal(
    const JointStatePV& goal, const JointLimits& limits) {
  distance_to_target_ = std::nullopt;
  current_goal_ = goal;

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

  return OkStatus();
}

RealtimeStatus StreamingJointPositionAction::Sense(SenseParameters params) {
  if (!current_goal_.has_value()) {
    return FailedPreconditionError(
        "Called Sense() when Action has no goal. OnEnter() "
        "must be called once before Sense() or Control() to set a goal.");
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const auto* streaming_params,
      params.streaming_io_access.PollInput<Params>(streaming_input_id_));
  if (streaming_params != nullptr) {
    current_params_ = *streaming_params;
    INTRINSIC_RT_RETURN_IF_ERROR(
        UpdateGoal(streaming_params->goal, streaming_params->joint_limits));
  }

  const JointPosition* position_interface =
      params.slot_map.GetInterfaceForSlot<JointPosition>(slot_id_);
  if (position_interface == nullptr) {
    return InternalError("Slot doesn't have JointPosition.");
  }
  const JointPositionSensor* position_sensor =
      params.slot_map.GetInterfaceForSlot<JointPositionSensor>(slot_id_);
  if (position_sensor == nullptr) {
    return InternalError("Slot doesn't have JointPositionSensor.");
  }
  const JointVelocityEstimator* velocity_estimator =
      params.slot_map.GetInterfaceForSlot<JointVelocityEstimator>(slot_id_);
  if (velocity_estimator == nullptr) {
    return InternalError("Slot doesn't have JointVelocityEstimator.");
  }
  const JointAccelerationEstimator* acceleration_estimator =
      params.slot_map.GetInterfaceForSlot<JointAccelerationEstimator>(slot_id_);
  if (acceleration_estimator == nullptr) {
    return InternalError("Slot doesn't have JointAccelerationEstimator.");
  }

  JointStatePVA sensed_state;
  INTRINSIC_RT_RETURN_IF_ERROR(sensed_state.SetSize(ndof_));
  sensed_state.position = position_sensor->GetSensedPosition().position;
  sensed_state.velocity = velocity_estimator->GetVelocityEstimate().velocity;
  sensed_state.acceleration =
      acceleration_estimator->GetAccelerationEstimate().acceleration;

  // If previous_setpoint_ is not set yet, obtain the values from the
  // previous position setpoints.
  if (!previous_setpoint_.has_value()) {
    JointPositionCommand previous_position_setpoints =
        position_interface->PreviousPositionSetpoints();
    previous_setpoint_ = JointStatePVA();
    previous_setpoint_->position = previous_position_setpoints.position();
    previous_setpoint_->velocity =
        previous_position_setpoints.velocity_feedforward().value_or(
            eigenmath::VectorNd::Zero(ndof_));
    previous_setpoint_->acceleration.setConstant(ndof_, 0.);
  }

  if (current_goal_.has_value()) {
    distance_to_target_ =
        (sensed_state.position - current_goal_.value().position).norm();
  }

  done_ = done_buffer_;

  return OkStatus();
}

RealtimeStatus StreamingJointPositionAction::Control(ControlParameters params) {
  JointPosition* position_interface =
      params.slot_map.GetMutableInterfaceForSlot<JointPosition>(slot_id_);
  if (position_interface == nullptr) {
    return InternalError("Slot doesn't have JointPosition.");
  }
  if (!current_goal_.has_value()) {
    return FailedPreconditionError(
        "Control() called when no goal is set. Must call OnEnter() once before "
        "Sense() or Control() to set a goal.");
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
  JointStatePVA new_target;
  INTRINSIC_RT_RETURN_IF_ERROR(new_target.SetSize(ndof_));
  if (params.speed_override > kSwitchSpeedOverride) {
    JointLimits scaled_limits = current_params_.joint_limits;
    scaled_limits.max_velocity *= params.speed_override;
    // TODO(b/254432130): scaling acceleration and jerk limits results in
    // infeasible scenarios.
    // scaled_limits.max_acceleration *=
    // ::intrinsic::IPow(params.speed_override, 2); scaled_limits.max_jerk
    // *= ::intrinsic::IPow(params.speed_override, 3);
    if (!trajectory_generator_position_.SetLimits(scaled_limits)) {
      return InternalError("Failed to set reflexxes limits.");
    }
    if (!trajectory_generator_position_.ComputeSetpoint(&new_target)) {
      return InternalError(
          "Failed to compute setpoint for new reflexxes position target.");
    }
  } else {
    if (!trajectory_generator_velocity_.SetLimits(
            current_params_.joint_limits)) {
      return InternalError("Failed to set reflexxes limits.");
    }
    if (!trajectory_generator_velocity_.ComputeSetpoint(&new_target)) {
      return InternalError(
          "Failed to compute setpoint for new reflexxes velocity target.");
    }
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const auto setpoints,
      JointPositionCommand::Create(new_target.position, new_target.velocity,
                                   new_target.acceleration));
  INTRINSIC_RT_RETURN_IF_ERROR(
      position_interface->SetPositionSetpoints(setpoints));
  // Only update previous_setpoint if everything went well.
  distance_to_target_ =
      (new_target.position - current_goal_.value().position).norm();
  previous_setpoint_ = new_target;
  done_buffer_ = trajectory_generator_position_.GetReflexxesStatus() ==
                 reflexxes::Status::kFinalStateReached;
  return OkStatus();
}

RealtimeStatusOr<StateVariableValue>
StreamingJointPositionAction::GetStateVariable(absl::string_view name) const {
  if (name == kIsDone) {
    return StateVariableValue(done_);
  }
  if (name == StreamingJointPositionInfo::kDistanceToSensed) {
    if (!distance_to_target_.has_value()) {
      return icon::UnavailableError(
          "Distance to target is unknown (did you set call "
          "SetParameters() Sense() at least once?");
    }
    return StateVariableValue(distance_to_target_.value());
  }
  return NotFoundError(FixedStrCat<RealtimeStatus::kMaxMessageLength>(
      "StreamingJointPositionAction, state variable not found ", name));
}

// static
absl::StatusOr<StreamingJointPositionAction::Params>
StreamingJointPositionAction::ParseStreamingInput(
    const JointLimits& application_limits,
    const StreamingJointPositionInfo::FixedParams& params) {
  if (!params.has_goal_position()) {
    return absl::InvalidArgumentError(
        "The StreamingJointPositionInfo::FixedParams must contain the "
        "`goal_position`.");
  }
  if (!params.has_goal_velocity()) {
    return absl::InvalidArgumentError(
        "The StreamingJointPositionInfo::FixedParams must contain the "
        "`goal_velocity`.");
  }
  if (params.goal_position().joints_size() !=
      params.goal_velocity().joints_size()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Mismatch with",
        " goal_position size: ", params.goal_position().joints_size(),
        " goal_velocity size: ", params.goal_velocity().joints_size()));
  }
  Params output;
  INTR_ASSIGN_OR_RETURN(output.goal.position,
                        FromProto(params.goal_position()));
  INTR_ASSIGN_OR_RETURN(output.goal.velocity,
                        FromProto(params.goal_velocity()));

  // Check and use the user provided limits, or fall back to Application Limits.
  output.joint_limits = application_limits;
  if (params.has_joint_limits()) {
    INTR_ASSIGN_OR_RETURN(output.joint_limits,
                          intrinsic::FromProto(params.joint_limits()));
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        auto limits_within_limits,
        IsWithinLimits(output.joint_limits, application_limits));
    if (!limits_within_limits) {
      return absl::InvalidArgumentError(absl::StrCat(
          "The provided joint limits violate the Application joint limits: ",
          ToFixedString(limits_within_limits)));
    }
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto within_limits, IsWithinLimits(output.goal, output.joint_limits));
  if (!within_limits) {
    return absl::InvalidArgumentError("Goal violates joint limits.");
  }

  return output;
}

}  // namespace intrinsic::icon
