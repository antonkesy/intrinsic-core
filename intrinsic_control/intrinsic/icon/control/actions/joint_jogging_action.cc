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

#include "intrinsic/icon/control/actions/joint_jogging_action.h"

#include <cstddef>
#include <memory>
#include <utility>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/actions/joint_jogging.pb.h"
#include "intrinsic/icon/actions/joint_jogging_info.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/algorithms/joint_velocity_reflexxes.h"
#include "intrinsic/icon/control/joint_position_command.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/slot_types.h"
#include "intrinsic/icon/control/streaming_io_types.h"
#include "intrinsic/icon/proto/eigen_conversion.h"
#include "intrinsic/icon/proto/generic_part_config.pb.h"
#include "intrinsic/icon/proto/joint_space.pb.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/reflexxes/flags.h"
#include "intrinsic/icon/utils/fixed_str_cat.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/check_joint_limits.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

// static
absl::StatusOr<std::unique_ptr<JointJoggingAction>> JointJoggingAction::Create(
    const JointJoggingInfo::FixedParams& params_proto,
    ActionFactoryContext& context) {
  INTRINSIC_ASSERT_NON_REALTIME();
  INTR_ASSIGN_OR_RETURN(SlotInfo arm_info,
                        context.GetSlotInfo(JointJoggingInfo::kSlotName));
  const ::intrinsic_proto::icon::GenericJointPositionConfig&
      joint_position_config =
          arm_info.config.generic_config().joint_position_config();

  size_t ndof = joint_position_config.num_joints();
  if (!params_proto.has_joint_limits()) {
    return absl::InvalidArgumentError(
        "Joint jogging action needs joint limits as parameter.");
  }
  FixedParams params;
  INTR_ASSIGN_OR_RETURN(params.joint_limits,
                        intrinsic::FromProto(params_proto.joint_limits()));
  INTR_ASSIGN_OR_RETURN(JointLimits application_limits,
                        ::intrinsic::FromProto(arm_info.config.generic_config()
                                                   .joint_limits_config()
                                                   .application_limits()));
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

  auto parser = [limits = params.joint_limits](
                    const JointJoggingInfo::StreamingParams& proto)
      -> absl::StatusOr<StreamingParams> {
    StreamingParams output;
    INTR_RETURN_IF_ERROR(
        output.goal.SetSize(proto.goal_velocity().joints().size()));
    INTR_ASSIGN_OR_RETURN(
        output.goal.velocity,
        RepeatedDoubleToVectorNd(proto.goal_velocity().joints()));
    INTRINSIC_RT_ASSIGN_OR_RETURN(bool is_within_limits,
                                  IsWithinLimits(output.goal, limits));
    if (!is_within_limits) {
      const auto error = absl::FailedPreconditionError(
          "Provided goal violates velocity limits.");
      LOG_EVERY_N_SEC(ERROR, 1) << error;
      return error;
    }
    return output;
  };
  INTR_ASSIGN_OR_RETURN(
      StreamingInputId streaming_input_id,
      (context.AddStreamingInputParser<JointJoggingInfo::StreamingParams,
                                       StreamingParams>(
          JointJoggingInfo::kStreamingInputName, parser)));

  // Two trajectory generators are used for jogging:
  // 1) the first one ignores positional limits such that it can generate
  // commands to jog the robot even under very constrained limits. This would
  // typically cause the robot to NOT jog at all given that reaching a stop
  // within positional limits after reaching the target velocity is infeasible.
  JointVelocityReflexxes trajectory_generator(
      ndof, context.ServerConfig().frequency_hz());
  // The trajectory generator should control all dofs.
  if (!trajectory_generator.SetSelection(
          eigenmath::VectorNb::Constant(ndof, 1, true))) {
    return InternalError(
        "Failed to call SetSelection in the trajectory_generator.");
  }
  if (!trajectory_generator.SetLimits(params.joint_limits)) {
    return InternalError(
        "Failed to set joint_limits in the trajectory_generator.");
  }
  // Avoid position limits.
  trajectory_generator.SetPositionalLimitsBehavior(
      reflexxes::Flags::PositionalLimitsBehavior::kIgnore);

  // 2) the second trajectory generator makes sure that a valid stop trajectory
  // exists from the new_target computed by the first trajectory generator. If
  // it does not exist, then a stop motion is immediately issued from the
  // current state for which a stop trajectory exists. Note that in this case
  // the robot would still jog though it would not reach the target velocity.
  JointVelocityReflexxes stop_generator(ndof,
                                        context.ServerConfig().frequency_hz());
  // The trajectory generator should control all dofs.
  if (!stop_generator.SetSelection(
          eigenmath::VectorNb::Constant(ndof, 1, true))) {
    return InternalError("Failed to call SetSelection in the stop_generator.");
  }
  if (!stop_generator.SetLimits(params.joint_limits)) {
    return InternalError("Failed to set joint_limits in the stop_generator.");
  }
  // The stop generator always drives the robot to a zero velocity.
  JointStateV zero_velocity;
  INTRINSIC_RT_RETURN_IF_ERROR(zero_velocity.SetSize(ndof));
  zero_velocity.velocity.setZero();
  stop_generator.SetTarget(zero_velocity);
  // Avoid position limits.
  stop_generator.SetPositionalLimitsBehavior(
      reflexxes::Flags::PositionalLimitsBehavior::kIgnore);

  return std::unique_ptr<JointJoggingAction>(new JointJoggingAction(
      arm_info.slot_id, ndof, context.ServerConfig().frequency_hz(),
      streaming_input_id, std::move(params), std::move(trajectory_generator),
      std::move(stop_generator)));
}

RealtimeStatus JointJoggingAction::OnEnter(OnEnterParameters params) {
  timed_out_.reset();
  internal_state_ = InternalState();

  // Set the initial target_velocity to zero.
  INTRINSIC_RT_RETURN_IF_ERROR(target_velocity_.SetSize(ndof_));
  target_velocity_.velocity.setConstant(0.);

  return OkStatus();
}

RealtimeStatus JointJoggingAction::Sense(SenseParameters params) {
  if (!internal_state_.has_value()) {
    return FailedPreconditionError(
        "OnEnter() must be called once before Sense() or Control() to "
        "set valid limits and initialize the trajectory generator.");
  }
  const JointPosition* const position_interface =
      params.slot_map.GetInterfaceForSlot<JointPosition>(slot_id_);
  if (position_interface == nullptr) {
    return InternalError("Slot doesn't have JointPosition.");
  }
  const JointVelocityEstimator* const velocity_estimator =
      params.slot_map.GetInterfaceForSlot<JointVelocityEstimator>(slot_id_);
  if (velocity_estimator == nullptr) {
    return InternalError("Slot doesn't have JointVelocityEstimator.");
  }

  // If previous_setpoint_ is not set yet, obtain the values from the
  // previous position setpoints.
  if (!internal_state_->previous_joint_command.has_value()) {
    JointPositionCommand previous_position_setpoints =
        position_interface->PreviousPositionSetpoints();
    JointStatePVA previous_setpoint;
    previous_setpoint.position = previous_position_setpoints.position();
    previous_setpoint.velocity =
        previous_position_setpoints.velocity_feedforward().value_or(
            velocity_estimator->GetVelocityEstimate().velocity);
    previous_setpoint.acceleration =
        previous_position_setpoints.acceleration_feedforward().value_or(
            eigenmath::VectorNd::Zero(ndof_));
    internal_state_->previous_joint_command = previous_setpoint;
  }

  // Update the StateVariable only after receiving the first streaming command.
  if (timed_out_.has_value()) {
    timed_out_ = (internal_state_->cycles_until_timeout <= 0);
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const StreamingParams* streaming_input_value,
      params.streaming_io_access.PollInput<StreamingParams>(
          streaming_input_id_));
  if (streaming_input_value != nullptr) {
    internal_state_->cycles_until_timeout =
        (JointJoggingInfo::kWatchdogTimeoutInSeconds * frequency_hz_);
    INTRINSIC_RT_LOG_THROTTLED(INFO)
        << JointJoggingInfo::kActionTypeName << " received streamed command.";
    timed_out_ = false;
    // Streaming input is checked against joint_limits by streaming parser
    // defined in JointJoggingAction::Create, so skip checking here.
    target_velocity_ = streaming_input_value->goal;
  }
  return OkStatus();
}

RealtimeStatus JointJoggingAction::Control(ControlParameters params) {
  if (!internal_state_.has_value()) {
    return FailedPreconditionError(
        "OnEnter() must be called once before Sense() or Control() to "
        "set valid limits and initialize the trajectory generator.");
  }
  const bool is_paused = (params.requested_behavior_override ==
                          intrinsic_proto::icon::v1::BehaviorOverrideRequest::
                              BEHAVIOR_OVERRIDE_REQUEST_PAUSE);
  if (is_paused) {
    INTRINSIC_RT_LOG_THROTTLED(INFO) << "PAUSE requested for Joint Jogging.";
  }
  const double effective_speed_override =
      is_paused ? 0.0 : params.speed_override;

  JointPosition* const position_interface =
      params.slot_map.GetMutableInterfaceForSlot<JointPosition>(slot_id_);
  if (position_interface == nullptr) {
    return InternalError("Slot doesn't have JointPosition.");
  }

  MoveOk* const move_ok_interface =
      params.slot_map.GetMutableInterfaceForSlot<MoveOk>(slot_id_);
  if (move_ok_interface == nullptr) {
    return InternalError("Slot doesn't have MoveOk.");
  }

  if (!internal_state_->previous_joint_command.has_value()) {
    return InternalError(
        "Previous joint command is not set. Did you forget to call Sense() "
        "before Control()?");
  }

  // Force a stop if cycles_until_timeout is zero, if not decrement counter and
  // update the target velocity according to the current speed_override value
  if (internal_state_->cycles_until_timeout-- <= 0) {
    internal_state_->cycles_until_timeout = 0;
    JointStateV zero_velocity;
    INTRINSIC_RT_RETURN_IF_ERROR(zero_velocity.SetSize(ndof_));
    zero_velocity.velocity.setConstant(0.);
    trajectory_generator_.SetTarget(zero_velocity);
  } else {
    JointStateV scaled_velocity;
    scaled_velocity.velocity =
        target_velocity_.velocity * effective_speed_override;
    trajectory_generator_.SetTarget(scaled_velocity);
  }

  auto previous_state = *internal_state_->previous_joint_command;
  if (!trajectory_generator_.SetPrevious(previous_state)) {
    return InternalError("Failed to set previous reflexxes target.");
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto limit_result,
      IsWithinLimits(previous_state, fixed_params_.joint_limits));

  JointStatePVA new_target;
  INTRINSIC_RT_RETURN_IF_ERROR(new_target.SetSize(ndof_));
  if (!trajectory_generator_.ComputeSetpoint(&new_target)) {
    return InternalError(
        "Failed to compute setpoint for new reflexxes target.");
  }

  // If the robot is within limits, attempt to compute a
  // `new_target_stop_candidate` to move towards the target velocity. If from
  // `new_target_stop_candidate` a stop trajectory within positional limits is
  // not feasible, compute within `new_target` the next state to follow a stop
  // trajectory.
  if (limit_result.p_ok) {
    if (!stop_generator_.SetPrevious(new_target)) {
      return InternalError("Failed to set previous reflexxes target.");
    }
    JointStatePVA new_target_stop_candidate;
    INTRINSIC_RT_RETURN_IF_ERROR(new_target_stop_candidate.SetSize(ndof_));
    if (!stop_generator_.ComputeSetpoint(&new_target_stop_candidate)) {
      return InternalError(
          "Failed to compute setpoint for new reflexxes target.");
    }
    // Compute a stop trajectory from the `previous_state` if the stop
    // trajectory from the `new_target` would violate positional limits.
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        const bool trajectory_breaches_limits,
        stop_generator_.DoesStopMotionFromNewStateBreachPositionalLimits());
    if (trajectory_breaches_limits) {
      if (!stop_generator_.SetPrevious(previous_state)) {
        return InternalError("Failed to set previous reflexxes target.");
      }
      if (!stop_generator_.ComputeSetpoint(&new_target)) {
        return InternalError(
            "Failed to compute setpoint for new reflexxes target.");
      }
    }
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      auto setpoints,
      JointPositionCommand::Create(new_target.position, new_target.velocity,
                                   new_target.acceleration));

  bool setpoint_is_ok = move_ok_interface->IsMoveOkWithUserLimits(
      setpoints, fixed_params_.joint_limits);
  if (!setpoint_is_ok) {
    // Recompute setpoint with zero velocity target
    INTRINSIC_RT_LOG_THROTTLED(INFO)
        << "Setpoint is not OK so commanding a zero velocity trajectory.";
    if (!stop_generator_.SetPrevious(previous_state)) {
      return InternalError("Failed to set previous reflexxes target.");
    }
    if (!stop_generator_.ComputeSetpoint(&new_target)) {
      return InternalError(
          "Failed to compute setpoint for new reflexxes target.");
    }
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        setpoints,
        JointPositionCommand::Create(new_target.position, new_target.velocity,
                                     new_target.acceleration));
  }
  INTRINSIC_RT_RETURN_IF_ERROR(
      position_interface->SetPositionSetpoints(setpoints));
  // Only update previous_setpoint if everything went well.
  internal_state_->previous_joint_command = new_target;
  return OkStatus();
}

RealtimeStatusOr<StateVariableValue> JointJoggingAction::GetStateVariable(
    absl::string_view name) const {
  if (name == JointJoggingInfo::kTimedOut) {
    if (!timed_out_.has_value()) {
      return icon::UnavailableError(FixedStrCat<
                                    RealtimeStatus::kMaxMessageLength>(
          JointJoggingInfo::kTimedOut,
          " is unavailable (did you send a Streaming Command at least once?"));
    }
    return StateVariableValue(*timed_out_);
  }
  return NotFoundError(FixedStrCat<RealtimeStatus::kMaxMessageLength>(
      JointJoggingInfo::kActionTypeName, ", state variable not found ", name));
}

}  // namespace intrinsic::icon
