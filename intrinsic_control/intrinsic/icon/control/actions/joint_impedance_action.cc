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

#include "intrinsic/icon/control/actions/joint_impedance_action.h"

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
#include "intrinsic/icon/actions/joint_impedance_action.pb.h"
#include "intrinsic/icon/actions/joint_impedance_action_info.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/common/builtins.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/algorithms/is_settled_criterion.h"
#include "intrinsic/icon/control/algorithms/joint_position_reflexxes.h"
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

JointImpedanceAction::JointImpedanceAction(
    RealtimeSlotId slot_id, StreamingInputId streaming_input_id, size_t ndof,
    double frequency_hz, Params initial_params,
    std::unique_ptr<IsSettledCriterion> is_settled_criterion)
    : slot_id_(slot_id),
      streaming_input_id_(streaming_input_id),
      ndof_(ndof),
      trajectory_generator_position_(ndof, frequency_hz),
      trajectory_generator_velocity_(ndof, frequency_hz),
      initial_params_(std::move(initial_params)),
      is_settled_criterion_(std::move(is_settled_criterion)) {
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
absl::StatusOr<std::unique_ptr<JointImpedanceAction>>
JointImpedanceAction::Create(JointImpedanceInfo::FixedParams params_proto,
                             ActionFactoryContext& context) {
  INTRINSIC_ASSERT_NON_REALTIME();
  INTR_ASSIGN_OR_RETURN(SlotInfo arm_info,
                        context.GetSlotInfo(JointImpedanceInfo::kSlotName));
  const ::intrinsic_proto::icon::GenericJointTorqueConfig& joint_torque_config =
      arm_info.config.generic_config().joint_torque_config();

  const int ndof = joint_torque_config.num_joints();
  if (!arm_info.config.generic_config().has_joint_limits_config()) {
    return absl::InvalidArgumentError(
        "JointImpedanceAction requires joint limits.");
  }
  const ::intrinsic_proto::icon::GenericJointLimitsConfig& joint_limits_config =
      arm_info.config.generic_config().joint_limits_config();
  INTR_ASSIGN_OR_RETURN(
      JointLimits application_limits,
      ::intrinsic::FromProto(joint_limits_config.application_limits()));

  // Use ParseStreamingInput to check the limits and goal.
  INTR_ASSIGN_OR_RETURN(Params initial_params,
                        ParseInput(application_limits, params_proto));

  INTR_ASSIGN_OR_RETURN(
      StreamingInputId streaming_input_id,
      (context.AddStreamingInputParser<JointImpedanceInfo::FixedParams, Params>(
          JointImpedanceInfo::kStreamingInputName,
          absl::bind_front(&ParseInput, application_limits))));

  INTR_ASSIGN_OR_RETURN(
      auto is_settled_criterion,
      IsSettledCriterion::Create(context.ServerConfig().frequency_hz()));

  return std::make_unique<JointImpedanceAction>(
      arm_info.slot_id, streaming_input_id, ndof,
      context.ServerConfig().frequency_hz(), std::move(initial_params),
      std::move(is_settled_criterion));
}

RealtimeStatus JointImpedanceAction::OnEnter(OnEnterParameters params) {
  // Unset previous_setpoint_ and distance_to_target_ – they will be populated
  // in Sense().
  previous_setpoint_ = std::nullopt;
  joint_state_sensed_ = std::nullopt;
  is_done_ = false;
  is_settled_ = false;
  INTRINSIC_RT_RETURN_IF_ERROR(is_settled_criterion_->Initialize());
  return UpdateGoal(current_params_.target_state, current_params_.joint_limits);
}

RealtimeStatus JointImpedanceAction::UpdateGoal(
    const JointStatePV& target_state, const JointLimits& limits) {
  distance_to_target_ = std::nullopt;
  is_done_ = false;
  is_settled_ = false;
  INTRINSIC_RT_RETURN_IF_ERROR(is_settled_criterion_->Initialize());

  if (!trajectory_generator_position_.SetTarget(target_state)) {
    return InternalError(
        "Failed to set joint position reflexxes target state.");
  }

  // The velocity generator is only used for commanding a zero velocity command
  // due to very small speed override.
  JointStateV zero_velocity;
  INTRINSIC_RT_RETURN_IF_ERROR(zero_velocity.SetSize(ndof_));
  zero_velocity.velocity.setConstant(0.);
  if (!trajectory_generator_velocity_.SetTarget(zero_velocity)) {
    return InternalError(
        "Failed to set joint velocity reflexxes zero velocity target.");
  }

  // Ignoring joint position limits so that stopping is attempted when starting
  // the action outside of the limits, or if the stop trajectory ends outside of
  // the limits. Such a violation needs to be handled by the part or safety
  // implementation.
  trajectory_generator_velocity_.SetPositionalLimitsBehavior(
      reflexxes::Flags::PositionalLimitsBehavior::kIgnore);

  return OkStatus();
}

RealtimeStatus JointImpedanceAction::Sense(SenseParameters params) {
  // First we check if there are any streaming input parameters.
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const auto* streaming_params,
      params.streaming_io_access.PollInput<Params>(streaming_input_id_));
  if (streaming_params != nullptr) {
    current_params_ = *streaming_params;
    INTRINSIC_RT_RETURN_IF_ERROR(UpdateGoal(streaming_params->target_state,
                                            streaming_params->joint_limits));
  }

  // Get all the sensed state interfaces
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
  // Get the acceleration estimator interface, if available. This is optional.
  const JointAccelerationEstimator* acceleration_estimator =
      params.slot_map.GetInterfaceForSlot<JointAccelerationEstimator>(slot_id_);

  joint_state_sensed_.emplace();
  INTRINSIC_RT_RETURN_IF_ERROR(joint_state_sensed_->SetSize(ndof_));
  joint_state_sensed_->position = position_sensor->GetSensedPosition().position;
  joint_state_sensed_->velocity =
      velocity_estimator->GetVelocityEstimate().velocity;
  joint_state_sensed_->acceleration = eigenmath::VectorNd::Zero(ndof_);
  if (acceleration_estimator != nullptr) {
    joint_state_sensed_->acceleration =
        acceleration_estimator->GetAccelerationEstimate().acceleration;
  }

  // If previous_setpoint_ is not set yet, use the sensed state as previous
  // setpoint.
  if (!previous_setpoint_.has_value()) {
    previous_setpoint_ = *joint_state_sensed_;
  }

  distance_to_target_ =
      (joint_state_sensed_->position - current_params_.target_state.position)
          .norm();

  is_done_ = trajectory_generator_position_.GetReflexxesStatus() ==
             reflexxes::Status::kFinalStateReached;

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      is_settled_,
      is_settled_criterion_->Update(joint_state_sensed_->velocity,
                                    previous_setpoint_->velocity, is_done_));

  return OkStatus();
}

RealtimeStatus JointImpedanceAction::Control(ControlParameters params) {
  JointTorque* torque_interface =
      params.slot_map.GetMutableInterfaceForSlot<JointTorque>(slot_id_);
  if (torque_interface == nullptr) {
    return InternalError("Slot doesn't have JointTorque.");
  }

  if (!previous_setpoint_.has_value()) {
    return InternalError(
        "Previous target is unknown. Did you forget to call Sense() before "
        "Control()?");
  }
  if (!joint_state_sensed_.has_value()) {
    return InternalError(
        "Joint state is unknown. Did you forget to call Sense() before "
        "Control()?");
  }
  if (!trajectory_generator_position_.SetPrevious(*previous_setpoint_)) {
    return InternalError("Failed to set previous reflexxes position target.");
  }
  if (!trajectory_generator_velocity_.SetPrevious(*previous_setpoint_)) {
    return InternalError("Failed to set previous reflexxes velocity target.");
  }

  // We generate smooth and joint limited setpoints for the impedance control
  // law
  JointStatePVA new_target;
  INTRINSIC_RT_RETURN_IF_ERROR(new_target.SetSize(ndof_));
  if (params.speed_override > kSwitchSpeedOverride) {
    JointLimits scaled_limits = current_params_.joint_limits;
    scaled_limits.max_velocity *= params.speed_override;

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

  // Compute the inverse dynamics torques if we have a dynamics interface.
  eigenmath::VectorNd inverse_dynamics_torque =
      eigenmath::VectorNd::Zero(ndof_);

  // Get the dynamics interface, if available. This is optional.
  Dynamics* dynamics_interface =
      params.slot_map.GetMutableInterfaceForSlot<Dynamics>(slot_id_);
  if (dynamics_interface != nullptr && joint_state_sensed_.has_value()) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        inverse_dynamics_torque,
        dynamics_interface->GetRigidBodyInterface().ComputeInverseDynamics(
            joint_state_sensed_.value().position,
            joint_state_sensed_.value().velocity, new_target.acceleration));
  }

  const eigenmath::VectorNd position_error =
      new_target.position - joint_state_sensed_->position;
  const eigenmath::VectorNd velocity_error =
      new_target.velocity - joint_state_sensed_->velocity;

  // Simple position and velocity impedance control law.
  const eigenmath::VectorNd joint_torques =
      current_params_.joint_stiffness.cwiseProduct(position_error) +
      current_params_.joint_damping.cwiseProduct(velocity_error);

  eigenmath::VectorNd feedforward_torque = eigenmath::VectorNd::Zero(ndof_);
  if (current_params_.feedforward_torque.has_value()) {
    feedforward_torque = current_params_.feedforward_torque->torque;
  }

  INTRINSIC_RT_RETURN_IF_ERROR(torque_interface->SetTorqueSetpoints(
      feedforward_torque + inverse_dynamics_torque + joint_torques));

  // Only update previous_setpoint if everything went well.
  distance_to_target_ =
      (new_target.position - current_params_.target_state.position).norm();
  previous_setpoint_ = new_target;

  return OkStatus();
}

RealtimeStatusOr<StateVariableValue> JointImpedanceAction::GetStateVariable(
    absl::string_view name) const {
  if (name == kIsDone) {
    return StateVariableValue(is_done_);
  }
  if (name == JointImpedanceInfo::kIsSettled) {
    return StateVariableValue(is_settled_);
  }
  if (name == JointImpedanceInfo::kDistanceToSensed) {
    if (!distance_to_target_.has_value()) {
      return icon::UnavailableError(
          "Distance to target is unknown (did you set call "
          "SetParameters() Sense() at least once?");
    }
    return StateVariableValue(distance_to_target_.value());
  }
  return NotFoundError(FixedStrCat<RealtimeStatus::kMaxMessageLength>(
      "JointImpedanceAction, state variable not found ", name));
}

// static
absl::StatusOr<JointImpedanceAction::Params> JointImpedanceAction::ParseInput(
    const JointLimits& application_limits,
    const JointImpedanceInfo::StreamingParams& params) {
  const int ndof = application_limits.size();
  if (!params.has_target_position()) {
    return absl::InvalidArgumentError(
        "The JointImpedanceInfo::StreamingParams must contain the "
        "`target_position`.");
  }
  if (!params.has_target_velocity()) {
    return absl::InvalidArgumentError(
        "The JointImpedanceInfo::StreamingParams must contain the "
        "`target_velocity`.");
  }

  if (params.target_position().joints_size() !=
          params.target_velocity().joints_size() ||
      params.target_position().joints_size() != ndof) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Incorrect goal size. Number of DoFs: ", ndof,
        " target_position size: ", params.target_position().joints_size(),
        " target_velocity size: ", params.target_velocity().joints_size()));
  }

  if (!params.has_joint_stiffness() || !params.has_joint_damping()) {
    return absl::InvalidArgumentError(
        "The JointImpedanceInfo::StreamingParams must contain the "
        "`joint_stiffness` and `joint_damping`.");
  }

  if (params.joint_stiffness().joints_size() !=
          params.joint_damping().joints_size() ||
      params.joint_stiffness().joints_size() != ndof) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Incorrect stiffness and damping size. Number of DoFs: ", ndof,
        " joint_stiffness size: ", params.joint_stiffness().joints_size(),
        " joint_damping size: ", params.joint_damping().joints_size()));
  }

  Params output;
  INTR_ASSIGN_OR_RETURN(output.target_state.position,
                        FromProto(params.target_position()));
  INTR_ASSIGN_OR_RETURN(output.target_state.velocity,
                        FromProto(params.target_velocity()));

  if (params.has_feedforward_torque()) {
    if (params.feedforward_torque().joints_size() != ndof) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Incorrect feedforward torque size. Number of DoFs: ", ndof,
          " feedforward_torque size: ",
          params.feedforward_torque().joints_size()));
    }
    INTR_ASSIGN_OR_RETURN(output.feedforward_torque,
                          FromProto(params.feedforward_torque()));
  }

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
      auto within_limits,
      IsWithinLimits(output.target_state, output.joint_limits));
  if (!within_limits) {
    return absl::InvalidArgumentError("Goal violates joint limits.");
  }

  INTR_ASSIGN_OR_RETURN(output.joint_damping,
                        FromProto(params.joint_damping()));
  INTR_ASSIGN_OR_RETURN(output.joint_stiffness,
                        FromProto(params.joint_stiffness()));

  return output;
}

}  // namespace intrinsic::icon
