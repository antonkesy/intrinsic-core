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

#include "intrinsic/icon/control/actions/rtcl_joint_stop_action.h"

#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <utility>

#include "absl/log/check.h"
#include "absl/memory/memory.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/actions/action_utils.h"
#include "intrinsic/icon/actions/stop_info.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/common/builtins.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/algorithms/is_settled_criterion.h"
#include "intrinsic/icon/control/algorithms/joint_velocity_reflexxes.h"
#include "intrinsic/icon/control/joint_position_command.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/slot_types.h"
#include "intrinsic/icon/proto/generic_part_config.pb.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/reflexxes/flags.h"
#include "intrinsic/icon/reflexxes/status.h"
#include "intrinsic/icon/utils/fixed_str_cat.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/kinematics/types/dynamic_limits_check_mode.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/thread/rt_trace.h"

namespace intrinsic::icon {

// If joint acceleration limits computed from dynamics are available use them to
// generate the stop trajectory. This will always lead to a faster stop than
// hard-coded limits. If valid (non-std::nullopt)
// `joint_acceleration_limits_from_dynamics` are available,
// `joint_limits.max_acceleration` is updated to the maximum valid joint
// acceleration limits from dynamics that can be used to stop, as well as
// `joint_dynamic_limits_check_mode` marks the setpoint as torque-limited.
// Otherwise, `joint_limits.max_acceleration` and
// `joint_dynamic_limits_check_mode` are untouched. An InternalError is returned
// if the size of `joint_acceleration_limits_from_dynamics` does not match the
// expected number of degrees of freedom.
icon::RealtimeStatus UpdateJointLimitsAndCheckModeWithDynamics(
    const JointLimitsInterface* const limits_interface,
    const JointStatePVA& previous_setpoint, const RealtimeSlotId& slot_id_,
    JointLimits& joint_limits,
    DynamicLimitsCheckMode& joint_dynamic_limits_check_mode) {
  // Pointer to `limits_interface` is validated before calling this method.
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      std::optional<JointLimitsInterface::JointAccelerationLimitsFromDynamics>
          joint_acceleration_limits_from_dynamics,
      limits_interface->GetJointAccelerationLimitsFromDynamics());
  if (joint_acceleration_limits_from_dynamics.has_value()) {
    if ((joint_acceleration_limits_from_dynamics
             ->joint_acceleration_limits_at_min_torque.size() !=
         joint_limits.size()) ||
        (joint_acceleration_limits_from_dynamics
             ->joint_acceleration_limits_at_max_torque.size() !=
         joint_limits.size())) {
      return InternalError(FixedStrCat<RealtimeStatus::kMaxMessageLength>(
          "Slot ", slot_id_.value(),
          ": acceleration limits via dynamics has incorrect size."));
    }

    const eigenmath::VectorNd& acc_at_min_torque =
        joint_acceleration_limits_from_dynamics
            ->joint_acceleration_limits_at_min_torque;
    const eigenmath::VectorNd& acc_at_max_torque =
        joint_acceleration_limits_from_dynamics
            ->joint_acceleration_limits_at_max_torque;

    // Take max acceleration/deceleration according to motion velocity.
    joint_limits.max_acceleration =
        (previous_setpoint.velocity.array() < 0.0)
            .select(acc_at_min_torque.cwiseMax(acc_at_max_torque),
                    acc_at_min_torque.cwiseMin(acc_at_max_torque))
            .cwiseAbs();

    // Marks the setpoint as torque-limited, which allows the MoveChecker to
    // validate it is safe to execute using a stop trajectory based on joint
    // acceleration limits computed from dynamics.
    joint_dynamic_limits_check_mode = DynamicLimitsCheckMode::kCheckNone;
  }

  return icon::OkStatus();
}

JointStopAction::JointStopAction(
    RealtimeSlotId slot_id, size_t ndof, double frequency_hz,
    std::unique_ptr<IsSettledCriterion> is_settled_criterion,
    bool use_joint_jerk_limits)
    : slot_id_(slot_id),
      ndof_(ndof),
      frequency_hz_(frequency_hz),
      trajectory_generator_(ndof, frequency_hz),
      is_settled_criterion_(std::move(is_settled_criterion)),
      use_joint_jerk_limits_(use_joint_jerk_limits) {
  trajectory_generator_.SetSelection(
      eigenmath::VectorNb::Constant(ndof, 1, true));
}

// static
absl::StatusOr<std::unique_ptr<JointStopAction>> JointStopAction::Create(
    ActionFactoryContext& context) {
  INTR_ASSIGN_OR_RETURN(SlotInfo stop_slot_info,
                        context.GetSlotInfo(kStopPartSlot));
  size_t ndof = stop_slot_info.config.generic_config()
                    .joint_position_config()
                    .num_joints();

  INTR_ASSIGN_OR_RETURN(
      auto is_settled_criterion,
      IsSettledCriterion::Create(context.ServerConfig().frequency_hz()));

  // Using absl::WrapUnique() below because of the private constructor of the
  // JointStopAction.
  return absl::WrapUnique(new JointStopAction(
      stop_slot_info.slot_id, ndof, context.ServerConfig().frequency_hz(),
      std::move(is_settled_criterion),
      /*use_joint_jerk_limits=*/true));
}

// static
intrinsic_proto::icon::v1::ActionSignature JointStopAction::GetStopSignature() {
  ActionSignatureBuilder builder(kStopAction, kStopActionDescription);
  CHECK_OK(builder.AddStateVariable<intrinsic_proto::icon::v1::ActionSignature::
                                        StateVariableInfo::TYPE_BOOL>(
      kIsDone, kIsDoneDescription));
  CHECK_OK(builder.AddStateVariable<intrinsic_proto::icon::v1::ActionSignature::
                                        StateVariableInfo::TYPE_DOUBLE>(
      kActionElapsedTime, kActionElapsedTimeDescription));
  CHECK_OK(builder.AddStateVariable<intrinsic_proto::icon::v1::ActionSignature::
                                        StateVariableInfo::TYPE_BOOL>(
      kIsStopped, kIsStoppedDescription));
  CHECK_OK(builder.AddStateVariable<intrinsic_proto::icon::v1::ActionSignature::
                                        StateVariableInfo::TYPE_BOOL>(
      StopInfo::kIsSettled, StopInfo::kIsSettledDescription));
  CHECK_OK(builder.AddPartSlot(
      kStopPartSlot, kStopPartSlotDescription,
      /*required_feature_interfaces=*/
      {
          intrinsic_proto::icon::v1::FeatureInterfaceTypes::
              FEATURE_INTERFACE_JOINT_POSITION,
          intrinsic_proto::icon::v1::FeatureInterfaceTypes::
              FEATURE_INTERFACE_JOINT_POSITION_SENSOR,
          intrinsic_proto::icon::v1::FeatureInterfaceTypes::
              FEATURE_INTERFACE_JOINT_LIMITS,
          intrinsic_proto::icon::v1::FeatureInterfaceTypes::
              FEATURE_INTERFACE_JOINT_VELOCITY_ESTIMATOR,
      },
      /*optional_feature_interfaces=*/
      {
          intrinsic_proto::icon::v1::FeatureInterfaceTypes::
              FEATURE_INTERFACE_JOINT_ACCELERATION_ESTIMATOR,
      }));
  CHECK_OK(builder.AddSupportedBehaviorOverride(
      intrinsic_proto::icon::v1::BehaviorOverrideRequest::
          BEHAVIOR_OVERRIDE_REQUEST_PAUSE,
      "No change in behavior, the action stops the robot."));
  return builder.Finish();
}

RealtimeStatus JointStopAction::OnEnter(OnEnterParameters params) {
  INTRINSIC_TRACE_SCOPED("JointStopAction::OnEnter");
  // Unset previous_setpoint_ – it will be populated
  // in Sense().
  previous_setpoint_ = std::nullopt;
  done_buffer_ = false;

  const JointLimitsInterface* const limits_interface =
      params.slot_map.GetInterfaceForSlot<JointLimitsInterface>(slot_id_);
  if (limits_interface == nullptr) {
    return InternalError("Slot doesn't have JointLimits.");
  }
  JointLimits joint_limits = limits_interface->GetSystemLimits();
  if (!use_joint_jerk_limits_) {
    joint_limits.max_jerk.setConstant(std::numeric_limits<double>::infinity());
  }
  if (!trajectory_generator_.SetLimits(joint_limits)) {
    return InternalError("Failed to set reflexxes limits.");
  }

  JointStateV zero_velocity;
  INTRINSIC_RT_RETURN_IF_ERROR(zero_velocity.SetSize(ndof_));
  zero_velocity.velocity.setConstant(0.);
  if (!trajectory_generator_.SetTarget(zero_velocity)) {
    return InternalError("Failed to set reflexxes target.");
  }

  // Ignoring joint position limits so that stopping is attempted when starting
  // the action outside of the limits, or if the stop trajectory ends outside of
  // the limits. Such a violation needs to be handled by the part or safety
  // implementation.
  trajectory_generator_.SetPositionalLimitsBehavior(
      reflexxes::Flags::PositionalLimitsBehavior::kIgnore);

  elapsed_time_seconds_ = 0.0;
  // Initialize the criterion to determine the settled state.
  is_settled_ = false;
  INTRINSIC_RT_RETURN_IF_ERROR(is_settled_criterion_->Initialize());

  return OkStatus();
}

RealtimeStatus JointStopAction::Sense(SenseParameters params) {
  INTRINSIC_TRACE_SCOPED("JointStopAction::Sense");
  const JointPosition* const joint_position_interface =
      params.slot_map.GetInterfaceForSlot<JointPosition>(slot_id_);
  if (joint_position_interface == nullptr) {
    return InternalError(
        "Slot doesn't have the JointPosition FeatureInterface.");
  }
  const JointVelocityEstimator* const velocity_estimator =
      params.slot_map.GetInterfaceForSlot<JointVelocityEstimator>(slot_id_);
  if (velocity_estimator == nullptr) {
    return InternalError(FixedStrCat<RealtimeStatus::kMaxMessageLength>(
        "Slot ", slot_id_.value(), " doesn't have JointVelocityEstimator."));
  }
  // The acceleration_estimator is optional.
  const JointAccelerationEstimator* const acceleration_estimator =
      params.slot_map.GetInterfaceForSlot<JointAccelerationEstimator>(slot_id_);

  if (!previous_setpoint_.has_value()) {
    // Initialize previous_setpoint_ with the previous setpoint as reported by
    // the part using fallbacks in case the optional feedforwards are missing.
    //
    // We don't read this every cycle, because we know what command we sent last
    // cycle.
    JointPositionCommand previous_command =
        joint_position_interface->PreviousPositionSetpoints();
    JointStatePVA previous_setpoint;
    INTRINSIC_RT_RETURN_IF_ERROR(previous_setpoint.SetSize(ndof_));
    previous_setpoint.position = previous_command.position();
    previous_setpoint.velocity =
        previous_command.velocity_feedforward().value_or(
            velocity_estimator->GetVelocityEstimate().velocity);
    eigenmath::VectorNd fallback_acceleration =
        eigenmath::VectorNd::Zero(ndof_);
    if (acceleration_estimator == nullptr) {
      INTRINSIC_RT_LOG_THROTTLED(INFO) << "Slot doesn't have the optional "
                                          "JointAccelerationEstimator. Initial "
                                          "Acceleration may be set to zero.";

    } else {
      fallback_acceleration =
          acceleration_estimator->GetAccelerationEstimate().acceleration;
    }
    previous_setpoint.acceleration =
        previous_command.acceleration_feedforward().value_or(
            fallback_acceleration);
    previous_setpoint_ = previous_setpoint;
  }
  done_ = done_buffer_;

  elapsed_time_seconds_ += 1 / frequency_hz_;

  // Update the settled state estimator with new measurements.
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      is_settled_,
      is_settled_criterion_->Update(
          /*measured_joint_velocities=*/velocity_estimator
              ->GetVelocityEstimate()
              .velocity,
          /*commanded_joint_velocities=*/previous_setpoint_->velocity,
          /*has_trajectory_ended=*/done_));

  return OkStatus();
}

RealtimeStatus JointStopAction::Control(ControlParameters params) {
  INTRINSIC_TRACE_SCOPED("JointStopAction::Control");
  if (!previous_setpoint_.has_value()) {
    return InternalError(
        "Previous target is unknown. Did you forget to call Sense() before "
        "Control()?");
  }
  JointPosition* const joint_position_interface =
      params.slot_map.GetMutableInterfaceForSlot<JointPosition>(slot_id_);
  if (joint_position_interface == nullptr) {
    return InternalError(
        "Slot doesn't have the JointPosition FeatureInterface.");
  }

  const JointLimitsInterface* const limits_interface =
      params.slot_map.GetInterfaceForSlot<JointLimitsInterface>(slot_id_);
  if (limits_interface == nullptr) {
    return InternalError("Slot doesn't have JointLimits.");
  }
  JointLimits joint_limits = limits_interface->GetSystemLimits();
  if (!use_joint_jerk_limits_) {
    // Note that deactivating joint jerk limits in torque-limited trajectories
    // can lead to current discontinuities, and that too low joint jerk limits
    // can lead to too slow stopping trajectories.
    joint_limits.max_jerk.setConstant(std::numeric_limits<double>::infinity());
  }
  DynamicLimitsCheckMode joint_dynamic_limits_check_mode =
      DynamicLimitsCheckMode::kCheckJointAcceleration;

  INTRINSIC_RT_RETURN_IF_ERROR(UpdateJointLimitsAndCheckModeWithDynamics(
      limits_interface, previous_setpoint_.value(), slot_id_, joint_limits,
      joint_dynamic_limits_check_mode));

  if (!trajectory_generator_.SetLimits(joint_limits)) {
    return InternalError("Failed to set reflexxes limits.");
  }

  JointStatePVA previous_setpoint_control;
  previous_setpoint_control.position = previous_setpoint_->position;
  previous_setpoint_control.velocity = previous_setpoint_->velocity;
  previous_setpoint_control.acceleration = previous_setpoint_->acceleration;
  if (!trajectory_generator_.SetPrevious(previous_setpoint_control)) {
    return InternalError("Failed to set previous reflexxes target.");
  }
  JointStatePVA new_target;
  INTRINSIC_RT_RETURN_IF_ERROR(new_target.SetSize(ndof_));
  if (!trajectory_generator_.ComputeSetpoint(&new_target)) {
    return InternalError(
        "Failed to compute setpoint for new reflexxes target.");
  }

  // Apply setpoints.
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const auto setpoints,
      JointPositionCommand::Create(
          /*position=*/new_target.position,
          /*velocity_feedforward=*/new_target.velocity,
          /*acceleration_feedforward=*/new_target.acceleration,
          /*joint_dynamic_limits_check_mode=*/joint_dynamic_limits_check_mode));
  INTRINSIC_RT_RETURN_IF_ERROR(
      joint_position_interface->SetPositionSetpoints(setpoints));

  // Only update previous setpoint if everything went well.
  previous_setpoint_ = new_target;
  done_buffer_ = trajectory_generator_.GetReflexxesStatus() ==
                 reflexxes::Status::kFinalStateReached;
  return OkStatus();
}

RealtimeStatusOr<StateVariableValue> JointStopAction::GetStateVariable(
    absl::string_view name) const {
  if (name == kIsDone || name == kIsStopped) {
    return StateVariableValue(done_);
  }
  if (name == kActionElapsedTime) {
    return StateVariableValue(elapsed_time_seconds_);
  }
  if (name == StopInfo::kIsSettled) {
    return StateVariableValue(is_settled_);
  }
  if (name == StopInfo::kIsSettledUncertainty) {
    return StateVariableValue(is_settled_criterion_->GetUncertainty());
  }
  return NotFoundError(FixedStrCat<RealtimeStatus::kMaxMessageLength>(
      "JointStopAction, state variable not found ", name));
}

}  // namespace intrinsic::icon
