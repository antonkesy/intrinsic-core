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

#include "intrinsic/icon/control/parts/feature_interfaces/joint_position_pid_torque_controller_feature.h"

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "flatbuffers/vector.h"
#include "intrinsic/eigenmath/clamp.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/algorithms/joint_position_pid_torque_controller.h"
#include "intrinsic/icon/control/joint_position_command.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/move_checker.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/flatbuffers/transform_view.h"
#include "intrinsic/icon/hal/interfaces/joint_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_state.fbs.h"
#include "intrinsic/icon/proto/joint_position_pid_torque_controller_config.pb.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

// static
absl::StatusOr<JointPositionPidTorqueControllerFeature>
JointPositionPidTorqueControllerFeature::Create(
    JointTorqueCommandHardwareInterface joint_torque_command_hardware_interface,
    JointPositionStateHardwareInterface joint_position_state_hardware_interface,
    JointVelocityStateHardwareInterface joint_velocity_state_hardware_interface,
    intrinsic_proto::icon::ArmPositionPidTorqueControllerConfig config,
    double control_frequency_hz,
    const JointLimitsInterface* joint_limits_interface) {
  if (*joint_torque_command_hardware_interface == nullptr) {
    return absl::FailedPreconditionError(
        "Hardware interface handle for JointTorqueCommandFeature is not "
        "initialized.");
  }
  if (*joint_position_state_hardware_interface == nullptr) {
    return absl::FailedPreconditionError(
        "Hardware interface handle for JointPositionStateFeature is not "
        "initialized.");
  }
  if (*joint_velocity_state_hardware_interface == nullptr) {
    return absl::FailedPreconditionError(
        "Hardware interface handle for JointVelocityStateFeature is not "
        "initialized.");
  }
  if (joint_position_state_hardware_interface->position()->size() !=
      joint_velocity_state_hardware_interface->velocity()->size()) {
    return absl::FailedPreconditionError(
        "The position and velocity hardware interfaces have different sizes.");
  }
  if (joint_position_state_hardware_interface->position()->size() !=
      joint_torque_command_hardware_interface->torque()->size()) {
    return absl::FailedPreconditionError(
        "The torque interface has different size from the position and "
        "velocity interfaces.");
  }

  INTR_ASSIGN_OR_RETURN(
      auto move_checker,
      MoveChecker::Create(
          joint_position_state_hardware_interface->position()->size(),
          control_frequency_hz));

  // Set the initial and previous setpoint based on the current sensed position
  // and zero velocity and acceleration.
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      JointPositionCommand position_setpoints,
      JointPositionCommand::Create(
          ::intrinsic_fbs ::ViewAs<::intrinsic ::eigenmath ::VectorNd>(
              (joint_position_state_hardware_interface)->position()),
          eigenmath::VectorNd::Zero(
              joint_position_state_hardware_interface->position()->size()),
          eigenmath::VectorNd::Zero(
              joint_position_state_hardware_interface->position()->size())));

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      JointPositionCommand previous_setpoints,
      JointPositionCommand::Create(
          ::intrinsic_fbs ::ViewAs<::intrinsic ::eigenmath ::VectorNd>(
              (joint_position_state_hardware_interface)->position()),
          eigenmath::VectorNd::Zero(
              joint_position_state_hardware_interface->position()->size()),
          eigenmath::VectorNd::Zero(
              joint_position_state_hardware_interface->position()->size())));

  INTRINSIC_RT_RETURN_IF_ERROR(
      move_checker->UpdatePreviousSetpoint(previous_setpoints));

  JointStatePV sensed_state;
  INTR_RETURN_IF_ERROR(sensed_state.SetSize(
      joint_position_state_hardware_interface->position()->size()));
  std::vector<std::unique_ptr<JointPositionPidTorqueController>> controllers;
  if (config.joint_configs().size() != sensed_state.size()) {
    return absl::InvalidArgumentError(
        "The number of configured joint controllers doesn't match the number "
        "of degrees of freedom.");
  }
  for (const auto& controller_config : config.joint_configs()) {
    INTR_ASSIGN_OR_RETURN(
        auto controller,
        JointPositionPidTorqueController::Create(controller_config));
    controllers.push_back(std::move(controller));
  }
  return JointPositionPidTorqueControllerFeature(
      std::move(joint_torque_command_hardware_interface),
      std::move(joint_position_state_hardware_interface),
      std::move(joint_velocity_state_hardware_interface),
      std::move(sensed_state), std::move(position_setpoints),
      std::move(previous_setpoints), control_frequency_hz,
      joint_limits_interface, std::move(move_checker), std::move(controllers));
}

RealtimeStatus JointPositionPidTorqueControllerFeature::SetPositionSetpoints(
    const JointPositionCommand& setpoints) {
  if (setpoints.Size() !=
      joint_torque_command_hardware_interface_->torque()->size()) {
    return icon::InvalidArgumentError(
        "Invalid setpoints: Wrong number of DoFs.");
  }
  // TODO(b/270674017): Limit check in HalArmPart should check against
  // application limits by default - and special case safety (and feedback
  // actions).
  INTRINSIC_RT_RETURN_IF_ERROR(move_checker_->CheckSetpoint(
      setpoints, joint_limits_interface_->GetSystemLimits(),
      joint_limits_interface_->GetSystemLimits()));

  position_setpoints_ = setpoints;
  updated_time_ = Clock::now();

  return OkStatus();
}

RealtimeStatus JointPositionPidTorqueControllerFeature::ReadStatus(
    RealtimePartInterface::ReadStatusParameters params) {
  // If we did not receive a position command in the previous cycle, then set
  // previous_setpoints_ to the current sensed values.
  if (position_setpoints_ == std::nullopt) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        previous_setpoints_,
        JointPositionCommand::Create(
            ::intrinsic_fbs ::ViewAs<::intrinsic ::eigenmath ::VectorNd>(
                (joint_position_state_hardware_interface_)->position()),
            eigenmath::VectorNd::Zero(
                joint_position_state_hardware_interface_->position()->size()),
            eigenmath::VectorNd::Zero(
                joint_position_state_hardware_interface_->position()->size())));

    INTRINSIC_RT_RETURN_IF_ERROR(
        move_checker_->UpdatePreviousSetpoint(previous_setpoints_));
  }

  updated_time_ = std::nullopt;
  position_setpoints_ = std::nullopt;

  for (int i = 0; i < sensed_state_.size(); ++i) {
    sensed_state_.position[i] =
        joint_position_state_hardware_interface_->position()->Get(i);
    sensed_state_.velocity[i] =
        joint_velocity_state_hardware_interface_->velocity()->Get(i);
  }

  return OkStatus();
}

RealtimeStatus JointPositionPidTorqueControllerFeature::ApplyCommand(
    RealtimePartInterface::ApplyCommandParameters params) {
  if (!position_setpoints_.has_value()) {
    position_setpoints_numdiff_.Append(sensed_state_.position);
    // This should only happen if ApplyCommand was called prior to the first
    // call to SetPositionSetpoints. In this case, we will just not do anything
    // and allow the hardware module to reason about the update time and take
    // care of itself.
    return OkStatus();
  }

  position_setpoints_numdiff_.Append(position_setpoints_->position());
  JointLimits system_limits = joint_limits_interface_->GetSystemLimits();

  eigenmath::VectorNd velocity_estimate;
  eigenmath::VectorNd acceleration_estimate;

  if (position_setpoints_.value().velocity_feedforward().has_value()) {
    velocity_estimate =
        position_setpoints_.value().velocity_feedforward().value();
  } else {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        velocity_estimate,
        position_setpoints_numdiff_.FirstDerivativeBackward(kNumDiffOrder));
    if (!eigenmath::ClampVector(-system_limits.max_velocity,
                                system_limits.max_velocity,
                                velocity_estimate)) {
      return icon::InternalError(
          "Clamping the velocity feedforwards failed. This is an ICON "
          "programming error.");
    }
  }
  if (position_setpoints_.value().acceleration_feedforward().has_value()) {
    for (int i = 0; i < position_setpoints_.value().Size(); ++i) {
      acceleration_estimate =
          position_setpoints_.value().acceleration_feedforward().value();
    }
  } else {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        acceleration_estimate,
        position_setpoints_numdiff_.SecondDerivativeBackward(kNumDiffOrder));
    if (!eigenmath::ClampVector(-system_limits.max_acceleration,
                                system_limits.max_acceleration,
                                acceleration_estimate)) {
      return icon::InternalError(
          "Clamping the acceleration feedforwards failed. This is an ICON "
          "programming error.");
    }
  }

  // Call the torque controller for each joint.
  for (int i = 0; i < controllers_.size(); ++i) {
    INTRINSIC_RT_ASSIGN_OR_RETURN(
        auto single_joint_command,
        JointPositionCommand::Create(
            eigenmath::VectorNd({{position_setpoints_.value().position()[i]}}),
            eigenmath::VectorNd({{velocity_estimate[i]}}),
            eigenmath::VectorNd({{acceleration_estimate[i]}})));

    // Extract the state for a single dof.
    JointStatePV single_sensed_state;
    INTRINSIC_RT_RETURN_IF_ERROR(single_sensed_state.SetSize(1));
    single_sensed_state.position(0) = sensed_state_.position(i);
    single_sensed_state.velocity(0) = sensed_state_.velocity(i);

    bool controller_success = controllers_[i]->CalculateFeedbackSetPoints(
        single_joint_command, single_sensed_state);
    if (!controller_success) {
      return icon::InternalError(
          "Failed to calculate the torque setpoint. Something went wrong.");
    }
    joint_torque_command_hardware_interface_->mutable_torque()->Mutate(
        i, controllers_[i]->GetTargetTorque());
  }

  if (updated_time_.has_value()) {
    joint_torque_command_hardware_interface_.UpdatedAt(*updated_time_);
  }

  previous_setpoints_ = position_setpoints_.value();

  INTRINSIC_RT_RETURN_IF_ERROR(
      move_checker_->UpdatePreviousSetpoint(previous_setpoints_));

  return icon::OkStatus();
}

RealtimeStatus JointPositionPidTorqueControllerFeature::Reset() {
  position_setpoints_numdiff_.Reset();

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      previous_setpoints_,
      JointPositionCommand::Create(
          ::intrinsic_fbs ::ViewAs<::intrinsic ::eigenmath ::VectorNd>(
              (joint_position_state_hardware_interface_)->position()),
          eigenmath::VectorNd::Zero(
              joint_position_state_hardware_interface_->position()->size()),
          eigenmath::VectorNd::Zero(
              joint_position_state_hardware_interface_->position()->size())));

  INTRINSIC_RT_RETURN_IF_ERROR(
      move_checker_->UpdatePreviousSetpoint(previous_setpoints_));

  for (int i = 0; i < controllers_.size(); ++i) {
    controllers_[i]->Reset();
    joint_torque_command_hardware_interface_->mutable_torque()->Mutate(
        i, controllers_[i]->GetTargetTorque());
  }

  return icon::OkStatus();
}

JointPositionCommand
JointPositionPidTorqueControllerFeature::PreviousPositionSetpoints() const {
  return previous_setpoints_;
}

JointPositionPidTorqueControllerFeature::
    JointPositionPidTorqueControllerFeature(
        JointTorqueCommandHardwareInterface
            joint_torque_command_hardware_interface,
        JointPositionStateHardwareInterface
            joint_position_state_hardware_interface,
        JointVelocityStateHardwareInterface
            joint_velocity_state_hardware_interface,
        JointStatePV sensed_state, JointPositionCommand position_setpoints,
        JointPositionCommand previous_setpoints, double control_frequency_hz,
        const JointLimitsInterface* joint_limits_interface,
        std::unique_ptr<MoveChecker> move_checker,
        std::vector<std::unique_ptr<JointPositionPidTorqueController>>
            controllers)
    : joint_limits_interface_(joint_limits_interface),
      position_setpoints_numdiff_(/*dt_sec=*/1.0 / control_frequency_hz),
      joint_torque_command_hardware_interface_(
          std::move(joint_torque_command_hardware_interface)),
      joint_position_state_hardware_interface_(
          std::move(joint_position_state_hardware_interface)),
      joint_velocity_state_hardware_interface_(
          std::move(joint_velocity_state_hardware_interface)),
      position_setpoints_(std::move(position_setpoints)),
      previous_setpoints_(std::move(previous_setpoints)),
      move_checker_(std::move(move_checker)),
      sensed_state_(sensed_state),
      controllers_(std::move(controllers)) {}

}  // namespace intrinsic::icon
