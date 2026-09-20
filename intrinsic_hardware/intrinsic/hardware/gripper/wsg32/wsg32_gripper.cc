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

#include "intrinsic/hardware/gripper/wsg32/wsg32_gripper.h"

#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/strings/substitute.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "google/protobuf/any.pb.h"
#include "google/protobuf/util/message_differencer.h"
#include "intrinsic/hardware/gripper/gripper.h"
#include "intrinsic/hardware/gripper/gripper.pb.h"
#include "intrinsic/hardware/gripper/service/generic_pinch_gripper_utils.h"
#include "intrinsic/hardware/gripper/service/proto/generic_pinch_gripper_configs.pb.h"
#include "intrinsic/hardware/gripper/wsg32/wsg32_client.h"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "intrinsic/util/cyclic_runner.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::gripper {
namespace {

using intrinsic::gripper::Wsg32Client;
using ::intrinsic_proto::gripper::GenericPinchGripperCommunicationConfig;
using ::intrinsic_proto::gripper::PinchGripperStatus;

intrinsic_proto::gripper::PinchGripperCommand DefaultPinchGripperCommand() {
  intrinsic_proto::gripper::PinchGripperCommand command;
  command.set_command_id(absl::ToUnixSeconds(absl::Now()));
  command.set_position_percentage(0.0);
  command.set_velocity_percentage(50.0);
  command.set_effort_percentage(100.0);
  return command;
}

}  // namespace

absl::StatusOr<std::unique_ptr<PinchGripperInterface>> Wsg32Gripper::Create(
    const std::optional<google::protobuf::Any>& any_config) {
  intrinsic_proto::gripper::GenericPinchGripperConfig config;
  if (any_config.has_value() && !any_config->UnpackTo(&config)) {
    return absl::InvalidArgumentError("Failed to unpack config.");
  }
  INTR_ASSIGN_OR_RETURN(auto comm_interface,
                        CreateCommunicationInterface(config.comm_config()));
  auto pinch_gripper = std::make_unique<Wsg32Gripper>(
      std::move(comm_interface), config.additional_config());
  return std::unique_ptr<PinchGripperInterface>(std::move(pinch_gripper));
}

Wsg32Gripper::Wsg32Gripper(
    std::unique_ptr<Wsg32Client> comm_interface,
    const intrinsic_proto::gripper::PinchGripperAdditionalConfig&
        additional_config)
    : PinchGripperInterface(),
      comm_interface_(std::move(comm_interface)),
      cyclic_routine_frequency_hz_(
          additional_config.cyclic_routine_frequency_hz()),
      status_topic_(additional_config.status_topic()),
      cyclic_keep_alive_routine_running_(false),
      last_gripper_command_(DefaultPinchGripperCommand()),
      pub_(pubsub_.CreatePublisher(status_topic_, TopicConfig()).value()) {}

absl::StatusOr<std::unique_ptr<Wsg32Client>>
Wsg32Gripper::CreateCommunicationInterface(
    const GenericPinchGripperCommunicationConfig& config) {
  return Wsg32Client::Create(config.ip_address(), config.port(), true);
}

absl::Status Wsg32Gripper::ValidateAdditionalConfig(
    const intrinsic_proto::gripper::PinchGripperAdditionalConfig&
        additional_config) {
  if (!additional_config.has_cyclic_routine_frequency_hz()) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "Missing cyclic_routine_frequency_hz parameter in the additional "
              "config.";
  } else if (additional_config.cyclic_routine_frequency_hz() <= 0) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "Gripper cyclic routine frequency: "
           << additional_config.cyclic_routine_frequency_hz()
           << " Hz is invalid! It should be > 0 Hz instead.";
  }
  if (!additional_config.has_status_topic()) {
    return ::intrinsic::InvalidArgumentErrorBuilder()
           << "Missing status_topic parameter in the additional config.";
  }
  return absl::OkStatus();
}

absl::Status Wsg32Gripper::Configure(
    const intrinsic_proto::gripper::PinchGripperPhysicalConfig& config,
    bool validate_si_position_config, bool validate_si_velocity_config,
    bool validate_si_effort_config) {
  INTR_RETURN_IF_ERROR(CheckConfigValidity(config, validate_si_position_config,
                                           validate_si_velocity_config,
                                           validate_si_effort_config))
      .LogError();
  absl::MutexLock lock(config_mutex_);
  config_ = config;
  return absl::OkStatus();
}

absl::Status Wsg32Gripper::Enable() {
  LOG(INFO) << "Enable gripper.";
  absl::MutexLock lock(channel_mutex_);
  if (comm_interface_ == nullptr) {
    return absl::InternalError("comm_interface_ is null.");
  }
  if (!comm_interface_->Connect(true).ok()) {
    return absl::InternalError("Failed to connect to gripper.");
  }
  return absl::OkStatus();
}

absl::Status Wsg32Gripper::Disable() {
  LOG(INFO) << "Disable gripper.";
  absl::MutexLock lock(channel_mutex_);
  comm_interface_->Disconnect();
  return absl::OkStatus();
}

absl::Status Wsg32Gripper::Reset() {
  LOG(INFO) << "Reset gripper.";
  INTR_RETURN_IF_ERROR(Disable());
  absl::SleepFor(absl::Seconds(1));
  INTR_RETURN_IF_ERROR(Enable());
  absl::SleepFor(absl::Seconds(1));
  return absl::OkStatus();
}

absl::StatusOr<PinchGripperStatus> Wsg32Gripper::ExecuteCommand(
    const intrinsic_proto::gripper::PinchGripperCommand& command,
    absl::Time deadline, bool waiting_motion_stopped) {
  {
    absl::MutexLock lock(command_mutex_);
    user_command_executed_ = false;
  }

  INTR_RETURN_IF_ERROR(CommandAndUpdate(command));

  if (waiting_motion_stopped && cyclic_keep_alive_routine_running_) {
    // Wait for the execution to terminate (i.e. user_command_executed_ flag
    // being set in the GetStatus() function) or for a time-out.
    const absl::Time start = absl::Now();
    if (command_mutex_.LockWhenWithDeadline(
            absl::Condition(&user_command_executed_), deadline)) {
      command_mutex_.unlock();
      absl::MutexLock lock(status_mutex_);
      return last_gripper_status_;
    } else {
      command_mutex_.unlock();
      const absl::Duration lock_duration = absl::Now() - start;
      return absl::DeadlineExceededError(absl::Substitute(
          "Failed to execute gripper command within deadline. Tried for $0 ms.",
          absl::ToDoubleMilliseconds(lock_duration)));
    }
  } else {
    return GetStatus();
  }
}

absl::Status Wsg32Gripper::CommandAndUpdate(
    const intrinsic_proto::gripper::PinchGripperCommand& command) {
  // Send the received gripper command.
  LOG(INFO) << "Received gripper command:\n" << command;
  {
    absl::MutexLock lock(command_mutex_);
    last_gripper_command_ = command;
  }
  return SendCommand(command);
}

absl::Status Wsg32Gripper::SendCommand(
    const intrinsic_proto::gripper::PinchGripperCommand& command) {
  VLOG(1) << "Sending gripper command:\n" << command;

  // Command_id has the time stamp of this command.
  wsg32_command_.id = command.command_id();

  // Extract command details, use defaults for missing fields.
  {
    absl::MutexLock lock(config_mutex_);

    // Converting position command either from SI units or percentage
    // to hardware values:
    if (command.has_position()) {
      INTR_ASSIGN_OR_RETURN(
          wsg32_command_.position,
          ConvertSiToHardwareValue(command.position(), config_.position()));
    } else if (command.has_position_percentage()) {
      INTR_ASSIGN_OR_RETURN(
          wsg32_command_.position,
          ConvertPercentageToHardwareValue(command.position_percentage(),
                                           config_.position()));
    } else {
      INTR_ASSIGN_OR_RETURN(
          wsg32_command_.position,
          ConvertSiToHardwareValue(config_.position().default_si_value(),
                                   config_.position()));
    }

    // Converting velocity command either from SI units or percentage
    // to hardware values:
    if (command.has_velocity()) {
      INTR_ASSIGN_OR_RETURN(
          wsg32_command_.speed,
          ConvertSiToHardwareValue(command.velocity(), config_.velocity()));
    } else if (command.has_velocity_percentage()) {
      INTR_ASSIGN_OR_RETURN(
          wsg32_command_.speed,
          ConvertPercentageToHardwareValue(command.velocity_percentage(),
                                           config_.velocity()));
    } else {
      INTR_ASSIGN_OR_RETURN(
          wsg32_command_.speed,
          ConvertSiToHardwareValue(config_.velocity().default_si_value(),
                                   config_.velocity()));
    }

    // Converting effort command either from SI units or percentage
    // to hardware values:
    if (command.has_effort()) {
      INTR_ASSIGN_OR_RETURN(
          wsg32_command_.force,
          ConvertSiToHardwareValue(command.effort(), config_.effort()));
    } else if (command.has_effort_percentage()) {
      INTR_ASSIGN_OR_RETURN(wsg32_command_.force,
                            ConvertPercentageToHardwareValue(
                                command.effort_percentage(), config_.effort()));
    } else {
      INTR_ASSIGN_OR_RETURN(
          wsg32_command_.force,
          ConvertSiToHardwareValue(config_.effort().default_si_value(),
                                   config_.effort()));
    }
  }

  // Keep memory of command in wsg32_last_command_.
  wsg32_last_command_ = wsg32_command_;

  // Send the command to WSG32 gripper.
  absl::MutexLock lock(channel_mutex_);
  if (comm_interface_->Move(wsg32_command_.position, wsg32_command_.speed,
                            wsg32_command_.force)) {
    return absl::OkStatus();
  }

  return absl::InternalError("Failed to send gripper command.");
}

absl::Status Wsg32Gripper::RunCyclicKeepAliveRoutine() {
  PinchGripperStatus gripper_status;
  PinchGripperStatus last_gripper_status;

  // Query and publish the gripper status at `rate` Hz.
  absl::Status receive_status;
  intrinsic::CyclicRunner cyclic_publisher;
  LOG(INFO) << "Publishing status to topic: " << status_topic_;
  LOG(INFO) << "Starting the CyclicRunner for periodically sending a command "
               "to the gripper at cyclic_routine_frequency_hz="
            << cyclic_routine_frequency_hz_ << " Hz.";
  cyclic_keep_alive_routine_running_ = true;
  if (auto status = cyclic_publisher.Start(
          [=, this, &receive_status, &gripper_status, &last_gripper_status]() {
            bool gripper_enabled;
            {
              absl::MutexLock lock(channel_mutex_);
              gripper_enabled = comm_interface_->IsConnected();
            }
            if (gripper_enabled) {
              // Get gripper status message.
              auto temp_status_or = GetStatus();
              if (!temp_status_or.ok()) {
                LOG(ERROR)
                    << "Failed to get status from gripper, with error message: "
                    << temp_status_or.status().message();
                receive_status = temp_status_or.status();
              } else {
                gripper_status = temp_status_or.value();

                // If publishing fails for any reason, stop publishing and quit.
                receive_status = pub_.Publish(gripper_status);
                if (!receive_status.ok()) {
                  LOG(ERROR) << absl::StrFormat(
                      "Failed to publish on topic: '%s', with error message: "
                      "'%s'",
                      status_topic_, receive_status.message());
                }
                if (!google::protobuf::util::MessageDifferencer::Equals(
                        gripper_status, last_gripper_status)) {
                  VLOG(1)
                      << "Published gripper message:\n"
                      << absl::StrCat(
                             "actual position: ", gripper_status.position(),
                             "\nrequested position: ",
                             gripper_status.position_requested(),
                             "\nreached: ", gripper_status.position_reached(),
                             "\nin_motion: ", gripper_status.in_motion(),
                             "\nenabled: ", gripper_status.gripper_enabled());

                  last_gripper_status = gripper_status;
                }
              }
            }
          },
          absl::Milliseconds(1000.0 / cyclic_routine_frequency_hz_));
      !status.ok()) {
    cyclic_keep_alive_routine_running_ = false;
    LOG(ERROR) << "cyclic_publisher.Start() returns with error: "
               << status.message();
  }
  return receive_status;
}

absl::StatusOr<PinchGripperStatus> Wsg32Gripper::GetStatus() {
  PinchGripperStatus status;

  {
    absl::MutexLock lock(channel_mutex_);
    if (comm_interface_ == nullptr) {
      return absl::UnavailableError("Invalid interface.");
    }
  }

  {
    // Read the status structure from WSG32 gripper.
    absl::MutexLock lock(channel_mutex_);
    wsg32_status_ = comm_interface_->GetStatus();
  }

  {
    absl::MutexLock lock(config_mutex_);
    status.set_position(
        PositionFromHardwareValue(config_, wsg32_status_.position));
    status.set_position_requested(
        PositionFromHardwareValue(config_, wsg32_last_command_.position));
  }

  status.set_in_motion(wsg32_status_.is_moving);
  status.set_command_id(wsg32_last_command_.id);

  // Determine whether the desired position was reached.
  if (fabs(wsg32_status_.position - wsg32_last_command_.position) <
      kPositionReachedThresholdMm) {
    status.set_position_reached(true);
  } else {
    status.set_position_reached(false);
  }

  // Determine whether an object was detected -- not very precise.
  if (!status.position_reached() && !status.in_motion()) {
    status.set_object_detected(true);
  } else {
    status.set_object_detected(false);
  }

  // Determine whether the gripper is enabled.
  {
    absl::MutexLock lock(channel_mutex_);
    status.set_gripper_enabled(comm_interface_->IsConnected());
  }

  VLOG(1) << "Gripper::GetStatus(): "
          << absl::StrCat("actual position: ", status.position(),
                          ", requested position: ", status.position_requested(),
                          ", reached: ", status.position_reached(),
                          ", in_motion: ", status.in_motion(),
                          ", enabled: ", status.gripper_enabled());

  {
    absl::MutexLock status_lock(status_mutex_);
    last_gripper_status_ = status;
  }

  absl::MutexLock command_lock(command_mutex_);
  const bool has_last_gripper_command_finished =
      !user_command_executed_ && !status.in_motion() &&
      (status.command_id() == last_gripper_command_.command_id());
  if (has_last_gripper_command_finished) {
    user_command_executed_ = true;
  }
  return status;
}

}  // namespace intrinsic::gripper
