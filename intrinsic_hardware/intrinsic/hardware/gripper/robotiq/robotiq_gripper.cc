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

#include "intrinsic/hardware/gripper/robotiq/robotiq_gripper.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

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
#include "intrinsic/hardware/gripper/robotiq/hardware_registers.h"
#include "intrinsic/hardware/gripper/service/generic_pinch_gripper_utils.h"
#include "intrinsic/hardware/gripper/service/proto/generic_pinch_gripper_configs.pb.h"
#include "intrinsic/hardware/modbus_tcp/modbus_tcp.h"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "intrinsic/util/cyclic_runner.h"
#include "intrinsic/util/status/status_builder.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::gripper {
namespace {

using ::intrinsic::hardware::ModbusTcp;
using ::intrinsic_proto::gripper::GenericPinchGripperCommunicationConfig;
using ::intrinsic_proto::gripper::PinchGripperStatus;

intrinsic_proto::gripper::PinchGripperCommand DefaultPinchGripperCommand() {
  intrinsic_proto::gripper::PinchGripperCommand command;
  command.set_command_id(absl::ToUnixSeconds(absl::Now()));
  command.set_position_percentage(0.0);
  command.set_velocity_percentage(0.0);
  command.set_effort_percentage(0.0);
  return command;
}

}  // namespace

constexpr absl::Duration kHardwareSmallSleepDuration = absl::Milliseconds(200);

absl::StatusOr<std::unique_ptr<PinchGripperInterface>> RobotiqGripper::Create(
    const std::optional<google::protobuf::Any>& any_config) {
  intrinsic_proto::gripper::GenericPinchGripperConfig config;
  if (any_config.has_value() && !any_config->UnpackTo(&config)) {
    return absl::InvalidArgumentError("Failed to unpack config.");
  }
  INTR_ASSIGN_OR_RETURN(auto comm_interface,
                        CreateCommunicationInterface(config.comm_config()));
  INTR_RETURN_IF_ERROR(ValidateAdditionalConfig(config.additional_config()));
  auto pinch_gripper = std::make_unique<RobotiqGripper>(
      std::move(comm_interface), config.additional_config());
  return std::unique_ptr<PinchGripperInterface>(std::move(pinch_gripper));
}

RobotiqGripper::RobotiqGripper(
    std::unique_ptr<ModbusTcp> comm_interface,
    const intrinsic_proto::gripper::PinchGripperAdditionalConfig&
        additional_config)
    : PinchGripperInterface(),
      comm_interface_(std::move(comm_interface)),
      cyclic_routine_frequency_hz_(
          additional_config.cyclic_routine_frequency_hz()),
      status_topic_(additional_config.status_topic()),
      cyclic_keep_alive_routine_running_(false),
      has_been_reset_(false),
      reset_signal_sent_(false),
      received_first_user_command_(false),
      blocking_command_executing_(false),
      last_gripper_command_(DefaultPinchGripperCommand()),
      pub_(pubsub_.CreatePublisher(status_topic_, TopicConfig()).value()) {
  if (additional_config.has_command_topic()) {
    // Set up pubsub listener for commands.
    auto sub_or = pubsub_.CreateSubscription<
        intrinsic_proto::gripper::PinchGripperCommand>(
        additional_config.command_topic(), TopicConfig(),
        [this](const intrinsic_proto::gripper::PinchGripperCommand& message) {
          // Note, we specifically set the deadline in the past as we treat the
          // commands that come here as best effort, and blocking commands
          // should take precedence.
          auto status =
              ExecuteCommand(message, absl::Now() - absl::Seconds(1), false);
          // Errors are dropped.
          if (!status.ok()) {
            LOG_EVERY_N_SEC(ERROR, 10) << "Failed to handle pubsub request:\n"
                                       << message;
          }
        },
        [](absl::string_view packet, absl::Status error) {
          LOG(ERROR) << "Failed to read incoming pubsub command. Error: "
                     << error;
        });
    if (!sub_or.ok()) {
      LOG(ERROR) << "Failed to set up pubsub listener for commands. Will not "
                 << "respond to pubsub commands! Error: " << sub_or.status();
    } else {
      sub_ = std::move(sub_or).value();
    }
  }
}

absl::StatusOr<std::unique_ptr<ModbusTcp>>
RobotiqGripper::CreateCommunicationInterface(
    const GenericPinchGripperCommunicationConfig& config) {
  return ModbusTcp::Create(config.ip_address(), config.port());
}

absl::Status RobotiqGripper::ValidateAdditionalConfig(
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

absl::Status RobotiqGripper::Configure(
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

absl::Status RobotiqGripper::Enable() {
  LOG(INFO) << "Enable gripper.";
  command_.action |= Action::Active;
  return Write();
}

absl::Status RobotiqGripper::Disable() {
  LOG(INFO) << "Disable gripper.";
  command_.action &= ~(Action::Active | Action::GoTo);
  return Write();
}

absl::Status RobotiqGripper::Reset() {
  LOG(INFO) << "Reset gripper.";
  INTR_RETURN_IF_ERROR(Disable());
  absl::SleepFor(absl::Seconds(1));
  INTR_RETURN_IF_ERROR(Enable());
  absl::SleepFor(absl::Seconds(1));
  has_been_reset_ = true;
  reset_signal_sent_ = true;
  return absl::OkStatus();
}

absl::StatusOr<PinchGripperStatus> RobotiqGripper::ExecuteCommand(
    const intrinsic_proto::gripper::PinchGripperCommand& command,
    absl::Time deadline, bool waiting_motion_stopped) {
  if (!has_been_reset_) {
    return absl::InternalError(
        "Gripper has not been reset! Cannot execute command.");
  }

  {
    absl::MutexLock lock(command_mutex_);
    user_command_executed_ = false;

    // Regardless of whether we are blocking or not, check whether another
    // blocking call is running, and return an error if so.
    //
    // One could argue that a blocking call with a timeout should wait to try to
    // execute, but it seems like a general anti-pattern if we're issuing
    // multiple commands to a single gripper at once. This can easily be
    // changed if we want to implement waiting behavior.
    if (blocking_command_executing_) {
      return absl::ResourceExhaustedError(
          "A blocking command is currently running.");
    }

    if (waiting_motion_stopped) {
      // We are blocking. Mark the boolean so other calls know a blocking call
      // is executing.
      blocking_command_executing_ = true;
    }
  }

  INTR_RETURN_IF_ERROR(CommandAndUpdate(command));

  if (waiting_motion_stopped && cyclic_keep_alive_routine_running_) {
    // Wait for the execution to terminate (i.e. user_command_executed_ flag
    // being set in the GetStatus() function) or for a time-out.
    const absl::Time start = absl::Now();
    if (command_mutex_.LockWhenWithDeadline(
            absl::Condition(&user_command_executed_), deadline)) {
      blocking_command_executing_ = false;
      command_mutex_.unlock();
      absl::ReaderMutexLock lock(status_mutex_);
      LOG(INFO)
          << "Returning from RobotiqGripper::ExecuteCommand() on `command`:\n"
          << command << "\n with `last_gripper_status_`:\n"
          << last_gripper_status_;
      return last_gripper_status_;
    } else {
      blocking_command_executing_ = false;
      command_mutex_.unlock();
      const absl::Duration lock_duration = absl::Now() - start;
      return absl::DeadlineExceededError(absl::Substitute(
          "Failed to execute gripper command within deadline. Tried for $0 ms.",
          absl::ToDoubleMilliseconds(lock_duration)));
    }
  } else {
    LOG(INFO) << "waiting_motion_stopped = " << waiting_motion_stopped;
    LOG(INFO) << "cyclic_keep_alive_routine_running_ = "
              << cyclic_keep_alive_routine_running_;
    INTR_ASSIGN_OR_RETURN(PinchGripperStatus pinch_gripper_status, GetStatus());
    LOG(INFO)
        << "Returning from RobotiqGripper::ExecuteCommand() after calling "
           "GetStatus() with return value:\n"
        << pinch_gripper_status;
    return pinch_gripper_status;
  }
}

absl::Status RobotiqGripper::CommandAndUpdate(
    const intrinsic_proto::gripper::PinchGripperCommand& command) {
  // Send the received gripper command.
  LOG(INFO) << "Received gripper command:\n" << command;
  {
    absl::MutexLock lock(command_mutex_);
    last_gripper_command_ = command;
  }
  received_first_user_command_ = true;
  return SendCommand(command);
}

absl::Status RobotiqGripper::SendCommand(
    const intrinsic_proto::gripper::PinchGripperCommand& command) {
  VLOG(1) << "Sending gripper command:\n" << command;
  if (command.has_position() || command.has_position_percentage() ||
      command.has_velocity() || command.has_velocity_percentage() ||
      command.has_effort() || command.has_effort_percentage()) {
    absl::MutexLock lock(config_mutex_);
    // Converting position command either from SI units or percentage
    // to hardware values:
    if (command.has_position()) {
      INTR_ASSIGN_OR_RETURN(
          command_.position,
          ConvertSiToHardwareValue(command.position(), config_.position()));
    } else if (command.has_position_percentage()) {
      INTR_ASSIGN_OR_RETURN(
          command_.position,
          ConvertPercentageToHardwareValue(command.position_percentage(),
                                           config_.position()));
    }

    // Converting velocity command either from SI units or percentage
    // to hardware values:
    if (command.has_velocity()) {
      INTR_ASSIGN_OR_RETURN(
          command_.speed,
          ConvertSiToHardwareValue(command.velocity(), config_.velocity()));
    } else if (command.has_velocity_percentage()) {
      INTR_ASSIGN_OR_RETURN(command_.speed, ConvertPercentageToHardwareValue(
                                                command.velocity_percentage(),
                                                config_.velocity()));
    }

    // Converting effort command either from SI units or percentage
    // to hardware values:
    if (command.has_effort()) {
      INTR_ASSIGN_OR_RETURN(
          command_.force,
          ConvertSiToHardwareValue(command.effort(), config_.effort()));
    } else if (command.has_effort_percentage()) {
      INTR_ASSIGN_OR_RETURN(command_.force,
                            ConvertPercentageToHardwareValue(
                                command.effort_percentage(), config_.effort()));
    }
  }

  last_command_.id = command.command_id();
  last_command_.position = command_.position;

  command_.action |= Action::GoTo;
  return Write();
}

absl::Status RobotiqGripper::RunCyclicKeepAliveRoutine() {
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
                LOG(INFO) << "Published gripper status message:\n"
                          << gripper_status;
                last_gripper_status = gripper_status;
              }

              // Re-sending the last command to keep the connection alive.
              if (received_first_user_command_ &&
                  gripper_status.gripper_enabled()) {
                absl::MutexLock lock(command_mutex_);
                if (!SendCommand(last_gripper_command_).ok()) {
                  LOG(ERROR) << "Error sending the last gripper command:\n"
                             << last_gripper_command_;
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

absl::StatusOr<PinchGripperStatus> RobotiqGripper::GetStatus() {
  PinchGripperStatus status;
  {
    absl::MutexLock lock(channel_mutex_);
    if (comm_interface_ == nullptr) {
      return absl::UnavailableError("Invalid interface.");
    }
  }

  std::vector<uint16_t> data;
  for (int i = 0; i < kMaxNumRegisterReadWriteRetrials; i++) {
    absl::StatusOr<std::vector<uint16_t>> read_status_or;
    {
      absl::MutexLock lock(channel_mutex_);
      read_status_or = comm_interface_->read(kStatusDataLength);
    }
    if (read_status_or.ok()) {
      data = read_status_or.value();
      break;
    } else {
      LOG(INFO) << "GetStatus(): register-reading retry #" << i + 1 << "/"
                << kMaxNumRegisterReadWriteRetrials
                << " fails with error message: "
                << read_status_or.status().message();
      if (i < kMaxNumRegisterReadWriteRetrials - 1) {
        // sleep for a small time to give the hardware sometime to finish prior
        // operations, before retrying...
        absl::SleepFor(kHardwareSmallSleepDuration);
      } else {
        LOG(ERROR) << "GetStatus() failed all "
                   << kMaxNumRegisterReadWriteRetrials << " retries.";
        return read_status_or.status();
      }
    }
  }

  for (size_t i = 0; i < kStatusDataLength; i++) {
    status_.raw[i] = data[i];
  }

  {
    absl::MutexLock lock(config_mutex_);
    status.set_position(PositionFromHardwareValue(config_, status_.position));
    status.set_position_requested(
        PositionFromHardwareValue(config_, status_.position_echo));
  }

  // TODO(williambaker): status.set_faults(status_.faults);
  // TODO(williambaker): status.set_effort(status_.current);
  bool in_motion = (status_.status & 0xc0) == 0x00;
  status.set_in_motion(in_motion);
  status.set_position_reached((status_.status & 0xc0) == 0xc0);
  status.set_gripper_enabled((status_.status & 0x09) == 0x09);
  status.set_object_detected(
      intrinsic::gripper::robotiq::ObjectDetected(status_.status));
  if (last_command_.position == status_.position_echo) {
    status.set_command_id(last_command_.id);
  }

  if (status_.faults) {
    has_been_reset_ = false;
    LOG_EVERY_N_SEC(ERROR, 1)
        << absl::StrCat("Gripper is faulted, reason : 0x",
                        absl::Hex(status_.faults, absl::kZeroPad2));
    if ((status_.faults == 0x05 || status_.faults == 0x50) &&
        !reset_signal_sent_) {
      // 0x05: gripper fault requiring reset.
      // 0x50: controller fault requiring reset.
      LOG(ERROR) << "Gripper is faulted and requires a reset";
      INTR_RETURN_IF_ERROR(Reset());
    }
  } else {
    if ((status_.status & 0x09) == 0x09) {
      // Gripper is activated and initialized.
      has_been_reset_ = true;
      reset_signal_sent_ = false;
    }
  }

  std::string status_string =
      absl::StrCat("status: 0x", absl::Hex(status_.status, absl::kZeroPad2),
                   ", actual position: ", status.position(),
                   ", requested position: ", status.position_requested(),
                   ", reached: ", status.position_reached(),
                   ", in_motion: ", status.in_motion(),
                   ", enabled: ", status.gripper_enabled());
  VLOG(1) << "RobotiqGripper::GetStatus(): " << status_string;

  absl::MutexLock status_lock(status_mutex_);
  last_gripper_status_ = status;
  absl::MutexLock command_lock(command_mutex_);
  if (!user_command_executed_ && !in_motion &&
      (last_gripper_status_.command_id() ==
       last_gripper_command_.command_id())) {
    LOG(INFO)
        << "RobotiqGripper::GetStatus(): user_command_executed_ is changing "
           "value to true, with `last_gripper_command_`:\n"
        << last_gripper_command_ << "\n `last_gripper_status_`:\n"
        << last_gripper_status_ << "\n `status_string`:\n"
        << status_string;
    user_command_executed_ = true;
  }
  return status;
}

absl::Status RobotiqGripper::Write() {
  {
    absl::MutexLock lock(channel_mutex_);
    if (comm_interface_ == nullptr) {
      return absl::UnavailableError("Invalid interface.");
    }
  }

  for (int i = 0; i < kMaxNumRegisterReadWriteRetrials; i++) {
    absl::Status write_status;
    {
      absl::MutexLock lock(channel_mutex_);
      write_status = comm_interface_->write(command_.raw);
    }
    if (write_status.ok()) {
      break;
    } else {
      LOG(INFO) << "Write(): register-writing retry #" << i + 1 << "/"
                << kMaxNumRegisterReadWriteRetrials
                << " fails with error message: " << write_status.message();
      if (i < kMaxNumRegisterReadWriteRetrials - 1) {
        // sleep for a small time to give the hardware sometime to finish prior
        // operations, before retrying...
        absl::SleepFor(kHardwareSmallSleepDuration);
      } else {
        LOG(ERROR) << "Write() failed all " << kMaxNumRegisterReadWriteRetrials
                   << " retries.";
        return write_status;
      }
    }
  }

  return absl::OkStatus();
}

}  // namespace intrinsic::gripper
