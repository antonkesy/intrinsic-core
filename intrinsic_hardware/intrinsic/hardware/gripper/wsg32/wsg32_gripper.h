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

#ifndef INTRINSIC_HARDWARE_GRIPPER_WSG32_WSG32_GRIPPER_H_
#define INTRINSIC_HARDWARE_GRIPPER_WSG32_WSG32_GRIPPER_H_

#include <stdbool.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "google/protobuf/any.pb.h"
#include "intrinsic/hardware/gripper/gripper.h"
#include "intrinsic/hardware/gripper/gripper.pb.h"
#include "intrinsic/hardware/gripper/service/proto/generic_pinch_gripper_configs.pb.h"
#include "intrinsic/hardware/gripper/wsg32/wsg32_client.h"
#include "intrinsic/platform/pubsub/publisher.h"
#include "intrinsic/platform/pubsub/pubsub.h"

namespace intrinsic::gripper {

// A Weiss Robotics WSG32 interface using the third_party/wsg32 libraries.
class Wsg32Gripper : public PinchGripperInterface {
 public:
  static absl::StatusOr<std::unique_ptr<PinchGripperInterface>> Create(
      const std::optional<google::protobuf::Any>& any_config);

  explicit Wsg32Gripper(
      std::unique_ptr<Wsg32Client> comm_interface,
      const intrinsic_proto::gripper::PinchGripperAdditionalConfig&
          additional_config);

  static absl::StatusOr<std::unique_ptr<Wsg32Client>>
  CreateCommunicationInterface(
      const intrinsic_proto::gripper::GenericPinchGripperCommunicationConfig&
          config);

  static absl::Status ValidateAdditionalConfig(
      const intrinsic_proto::gripper::PinchGripperAdditionalConfig&
          additional_config);

  // (Re)configures the device settings. Before the configuration is applied,
  // the function checks the validity of the pinch gripper config.
  // The flag `validate_si_position_config` is specifying whether to validate
  // the config for position SI unit properties, or not; this is useful
  // for example when the user only provides position commands in percentage,
  // in which the SI unit properties are irrelevant and no need to be validated.
  // Similarly for `validate_si_velocity_config` and `validate_si_effort_config`
  // for velocity and effort, respectively. These flags are separated to provide
  // a better granularity, so that for example the user can specify position
  // command in SI unit, and specify velocity and effort commands in
  // percentages.
  absl::Status Configure(
      const intrinsic_proto::gripper::PinchGripperPhysicalConfig& config,
      bool validate_si_position_config, bool validate_si_velocity_config,
      bool validate_si_effort_config)
      ABSL_LOCKS_EXCLUDED(config_mutex_) override;

  // Sends a non-blocking enable command to the gripper.
  absl::Status Enable() override;

  // Sends a non-blocking Disable command to disable motion.
  absl::Status Disable() override;

  // Resets the device to a ready state. Required after power-cycle for safety.
  absl::Status Reset() override;

  // Execute a user command on the gripper.
  // If waiting_motion_stopped is set to true, then the function only returns
  // after either the gripper motion is stopped or the deadline is reached;
  // otherwise the function returns immediately with the current gripper status.
  absl::StatusOr<intrinsic_proto::gripper::PinchGripperStatus> ExecuteCommand(
      const intrinsic_proto::gripper::PinchGripperCommand& command,
      absl::Time deadline, bool waiting_motion_stopped)
      ABSL_LOCKS_EXCLUDED(command_mutex_)
          ABSL_LOCKS_EXCLUDED(status_mutex_) override;

  // Retrieves the status of the gripper. Underneath this function,
  // it reads the status from the register of the gripper hardware.
  absl::StatusOr<intrinsic_proto::gripper::PinchGripperStatus> GetStatus()
      ABSL_LOCKS_EXCLUDED(channel_mutex_) ABSL_LOCKS_EXCLUDED(command_mutex_)
          ABSL_LOCKS_EXCLUDED(status_mutex_) override;

  // Runs a cyclic routine periodically for the purpose of keeping the
  // connection alive with the gripper. Also will publish the gripper status via
  // DDS when a new status occurs.
  absl::Status RunCyclicKeepAliveRoutine()
      ABSL_LOCKS_EXCLUDED(command_mutex_) override;

 protected:
  struct Wsg32GripperCommand {
    int64_t id = 0;
    float position = 0;
    float speed = 0;
    float force = 0;
  } wsg32_command_;

  Wsg32Client::Status wsg32_status_ = {0, 0, 0, false};
  Wsg32GripperCommand wsg32_last_command_ = {0, 0, 0, 0};

 private:
  // Threshold in mm to determine whether a gripper position was reached.
  static constexpr double kPositionReachedThresholdMm = 2.0;

  // Sends a command to the gripper and updates the internal states.
  // Additionally, at the beginning this function also checks the gripper status
  // and re-enables the gripper if it's not already enabled.
  absl::Status CommandAndUpdate(
      const intrinsic_proto::gripper::PinchGripperCommand& command)
      ABSL_LOCKS_EXCLUDED(command_mutex_);

  // Sets the desired state command of the gripper.
  absl::Status SendCommand(
      const intrinsic_proto::gripper::PinchGripperCommand& command)
      ABSL_LOCKS_EXCLUDED(config_mutex_);

  absl::Mutex channel_mutex_;
  absl::Mutex config_mutex_;  // to guard against concurrent access to
                              // Configure() and SendCommand()
  absl::Mutex command_mutex_;
  absl::Mutex status_mutex_;
  std::unique_ptr<Wsg32Client> comm_interface_ ABSL_GUARDED_BY(channel_mutex_);

  double cyclic_routine_frequency_hz_;
  std::string status_topic_;
  bool cyclic_keep_alive_routine_running_;
  bool user_command_executed_ ABSL_GUARDED_BY(command_mutex_);
  intrinsic_proto::gripper::PinchGripperPhysicalConfig config_
      ABSL_GUARDED_BY(config_mutex_);
  intrinsic_proto::gripper::PinchGripperCommand last_gripper_command_
      ABSL_GUARDED_BY(command_mutex_);
  intrinsic_proto::gripper::PinchGripperStatus last_gripper_status_
      ABSL_GUARDED_BY(status_mutex_);
  intrinsic::PubSub pubsub_;
  intrinsic::Publisher pub_;
};

REGISTER_PINCH_GRIPPER(Wsg32Gripper, "Wsg32Gripper", Wsg32Gripper::Create);

}  // namespace intrinsic::gripper

#endif  // INTRINSIC_HARDWARE_GRIPPER_WSG32_WSG32_GRIPPER_H_
