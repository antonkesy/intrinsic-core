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

#ifndef INTRINSIC_HARDWARE_GRIPPER_ROBOTIQ_ROBOTIQ_GRIPPER_H_
#define INTRINSIC_HARDWARE_GRIPPER_ROBOTIQ_ROBOTIQ_GRIPPER_H_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "absl/base/attributes.h"
#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "google/protobuf/any.pb.h"
#include "intrinsic/hardware/gripper/gripper.h"
#include "intrinsic/hardware/gripper/gripper.pb.h"
#include "intrinsic/hardware/gripper/service/proto/generic_pinch_gripper_configs.pb.h"
#include "intrinsic/hardware/modbus_tcp/modbus_tcp.h"
#include "intrinsic/platform/pubsub/publisher.h"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "intrinsic/platform/pubsub/subscription.h"

namespace intrinsic::gripper {

// Register     | Robot Output / Functionalities | Robot Input / Status
//--------------|--------------------------------|-------------------------
// Byte 0       | ACTION                         | REQUEST GRIPPER STATUS
// Byte 1       | RESERVED                       | RESERVED
// Byte 2       | RESERVED                       | FAULT STATUS
// Byte 3       | POSITION                       | REQUEST POS REQUEST ECHO
// Byte 4       | SPEED                          | POSITION
// Byte 5       | FORCE                          | CURRENT
// Byte 6 to 15 | RESERVED                       | RESERVED
//
// Modbus RTU is a communication protocol based on a Big Endian byte order.
// Therefore, the 16-bit register addresses are transmitted with the most
// significant byte first. However, the data port is in the case of Robotiq
// products based on the Little Endian byte order. As such, the data parts of
// Modbus RTU messages are sent with the less significant byte first

// TODO(williambaker): Instead use bitfield
namespace Action {
// Functionalities
// Bits    | 7 6      | 5    | 4    | 3    | 2 1      | 0
// Symbols | Reserved | rARD | rATR | rGTO | Reserved | rACT
// Status
// Bits    | 7 6  | 5 4  | 3    | 2 1      | 0
// Symbols | gOBJ | gSTA | gGTO | Reserved | gACT |
const uint8_t Active = 0x1;
const uint8_t GoTo = 0x8;

}  // namespace Action

// A RobotiqGripper interface using ModbusTCP.
//
// After initialization of gripper using IP and port, a Reset() command must be
// sent to enable motion (safety requirement by robotiq).
// Note Enable/Disable/Reset are non-blocking commands.
// TODO(b/182798351): Add blocking Enable/Disable/Reset methods.
class RobotiqGripper : public PinchGripperInterface {
 public:
  static absl::StatusOr<std::unique_ptr<PinchGripperInterface>> Create(
      const std::optional<google::protobuf::Any>& any_config);

  explicit RobotiqGripper(
      std::unique_ptr<hardware::ModbusTcp> comm_interface,
      const intrinsic_proto::gripper::PinchGripperAdditionalConfig&
          additional_config);

  static absl::StatusOr<std::unique_ptr<hardware::ModbusTcp>>
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
  // This function eventually writes the command to the register of the gripper
  // hardware, and therefore we allow retries as many as
  // `kMaxNumRegisterReadWriteRetrials` to handle occasional failures.
  absl::Status Enable() override;

  // Sends a non-blocking Disable command to disable motion.
  // This function eventually writes the command to the register of the gripper
  // hardware, and therefore we allow retries as many as
  // `kMaxNumRegisterReadWriteRetrials` to handle occasional failures.
  absl::Status Disable() override;

  // Resets the device to a ready state. Required after power-cycle for safety.
  // This function eventually writes the command to the register of the gripper
  // hardware, and therefore we allow retries as many as
  // `kMaxNumRegisterReadWriteRetrials` to handle occasional failures.
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
  // Because this register-reading may fail occasionally due to one or other
  // reasons, we allow retries as many as `kMaxNumRegisterReadWriteRetrials`.
  // TODO(b/181350040) status does not report correct position value
  absl::StatusOr<intrinsic_proto::gripper::PinchGripperStatus> GetStatus()
      ABSL_LOCKS_EXCLUDED(channel_mutex_) ABSL_LOCKS_EXCLUDED(config_mutex_)
          ABSL_LOCKS_EXCLUDED(status_mutex_)
              ABSL_LOCKS_EXCLUDED(command_mutex_) override;

  // Runs a cyclic routine periodically for the purpose of keeping the
  // connection alive with the gripper. Also will publish the gripper status via
  // DDS when a new status occurs.
  absl::Status RunCyclicKeepAliveRoutine()
      ABSL_LOCKS_EXCLUDED(command_mutex_) override;

 protected:
  static const int kCommandDataLength = 3;
  static const int kStatusDataLength = 3;

  union ABSL_ATTRIBUTE_PACKED {
    uint16_t raw[kCommandDataLength];
    uint8_t bytes[kCommandDataLength * 2];
    struct {
      uint8_t : 8;  // reserved
      uint8_t action;
      uint8_t position;
      uint8_t : 8;  // reserved
      uint8_t force;
      uint8_t speed;
    };
  } command_ = {{0, 0, 0}};

  union ABSL_ATTRIBUTE_PACKED {
    uint16_t raw[kStatusDataLength];
    uint8_t bytes[kStatusDataLength * 2];
    struct {
      // `raw` is in Big Endian order.
      uint8_t : 8;            // [first byte: high]; reserved
      uint8_t status;         // [first byte: low]
      uint8_t position_echo;  // [second byte: high]
      uint8_t faults;         // [second byte: low]
      uint8_t current;        // [third byte: high]
      uint8_t position;       // [third byte: low]
    };
  } status_ = {{0, 0, 0}};

  struct {
    int64_t id;
    uint8_t position;
  } last_command_ = {0, 0};

 private:
  // Sends a command to the gripper and update the internal states.
  // Additionally, at the beginning this function also checks the gripper status
  // and re-enables the gripper if it's not already enabled; note, this step is
  // skipped if the gripper was reset on startup, and we simply ignore errors
  // when sending a command.
  absl::Status CommandAndUpdate(
      const intrinsic_proto::gripper::PinchGripperCommand& command)
      ABSL_LOCKS_EXCLUDED(command_mutex_);

  // Sets the desired state of the gripper.
  // This function eventually writes the command to the register of the gripper
  // hardware, and therefore we allow retries as many as
  // `kMaxNumRegisterReadWriteRetrials` to handle occasional failures.
  absl::Status SendCommand(
      const intrinsic_proto::gripper::PinchGripperCommand& command)
      ABSL_LOCKS_EXCLUDED(config_mutex_);

  // Writes the command to the register of the gripper hardware.
  // Because this register-writing may fail occasionally due to one or other
  // reasons, we allow retries as many as `kMaxNumRegisterReadWriteRetrials`.
  absl::Status Write() ABSL_LOCKS_EXCLUDED(channel_mutex_);

  absl::Mutex channel_mutex_;
  absl::Mutex config_mutex_;  // to guard against concurrent access to
                              // Configure(), GetStatus(), and SendCommand()
  absl::Mutex command_mutex_;
  absl::Mutex status_mutex_;
  std::unique_ptr<hardware::ModbusTcp> comm_interface_
      ABSL_GUARDED_BY(channel_mutex_);
  double cyclic_routine_frequency_hz_;
  std::string status_topic_;
  bool cyclic_keep_alive_routine_running_;
  // Although has_been_reset_ and reset_signal_sent_ below sound similar, they
  // are behaviorally different:
  // >> has_been_reset_ flag is set to false in the constructor, set to true
  // after the gripper Reset() has been successful and afterwards *kept being
  // hold to true*, except when a gripper error (that requires a gripper reset)
  // suddenly occurs, at which time the has_been_reset_ flag is set to false
  // again.
  // >> reset_signal_sent_ is set to false in the constructor, set to true when
  // the gripper Reset() is executed, and (not long after that) when the gripper
  // has been indicated as activated and initialized the reset_signal_sent_ is
  // set to false again.
  // So to summarize, the difference is that the has_been_reset_ flag behaves
  // like a step function at the Reset() event, while the reset_signal_sent_
  // behaves like an impulse function at the Reset() event.
  bool has_been_reset_;
  bool reset_signal_sent_;
  bool received_first_user_command_;
  bool user_command_executed_ ABSL_GUARDED_BY(command_mutex_);
  // If true, then a caller has triggered ExecuteCommand with
  // waiting_motion_stopped == true. This will block non-blocking calls to
  // ExecuteCommand.
  bool blocking_command_executing_ ABSL_GUARDED_BY(command_mutex_);
  intrinsic_proto::gripper::PinchGripperPhysicalConfig config_
      ABSL_GUARDED_BY(config_mutex_);
  intrinsic_proto::gripper::PinchGripperCommand last_gripper_command_
      ABSL_GUARDED_BY(command_mutex_);
  intrinsic_proto::gripper::PinchGripperStatus last_gripper_status_
      ABSL_GUARDED_BY(status_mutex_);
  intrinsic::PubSub pubsub_;
  intrinsic::Publisher pub_;
  // If the configuration, provides a command topic, this field will be set with
  // a subscription that receives commands and executes them.
  //
  // The callback for subscription is designed in such a way blocking calls take
  // precedence over non-blocking calls, so there is no guarantee that commands
  // sent to the topic will be executed.
  std::optional<intrinsic::Subscription> sub_;
};

REGISTER_PINCH_GRIPPER(RobotiqGripper, "RobotiqGripper",
                       RobotiqGripper::Create);

}  // namespace intrinsic::gripper

#endif  // INTRINSIC_HARDWARE_GRIPPER_ROBOTIQ_ROBOTIQ_GRIPPER_H_
