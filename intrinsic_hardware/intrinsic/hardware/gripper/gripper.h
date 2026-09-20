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

#ifndef INTRINSIC_HARDWARE_GRIPPER_GRIPPER_H_
#define INTRINSIC_HARDWARE_GRIPPER_GRIPPER_H_

#include <memory>
#include <optional>
#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "absl/types/optional.h"
#include "cppregpattern/registry.h"
#include "google/protobuf/any.pb.h"
#include "intrinsic/hardware/gripper/gripper.pb.h"

namespace intrinsic::gripper {

constexpr int kMaxNumRegisterReadWriteRetrials = 3;

class PinchGripperInterface {
 public:
  PinchGripperInterface() = default;
  virtual ~PinchGripperInterface() = default;

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
  virtual absl::Status Configure(
      const intrinsic_proto::gripper::PinchGripperPhysicalConfig& config,
      bool validate_si_position_config, bool validate_si_velocity_config,
      bool validate_si_effort_config) = 0;

  // Enables the gripper s.t. it is ready to receive and act on commands.
  // This function eventually writes the command to the register of the gripper
  // hardware, and therefore we allow retries as many as
  // `kMaxNumRegisterReadWriteRetrials` to handle occasional failures.
  virtual absl::Status Enable() = 0;

  // Disables the gripper s.t. it will not act on any received commands.
  // This function eventually writes the command to the register of the gripper
  // hardware, and therefore we allow retries as many as
  // `kMaxNumRegisterReadWriteRetrials` to handle occasional failures.
  virtual absl::Status Disable() = 0;

  // Resets the gripper s.t. it is reinitialized but not yet enabled.
  // This function eventually writes the command to the register of the gripper
  // hardware, and therefore we allow retries as many as
  // `kMaxNumRegisterReadWriteRetrials` to handle occasional failures.
  virtual absl::Status Reset() = 0;

  // Execute a user command on the gripper.
  // If waiting_motion_stopped is set to true, then the function only returns
  // after either the gripper motion is stopped or the deadline is reached;
  // otherwise the function returns immediately with the current gripper status.
  virtual absl::StatusOr<intrinsic_proto::gripper::PinchGripperStatus>
  ExecuteCommand(const intrinsic_proto::gripper::PinchGripperCommand& command,
                 absl::Time deadline, bool waiting_motion_stopped) = 0;

  // Retrieves the current state of the gripper. Underneath this function,
  // it reads the status from the register of the gripper hardware.
  // Because this register-reading may fail occasionally due to one or other
  // reasons, we allow retries as many as `kMaxNumRegisterReadWriteRetrials`.
  virtual absl::StatusOr<intrinsic_proto::gripper::PinchGripperStatus>
  GetStatus() = 0;

  // Runs a cyclic routine at `cyclic_routine_frequency_hz` for the purpose of
  // keeping the connection alive with the gripper. If a gripper does not need
  // to run any cyclic keep-alive routine, then this function can directly
  // return an absl::OkStatus().
  virtual absl::Status RunCyclicKeepAliveRoutine() = 0;
};

// Classes and macros below define a distributed way for registering subclasses
// of PinchGripperInterface. Example usage:
//
// <To register a pinch gripper subclass, in 'magic_pinch_gripper.h' file>
//
// #include ".../gripper.h"
//
// class MagicPinchGripper : public PinchGripperInterface {
//  public:
//   explicit MagicPinchGripper(const MagicPinchGripperConfigProto&
//   config);
// };
//
// REGISTER_PINCH_GRIPPER(MagicPinchGripper,
// MagicPinchGripperConfigProto);
//
//
// <To use the registered class in an 'apply_magic_pinch_gripper.cc' file>
//
// #include ".../magic_pinch_gripper.h"
//
// ...
//
// std::unique_ptr<PinchGripperInterface> pinch_gripper;
//
// INTR_ASSIGN_OR_RETURN(pinch_gripper,
//                  PinchGripperFactory::Create("MagicPinchGripper",
//                                              any_config));
//
// MagicPinchGripper->SendCommand(...);
//
// ...
//
// <End of example>
//
// Note: 'any_config' must be of proto 'Any'.  If not, use
//
//   'google::protobuf::Any any_config; any_config.PackFrom(config);'
//
// to get 'any_config'.

class PinchGripperFactory {
 public:
  static absl::StatusOr<std::unique_ptr<PinchGripperInterface>> Create(
      absl::string_view pinch_gripper_name,
      const std::optional<google::protobuf::Any>& any_config);
};

using PinchGripperRegistry = registry::Registry<
    std::string,
    absl::StatusOr<std::unique_ptr<PinchGripperInterface>>(
        const std::optional<google::protobuf::Any>& any_config),
    registry::MissingKeyPolicy::default_construct>;

intrinsic::gripper::PinchGripperRegistry& GetPinchGripperRegistry();

#define REGISTER_PINCH_GRIPPER(name, alias, FactoryName)                      \
  [[maybe_unused]] const bool kUnused##name =                                 \
      intrinsic::gripper::PinchGripperRegistry::Register(                     \
          alias, [](const std::optional<google::protobuf::Any>& any_config) { \
            return FactoryName(any_config);                                   \
          });

}  // namespace intrinsic::gripper

#endif  // INTRINSIC_HARDWARE_GRIPPER_GRIPPER_H_
