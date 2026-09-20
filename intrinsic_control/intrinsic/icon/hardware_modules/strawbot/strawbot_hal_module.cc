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

#include <cstddef>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "google/protobuf/empty.pb.h"
#include "intrinsic/icon/hal/default_hardware_interfaces.h"  // IWYU pragma: keep
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/hardware_interface_registry.h"
#include "intrinsic/icon/hal/hardware_module_init_context.h"
#include "intrinsic/icon/hal/hardware_module_interface.h"
#include "intrinsic/icon/hal/hardware_module_registry.h"
#include "intrinsic/icon/hal/interfaces/joint_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_state.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/util/status/status_macros.h"

namespace strawbot_demo_module {

using ::intrinsic::icon::MutableHardwareInterfaceHandle;
using ::intrinsic::icon::OkStatus;
using ::intrinsic::icon::RealtimeStatus;
using ::intrinsic::icon::StrictHardwareInterfaceHandle;
using ::intrinsic_fbs::JointAccelerationState;
using ::intrinsic_fbs::JointPositionCommand;
using ::intrinsic_fbs::JointPositionState;
using ::intrinsic_fbs::JointVelocityState;

// Corresponding module config in test_hardware_module_config.h
class StrawbotHalModule final
    : public intrinsic::icon::HardwareModuleInterface {
 public:
  absl::Status Init(
      intrinsic::icon::HardwareModuleInitContext& init_context) override {
    intrinsic::icon::HardwareInterfaceRegistry& interface_registry =
        init_context.GetInterfaceRegistry();
    size_t dof = 6;

    INTR_ASSIGN_OR_RETURN(
        joint_position_command_,
        interface_registry.AdvertiseStrictInterface<JointPositionCommand>(
            "joint_position_command", dof));
    INTR_ASSIGN_OR_RETURN(
        joint_position_state_,
        interface_registry.AdvertiseMutableInterface<JointPositionState>(
            "joint_position_state", dof));
    INTR_ASSIGN_OR_RETURN(
        joint_velocity_state_,
        interface_registry.AdvertiseMutableInterface<JointVelocityState>(
            "joint_velocity_state", dof));
    INTR_ASSIGN_OR_RETURN(
        joint_acceleration_state_,
        interface_registry.AdvertiseMutableInterface<JointAccelerationState>(
            "joint_acceleration_state", dof));

    LOG(INFO) << "Strawbot hardware module successfully initiated.";
    return absl::OkStatus();
  }

  RealtimeStatus Activate() override {
    LOG(INFO) << "Strawbot is activated";
    return OkStatus();
  }

  RealtimeStatus Deactivate() override {
    LOG(INFO) << "Strawbot is deactivated";
    return OkStatus();
  }

  absl::Status EnableMotion() override {
    LOG(INFO) << "Strawbot is motion enabled";
    return absl::OkStatus();
  }

  absl::Status DisableMotion() override {
    LOG(INFO) << "Strawbot is motion disabled";
    return absl::OkStatus();
  }

  absl::Status Shutdown() override {
    LOG(INFO) << "Strawbot is shutting down";
    return absl::OkStatus();
  }

  absl::Status ClearFaults() override {
    LOG(INFO) << "Strawbot is clearing faults";
    return absl::OkStatus();
  }

  RealtimeStatus ApplyCommand() override {
    LOG_EVERY_N(INFO, 100) << "Strawbot is calling ApplyCommand";

    // Do not command a position if the command was not updated this cycle.
    INTRINSIC_RT_ASSIGN_OR_RETURN(const auto joint_position_command,
                                  joint_position_command_.Value());

    for (int i = 0; i < joint_position_command->position()->size(); ++i) {
      joint_position_state_->mutable_position()->Mutate(
          i, joint_position_command->position()->Get(i));
      joint_velocity_state_->mutable_velocity()->Mutate(i, 0.0);
      joint_acceleration_state_->mutable_acceleration()->Mutate(i, 0.0);
    }

    return OkStatus();
  }

  RealtimeStatus ReadStatus() override {
    LOG_EVERY_N(INFO, 100) << "Strawbot is calling ReadStatus";
    return OkStatus();
  }

 private:
  StrictHardwareInterfaceHandle<JointPositionCommand> joint_position_command_;
  MutableHardwareInterfaceHandle<JointPositionState> joint_position_state_;
  MutableHardwareInterfaceHandle<JointVelocityState> joint_velocity_state_;
  MutableHardwareInterfaceHandle<JointAccelerationState>
      joint_acceleration_state_;
};

}  // namespace strawbot_demo_module

REGISTER_HARDWARE_MODULE(strawbot_demo_module::StrawbotHalModule)
