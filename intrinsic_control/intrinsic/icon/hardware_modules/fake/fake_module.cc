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

#include "intrinsic/icon/hardware_modules/fake/fake_module.h"

#include <sched.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/synchronization/notification.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "flatbuffers/vector.h"
#include "intrinsic/icon/control/safety/extern/safety_status.fbs.h"
#include "intrinsic/icon/control/safety/safety_messages.fbs.h"
#include "intrinsic/icon/flatbuffers/control_types_view.h"
#include "intrinsic/icon/flatbuffers/flatbuffer_utils.h"
#include "intrinsic/icon/flatbuffers/transform_types.fbs.h"
#include "intrinsic/icon/hal/default_hardware_interfaces.h"  // IWYU pragma: keep
#include "intrinsic/icon/hal/hardware_interface_registry.h"
#include "intrinsic/icon/hal/hardware_module_init_context.h"
#include "intrinsic/icon/hal/interfaces/adio.fbs.h"
#include "intrinsic/icon/hal/interfaces/control_mode.fbs.h"
#include "intrinsic/icon/hal/interfaces/force_sensor.fbs.h"
#include "intrinsic/icon/hal/interfaces/force_torque.fbs.h"
#include "intrinsic/icon/hal/interfaces/io_controller.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_limits.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_state.fbs.h"
#include "intrinsic/icon/hal/interfaces/payload_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/payload_state.fbs.h"
#include "intrinsic/icon/hal/interfaces/rangefinder.fbs.h"
#include "intrinsic/icon/hal/module_config.h"
#include "intrinsic/icon/hal/proto/hardware_module_inspection.pb.h"
#include "intrinsic/icon/hardware_modules/fake/fake_module_config.pb.h"
#include "intrinsic/icon/proto/cart_space_conversion.h"
#include "intrinsic/icon/proto/part_status.pb.h"
#include "intrinsic/icon/proto/safety_status.pb.h"
#include "intrinsic/icon/proto/safety_status_conversion.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/kinematics/types/joint_limits.pb.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/thread/rt_thread.h"
#include "intrinsic/util/thread/thread.h"
#include "intrinsic/util/thread/thread_options.h"
#include "magic_enum/magic_enum.hpp"

namespace intrinsic::timeslicer::fake_module {

using ::intrinsic_fbs::ControlMode;
using ::intrinsic_fbs::ControlModeStatus;
using ::intrinsic_fbs::ForceTorqueCommand;
using ::intrinsic_fbs::ForceTorqueStatus;
using ::intrinsic_fbs::HandGuidingCommand;
using ::intrinsic_fbs::JointAccelerationState;
using ::intrinsic_fbs::JointCommandedPosition;
using ::intrinsic_fbs::JointPositionCommand;
using ::intrinsic_fbs::JointPositionState;
using ::intrinsic_fbs::JointTorqueCommand;
using ::intrinsic_fbs::JointTorqueState;
using ::intrinsic_fbs::JointVelocityCommand;
using ::intrinsic_fbs::JointVelocityState;
using ::intrinsic_fbs::RangeFinderStatus;
using ::intrinsic_proto::icon::PartJointState;

namespace {

static constexpr size_t kLogTimingEveryNCycles = 2000;

}  // namespace

absl::Status FakeModule::InitializeArmPartInterfaces(
    icon::HardwareInterfaceRegistry& interface_registry,
    const intrinsic_proto::icon::FakeModuleConfig::ArmInterfaces& config) {
  if (config.initial_joint_state().empty()) {
    return absl::FailedPreconditionError(
        "`initial_joint_state` is required to initialize `arm_interfaces`.");
  }

  const std::size_t ndof = config.initial_joint_state().size();
  // Register Command Interfaces
  if (!config.joint_position_command_name().empty()) {
    INTR_ASSIGN_OR_RETURN(
        joint_position_command_,
        interface_registry
            .AdvertiseMutableStrictInterface<JointPositionCommand>(
                config.joint_position_command_name(), ndof));
  }
  if (!config.joint_commanded_position_name().empty()) {
    INTR_ASSIGN_OR_RETURN(
        joint_commanded_position_,
        interface_registry
            .AdvertiseMutableStrictInterface<JointCommandedPosition>(
                config.joint_commanded_position_name(), ndof));
  }
  if (!config.joint_velocity_command_name().empty()) {
    INTR_ASSIGN_OR_RETURN(
        joint_velocity_command_,
        interface_registry.AdvertiseMutableInterface<JointVelocityCommand>(
            config.joint_velocity_command_name(), ndof));
  }
  if (!config.joint_torque_command_name().empty()) {
    INTR_ASSIGN_OR_RETURN(
        joint_torque_command_,
        interface_registry.AdvertiseMutableInterface<JointTorqueCommand>(
            config.joint_torque_command_name(), ndof));
  }
  if (!config.process_wrench_command_name().empty()) {
    INTR_ASSIGN_OR_RETURN(
        process_wrench_command_,
        interface_registry.AdvertiseMutableInterface<intrinsic_fbs::Wrench>(
            config.process_wrench_command_name()));
  }
  if (!config.payload_command_name().empty()) {
    INTR_ASSIGN_OR_RETURN(
        payload_command_,
        interface_registry
            .AdvertiseMutableInterface<intrinsic_fbs::PayloadCommand>(
                config.payload_command_name()));
  }
  // Register State Interfaces
  if (!config.joint_position_state_name().empty()) {
    INTR_ASSIGN_OR_RETURN(
        joint_position_state_,
        interface_registry.AdvertiseMutableInterface<JointPositionState>(
            config.joint_position_state_name(), ndof));
  }
  if (!config.joint_velocity_state_name().empty()) {
    INTR_ASSIGN_OR_RETURN(
        joint_velocity_state_,
        interface_registry.AdvertiseMutableInterface<JointVelocityState>(
            config.joint_velocity_state_name(), ndof));
  }
  if (!config.joint_acceleration_state_name().empty()) {
    INTR_ASSIGN_OR_RETURN(
        joint_acceleration_state_,
        interface_registry.AdvertiseMutableInterface<JointAccelerationState>(
            config.joint_acceleration_state_name(), ndof));
  }
  if (!config.joint_torque_state_name().empty()) {
    INTR_ASSIGN_OR_RETURN(
        joint_torque_state_,
        interface_registry.AdvertiseMutableInterface<JointTorqueState>(
            config.joint_torque_state_name(), ndof));
  }
  if (!config.payload_state_name().empty()) {
    INTR_ASSIGN_OR_RETURN(
        payload_state_,
        interface_registry
            .AdvertiseMutableInterface<intrinsic_fbs::PayloadState>(
                config.payload_state_name()));
  }
  if (!config.joint_system_limits_name().empty()) {
    INTR_ASSIGN_OR_RETURN(
        joint_system_limits_,
        interface_registry.AdvertiseInterface<intrinsic_fbs::JointLimits>(
            config.joint_system_limits_name(), ndof));
  }

  std::string hand_guiding_command_name = "hand_guiding_command";
  if (config.hand_guiding_command_name().empty()) {
    INTRINSIC_RT_LOG(INFO) << "No 'hand_guiding_command_name' defined. Using: "
                           << hand_guiding_command_name;
  } else {
    hand_guiding_command_name = config.hand_guiding_command_name();
  }
  INTR_ASSIGN_OR_RETURN(
      hand_guiding_command_,
      interface_registry.AdvertiseInterface<HandGuidingCommand>(
          hand_guiding_command_name));

  std::string control_mode_state_name = "control_mode_state";
  if (config.control_mode_state_name().empty()) {
    INTRINSIC_RT_LOG(INFO) << "No 'control_mode_state_name' defined. Using: "
                           << control_mode_state_name;
  } else {
    control_mode_state_name = config.control_mode_state_name();
  }
  INTR_ASSIGN_OR_RETURN(
      control_mode_state_,
      interface_registry.AdvertiseMutableInterface<ControlModeStatus>(
          control_mode_state_name, ControlMode::kCyclicPosition));

  // Set initial state
  for (std::size_t i = 0; i < ndof; ++i) {
    const PartJointState& joint_state = config.initial_joint_state().at(i);
    // Set initial values for Command Interfaces.
    if (joint_position_command_) {
      joint_position_command_->MutableValue()->mutable_position()->Mutate(
          i, joint_state.position_commanded_last_cycle());
      joint_position_command_->MutableValue()
          ->mutable_velocity_feedforward()
          ->Mutate(i, joint_state.velocity_commanded_last_cycle());
      joint_position_command_->MutableValue()
          ->mutable_acceleration_feedforward()
          ->Mutate(i, joint_state.acceleration_commanded_last_cycle());
    }
    if (joint_velocity_command_) {
      joint_velocity_command_.value()->mutable_velocity()->Mutate(
          i, joint_state.velocity_commanded_last_cycle());
      joint_velocity_command_.value()
          ->mutable_acceleration_feedforward()
          ->Mutate(i, joint_state.acceleration_commanded_last_cycle());
    }
    if (joint_torque_command_) {
      // TODO(b/276430306): Initialize torque command.
      INTRINSIC_RT_LOG(ERROR)
          << "Can't initialize torque command with custom values, as "
             "`PartJointState` does not support `torque_commanded_last_cycle`. "
             "Using defaults.";
    }
    // Set initial values for State Interfaces.
    if (joint_position_state_) {
      joint_position_state_.value()->mutable_position()->Mutate(
          i, joint_state.position_sensed());
    }
    if (joint_velocity_state_) {
      joint_velocity_state_.value()->mutable_velocity()->Mutate(
          i, joint_state.velocity_sensed());
    }
    if (joint_acceleration_state_) {
      joint_acceleration_state_.value()->mutable_acceleration()->Mutate(
          i, joint_state.acceleration_sensed());
    }
    if (joint_torque_state_) {
      joint_torque_state_.value()->mutable_torque()->Mutate(
          i, joint_state.torque_sensed());
    }
  }

  return absl::OkStatus();
}

absl::Status FakeModule::InitializeForceTorquePartInterface(
    icon::HardwareInterfaceRegistry& interface_registry,
    const intrinsic_proto::icon::FakeModuleConfig::ForceTorqueInterface&
        config) {
  INTR_ASSIGN_OR_RETURN(
      force_torque_status_,
      interface_registry.AdvertiseMutableInterface<ForceTorqueStatus>(
          config.force_torque_status_name()));
  INTR_ASSIGN_OR_RETURN(
      force_torque_command_,
      interface_registry.AdvertiseMutableInterface<ForceTorqueCommand>(
          config.force_torque_command_name()));
  force_torque_status_.value()->mutate_enabled(true);
  force_torque_status_.value()->mutate_status_code(
      intrinsic_fbs::ForceSensorStatusCode::Ok);
  force_torque_status_.value()->mutate_raw_status_code(0);
  if (config.has_initial_force_torque_state()) {
    *intrinsic_fbs::FromSchema(force_torque_status_.value()->mutable_wrench()) =
        intrinsic::icon::FromProto(config.initial_force_torque_state());
  }
  return absl::OkStatus();
}

absl::Status FakeModule::InitializeRangefinderPartInterface(
    icon::HardwareInterfaceRegistry& interface_registry,
    const intrinsic_proto::icon::FakeModuleConfig::RangefinderInterface&
        config) {
  INTR_ASSIGN_OR_RETURN(
      rangefinder_status_,
      interface_registry.AdvertiseMutableInterface<RangeFinderStatus>(
          config.rangefinder_status_name()));
  if (config.has_initial_rangefinder_distance()) {
    rangefinder_status_.value()->mutate_distance(
        config.initial_rangefinder_distance());
  }
  return absl::OkStatus();
}

absl::Status FakeModule::InitializeADIOPartInterface(
    icon::HardwareInterfaceRegistry& interface_registry,
    const intrinsic_proto::icon::FakeModuleConfig::ADIOInterface& config) {
  const auto& initial = config.initial_adio_state();
  if (initial.digital_inputs().empty() && initial.analog_inputs().empty() &&
      initial.digital_outputs().empty()) {
    return absl::FailedPreconditionError(
        "At least one of the ADIOs must be set.");
  }

  for (const auto& [name, value] : initial.digital_inputs()) {
    uint32_t max_key = 0;
    for (const auto& [key, signal] : value.signals()) {
      max_key = std::max(max_key, key);
    }
    std::vector<std::string> digital_input_descriptions;
    digital_input_descriptions.reserve(max_key);
    for (int i = 0; i < max_key + 1; ++i) {
      digital_input_descriptions.push_back(std::to_string(i));
    }
    LOG(INFO) << "Creating digital inputs for name " << name << ": "
              << absl::StrJoin(digital_input_descriptions, ", ");
    INTR_ASSIGN_OR_RETURN(
        auto digital_input_status,
        interface_registry.AdvertiseMutableInterface<intrinsic_fbs::DIOStatus>(
            name, digital_input_descriptions));
    if (config.setup_loopback_interfaces()) {
      INTR_ASSIGN_OR_RETURN(
          auto digital_input_status_loopback_command,
          interface_registry.AdvertiseInterface<intrinsic_fbs::DIOCommand>(
              name + "_loopback_command", digital_input_descriptions));
      digital_input_status_loopback_command_vector_.emplace_back(
          std::move(digital_input_status_loopback_command));
    }
    for (const auto& [key, signal] : value.signals()) {
      LOG(INFO) << "Setting initial value " << signal.value() << " for key "
                << key;
      if (!digital_input_status->mutable_signals()
               ->GetMutableObject(key)
               ->mutate_value(signal.value())) {
        return absl::InternalError("Failed to mutate digital input value");
      }
    }
    digital_input_status_vector_.emplace_back(std::move(digital_input_status));
  }

  for (const auto& [name, value] : initial.analog_inputs()) {
    uint32_t max_key = 0;
    for (const auto& [key, signal] : value.signals()) {
      max_key = std::max(max_key, key);
    }
    std::vector<std::string> analog_input_descriptions;
    analog_input_descriptions.reserve(max_key);
    for (int i = 0; i < max_key + 1; ++i) {
      analog_input_descriptions.push_back(std::to_string(i));
    }
    LOG(INFO) << "Creating analog inputs for name " << name << ": "
              << absl::StrJoin(analog_input_descriptions, ", ");
    INTR_ASSIGN_OR_RETURN(
        auto analog_input_status,
        interface_registry.AdvertiseMutableInterface<intrinsic_fbs::AIOStatus>(
            name, analog_input_descriptions));
    if (config.setup_loopback_interfaces()) {
      INTR_ASSIGN_OR_RETURN(
          auto analog_input_status_loopback_command,
          interface_registry.AdvertiseInterface<intrinsic_fbs::AIOStatus>(
              name + "_loopback_command", analog_input_descriptions));
      analog_input_status_loopback_command_vector_.emplace_back(
          std::move(analog_input_status_loopback_command));
    }
    for (const auto& [key, pb_signal] : value.signals()) {
      auto* fb_signal =
          analog_input_status->mutable_signals()->GetMutableObject(key);
      if (!fb_signal->mutate_value(pb_signal.value())) {
        return absl::InternalError("Failed to mutate value");
      }
      auto enum_value = magic_enum::enum_cast<intrinsic_fbs::AnalogInputUnit>(
          std::string("k") + pb_signal.unit());
      if (enum_value) {
        if (!fb_signal->mutate_unit(*enum_value)) {
          return absl::InternalError("Failed to mutate unit");
        }
      } else {
        return absl::InvalidArgumentError(absl::StrCat(
            "Could not convert unit string to enum: ", pb_signal.unit()));
      }
      LOG(INFO) << "Setting initial value " << pb_signal.value() << " for key "
                << key;
    }
    analog_input_status_vector_.emplace_back(std::move(analog_input_status));
  }

  for (const auto& [name, value] : initial.digital_outputs()) {
    uint32_t max_key = 0;
    for (const auto& [key, signal] : value.signals()) {
      max_key = std::max(max_key, key);
    }
    std::vector<std::string> digital_output_descriptions;
    digital_output_descriptions.reserve(max_key);
    for (int i = 0; i < max_key + 1; ++i) {
      digital_output_descriptions.push_back(std::to_string(i));
    }
    LOG(INFO) << "Creating digital outputs for name " << name << ": "
              << absl::StrJoin(digital_output_descriptions, ", ");

    INTR_ASSIGN_OR_RETURN(
        auto digital_output_command,
        interface_registry.AdvertiseInterface<intrinsic_fbs::DIOCommand>(
            name, digital_output_descriptions));
    digital_output_command_vector_.emplace_back(
        std::move(digital_output_command));
    if (config.setup_loopback_interfaces()) {
      INTR_ASSIGN_OR_RETURN(
          auto digital_output_command_loopback_status,
          interface_registry
              .AdvertiseMutableInterface<intrinsic_fbs::DIOStatus>(
                  name + "_loopback_status", digital_output_descriptions));
      digital_output_command_loopback_status_vector_.emplace_back(
          std::move(digital_output_command_loopback_status));
    }
  }

  for (const auto& [name, value] : initial.analog_outputs()) {
    uint32_t max_key = 0;
    for (const auto& [key, signal] : value.signals()) {
      max_key = std::max(max_key, key);
    }
    std::vector<std::string> analog_output_descriptions;
    analog_output_descriptions.reserve(max_key);
    for (int i = 0; i < max_key + 1; ++i) {
      analog_output_descriptions.push_back(std::to_string(i));
    }
    LOG(INFO) << "Creating analog outputs for name " << name << ": "
              << absl::StrJoin(analog_output_descriptions, ", ");
    INTR_ASSIGN_OR_RETURN(
        auto analog_output_command,
        interface_registry.AdvertiseInterface<intrinsic_fbs::AIOCommand>(
            name, analog_output_descriptions));
    analog_output_command_vector_.emplace_back(
        std::move(analog_output_command));
    if (config.setup_loopback_interfaces()) {
      INTR_ASSIGN_OR_RETURN(
          auto analog_output_command_loopback_status,
          interface_registry
              .AdvertiseMutableInterface<intrinsic_fbs::AIOStatus>(
                  name + "_loopback_status", analog_output_descriptions));
      analog_output_command_loopback_status_vector_.emplace_back(
          std::move(analog_output_command_loopback_status));
    }
  }

  return absl::OkStatus();
}

absl::Status FakeModule::Init(
    intrinsic::icon::HardwareModuleInitContext& init_context) {
  INTRINSIC_ASSERT_NON_REALTIME();
  intrinsic::icon::HardwareInterfaceRegistry& interface_registry =
      init_context.GetInterfaceRegistry();
  const intrinsic::icon::ModuleConfig& config = init_context.GetModuleConfig();

  init_context.RegisterGrpcService(dummy_service_);

  INTRINSIC_RT_LOG(INFO) << "Configuring FakeModule with name ["
                         << config.GetName() << "].";
  INTR_ASSIGN_OR_RETURN(
      const auto module_config,
      config.GetConfig<::intrinsic_proto::icon::FakeModuleConfig>());

  INTR_ASSIGN_OR_RETURN(auto cycle_time, config.GetControlPeriod());
  if (cycle_time <= absl::ZeroDuration()) {
    return absl::InvalidArgumentError("cycle_time must be positive");
  }
  cycle_time_.emplace(cycle_time);

  module_state_ = ModuleState::kInactive;

  realtime_clock_ = config.GetRealtimeClock();
  if (realtime_clock_ == nullptr) {
    INTRINSIC_RT_LOG(INFO) << "ICON is driving the clock of ["
                           << config.GetName() << "].";
  } else {
    INTRINSIC_RT_LOG(INFO) << "The clock is driven by [" << config.GetName()
                           << "].";
    clock_ticking_thread_options_ = config.GetIconThreadOptions();
    clock_ticking_thread_options_.SetName("FakeModuleThread");
  }

  if (module_config.has_arm_interfaces()) {
    INTR_RETURN_IF_ERROR(InitializeArmPartInterfaces(
        interface_registry, module_config.arm_interfaces()));
  }

  if (module_config.has_adio_interface()) {
    INTR_RETURN_IF_ERROR(InitializeADIOPartInterface(
        interface_registry, module_config.adio_interface()));
  }

  if (module_config.has_force_torque_interface()) {
    INTR_RETURN_IF_ERROR(InitializeForceTorquePartInterface(
        interface_registry, module_config.force_torque_interface()));
  }

  if (module_config.has_rangefinder_interface()) {
    INTR_RETURN_IF_ERROR(InitializeRangefinderPartInterface(
        interface_registry, module_config.rangefinder_interface()));
  }

  if (module_config.has_safety_interface()) {
    if (module_config.safety_interface().safety_status_name().empty()) {
      return absl::FailedPreconditionError(
          "Config defines safety_interface, but no safety_status_name");
    }

    intrinsic_fbs::ModeOfSafeOperation mode_of_safe_operation =
        icon::FromProto(module_config.safety_interface()
                            .initial_safety_status()
                            .mode_of_safe_operation());

    intrinsic_fbs::ButtonStatus estop_button_status =
        icon::FromProto(module_config.safety_interface()
                            .initial_safety_status()
                            .estop_button_status());

    intrinsic_fbs::ButtonStatus enable_button_status =
        icon::FromProto(module_config.safety_interface()
                            .initial_safety_status()
                            .enable_button_status());

    intrinsic_fbs::RequestedBehavior requested_behavior =
        icon::FromProto(module_config.safety_interface()
                            .initial_safety_status()
                            .requested_behavior());

    INTR_ASSIGN_OR_RETURN(
        safety_status_,
        interface_registry
            .AdvertiseMutableInterface<intrinsic_fbs::SafetyStatusMessage>(
                module_config.safety_interface().safety_status_name(),
                /*mode_of_safe_operation=*/
                mode_of_safe_operation,
                /*estop_button_status=*/estop_button_status,
                /*enable_button_status=*/enable_button_status,
                /*requested_behavior=*/requested_behavior));
  }

  // Realtime metrics and warnings are not useful when not executing in a
  // realtime context.
  if (config.GetIconThreadOptions().GetSchedulePolicy() != SCHED_FIFO) {
    LOG(INFO) << "Not enabling realtime metrics because the module is not "
                 "running in realtime.";
  } else {
    init_context.EnableCycleTimeMetrics(*cycle_time_,
                                        /*log_cycle_time_warnings=*/true);
  }

  return absl::OkStatus();
}

icon::RealtimeStatus FakeModule::ApplyArmCommand() {
  if (!control_mode_state_) {
    return icon::OkStatus();
  }

  // Select the ControlMode based on the most recently updated type of command.
  auto last_updated = intrinsic::Clock::Zero();
  ControlMode last_updated_control_mode = ControlMode::kUnknown;

  if (joint_position_command_) {
    last_updated_control_mode = ControlMode::kCyclicPosition;
    last_updated = joint_position_command_->LastUpdatedTime();
  }
  if (joint_velocity_command_) {
    auto last_updated_velocity = joint_velocity_command_->LastUpdatedTime();
    if (last_updated_velocity > last_updated) {
      last_updated = last_updated_velocity;
      last_updated_control_mode = ControlMode::kCyclicVelocity;
    }
  }
  if (joint_torque_command_) {
    auto last_updated_torque = joint_torque_command_->LastUpdatedTime();
    if (last_updated_torque > last_updated) {
      last_updated = last_updated_torque;
      last_updated_control_mode = ControlMode::kCyclicTorque;
    }
  }
  if (hand_guiding_command_) {
    auto last_updated_hand_guiding = hand_guiding_command_->LastUpdatedTime();
    if (last_updated_hand_guiding > last_updated) {
      last_updated = last_updated_hand_guiding;
      last_updated_control_mode = ControlMode::kHandguiding;
    }
  }

  control_mode_state_.value()->mutate_status(last_updated_control_mode);
  INTRINSIC_RT_LOG_THROTTLED(INFO)
      << "Control mode: "
      << EnumNameControlMode(control_mode_state_.value()->status());

  switch (control_mode_state_.value()->status()) {
    case ControlMode::kCyclicPosition: {
      if (!joint_position_command_) {
        return icon::InternalError(
            "ControlMode is kCyclicPosition, but no joint position command is "
            "defined.");
      }
      INTRINSIC_RT_ASSIGN_OR_RETURN(const auto joint_position_command,
                                    joint_position_command_->Value());
      if (joint_commanded_position_.has_value()) {
        intrinsic_fbs::JointCommandedPosition* const joint_commanded_position =
            joint_commanded_position_->MutableValue();
        INTRINSIC_RT_RETURN_IF_ERROR(intrinsic_fbs::CopyFbsVector(
            *joint_position_command->position(),
            *(joint_commanded_position->mutable_position())));
        INTRINSIC_RT_RETURN_IF_ERROR(intrinsic_fbs::CopyFbsVector(
            *joint_position_command->velocity_feedforward(),
            *(joint_commanded_position->mutable_velocity_feedforward())));
        INTRINSIC_RT_RETURN_IF_ERROR(intrinsic_fbs::CopyFbsVector(
            *joint_position_command->acceleration_feedforward(),
            *(joint_commanded_position->mutable_acceleration_feedforward())));
      }

      if (joint_position_state_) {
        INTRINSIC_RT_RETURN_IF_ERROR(intrinsic_fbs::CopyFbsVector(
            *joint_position_command->position(),
            *joint_position_state_.value()->mutable_position()));
      }
      if (joint_velocity_state_) {
        INTRINSIC_RT_RETURN_IF_ERROR(intrinsic_fbs::CopyFbsVector(
            *joint_position_command->velocity_feedforward(),
            *joint_velocity_state_.value()->mutable_velocity()));
      }
      if (joint_acceleration_state_) {
        INTRINSIC_RT_RETURN_IF_ERROR(intrinsic_fbs::CopyFbsVector(
            *joint_position_command->acceleration_feedforward(),
            *joint_acceleration_state_.value()->mutable_acceleration()));
      }
      break;
    }
    case ControlMode::kCyclicVelocity: {
      if (joint_velocity_command_ && joint_velocity_state_) {
        INTRINSIC_RT_RETURN_IF_ERROR(intrinsic_fbs::CopyFbsVector(
            *joint_velocity_command_.value()->velocity(),
            *joint_velocity_state_.value()->mutable_velocity()));
      }
      if (joint_velocity_command_ && joint_acceleration_state_) {
        INTRINSIC_RT_RETURN_IF_ERROR(intrinsic_fbs::CopyFbsVector(
            *joint_velocity_command_.value()->acceleration_feedforward(),
            *joint_acceleration_state_.value()->mutable_acceleration()));
      }
      break;
    }
    case ControlMode::kCyclicTorque: {
      if (joint_torque_command_ && joint_torque_state_) {
        INTRINSIC_RT_RETURN_IF_ERROR(intrinsic_fbs::CopyFbsVector(
            *joint_torque_command_.value()->torque(),
            *joint_torque_state_.value()->mutable_torque()));
      }
      break;
    }
    case ControlMode::kHandguiding: {
      // NO-OP.
      break;
    }
    default:
      INTRINSIC_RT_LOG(ERROR)
          << "Unsupported Control mode ["
          << EnumNameControlMode(control_mode_state_.value()->status())
          << "]. Command is not looped back.";
  }
  return icon::OkStatus();
}

icon::RealtimeStatus FakeModule::ApplyAdioCommand() {
  if (digital_input_status_loopback_command_vector_.size() ==
      digital_input_status_vector_.size()) {
    for (int j = 0; j < digital_input_status_vector_.size(); ++j) {
      for (int i = 0; i < digital_input_status_vector_.at(j)->signals()->size();
           ++i) {
        digital_input_status_vector_.at(j)
            ->mutable_signals()
            ->GetMutableObject(i)
            ->mutate_value(digital_input_status_loopback_command_vector_.at(j)
                               ->signals()
                               ->Get(i)
                               ->value());
      }
    }
  }
  if (analog_input_status_loopback_command_vector_.size() ==
      analog_input_status_vector_.size()) {
    for (int j = 0; j < analog_input_status_vector_.size(); ++j) {
      for (int i = 0; i < analog_input_status_loopback_command_vector_.at(j)
                              ->signals()
                              ->size();
           ++i) {
        analog_input_status_vector_.at(j)
            ->mutable_signals()
            ->GetMutableObject(i)
            ->mutate_value(analog_input_status_loopback_command_vector_.at(j)
                               ->signals()
                               ->Get(i)
                               ->value());
      }
    }
  }
  if (digital_output_command_loopback_status_vector_.size() ==
      digital_output_command_vector_.size()) {
    for (int j = 0; j < digital_output_command_vector_.size(); ++j) {
      for (int i = 0;
           i < digital_output_command_vector_.at(j)->signals()->size(); ++i) {
        digital_output_command_loopback_status_vector_.at(j)
            ->mutable_signals()
            ->GetMutableObject(i)
            ->mutate_value(digital_output_command_vector_.at(j)
                               ->signals()
                               ->Get(i)
                               ->value());
      }
    }
  }
  if (analog_output_command_loopback_status_vector_.size() ==
      analog_output_command_vector_.size()) {
    for (int j = 0; j < analog_output_command_vector_.size(); ++j) {
      for (int i = 0;
           i < analog_output_command_vector_.at(j)->signals()->size(); ++i) {
        analog_output_command_loopback_status_vector_.at(j)
            ->mutable_signals()
            ->GetMutableObject(i)
            ->mutate_value(analog_output_command_vector_.at(j)
                               ->signals()
                               ->Get(i)
                               ->value());
      }
    }
  }
  return icon::OkStatus();
}

icon::RealtimeStatus FakeModule::ApplyCommand() {
  INTRINSIC_RT_RETURN_IF_ERROR(ApplyArmCommand());
  return ApplyAdioCommand();
}

void FakeModule::RuntimeLoop() {
  INTRINSIC_RT_LOG(INFO) << "Entering runtime loop";
  absl::Duration longest_tick_blocking_in_n_cycles = absl::ZeroDuration();
  size_t cycles_since_last_timing_log = 0;
  QCHECK(realtime_clock_ != nullptr);
  QCHECK(cycle_time_.has_value());
  while (!cancel_clock_ticking_thread_) {
    auto cycle_start_time = absl::Now();
    if (module_state_ == ModuleState::kActive ||
        module_state_ == ModuleState::kMotionEnabled) {
      absl::Time before_tick_blocking = absl::Now();
      if (auto ret = realtime_clock_->TickBlockingWithTimeout(
              // TODO(b/276430306): Add option to configure the timeout.
              intrinsic::Clock::Now(), absl::InfiniteDuration());
          !ret.ok()) {
        // TODO(b/276430306): Configure the timeout behavior.
        INTRINSIC_RT_LOG_THROTTLED(ERROR)
            << "TickBlocking returned an error: " << ret.message();
        // Explicitly calling sleep so other threads get a chance to run.
        absl::SleepFor(0.5 * cycle_time_.value());
      }
      absl::Time after_tick_blocking = absl::Now();
      absl::Duration tick_blocking = after_tick_blocking - before_tick_blocking;
      longest_tick_blocking_in_n_cycles =
          std::max(longest_tick_blocking_in_n_cycles, tick_blocking);

      if (++cycles_since_last_timing_log >= kLogTimingEveryNCycles) {
        INTRINSIC_RT_LOG(INFO)
            << "Longest ICON control phase in the last "
            << kLogTimingEveryNCycles << " cycles took "
            << absl::ToDoubleMicroseconds(longest_tick_blocking_in_n_cycles)
            << " us.";
        longest_tick_blocking_in_n_cycles = absl::ZeroDuration();
        cycles_since_last_timing_log = 0;
      }
    }
    absl::Duration tick_duration = absl::Now() - cycle_start_time;
    absl::SleepFor(cycle_time_.value() - tick_duration);
  }
  LOG(ERROR) << "Finishing runtime loop.";
}

FakeModule::~FakeModule() {
  module_state_ = ModuleState::kShutdown;
  cancel_clock_ticking_thread_ = true;
  if (runtime_loop_thread_.joinable()) {
    runtime_loop_thread_.join();
  }
}

icon::RealtimeStatus FakeModule::ReadStatus() {
  // Return the latest injected error from the dummy service if there is one.
  return dummy_service_.GetInjectedError();
}

absl::Status FakeModule::Prepare() {
  if (realtime_clock_ != nullptr) {
    cancel_clock_ticking_thread_ = true;
    auto runtime_loop_running = std::make_shared<absl::Notification>();
    if (runtime_loop_thread_.joinable()) {
      LOG(INFO) << "Joining runtime loop thread.";
      runtime_loop_thread_.join();
    }
    runtime_loop_thread_ = intrinsic::Thread();
    cancel_clock_ticking_thread_ = false;
    if (auto status = realtime_clock_->Reset(absl::InfiniteDuration());
        !status.ok()) {
      LOG(ERROR) << "Failed to reset the clock: " << status;
    }
    INTR_ASSIGN_OR_RETURN(
        runtime_loop_thread_,
        CreateRealtimeCapableThread(clock_ticking_thread_options_,
                                    [this, runtime_loop_running]() {
                                      runtime_loop_running->Notify();
                                      RuntimeLoop();
                                    }));
    auto start_up_timeout = absl::Seconds(3);
    // Wait until the thread is running.
    if (!runtime_loop_running->WaitForNotificationWithTimeout(
            start_up_timeout)) {
      return absl::DeadlineExceededError(
          absl::StrFormat("Timeout after %s starting the clock thread.",
                          absl::FormatDuration(start_up_timeout)));
    }
  }

  return absl::OkStatus();
}

icon::RealtimeStatus FakeModule::Activate() {
  INTRINSIC_RT_LOG(INFO) << "Activate";
  module_state_ = ModuleState::kActive;
  return icon::OkStatus();
}

icon::RealtimeStatus FakeModule::Deactivate() {
  INTRINSIC_RT_LOG(INFO) << "Deactivate";
  module_state_ = ModuleState::kInactive;
  cancel_clock_ticking_thread_ = true;
  return icon::OkStatus();
}

absl::Status FakeModule::EnableMotion() {
  INTRINSIC_RT_LOG(INFO) << "EnableMotion";

  return absl::OkStatus();
}

icon::RealtimeStatus FakeModule::Enabled() {
  module_state_ = ModuleState::kMotionEnabled;
  return icon::OkStatus();
}

icon::RealtimeStatus FakeModule::Disabled() {
  module_state_ = ModuleState::kActive;
  return icon::OkStatus();
}

absl::Status FakeModule::DisableMotion() {
  INTRINSIC_RT_LOG(INFO) << "DisableMotion";

  return absl::OkStatus();
}

absl::Status FakeModule::ClearFaults() {
  INTRINSIC_RT_LOG(INFO) << "ClearFaults";
  module_state_ = ModuleState::kActive;
  return absl::OkStatus();
}

absl::Status FakeModule::Shutdown() {
  INTRINSIC_RT_LOG(INFO) << "Shutdown";
  module_state_ = ModuleState::kShutdown;
  cancel_clock_ticking_thread_ = true;
  return absl::OkStatus();
}

absl::Status FakeModule::ProvideInspectionData(
    intrinsic_proto::icon::v1::HardwareModuleInspectionData& data) {
  intrinsic_proto::icon::v1::Event event;
  event.mutable_timestamp()->set_seconds(1625097600);
  event.set_severity(intrinsic_proto::icon::v1::Event::INFO);
  event.set_message("This is a demo event");
  *data.mutable_event_history()->add_events() = event;
  return absl::OkStatus();
}

}  // namespace intrinsic::timeslicer::fake_module
