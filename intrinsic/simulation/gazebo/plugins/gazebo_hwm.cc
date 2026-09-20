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

#include "intrinsic/simulation/gazebo/plugins/gazebo_hwm.h"

#include <cstddef>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "absl/cleanup/cleanup.h"
#include "absl/container/flat_hash_map.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/synchronization/notification.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "gz/common/Profiler.hh"
#include "gz/sim/Entity.hh"
#include "intrinsic/icon/control/safety/extern/safety_status.fbs.h"
#include "intrinsic/icon/control/safety/safety_messages.fbs.h"
#include "intrinsic/icon/flatbuffers/transform_types.fbs.h"
#include "intrinsic/icon/hal/command_validator.h"
#include "intrinsic/icon/hal/default_hardware_interfaces.h"  // IWYU pragma: keep (this module uses standard hardware interfaces!)
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/hardware_interface_registry.h"
#include "intrinsic/icon/hal/hardware_module_init_context.h"
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
#include "intrinsic/icon/proto/safety_status.pb.h"
#include "intrinsic/icon/proto/safety_status_conversion.h"
#include "intrinsic/icon/server/config/dio_config.pb.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/simulation/gazebo/plugins/sim_hardware_interface_data.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/thread/rt_thread.h"
#include "intrinsic/util/thread/thread.h"
#include "intrinsic/util/thread/thread_options.h"

namespace intrinsic::simulation {
namespace {

constexpr absl::Duration kClockResetTimeout = absl::Seconds(10);
constexpr absl::Duration kClockQueueTimeout = absl::Seconds(10);
constexpr absl::Duration kTickTimeout = absl::Seconds(10);

constexpr intrinsic_fbs::ModeOfSafeOperation kDefaultModeOfSafeOperation =
    intrinsic_fbs::ModeOfSafeOperation::AUTOMATIC;
constexpr intrinsic_fbs::ButtonStatus kDefaultEstopButtonStatus =
    intrinsic_fbs::ButtonStatus::DISENGAGED;
constexpr intrinsic_fbs::ButtonStatus kDefaultEnableButtonStatus =
    intrinsic_fbs::ButtonStatus::ENGAGED;
constexpr intrinsic_fbs::RequestedBehavior kDefaultRequestedBehavior =
    intrinsic_fbs::RequestedBehavior::NORMAL_OPERATION;

absl::StatusOr<size_t> LookupNumJoints(
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups,
    absl::string_view name) {
  auto group = joint_groups.find(name);
  if (group == joint_groups.end()) {
    return absl::NotFoundError(
        absl::StrCat("There is no joint group '", name, "'"));
  }
  return group->second.size();
}

std::vector<std::string> BitAliasListFromSparseMap(
    const absl::flat_hash_map<size_t, std::string>& bit_index_to_alias,
    size_t num_bits, absl::string_view prefix_for_unspecified_bits) {
  std::vector<std::string> aliases;
  aliases.reserve(num_bits);
  for (size_t i = 0; i < num_bits; ++i) {
    if (auto index_and_alias = bit_index_to_alias.find(i);
        index_and_alias != bit_index_to_alias.end()) {
      aliases.push_back(index_and_alias->second);
    } else {
      aliases.push_back(absl::StrCat(prefix_for_unspecified_bits, i));
    }
  }
  return aliases;
}

// LINT.IfChange(register_interfaces)
struct RegisterInterfaceVisitor {
  const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
      joint_groups_by_name;
  icon::HardwareInterfaceRegistry& interface_registry;
  absl::string_view interface_name;

  absl::StatusOr<GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes>
  operator()(const hardware_interface_data::StrictJointPositionCommandData&
                 strict_jpos_data) {
    INTR_ASSIGN_OR_RETURN(size_t num_joints,
                          LookupNumJoints(joint_groups_by_name,
                                          strict_jpos_data.joint_group_name));
    // Assign to intermediate variable, so that we don't convert
    // StatusOr<HandleT> ->
    // GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes (instead
    // of the correct conversion, HandleT ->
    // GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes).
    INTR_ASSIGN_OR_RETURN(
        auto handle,
        interface_registry
            .AdvertiseStrictInterface<intrinsic_fbs::JointPositionCommand>(
                interface_name, num_joints));
    return handle;
  }

  absl::StatusOr<GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes>
  operator()(const hardware_interface_data::NonStrictJointPositionCommandData&
                 non_strict_jpos_data) {
    INTR_ASSIGN_OR_RETURN(
        size_t num_joints,
        LookupNumJoints(joint_groups_by_name,
                        non_strict_jpos_data.joint_group_name));
    // Assign to intermediate variable, so that we don't convert
    // StatusOr<HandleT> ->
    // GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes (instead
    // of the correct conversion, HandleT ->
    // GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes).
    INTR_ASSIGN_OR_RETURN(
        auto handle,
        interface_registry
            .AdvertiseInterface<intrinsic_fbs::JointPositionCommand>(
                interface_name, num_joints));
    LOG(INFO) << "Registered non-strict joint position command interface '"
              << interface_name << "' with " << num_joints << " joints";
    return handle;
  }

  absl::StatusOr<GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes>
  operator()(const hardware_interface_data::StrictJointTorqueCommandData&
                 strict_jtorque_data) {
    INTR_ASSIGN_OR_RETURN(
        size_t num_joints,
        LookupNumJoints(joint_groups_by_name,
                        strict_jtorque_data.joint_group_name));
    // Assign to intermediate variable, so that we don't convert
    // StatusOr<HandleT> ->
    // GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes (instead
    // of the correct conversion, HandleT ->
    // GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes).
    INTR_ASSIGN_OR_RETURN(
        auto handle,
        interface_registry
            .AdvertiseStrictInterface<intrinsic_fbs::JointTorqueCommand>(
                interface_name, num_joints));
    return handle;
  }

  absl::StatusOr<GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes>
  operator()(const hardware_interface_data::NonStrictJointTorqueCommandData&
                 non_strict_jtorque_data) {
    INTR_ASSIGN_OR_RETURN(
        size_t num_joints,
        LookupNumJoints(joint_groups_by_name,
                        non_strict_jtorque_data.joint_group_name));
    // Assign to intermediate variable, so that we don't convert
    // StatusOr<HandleT> ->
    // GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes (instead
    // of the correct conversion, HandleT ->
    // GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes).
    INTR_ASSIGN_OR_RETURN(
        auto handle, interface_registry
                         .AdvertiseInterface<intrinsic_fbs::JointTorqueCommand>(
                             interface_name, num_joints));
    return handle;
  }

  absl::StatusOr<GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes>
  operator()(const hardware_interface_data::JointPositionStateData&
                 position_state_data) {
    INTR_ASSIGN_OR_RETURN(
        size_t num_joints,
        LookupNumJoints(joint_groups_by_name,
                        position_state_data.joint_group_name));
    // Assign to intermediate variable, so that we don't convert
    // StatusOr<HandleT> ->
    // GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes (instead
    // of the correct conversion, HandleT ->
    // GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes).
    INTR_ASSIGN_OR_RETURN(
        auto handle,
        interface_registry
            .AdvertiseMutableInterface<intrinsic_fbs::JointPositionState>(
                interface_name, num_joints));
    return handle;
  }

  absl::StatusOr<GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes>
  operator()(const hardware_interface_data::JointCommandedPositionData&
                 commanded_position_data) {
    INTR_ASSIGN_OR_RETURN(
        size_t num_joints,
        LookupNumJoints(joint_groups_by_name,
                        commanded_position_data.joint_group_name));
    // Assign to intermediate variable, so that we don't convert
    // StatusOr<HandleT> ->
    // GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes (instead
    // of the correct conversion, HandleT ->
    // GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes).
    INTR_ASSIGN_OR_RETURN(
        auto handle,
        interface_registry
            .AdvertiseMutableInterface<intrinsic_fbs::JointCommandedPosition>(
                interface_name, num_joints));
    return handle;
  }

  absl::StatusOr<GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes>
  operator()(const hardware_interface_data::JointVelocityStateData&
                 velocity_state_data) {
    INTR_ASSIGN_OR_RETURN(
        size_t num_joints,
        LookupNumJoints(joint_groups_by_name,
                        velocity_state_data.joint_group_name));
    // Assign to intermediate variable, so that we don't convert
    // StatusOr<HandleT> ->
    // GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes (instead
    // of the correct conversion, HandleT ->
    // GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes).
    INTR_ASSIGN_OR_RETURN(
        auto handle,
        interface_registry
            .AdvertiseMutableInterface<intrinsic_fbs::JointVelocityState>(
                interface_name, num_joints));
    return handle;
  }

  absl::StatusOr<GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes>
  operator()(const hardware_interface_data::JointAccelerationStateData&
                 acceleration_state_data) {
    INTR_ASSIGN_OR_RETURN(
        size_t num_joints,
        LookupNumJoints(joint_groups_by_name,
                        acceleration_state_data.joint_group_name));
    // Assign to intermediate variable, so that we don't convert
    // StatusOr<HandleT> ->
    // GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes (instead
    // of the correct conversion, HandleT ->
    // GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes).
    INTR_ASSIGN_OR_RETURN(
        auto handle,
        interface_registry
            .AdvertiseMutableInterface<intrinsic_fbs::JointAccelerationState>(
                interface_name, num_joints));
    return handle;
  }

  absl::StatusOr<GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes>
  operator()(
      const hardware_interface_data::JointTorqueStateData& torque_state_data) {
    INTR_ASSIGN_OR_RETURN(size_t num_joints,
                          LookupNumJoints(joint_groups_by_name,
                                          torque_state_data.joint_group_name));
    // Assign to intermediate variable, so that we don't convert
    // StatusOr<HandleT> ->
    // GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes (instead
    // of the correct conversion, HandleT ->
    // GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes).
    INTR_ASSIGN_OR_RETURN(
        auto handle,
        interface_registry
            .AdvertiseMutableInterface<intrinsic_fbs::JointTorqueState>(
                interface_name, num_joints));
    return handle;
  }

  absl::StatusOr<GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes>
  operator()(const hardware_interface_data::KinematicChainPayloadCommandData&
                 payload_command_data) {
    INTR_ASSIGN_OR_RETURN(
        auto handle,
        interface_registry.AdvertiseInterface<intrinsic_fbs::PayloadCommand>(
            interface_name));
    return handle;
  }

  absl::StatusOr<GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes>
  operator()(const hardware_interface_data::KinematicChainPayloadStateData&
                 payload_state_data) {
    INTR_ASSIGN_OR_RETURN(
        auto handle,
        interface_registry
            .AdvertiseMutableInterface<intrinsic_fbs::PayloadState>(
                interface_name));
    return handle;
  }

  absl::StatusOr<GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes>
  operator()(
      const hardware_interface_data::KinematicChainProcessWrenchCommandData&
          process_wrench_command_data) {
    INTR_ASSIGN_OR_RETURN(
        auto handle,
        interface_registry.AdvertiseInterface<intrinsic_fbs::Wrench>(
            interface_name));
    return handle;
  }

  absl::StatusOr<GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes>
  operator()(const hardware_interface_data::DigitalInputStatusData&
                 digital_input_status_data) {
    INTR_ASSIGN_OR_RETURN(
        auto handle,
        interface_registry.AdvertiseMutableInterface<intrinsic_fbs::DIOStatus>(
            interface_name, BitAliasListFromSparseMap(
                                digital_input_status_data.bit_index_to_alias,
                                digital_input_status_data.num_inputs,
                                absl::StrCat(interface_name, "_"))));
    return handle;
  }

  absl::StatusOr<GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes>
  operator()(const hardware_interface_data::DigitalOutputCommandData&
                 digital_output_command_data) {
    std::vector<std::string> output_names(
        digital_output_command_data.num_outputs);
    for (int i = 0; i < digital_output_command_data.num_outputs; ++i) {
      output_names[i] = absl::StrCat(interface_name, "_", i);
    }
    INTR_ASSIGN_OR_RETURN(
        auto handle,
        interface_registry.AdvertiseInterface<intrinsic_fbs::DIOCommand>(
            interface_name, BitAliasListFromSparseMap(
                                digital_output_command_data.bit_index_to_alias,
                                digital_output_command_data.num_outputs,
                                absl::StrCat(interface_name, "_"))));
    return handle;
  }

  absl::StatusOr<GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes>
  operator()(const hardware_interface_data::ForceTorqueCommandData&
                 force_torque_command_data) {
    INTR_ASSIGN_OR_RETURN(
        auto handle, interface_registry
                         .AdvertiseInterface<intrinsic_fbs::ForceTorqueCommand>(
                             interface_name));
    return handle;
  }

  absl::StatusOr<GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes>
  operator()(const hardware_interface_data::ForceTorqueStatusData&
                 force_torque_status_data) {
    INTR_ASSIGN_OR_RETURN(
        auto handle,
        interface_registry
            .AdvertiseMutableInterface<intrinsic_fbs::ForceTorqueStatus>(
                interface_name));
    // Initialize FT sensor status code to Ok. This field defaults to
    // GenericError, and if the sensor reports an error during ICON startup,
    // ICON will fail.
    //
    // So we set the status here to ensure that ICON doesn't see the
    // GenericError code.
    handle->mutate_status_code(intrinsic_fbs::ForceSensorStatusCode::Ok);
    return handle;
  }

  absl::StatusOr<GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes>
  operator()(const hardware_interface_data::AnalogInputStatusData&
                 analog_input_status_data) {
    std::vector<std::string> input_names(analog_input_status_data.num_inputs);
    for (int i = 0; i < analog_input_status_data.num_inputs; ++i) {
      input_names[i] = absl::StrCat(interface_name, "_", i);
    }
    INTR_ASSIGN_OR_RETURN(
        auto handle,
        interface_registry.AdvertiseMutableInterface<intrinsic_fbs::AIOStatus>(
            interface_name, input_names));
    return handle;
  }

  absl::StatusOr<GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes>
  operator()(const hardware_interface_data::AnalogOutputCommandData&
                 analog_output_command_data) {
    std::vector<std::string> output_names(
        analog_output_command_data.num_outputs);
    for (int i = 0; i < analog_output_command_data.num_outputs; ++i) {
      output_names[i] = absl::StrCat(interface_name, "_", i);
    }
    INTR_ASSIGN_OR_RETURN(
        auto handle,
        interface_registry.AdvertiseInterface<intrinsic_fbs::AIOCommand>(
            interface_name, output_names));
    return handle;
  }

  absl::StatusOr<GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes>
  operator()(const hardware_interface_data::JointLimitsCommandData&
                 joint_limits_command_data) {
    INTR_ASSIGN_OR_RETURN(
        size_t num_joints,
        LookupNumJoints(joint_groups_by_name,
                        joint_limits_command_data.joint_group_name));
    INTR_ASSIGN_OR_RETURN(
        auto handle,
        interface_registry.AdvertiseInterface<intrinsic_fbs::JointLimits>(
            interface_name, num_joints));
    return handle;
  }

  absl::StatusOr<GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes>
  operator()(const hardware_interface_data::RangefinderStatusData& /*unused*/) {
    return interface_registry
        .AdvertiseMutableInterface<intrinsic_fbs::RangeFinderStatus>(
            interface_name);
  }
};

absl::StatusOr<absl::flat_hash_map<
    std::string,
    GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes>>
RegisterInterfaces(
    const absl::flat_hash_map<std::string, std::vector<gz::sim::Entity>>&
        joint_groups_by_name,
    const absl::flat_hash_map<std::string,
                              hardware_interface_data::HardwareInterfaceData>&
        hardware_interface_name_to_gazebo_data,
    icon::HardwareInterfaceRegistry& interface_registry) {
  absl::flat_hash_map<
      std::string,
      GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes>
      interface_handle_map;
  for (const auto& [interface_name, hardware_interface_handle] :
       hardware_interface_name_to_gazebo_data) {
    INTR_ASSIGN_OR_RETURN(interface_handle_map[interface_name],
                          std::visit(
                              RegisterInterfaceVisitor{
                                  .joint_groups_by_name = joint_groups_by_name,
                                  .interface_registry = interface_registry,
                                  .interface_name = interface_name,
                              },
                              hardware_interface_handle));
  }

  // Safety interface. This is a dummy.
  // We create it unconditionally, and with a hard-coded name. It's safe enough
  // to assume all safety interfaces should be called "safety_status" – if ICON
  // doesn't expect one, then it will just ignore the interface.
  INTR_ASSIGN_OR_RETURN(
      interface_handle_map["safety_status"],
      interface_registry
          .AdvertiseMutableInterface<intrinsic_fbs::SafetyStatusMessage>(
              "safety_status",
              /*mode_of_safe_operation=*/kDefaultModeOfSafeOperation,
              /*estop_button_status=*/kDefaultEstopButtonStatus,
              /*enable_button_status=*/kDefaultEnableButtonStatus,
              /*requested_behavior=*/kDefaultRequestedBehavior));
  return interface_handle_map;
}
// LINT.ThenChange(///intrinsic/simulation/gazebo/plugins/gazebo_hwm.h:interface_handle_types)

}  // namespace

GazeboHardwareModule::GazeboHardwareModule(GazeboHardwareModule::Config config)
    : config_(std::move(config)) {}

GazeboHardwareModule::~GazeboHardwareModule() {
  if (auto shutdown_status = Shutdown(); !shutdown_status.ok()) {
    LOG(ERROR) << "[" << config_.hwm_name
               << "] Error during shutdown: " << shutdown_status;
  }
  if (tick_realtime_clock_thread_.joinable()) {
    tick_realtime_clock_thread_.join();
  }
}

icon::RealtimeStatus GazeboHardwareModule::WaitForIconTicksToFinish() {
  GZ_PROFILE("GazeboHardwareModule::WaitForIconTicksToFinish");
  INTRINSIC_RT_RETURN_IF_ERROR(icon::RealtimeStatus(
      config_.init_error.code(), config_.init_error.message()));

  // Block Gazebo stepping if we are in Preparing or Prepared state.
  // Wait until we transition to Activated (active) or Deactivated
  // (shutdown/inactive).
  {
    absl::MutexLock clock_lock(&sim_clock_mutex_);
    bool await_success = sim_clock_mutex_.AwaitWithTimeout(
        absl::Condition(this, &GazeboHardwareModule::ReadyToStepOrShutdown),
        kClockQueueTimeout);
    if (!await_success) {
      LOG(ERROR) << "[" << config_.hwm_name
                 << "] Timed out waiting to enter active or deactivated state.";
      return icon::InternalError(
          "Timed out waiting to enter active or deactivated state.");
    }
  }

  if (!active_) {
    return icon::OkStatus();
  }

  // If this simulated HWM is a clock driver (has a non-null realtime clock),
  // then we wait for the sim time queue to be empty.
  if (realtime_clock_ != nullptr) {
    absl::MutexLock clock_lock(sim_clock_mutex_);
    // Block until the sim time queue is empty, that is until ICON has handled
    // the previous gazebo tick.
    bool await_success = sim_clock_mutex_.AwaitWithTimeout(
        absl::Condition(this,
                        &GazeboHardwareModule::SimTimeQueueEmptyOrShutdown),
        kClockQueueTimeout);
    if (!await_success) {
      LOG(ERROR) << "[" << config_.hwm_name
                 << "] Timed out waiting for empty sim time queue.";
      return icon::InternalError("Timed out waiting for empty sim time queue.");
    }
  }
  // Regardless of whether we have a realtime clock or not, we wait for ICON to
  // call ReadStatus() as many times as we expect.
  {
    absl::MutexLock l(read_status_calls_mutex_);
    // Similar, but for the count of expected ReadStatus calls.
    bool await_success = read_status_calls_mutex_.AwaitWithTimeout(
        absl::Condition(
            this,
            &GazeboHardwareModule::NoMoreExpectedReadStatusCallsOrShutdown),
        kClockQueueTimeout);
    if (!await_success) {
      LOG(ERROR) << "[" << config_.hwm_name
                 << "] Timed out waiting for ICON to make the expected "
                    "ReadStatus calls.";
      return icon::InternalError(
          "Timed out waiting for ICON to make the expected ReadStatus calls.");
    }
  }
  return icon::OkStatus();
}

icon::RealtimeStatus GazeboHardwareModule::RequestIconTick(
    ::intrinsic::Time sim_time) {
  GZ_PROFILE("GazeboHardwareModule::RequestIconTick");
  INTRINSIC_RT_RETURN_IF_ERROR(icon::RealtimeStatus(
      config_.init_error.code(), config_.init_error.message()));
  if (!active_) {
    return icon::OkStatus();
  }
  absl::MutexLock read_status_count_lock(read_status_calls_mutex_);
  if (realtime_clock_ != nullptr) {
    absl::MutexLock l(sim_clock_mutex_);
    sim_time_queue_.push(sim_time);
  }
  ++num_expected_read_status_calls_;
  return icon::OkStatus();
}

absl::Status GazeboHardwareModule::Init(
    intrinsic::icon::HardwareModuleInitContext& init_context) {
  // Note: We don't register any gRPC services with the server builder
  // passed in `init_context`. The server builder is a dummy that is not
  // actually started.
  // cf. `HardwareModuleLauncher::PrepareRuntimeData`.

  INTR_RETURN_IF_ERROR(config_.init_error);
  icon::HardwareInterfaceRegistry& interface_registry =
      init_context.GetInterfaceRegistry();
  const icon::ModuleConfig& config = init_context.GetModuleConfig();
  realtime_clock_ = config.GetRealtimeClock();
  INTR_ASSIGN_OR_RETURN(
      (absl::flat_hash_map<
          std::string,
          GazeboHardwareModule::HardwareInterfaces::InterfaceHandleTypes>
           interface_name_to_handle),
      RegisterInterfaces(config_.joint_groups_by_name,
                         config_.hardware_interface_name_to_gazebo_data,
                         interface_registry));
  {
    absl::MutexLock l(hardware_interfaces_.mutex);
    hardware_interfaces_.handles =
        HardwareInterfaces::Handles(std::move(interface_name_to_handle));
  }

  {
    absl::MutexLock l(hardware_interfaces_.mutex);
    INTR_ASSIGN_OR_RETURN(
        hardware_interfaces_.command_validator,
        intrinsic::icon::Validator::Create(interface_registry));
  }

  tick_realtime_clock_thread_options_.SetName(
      absl::StrCat("tick_realtime_clock", config.GetName()));
  {
    absl::MutexLock sim_clock_lock(sim_clock_mutex_);
    shutdown_ = std::make_unique<absl::Notification>();
  }
  {
    absl::MutexLock l(read_status_calls_mutex_);
    read_status_shutdown_ = std::make_unique<absl::Notification>();
  }
  LOG(INFO) << "[" << config_.hwm_name << "] Initialized GazeboHWM";
  return absl::OkStatus();
}

absl::Status GazeboHardwareModule::Prepare() {
  INTR_RETURN_IF_ERROR(config_.init_error);
  {
    absl::MutexLock sim_clock_lock(&sim_clock_mutex_);
    state_ = intrinsic_fbs::StateCode::kPreparing;
  }
  auto cleanup_state = absl::MakeCleanup([&]() {
    absl::MutexLock sim_clock_lock(&sim_clock_mutex_);
    if (state_ == intrinsic_fbs::StateCode::kPreparing) {
      state_ = intrinsic_fbs::StateCode::kDeactivated;
    }
  });

  // Shut down the tick thread if it's running. And then start it (again).
  {
    absl::MutexLock read_status_count_lock(read_status_calls_mutex_);
    if (realtime_clock_ != nullptr) {
      absl::MutexLock sim_clock_lock(sim_clock_mutex_);
      if (shutdown_) {
        shutdown_->Notify();
      }
    }
    if (read_status_shutdown_) {
      read_status_shutdown_->Notify();
    }
  }
  if (tick_realtime_clock_thread_.joinable()) {
    tick_realtime_clock_thread_.join();
  }

  {
    absl::MutexLock read_status_calls_lock(read_status_calls_mutex_);
    num_expected_read_status_calls_ = 0;
  }
  {
    absl::MutexLock sim_clock_lock(sim_clock_mutex_);
    sim_time_queue_ = {};
    shutdown_ = std::make_unique<absl::Notification>();
    if (realtime_clock_ != nullptr) {
      // reset the realtime clock, in case the other side (ICON) has cancelled
      // the underlying lockstep.
      INTR_RETURN_IF_ERROR(realtime_clock_->Reset(kClockResetTimeout));
    }
  }
  {
    absl::MutexLock l(read_status_calls_mutex_);
    read_status_shutdown_ = std::make_unique<absl::Notification>();
  }
  if (realtime_clock_ != nullptr) {
    INTR_ASSIGN_OR_RETURN(tick_realtime_clock_thread_,
                          CreateRealtimeCapableThread(
                              tick_realtime_clock_thread_options_,
                              &GazeboHardwareModule::TickRealtimeClock, this));
  }

  std::move(cleanup_state).Cancel();
  {
    absl::MutexLock sim_clock_lock(&sim_clock_mutex_);
    state_ = intrinsic_fbs::StateCode::kPrepared;
  }
  return absl::OkStatus();
}

icon::RealtimeStatus GazeboHardwareModule::Activate() {
  INTRINSIC_RT_RETURN_IF_ERROR(icon::RealtimeStatus(
      config_.init_error.code(), config_.init_error.message()));
  {
    absl::MutexLock sim_clock_lock(&sim_clock_mutex_);
    state_ = intrinsic_fbs::StateCode::kActivated;
  }
  active_ = true;
  LOG(INFO) << "[" << config_.hwm_name << "] Now active";
  return icon::OkStatus();
}

icon::RealtimeStatus GazeboHardwareModule::Deactivate() {
  INTRINSIC_RT_RETURN_IF_ERROR(icon::RealtimeStatus(
      config_.init_error.code(), config_.init_error.message()));
  DeactivateAndClearQueues();
  LOG(INFO) << "[" << config_.hwm_name << "] Now inactive";
  return icon::OkStatus();
}

absl::Status GazeboHardwareModule::EnableMotion() {
  INTR_RETURN_IF_ERROR(config_.init_error);
  LOG(INFO) << "[" << config_.hwm_name << "] EnableMotion";
  return absl::OkStatus();
}

icon::RealtimeStatus GazeboHardwareModule::Enabled() {
  INTRINSIC_RT_RETURN_IF_ERROR(icon::RealtimeStatus(
      config_.init_error.code(), config_.init_error.message()));
  enabled_ = true;
  LOG(INFO) << "[" << config_.hwm_name << "] Now enabled";
  return icon::OkStatus();
}

icon::RealtimeStatus GazeboHardwareModule::Disabled() {
  INTRINSIC_RT_RETURN_IF_ERROR(icon::RealtimeStatus(
      config_.init_error.code(), config_.init_error.message()));
  enabled_ = false;
  LOG(INFO) << "[" << config_.hwm_name << "] Now disabled";
  return icon::OkStatus();
}

absl::Status GazeboHardwareModule::DisableMotion() {
  INTRINSIC_RT_RETURN_IF_ERROR(icon::RealtimeStatus(
      config_.init_error.code(), config_.init_error.message()));
  enabled_ = false;
  LOG(INFO) << "[" << config_.hwm_name << "] DisableMotion";
  return absl::OkStatus();
}

absl::Status GazeboHardwareModule::ClearFaults() {
  INTR_RETURN_IF_ERROR(config_.init_error);
  return absl::OkStatus();
}

absl::Status GazeboHardwareModule::Shutdown() {
  INTR_RETURN_IF_ERROR(config_.init_error);
  LOG(INFO) << "[" << config_.hwm_name << "] GazeboHWM::Shutdown()";
  DeactivateAndClearQueues();
  {
    absl::MutexLock sim_clock_lock(sim_clock_mutex_);
    if (shutdown_ == nullptr) {
      return absl::FailedPreconditionError("Shutdown() called before Init()");
    }
    if (!shutdown_->HasBeenNotified()) {
      shutdown_->Notify();
    }
  }
  {
    absl::MutexLock l(read_status_calls_mutex_);
    if (read_status_shutdown_ == nullptr) {
      return absl::FailedPreconditionError("Shutdown() called before Init()");
    }
    if (!read_status_shutdown_->HasBeenNotified()) {
      read_status_shutdown_->Notify();
    }
  }
  if (tick_realtime_clock_thread_.joinable()) {
    tick_realtime_clock_thread_.join();
  }
  if (realtime_clock_ != nullptr) {
    INTR_RETURN_IF_ERROR(realtime_clock_->Reset(kClockResetTimeout));
  }
  return absl::OkStatus();
}

icon::RealtimeStatus GazeboHardwareModule::ReadStatus() {
  INTRINSIC_RT_RETURN_IF_ERROR(icon::RealtimeStatus(
      config_.init_error.code(), config_.init_error.message()));
  while (true) {
    absl::MutexLock l(read_status_calls_mutex_);
    bool await_success = read_status_calls_mutex_.AwaitWithTimeout(
        absl::Condition(
            this,
            &GazeboHardwareModule::NonZeroExpectedReadStatusCallsOrShutdown),
        kClockQueueTimeout);
    if (await_success) {
      --num_expected_read_status_calls_;
      break;
    }
    LOG(ERROR) << "[" << config_.hwm_name
               << "] Timed out waiting for a new sim step, will try again.";
  }
  return icon::OkStatus();
}

icon::RealtimeStatus GazeboHardwareModule::ApplyCommand() {
  INTRINSIC_RT_RETURN_IF_ERROR(icon::RealtimeStatus(
      config_.init_error.code(), config_.init_error.message()));
  return icon::OkStatus();
}

absl::Status GazeboHardwareModule::ProvideInspectionData(
    intrinsic_proto::icon::v1::HardwareModuleInspectionData& data) {
  auto safety_status = data.mutable_safety_status();
  safety_status->set_estop_button_status(
      icon::ToProto(kDefaultEstopButtonStatus));
  safety_status->set_mode_of_safe_operation(
      icon::ToProto(kDefaultModeOfSafeOperation));
  safety_status->set_enable_button_status(
      icon::ToProto(kDefaultEnableButtonStatus));
  safety_status->set_requested_behavior(
      icon::ToProto(kDefaultRequestedBehavior));

  return absl::OkStatus();
}

bool GazeboHardwareModule::NewSimTimeOrShutdown() {
  if (shutdown_ == nullptr) {
    return false;
  }
  return shutdown_->HasBeenNotified() || !sim_time_queue_.empty();
}

bool GazeboHardwareModule::SimTimeQueueEmptyOrShutdown() {
  return (shutdown_ != nullptr && shutdown_->HasBeenNotified()) ||
         sim_time_queue_.empty();
}

bool GazeboHardwareModule::NonZeroExpectedReadStatusCallsOrShutdown() {
  return (read_status_shutdown_ != nullptr &&
          read_status_shutdown_->HasBeenNotified()) ||
         num_expected_read_status_calls_ > 0;
}
bool GazeboHardwareModule::NoMoreExpectedReadStatusCallsOrShutdown() {
  return (read_status_shutdown_ != nullptr &&
          read_status_shutdown_->HasBeenNotified()) ||
         num_expected_read_status_calls_ <= 0;
}

bool GazeboHardwareModule::ReadyToStepOrShutdown() {
  return (shutdown_ != nullptr && shutdown_->HasBeenNotified()) ||
         state_ == intrinsic_fbs::StateCode::kActivated ||
         state_ == intrinsic_fbs::StateCode::kDeactivated;
}

void GazeboHardwareModule::DeactivateAndClearQueues() {
  active_ = false;
  {
    absl::MutexLock sim_clock_lock(&sim_clock_mutex_);
    state_ = intrinsic_fbs::StateCode::kDeactivated;
    sim_time_queue_ = {};
  }
  {
    absl::MutexLock read_status_calls_lock(&read_status_calls_mutex_);
    num_expected_read_status_calls_ = 0;
  }
}

void GazeboHardwareModule::TickRealtimeClock() {
  if (!config_.init_error.ok()) {
    return;
  }
  while (true) {
    ::intrinsic::Time sim_time;
    {
      absl::MutexLock clock_lock(sim_clock_mutex_);
      bool await_success = sim_clock_mutex_.AwaitWithTimeout(
          absl::Condition(this, &GazeboHardwareModule::NewSimTimeOrShutdown),
          kClockQueueTimeout);
      if (!await_success) {
        if (active_) {
          LOG(ERROR)
              << "[" << config_.hwm_name
              << "] Timed out waiting for sim time queue, will try again.";
          LOG(ERROR) << "sim_time_queue.size(): " << sim_time_queue_.size();
        } else {
          LOG(INFO) << "[" << config_.hwm_name
                    << "] Timed out waiting for sim time queue while "
                       "deactivated. This is expected, will try again.";
        }
        continue;
      }
      if (shutdown_ == nullptr) {
        INTRINSIC_RT_LOG_THROTTLED(ERROR)
            << "[" << config_.hwm_name
            << "] Shutdown notification pointer is null "
               "- cannot notify the thread.";
        continue;
      } else if (shutdown_->HasBeenNotified()) {
        break;
      }
      sim_time = sim_time_queue_.front();
    }
    // Lock access to hardware interfaces while ICON is ticking.
    absl::MutexLock hardware_interfaces_lock(hardware_interfaces_.mutex);
    if (icon::RealtimeStatus status =
            realtime_clock_->TickBlockingWithTimeout(sim_time, kTickTimeout);
        status.ok()) {
      // Only pop the sim time from the queue if we succeeded in ticking.
      // Otherwise, we want to loop around and retry (see else block below).
      absl::MutexLock clock_lock(sim_clock_mutex_);
      sim_time_queue_.pop();
    } else {
      LOG(ERROR)
          << "[" << config_.hwm_name
          << "] Failed to tick the ICON clock in TickRealtimeClock thread. "
             "Resetting clock and retrying";
      if (const auto status = realtime_clock_->Reset(kClockResetTimeout);
          !status.ok()) {
        LOG_EVERY_N_SEC(ERROR, 1)
            << "[" << config_.hwm_name
            << "] Failed to reset clock. With: " << status.message();
      }
      absl::SleepFor(absl::Milliseconds(100));
    }
  }
  LOG(INFO) << "[" << config_.hwm_name << "] Exiting TickRealtimeClock thread.";
}
}  // namespace intrinsic::simulation
