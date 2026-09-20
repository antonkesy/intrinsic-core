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

#ifndef INTRINSIC_ICON_HARDWARE_MODULES_FAKE_FAKE_MODULE_H_
#define INTRINSIC_ICON_HARDWARE_MODULES_FAKE_FAKE_MODULE_H_

#include <atomic>
#include <cstdint>
#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/time/time.h"
#include "grpcpp/server_context.h"
#include "grpcpp/support/status.h"
#include "intrinsic/icon/control/realtime_clock_interface.h"
#include "intrinsic/icon/control/safety/safety_messages.fbs.h"
#include "intrinsic/icon/flatbuffers/transform_types.fbs.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/hardware_interface_registry.h"
#include "intrinsic/icon/hal/hardware_module_init_context.h"
#include "intrinsic/icon/hal/hardware_module_interface.h"
#include "intrinsic/icon/hal/interfaces/control_mode.fbs.h"
#include "intrinsic/icon/hal/interfaces/force_torque.fbs.h"
#include "intrinsic/icon/hal/interfaces/io_controller.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_limits.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_state.fbs.h"
#include "intrinsic/icon/hal/interfaces/payload_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/payload_state.fbs.h"
#include "intrinsic/icon/hal/interfaces/rangefinder.fbs.h"
#include "intrinsic/icon/hal/proto/hardware_module_inspection.pb.h"
#include "intrinsic/icon/hardware_modules/fake/fake_module_config.pb.h"
#include "intrinsic/icon/hardware_modules/fake/hardware_module_dummy_service.grpc.pb.h"
#include "intrinsic/icon/hardware_modules/fake/hardware_module_dummy_service.pb.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/async_buffer.h"
#include "intrinsic/icon/utils/inspection_publisher.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/util/invalid_until_set.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/thread/thread.h"
#include "intrinsic/util/thread/thread_options.h"

namespace intrinsic::timeslicer::fake_module {

// Used to inject errors in integration tests and to test the registration of
// the HardwareModuleRuntime during FakeModule::Init().
class HardwareModuleDummyServiceImpl
    : public ::intrinsic_proto::HardwareModuleDummyService::Service {
  ::grpc::Status DoRequest(
      ::grpc::ServerContext* context,
      const ::intrinsic_proto::HardwareModuleDummyRequest* request,
      ::intrinsic_proto::HardwareModuleDummyResponse* response) override {
    return ToGrpcStatus(absl::OkStatus());
  }

  ::grpc::Status InjectError(
      ::grpc::ServerContext* context,
      const ::intrinsic_proto::HardwareModuleInjectErrorRequest* request,
      ::intrinsic_proto::HardwareModuleInjectErrorResponse* response) override {
    icon::RealtimeStatus* status_buffer = injected_error_.GetFreeBuffer();
    *status_buffer = icon::RealtimeStatus(
        absl::StatusCode(request->error_code()), request->error_message());
    injected_error_.CommitFreeBuffer();
    return ToGrpcStatus(absl::OkStatus());
  }

 public:
  icon::RealtimeStatus GetInjectedError() {
    icon::RealtimeStatus* status_ptr;
    if (injected_error_.GetActiveBuffer(&status_ptr)) {
      return *status_ptr;
    }
    return icon::OkStatus();
  }

 private:
  AsyncBuffer<icon::RealtimeStatus> injected_error_;
};

// A special version of a loopback_hardware_module that allows configuring the
// cycle_time, exposed Interfaces and setting the initial state.
// Currently only the joint commands for the active `ControlMode` are
// looped back to the corresponding joint_states.
class FakeModule : public intrinsic::icon::HardwareModuleInterface {
 public:
  FakeModule() = default;
  ~FakeModule() override;

  absl::Status Init(
      intrinsic::icon::HardwareModuleInitContext& init_context) override;

  absl::Status Prepare() override;

  icon::RealtimeStatus Activate() override;

  icon::RealtimeStatus Deactivate() override;

  absl::Status EnableMotion() override;

  icon::RealtimeStatus Enabled() override;

  icon::RealtimeStatus Disabled() override;

  absl::Status DisableMotion() override;

  absl::Status ClearFaults() override;

  absl::Status Shutdown() override;

  icon::RealtimeStatus ApplyCommand() override;

  icon::RealtimeStatus ReadStatus() override;

  absl::Status ProvideInspectionData(
      intrinsic_proto::icon::v1::HardwareModuleInspectionData& data) override;

 private:
  enum class ModuleState : uint8_t {
    kShutdown = 0,
    kInactive = 1,
    kActive = 2,
    kMotionEnabled = 3,
  };

  absl::Status InitializeArmPartInterfaces(
      icon::HardwareInterfaceRegistry& interface_registry,
      const intrinsic_proto::icon::FakeModuleConfig::ArmInterfaces& config);

  absl::Status InitializeADIOPartInterface(
      icon::HardwareInterfaceRegistry& interface_registry,
      const intrinsic_proto::icon::FakeModuleConfig::ADIOInterface& config);

  absl::Status InitializeForceTorquePartInterface(
      icon::HardwareInterfaceRegistry& interface_registry,
      const intrinsic_proto::icon::FakeModuleConfig::ForceTorqueInterface&
          config);

  absl::Status InitializeRangefinderPartInterface(
      icon::HardwareInterfaceRegistry& interface_registry,
      const intrinsic_proto::icon::FakeModuleConfig::RangefinderInterface&
          config);

  // Loops back arm interface commands if they are configured.
  icon::RealtimeStatus ApplyArmCommand();
  // Loops back ADIO interface commands if they are configured.
  icon::RealtimeStatus ApplyAdioCommand();

  absl::Status ReadAndPublishInspectionData();

  // The logic to tick the clock.
  void RuntimeLoop() INTRINSIC_CHECK_REALTIME_SAFE;

  std::atomic<ModuleState> module_state_;
  static_assert(decltype(module_state_)::is_always_lock_free);

  std::optional<const absl::Duration> cycle_time_;

  intrinsic::icon::RealtimeClockInterface* realtime_clock_;
  intrinsic::Thread runtime_loop_thread_;

  // Command Interfaces
  std::optional<::intrinsic::icon::MutableStrictHardwareInterfaceHandle<
      ::intrinsic_fbs::JointPositionCommand>>
      joint_position_command_;
  std::optional<::intrinsic::icon::MutableStrictHardwareInterfaceHandle<
      ::intrinsic_fbs::JointCommandedPosition>>
      joint_commanded_position_;
  std::optional<::intrinsic::icon::MutableHardwareInterfaceHandle<
      ::intrinsic_fbs::JointVelocityCommand>>
      joint_velocity_command_;
  std::optional<::intrinsic::icon::MutableHardwareInterfaceHandle<
      ::intrinsic_fbs::JointTorqueCommand>>
      joint_torque_command_;
  std::optional<
      ::intrinsic::icon::MutableHardwareInterfaceHandle<intrinsic_fbs::Wrench>>
      process_wrench_command_;
  std::optional<::intrinsic::icon::HardwareInterfaceHandle<
      ::intrinsic_fbs::HandGuidingCommand>>
      hand_guiding_command_;
  std::optional<::intrinsic::icon::MutableHardwareInterfaceHandle<
      ::intrinsic_fbs::PayloadCommand>>
      payload_command_;
  // State Interfaces
  std::optional<::intrinsic::icon::MutableHardwareInterfaceHandle<
      ::intrinsic_fbs::JointPositionState>>
      joint_position_state_;
  std::optional<::intrinsic::icon::MutableHardwareInterfaceHandle<
      ::intrinsic_fbs::JointVelocityState>>
      joint_velocity_state_;
  std::optional<::intrinsic::icon::MutableHardwareInterfaceHandle<
      ::intrinsic_fbs::JointAccelerationState>>
      joint_acceleration_state_;
  std::optional<::intrinsic::icon::MutableHardwareInterfaceHandle<
      ::intrinsic_fbs::JointTorqueState>>
      joint_torque_state_;
  std::optional<::intrinsic::icon::MutableHardwareInterfaceHandle<
      ::intrinsic_fbs::ControlModeStatus>>
      control_mode_state_;
  std::optional<::intrinsic::icon::MutableHardwareInterfaceHandle<
      ::intrinsic_fbs::PayloadState>>
      payload_state_;
  std::optional<
      ::intrinsic::icon::HardwareInterfaceHandle<::intrinsic_fbs::JointLimits>>
      joint_system_limits_;

  // Force Torque Sensor Interfaces
  std::optional<::intrinsic::icon::MutableHardwareInterfaceHandle<
      intrinsic_fbs::ForceTorqueCommand>>
      force_torque_command_;
  std::optional<::intrinsic::icon::MutableHardwareInterfaceHandle<
      intrinsic_fbs::ForceTorqueStatus>>
      force_torque_status_;

  // DIO interfaces
  std::vector<icon::MutableHardwareInterfaceHandle<intrinsic_fbs::DIOStatus>>
      digital_input_status_vector_;
  std::vector<icon::HardwareInterfaceHandle<intrinsic_fbs::DIOCommand>>
      digital_input_status_loopback_command_vector_;
  std::vector<icon::MutableHardwareInterfaceHandle<intrinsic_fbs::AIOStatus>>
      analog_input_status_vector_;
  std::vector<icon::HardwareInterfaceHandle<intrinsic_fbs::AIOStatus>>
      analog_input_status_loopback_command_vector_;
  std::vector<icon::HardwareInterfaceHandle<intrinsic_fbs::DIOCommand>>
      digital_output_command_vector_;
  std::vector<icon::MutableHardwareInterfaceHandle<intrinsic_fbs::DIOStatus>>
      digital_output_command_loopback_status_vector_;
  std::vector<icon::HardwareInterfaceHandle<intrinsic_fbs::AIOCommand>>
      analog_output_command_vector_;
  std::vector<icon::MutableHardwareInterfaceHandle<intrinsic_fbs::AIOStatus>>
      analog_output_command_loopback_status_vector_;
  std::optional<::intrinsic::icon::MutableHardwareInterfaceHandle<
      intrinsic_fbs::SafetyStatusMessage>>
      safety_status_;

  std::optional<::intrinsic::icon::MutableHardwareInterfaceHandle<
      intrinsic_fbs::RangeFinderStatus>>
      rangefinder_status_;

  HardwareModuleDummyServiceImpl dummy_service_;

  intrinsic::ThreadOptions clock_ticking_thread_options_;
  std::atomic<bool> cancel_clock_ticking_thread_ = false;
  static_assert(decltype(cancel_clock_ticking_thread_)::is_always_lock_free);

  InvalidUntilSet<icon::InspectionPublisher> inspection_publisher_;
  intrinsic::Thread inspection_publisher_thread_;
};

}  // namespace intrinsic::timeslicer::fake_module

#endif  // INTRINSIC_ICON_HARDWARE_MODULES_FAKE_FAKE_MODULE_H_
