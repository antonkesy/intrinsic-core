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

#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_hardware_module.h"

#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/time/time.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/hal/default_hardware_interfaces.h"  // IWYU pragma: keep
#include "intrinsic/icon/hal/hardware_interface_registry.h"
#include "intrinsic/icon/hal/hardware_module_init_context.h"
#include "intrinsic/icon/hal/hardware_module_registry.h"
#include "intrinsic/icon/hal/interfaces/adio.fbs.h"
#include "intrinsic/icon/hal/interfaces/io_controller.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_state.fbs.h"
#include "intrinsic/icon/hal/interfaces/payload_command.fbs.h"
#include "intrinsic/icon/hal/interfaces/payload_state.fbs.h"
#include "intrinsic/icon/hal/interfaces/robot_payload_utils.h"
#include "intrinsic/icon/hal/module_config.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi.pb.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_client.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_util.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/util/fixed_vector.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/robot_payload/robot_payload_base.h"

namespace intrinsic::icon {

using ::intrinsic_fbs::JointAccelerationState;
using ::intrinsic_fbs::JointPositionCommand;
using ::intrinsic_fbs::JointPositionState;
using ::intrinsic_fbs::JointTorqueState;
using ::intrinsic_fbs::JointVelocityState;
using ::intrinsic_proto::icon::KukaRsiModule;
using kuka::kKukaNumJoints;

KukaRsiHwModule::KukaRsiHwModule(
    std::unique_ptr<kuka::RealtimeKukaRsiClient> client)
    : client_(std::move(client)) {}

absl::Status KukaRsiHwModule::Init(
    intrinsic::icon::HardwareModuleInitContext& init_context) {
  INTRINSIC_ASSERT_NON_REALTIME();
  icon::HardwareInterfaceRegistry& interface_registry =
      init_context.GetInterfaceRegistry();
  const icon::ModuleConfig& module_config = init_context.GetModuleConfig();
  INTR_ASSIGN_OR_RETURN(auto control_period, module_config.GetControlPeriod());
  double control_frequency = 1.0 / absl::ToDoubleSeconds(control_period);
  if (std::abs(control_frequency - kFrequency) > 1e-6) {
    return absl::InvalidArgumentError(
        absl::StrCat("Invalid control frequency for RSI hardware module. "
                     "Must be 250Hz, but is ",
                     control_frequency));
  }

  if (client_ == nullptr) {
    client_ = std::make_unique<kuka::RealtimeKukaRsiClient>(
        module_config.GetIconThreadOptions(), module_config.GetRealtimeClock());
  }

  INTR_ASSIGN_OR_RETURN(
      auto config,
      module_config.GetConfig<::intrinsic_proto::icon::KukaRsiModule>());

  INTR_ASSIGN_OR_RETURN(
      joint_position_command_,
      interface_registry.AdvertiseStrictInterface<JointPositionCommand>(
          "joint_position_command", kKukaNumJoints));
  INTR_ASSIGN_OR_RETURN(
      joint_position_state_,
      interface_registry.AdvertiseMutableInterface<JointPositionState>(
          "joint_position_state", kKukaNumJoints));
  if (!butterworth_joint_velocity_filter_.Init(
          eigenmath::VectorNd::Zero(kKukaNumJoints), kFrequency,
          config.velocity_settling_cutoff_frequency())) {
    return absl::InternalError(
        "Failed to initialize joint velocity filter in RSI HWM.");
  }
  filtered_joint_velocities_.setZero();
  INTR_ASSIGN_OR_RETURN(
      joint_velocity_state_,
      interface_registry.AdvertiseMutableInterface<JointVelocityState>(
          "joint_velocity_state", kKukaNumJoints));
  INTR_ASSIGN_OR_RETURN(
      joint_acceleration_state_,
      interface_registry.AdvertiseMutableInterface<JointAccelerationState>(
          "joint_acceleration_state", kKukaNumJoints));
  INTR_ASSIGN_OR_RETURN(
      joint_torque_state_,
      interface_registry.AdvertiseMutableInterface<JointTorqueState>(
          "joint_torque_state", kKukaNumJoints));
  std::vector<std::string> digital_input_descriptions;
  for (auto& name : config.digital_input_names()) {
    digital_input_descriptions.push_back(name);
  }
  LOG(INFO) << "Creating digital inputs: "
            << absl::StrJoin(digital_input_descriptions, ", ");

  INTR_ASSIGN_OR_RETURN(
      digital_input_status_,
      interface_registry.AdvertiseMutableInterface<intrinsic_fbs::DIOStatus>(
          "digital_input_status", digital_input_descriptions));

  std::vector<std::string> digital_output_descriptions;
  for (auto& name : config.digital_output_names()) {
    digital_output_descriptions.push_back(name);
  }
  LOG(INFO) << "Creating digital outputs: "
            << absl::StrJoin(digital_output_descriptions, ", ");
  INTR_ASSIGN_OR_RETURN(
      digital_output_command_,
      interface_registry.AdvertiseInterface<intrinsic_fbs::DIOCommand>(
          "digital_output_command", digital_output_descriptions));
  INTR_ASSIGN_OR_RETURN(
      digital_output_status_,
      interface_registry.AdvertiseMutableInterface<intrinsic_fbs::DIOStatus>(
          "digital_output_status", digital_output_descriptions));

  INTR_ASSIGN_OR_RETURN(
      payload_command_,
      interface_registry.AdvertiseInterface<intrinsic_fbs::PayloadCommand>(
          "payload_command"));

  INTR_ASSIGN_OR_RETURN(
      payload_state_,
      interface_registry.AdvertiseMutableInterface<intrinsic_fbs::PayloadState>(
          "payload_state"));

  INTR_ASSIGN_OR_RETURN(auto struct_config,
                        kuka::KukaConfig::FromProto(config));

  init_context.EnableCycleTimeMetrics(absl::Seconds(1.0 / kFrequency),
                                      /*log_cycle_time_warnings=*/true);

  return client_->Init(struct_config);
}

absl::Status KukaRsiHwModule::Prepare() { return client_->Prepare(); }

RealtimeStatus KukaRsiHwModule::Activate() { return client_->Activate(); }

RealtimeStatus KukaRsiHwModule::Deactivate() { return client_->Deactivate(); }

RealtimeStatus KukaRsiHwModule::Enabled() { return client_->Enabled(); }

RealtimeStatus KukaRsiHwModule::Disabled() { return client_->Disabled(); }

absl::Status KukaRsiHwModule::EnableMotion() {
  LOG(INFO) << "EnableMotion";

  std::optional<RobotPayloadBase> full_payload;
  INTR_RETURN_IF_ERROR(CopyTo(*payload_command_->full_payload(), full_payload));

  if (full_payload.has_value()) {
    INTR_RETURN_IF_ERROR(client_->SetPayload(*full_payload));
  } else {
    LOG(INFO) << "Not setting custom payload. Using payload configured on "
                 "robot controller.";
  }

  // Copy the commanded payload to the state. If no payload was commanded, it is
  // nullopt.
  INTR_RETURN_IF_ERROR(intrinsic_fbs::CopyTo(
      full_payload, *payload_state_->mutable_full_payload()));

  return client_->Enable();
}

absl::Status KukaRsiHwModule::DisableMotion() {
  LOG(INFO) << "DisableMotion";
  return client_->Disable();
}

absl::Status KukaRsiHwModule::ClearFaults() {
  LOG(INFO) << "ClearFaults";
  absl::Status ret = client_->ClearFaults();
  LOG(INFO) << "ClearFaults finished with: " << ret;
  return ret;
}

absl::Status KukaRsiHwModule::Shutdown() {
  LOG(INFO) << "Shutdown";
  if (client_ != nullptr) {
    return client_->Shutdown();
  }
  LOG(INFO) << "No client to shutdown";
  return absl::OkStatus();
}

RealtimeStatus KukaRsiHwModule::ReadStatus() {
  const kuka::RsiTelemetry telemetry = client_->GetTelemetry();
  eigenmath::Vectord<kKukaNumJoints> positions;
  for (int dof = 0; dof < kKukaNumJoints; dof++) {
    joint_position_state_->mutable_position()->Mutate(dof,
                                                      telemetry.position[dof]);
    positions(dof) = telemetry.position[dof];
    joint_torque_state_->mutable_torque()->Mutate(dof, telemetry.torque[dof]);
  }

  ComputeAndUpdateJointVelocities(positions, telemetry.interpolator_counter_ms);
  if (client_->IsEnabled()) {
    if (telemetry.digital_input.size() !=
        digital_input_status_->signals()->size()) {
      return FailedPreconditionError(RealtimeStatus::StrCat(
          "The received digital input size does not match the configuration "
          "(received: ",
          telemetry.digital_input.size(),
          ", reserved: ", digital_input_status_->signals()->size(), ")"));
    }
    for (uint32_t i = 0; i < digital_input_status_->mutable_signals()->size();
         i++) {
      digital_input_status_->mutable_signals()
          ->GetMutableObject(i)
          ->mutate_value(telemetry.digital_input.at(i));
    }

    if (telemetry.digital_output.size() !=
        digital_output_status_->signals()->size()) {
      return FailedPreconditionError(RealtimeStatus::StrCat(
          "The received digital output size does not match the configuration "
          "(received: ",
          telemetry.digital_output.size(),
          ", reserved: ", digital_output_status_->signals()->size(), ")"));
    }
    for (uint32_t i = 0; i < digital_output_status_->mutable_signals()->size();
         ++i) {
      digital_output_status_->mutable_signals()
          ->GetMutableObject(i)
          ->mutate_value(telemetry.digital_output.at(i));
    }
  }
  // Telemetry has been published, but there could still be a fault. It is
  // important to not return early on faults, so that e.g. during
  // jogging with the teach pendant the robot pose and IO states get updated.
  return client_->GetFault();
}

RealtimeStatus KukaRsiHwModule::ApplyCommand() {
  if (!client_->IsActive()) {
    return UnavailableError("RSI is not active");
  }
  // Do not command a position if the command was not updated this cycle.
  INTRINSIC_RT_ASSIGN_OR_RETURN(const auto joint_position_command,
                                joint_position_command_.Value());
  for (int dof = 0; dof < kKukaNumJoints; ++dof) {
    last_position_command_[dof] = joint_position_command->position()->Get(dof);
  }
  FixedVector<bool, kuka::kKukaMaxDigitalSignals> digital_outputs;
  for (uint32_t i = 0; i < digital_output_command_->signals()->size(); ++i) {
    digital_outputs.push_back(
        digital_output_command_->signals()->Get(i)->value());
  }
  return client_->SendCommand(last_position_command_, digital_outputs);
}

void KukaRsiHwModule::ComputeAndUpdateJointVelocities(
    const eigenmath::Vectord<kKukaNumJoints>& positions,
    uint64_t timestamp_ms) {
  // Compute velocity from position delta and filter the velocity.
  if (previous_cycle_timestamp_ms_.has_value() &&
      previous_joint_position_state_.has_value()) {
    const double time_delta =
        (timestamp_ms - *previous_cycle_timestamp_ms_) * 1e-3;
    // Only compute when there is new data.
    if (time_delta > 0) {
      const eigenmath::Vectord<kKukaNumJoints> position_delta =
          positions - *previous_joint_position_state_;
      const eigenmath::Vectord<kKukaNumJoints> velocities =
          position_delta / time_delta;
      butterworth_joint_velocity_filter_.Update(velocities);
      filtered_joint_velocities_ =
          butterworth_joint_velocity_filter_.GetOutput();
      // Always compute velocity based on last valid value and timestamp.
      previous_joint_position_state_ = positions;
      previous_cycle_timestamp_ms_ = timestamp_ms;
    } else if (client_->IsEnabled()) {
      INTRINSIC_RT_LOG_THROTTLED(WARNING)
          << "Timestamp duplicate : " << timestamp_ms
          << " ms. Cannot derive new velocity in this "
             "frame. Will reuse previous velocity.";
    }
  } else {
    previous_joint_position_state_ = positions;
    previous_cycle_timestamp_ms_ = timestamp_ms;
  }
  for (int dof = 0; dof < kKukaNumJoints; dof++) {
    joint_velocity_state_->mutable_velocity()->Mutate(
        dof, filtered_joint_velocities_[dof]);
  }
}

absl::Status KukaRsiHwModule::ProvideInspectionData(
    intrinsic_proto::icon::v1::HardwareModuleInspectionData& data) {
  return client_->ProvideInspectionData(data);
}

}  // namespace intrinsic::icon

REGISTER_HARDWARE_MODULE(intrinsic::icon::KukaRsiHwModule);
