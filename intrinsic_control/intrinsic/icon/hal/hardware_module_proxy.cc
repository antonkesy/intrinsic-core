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

#include "intrinsic/icon/hal/hardware_module_proxy.h"

#include <cerrno>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "intrinsic/icon/hal/control_period_register.h"
#include "intrinsic/icon/hal/get_hardware_interface.h"
#include "intrinsic/icon/hal/hardware_interface_traits.h"
#include "intrinsic/icon/hal/icon_state_register.h"
#include "intrinsic/icon/hal/interfaces/hardware_module_state.fbs.h"
#include "intrinsic/icon/hal/interfaces/hardware_module_state_utils.h"
#include "intrinsic/icon/hal/interfaces/icon_state.fbs.h"
#include "intrinsic/icon/hal/module_config.h"
#include "intrinsic/icon/interprocess/remote_trigger/remote_trigger_client.h"
#include "intrinsic/icon/interprocess/shared_memory_manager/domain_socket_utils.h"
#include "intrinsic/icon/interprocess/shared_memory_manager/memory_segment.h"
#include "intrinsic/icon/server/config/icon_main_config.pb.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/current_cycle.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/thread/rt_trace.h"

namespace intrinsic::icon {

namespace hardware_interface_traits {
INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::HardwareModuleState,
                                 intrinsic_fbs::BuildHardwareModuleState,
                                 "intrinsic_fbs.HardwareModuleState")
}  // namespace hardware_interface_traits

namespace {

// Allow 1us tolerance to account for rounding errors.
const size_t kMaxControlPeriodMismatchNs = 1000;

// Validates that the control period exposed by the `hardware_module_proxy`
// matches the `expected_control_period`. To support connecting to legacy
// hardware modules, it skips the check when the `control_period` interface is
// not exposed by the HWM.
// TODO(/b/515288874): Remove support for hardware modules not exposing the
// control_period interface after a grace period.
absl::Status ValidateControlPeriod(
    intrinsic::Duration expected_control_period,
    const HardwareModuleProxy& hardware_module_proxy) {
  // Should only be possible in tests, as ICON already enforces this.
  if (expected_control_period <= intrinsic::ZeroDuration()) {
    return absl::FailedPreconditionError(
        absl::StrCat("Invalid expected_control_period ",
                     intrinsic::ToInt64Nanoseconds(expected_control_period),
                     "ns provided. It must be positive."));
  }

  // Validate the control period if the interface is available.
  auto control_period_handle =
      hardware_module_proxy.GetHardwareInterface<intrinsic_fbs::ControlPeriod>(
          kControlPeriodInterfaceName);

  if (control_period_handle.ok()) {
    intrinsic::Duration reported_period =
        intrinsic::Nanoseconds((*control_period_handle)->control_period_ns());

    // Allow 1us tolerance to account for rounding errors.
    if (std::abs(intrinsic::ToInt64Nanoseconds(reported_period) -
                 intrinsic::ToInt64Nanoseconds(expected_control_period)) >
        kMaxControlPeriodMismatchNs) {
      return absl::FailedPreconditionError(
          intrinsic_fbs::FormatControlPeriodMismatchError(
              hardware_module_proxy.Name(), expected_control_period,
              reported_period));
    }
  } else {
    // TODO(/b/515288874): Remove support hardware modules not exposing the
    // control_period interface after a grace period.
    LOG(WARNING) << "Hardware Module '" << hardware_module_proxy.Name()
                 << "' does not export the '" << kControlPeriodInterfaceName
                 << "' interface. Skipping validation.";
  }
  return absl::OkStatus();
}

}  // namespace

HardwareModuleProxy::HardwareModuleProxy(HardwareModuleProxy&& other)
    : shared_memory_namespace_(std::move(other.shared_memory_namespace_)),
      module_name_(std::move(other.module_name_)),
      module_info_(std::move(other.module_info_)),
      interface_names_(std::move(other.interface_names_)),
      hardware_module_state_(std::move(other.hardware_module_state_)),
      icon_state_(std::move(other.icon_state_)),
      prepare_client_(std::move(other.prepare_client_)),
      activate_client_(std::move(other.activate_client_)),
      deactivate_client_(std::move(other.deactivate_client_)),
      enable_motion_client_(std::move(other.enable_motion_client_)),
      disable_motion_client_(std::move(other.disable_motion_client_)),
      clear_faults_client_(std::move(other.clear_faults_client_)),
      read_status_client_(std::move(other.read_status_client_)),
      apply_command_client_(std::move(other.apply_command_client_)),
      restart_client_(std::move(other.restart_client_)) {
  absl::MutexLock l(&other.used_interfaces_mutex_);
  absl::MutexLock l2(&used_interfaces_mutex_);
  used_interfaces_ = std::move(other.used_interfaces_);
  segment_name_to_file_descriptor_map_ =
      std::move(other.segment_name_to_file_descriptor_map_);
  other.segment_name_to_file_descriptor_map_.clear();
}

HardwareModuleProxy& HardwareModuleProxy::operator=(
    HardwareModuleProxy&& other) noexcept {
  shared_memory_namespace_ = std::move(other.shared_memory_namespace_);
  module_name_ = std::move(other.module_name_);
  module_info_ = std::move(other.module_info_);
  interface_names_ = std::move(other.interface_names_);
  hardware_module_state_ = std::move(other.hardware_module_state_);
  icon_state_ = std::move(other.icon_state_);
  prepare_client_ = std::move(other.prepare_client_);
  activate_client_ = std::move(other.activate_client_);
  deactivate_client_ = std::move(other.deactivate_client_);
  enable_motion_client_ = std::move(other.enable_motion_client_);
  disable_motion_client_ = std::move(other.disable_motion_client_);
  clear_faults_client_ = std::move(other.clear_faults_client_);
  read_status_client_ = std::move(other.read_status_client_);
  apply_command_client_ = std::move(other.apply_command_client_);
  restart_client_ = std::move(other.restart_client_);

  absl::MutexLock l(&other.used_interfaces_mutex_);
  absl::MutexLock l2(&used_interfaces_mutex_);
  used_interfaces_ = std::move(other.used_interfaces_);
  segment_name_to_file_descriptor_map_ =
      std::move(other.segment_name_to_file_descriptor_map_);
  other.segment_name_to_file_descriptor_map_.clear();
  return *this;
}

absl::StatusOr<HardwareModuleProxy> HardwareModuleProxy::Create(
    const ModuleConfig& module_config) {
  return Create(module_config.GetSharedMemoryNamespace(),
                module_config.GetName());
}

absl::StatusOr<HardwareModuleProxy> HardwareModuleProxy::Create(
    absl::string_view shared_memory_namespace, absl::string_view module_name) {
  if (module_name.empty()) {
    return absl::InvalidArgumentError(
        "No name specified in hardware module config.");
  }
  return HardwareModuleProxy(shared_memory_namespace, module_name);
}

absl::StatusOr<HardwareModuleProxy> HardwareModuleProxy::Attach(
    const ModuleConfig& module_config, absl::Duration connection_timeout) {
  return Attach(module_config.GetSharedMemoryNamespace(),
                module_config.GetName(), connection_timeout);
}

absl::StatusOr<HardwareModuleProxy> HardwareModuleProxy::Attach(
    absl::string_view shared_memory_namespace, absl::string_view module_name,
    absl::Duration connection_timeout) {
  INTR_ASSIGN_OR_RETURN(
      auto hardware_module_proxy,
      HardwareModuleProxy::Create(shared_memory_namespace, module_name));
  INTR_RETURN_IF_ERROR(hardware_module_proxy.Connect(connection_timeout))
      << "PUBLIC: Is the hardware module " << module_name << " running?";

  return hardware_module_proxy;
}

HardwareModuleProxy::HardwareModuleProxy(
    absl::string_view shared_memory_namespace, absl::string_view module_name)
    : shared_memory_namespace_(shared_memory_namespace),
      module_name_(module_name) {}

absl::string_view HardwareModuleProxy::Name() const { return module_name_; }

absl::Status HardwareModuleProxy::Connect(absl::Duration connection_timeout) {
  if (IsConnected()) {
    return absl::OkStatus();
  }

  // Connects to the domain socket of the hardware module and gets the
  // SegmentNameToFileDescriptorMap.
  INTR_ASSIGN_OR_RETURN(
      segment_name_to_file_descriptor_map_,
      ::intrinsic::icon::GetSegmentNameToFileDescriptorMap(
          SocketDirectoryFromNamespace(shared_memory_namespace_), module_name_,
          connection_timeout));

  INTR_ASSIGN_OR_RETURN(
      hardware_module_state_,
      GetInterfaceHandle<intrinsic_fbs::HardwareModuleState>(
          segment_name_to_file_descriptor_map_, "hardware_module_state"));

  // Allows ICON to share state information with the module.
  INTR_ASSIGN_OR_RETURN(
      icon_state_,
      GetMutableInterfaceHandle<intrinsic_fbs::IconState>(
          segment_name_to_file_descriptor_map_, kIconStateInterfaceName));

  INTR_ASSIGN_OR_RETURN(
      module_info_,
      GetHardwareModuleInfo(segment_name_to_file_descriptor_map_));
  INTR_ASSIGN_OR_RETURN(interface_names_,
                        GetInterfacesFromModuleInfo(module_info_.GetValue()));

  INTR_ASSIGN_OR_RETURN(auto prepare_client,
                        RemoteTriggerClient::Create(
                            segment_name_to_file_descriptor_map_, "prepare"));
  prepare_client_ =
      std::make_unique<RemoteTriggerClient>(std::move(prepare_client));

  INTR_ASSIGN_OR_RETURN(auto activate_client,
                        RemoteTriggerClient::Create(
                            segment_name_to_file_descriptor_map_, "activate"));
  activate_client_ =
      std::make_unique<RemoteTriggerClient>(std::move(activate_client));

  INTR_ASSIGN_OR_RETURN(
      auto deactivate_client,
      RemoteTriggerClient::Create(segment_name_to_file_descriptor_map_,
                                  "deactivate"));
  deactivate_client_ =
      std::make_unique<RemoteTriggerClient>(std::move(deactivate_client));

  INTR_ASSIGN_OR_RETURN(
      auto enable_motion_client,
      RemoteTriggerClient::Create(segment_name_to_file_descriptor_map_,
                                  "enable_motion"));
  enable_motion_client_ =
      std::make_unique<RemoteTriggerClient>(std::move(enable_motion_client));

  INTR_ASSIGN_OR_RETURN(
      auto disable_motion_client,
      RemoteTriggerClient::Create(segment_name_to_file_descriptor_map_,
                                  "disable_motion"));
  disable_motion_client_ =
      std::make_unique<RemoteTriggerClient>(std::move(disable_motion_client));
  INTR_ASSIGN_OR_RETURN(
      auto clear_faults_client,
      RemoteTriggerClient::Create(segment_name_to_file_descriptor_map_,
                                  "clear_faults"));
  clear_faults_client_ =
      std::make_unique<RemoteTriggerClient>(std::move(clear_faults_client));

  INTR_ASSIGN_OR_RETURN(
      auto read_status_client,
      RemoteTriggerClient::Create(segment_name_to_file_descriptor_map_,
                                  "read_status"));
  read_status_client_ =
      std::make_unique<RemoteTriggerClient>(std::move(read_status_client));

  INTR_ASSIGN_OR_RETURN(
      auto apply_command_client,
      RemoteTriggerClient::Create(segment_name_to_file_descriptor_map_,
                                  "apply_command"));
  apply_command_client_ =
      std::make_unique<RemoteTriggerClient>(std::move(apply_command_client));

  auto restart_client = RemoteTriggerClient::Create(
      segment_name_to_file_descriptor_map_, "restart");
  if (restart_client.ok()) {
    restart_client_ =
        std::make_unique<RemoteTriggerClient>(std::move(*restart_client));
  } else {
    LOG(WARNING) << "Restart client not available for hardware module '"
                 << module_name_ << "'- continuing anyways.";
  }

  return absl::OkStatus();
}

HardwareModuleProxy::~HardwareModuleProxy() {
  if (!segment_name_to_file_descriptor_map_.empty()) {
    LOG(INFO) << "Closing file descriptors for " << module_name_;
  }

  for (const auto& [segment_name, file_descriptor] :
       segment_name_to_file_descriptor_map_) {
    if (close(file_descriptor) == -1) {
      LOG(WARNING) << "Failed to close shm_fd for '" << segment_name << "'. "
                   << strerror(errno) << ". Continuing anyways.";
    }
  }
}

bool HardwareModuleProxy::IsConnected() const {
  return read_status_client_ && apply_command_client_ &&
         read_status_client_->IsConnected() &&
         apply_command_client_->IsConnected();
}

std::vector<std::string> HardwareModuleProxy::GetHardwareInterfaceNames()
    const {
  return interface_names_;
}

absl::StatusOr<std::vector<std::string>>
HardwareModuleProxy::GetRequiredInterfaceNames() const {
  return GetRequiredInterfacesFromModuleInfo(module_info_.GetValue());
}

absl::StatusOr<std::vector<std::string>>
HardwareModuleProxy::GetUnusedRequiredInterfaceNames() const {
  INTR_ASSIGN_OR_RETURN(
      const auto required_interfaces,
      GetRequiredInterfacesFromModuleInfo(module_info_.GetValue()));
  absl::flat_hash_set<std::string> unused_required_interfaces;
  unused_required_interfaces.reserve(required_interfaces.size());
  {
    absl::MutexLock l(&used_interfaces_mutex_);
    for (const auto& required_interface : required_interfaces) {
      if (!used_interfaces_.contains(required_interface)) {
        unused_required_interfaces.insert(required_interface);
      }
    }
  }

  return std::vector<std::string>{unused_required_interfaces.begin(),
                                  unused_required_interfaces.end()};
}

absl::Status HardwareModuleProxy::Prepare(absl::Time deadline) const {
  INTRINSIC_RT_RETURN_IF_ERROR(prepare_client_->Trigger(deadline));
  // It is OK to read the HardwareModuleState since the ICON rt loop is not
  // running yet.
  if (GetHardwareModuleState()->code() != intrinsic_fbs::StateCode::kPrepared) {
    return InternalError(RealtimeStatus::StrCat(
        "Failed to prepare. Resulting state: ",
        intrinsic_fbs::EnumNameStateCode(GetHardwareModuleState()->code()),
        ": ", intrinsic_fbs::GetMessage(GetHardwareModuleState())));
  }
  return OkStatus();
}

RealtimeStatus HardwareModuleProxy::Activate(absl::Time deadline) const {
  INTRINSIC_RT_RETURN_IF_ERROR(activate_client_->Trigger(deadline));
  // It is OK to read the HardwareModuleState since the ICON rt loop is not
  // running yet.
  if (GetHardwareModuleState()->code() !=
      intrinsic_fbs::StateCode::kActivated) {
    return InternalError(RealtimeStatus::StrCat(
        "Failed to activate. Resulting state: ",
        intrinsic_fbs::EnumNameStateCode(GetHardwareModuleState()->code()),
        ": ", intrinsic_fbs::GetMessage(GetHardwareModuleState())));
  }
  return OkStatus();
}

RealtimeStatus HardwareModuleProxy::Deactivate(absl::Time deadline) const {
  INTRINSIC_RT_RETURN_IF_ERROR(deactivate_client_->Trigger(deadline));
  // It is OK to read the HardwareModuleState since the ICON rt loop is not
  // running anymore.
  if (GetHardwareModuleState()->code() !=
      intrinsic_fbs::StateCode::kDeactivated) {
    return InternalError(RealtimeStatus::StrCat(
        "Failed to deactivate. Resulting state: ",
        intrinsic_fbs::EnumNameStateCode(GetHardwareModuleState()->code()),
        ": ", intrinsic_fbs::GetMessage(GetHardwareModuleState())));
  }
  return OkStatus();
}

absl::StatusOr<SegmentNameToFileDescriptorMap>
HardwareModuleProxy::GetSegmentNameToFileDescriptorMap() const {
  if (segment_name_to_file_descriptor_map_.empty()) {
    return absl::FailedPreconditionError(
        "SegmentNameToFileDescriptorMap is empty. Call Connect() first");
  }

  return segment_name_to_file_descriptor_map_;
}

RealtimeStatusOr<RemoteTriggerClient::AsyncRequest>
HardwareModuleProxy::PrepareAsync() const {
  return prepare_client_->TriggerAsync();
}

RealtimeStatusOr<RemoteTriggerClient::AsyncRequest>
HardwareModuleProxy::EnableMotionAsync() const {
  return enable_motion_client_->TriggerAsync();
}

RealtimeStatusOr<RemoteTriggerClient::AsyncRequest>
HardwareModuleProxy::DisableMotionAsync() const {
  return disable_motion_client_->TriggerAsync();
}

RealtimeStatusOr<RemoteTriggerClient::AsyncRequest>
HardwareModuleProxy::ClearFaultsAsync() const {
  return clear_faults_client_->TriggerAsync();
}

RealtimeStatusOr<RemoteTriggerClient::AsyncRequest>
HardwareModuleProxy::ActivateAsync() const {
  return activate_client_->TriggerAsync();
}
RealtimeStatusOr<RemoteTriggerClient::AsyncRequest>
HardwareModuleProxy::DeactivateAsync() const {
  return deactivate_client_->TriggerAsync();
}

RealtimeStatus HardwareModuleProxy::ReadStatus(absl::Time deadline) const {
  INTRINSIC_TRACE_VA_SCOPED("ReadStatus:", Name());
  return read_status_client_->Trigger(deadline);
}

RealtimeStatusOr<RemoteTriggerClient::AsyncRequest>
HardwareModuleProxy::ReadStatusAsync() const {
  return read_status_client_->TriggerAsync();
}

RealtimeStatus HardwareModuleProxy::ApplyCommand(absl::Time deadline) {
  INTRINSIC_TRACE_VA_SCOPED("ApplyCommand:", Name());
  // ICON state is used to validate that the interface was updated in the same
  // cycle IconState reports as the current cycle.
  // Explicitly updating the stored `current_cycle` and `UpdatedAt` so that the
  // internal state of IconState is consistent.
  icon_state_->mutate_current_cycle(Cycle::GetCurrentCycle());
  icon_state_.UpdatedAt(intrinsic::Clock::Now());
  return apply_command_client_->Trigger(deadline);
}

RealtimeStatusOr<RemoteTriggerClient::AsyncRequest>
HardwareModuleProxy::ApplyCommandAsync() {
  // ICON state is used to validate that the interface was updated in the same
  // cycle IconState reports as the current cycle.
  // Explicitly updating the stored `current_cycle` and `UpdatedAt` so that the
  // internal state of IconState is consistent.
  icon_state_->mutate_current_cycle(Cycle::GetCurrentCycle());
  icon_state_.UpdatedAt(intrinsic::Clock::Now());
  return apply_command_client_->TriggerAsync();
}

absl::Status HardwareModuleProxy::Restart(absl::Time deadline) const {
  if (GetHardwareModuleState()->code() ==
      intrinsic_fbs::StateCode::kMotionEnabled) {
    return absl::FailedPreconditionError(
        "Cannot restart a hardware module while it is enabled.");
  }
  if (!restart_client_) {
    return absl::FailedPreconditionError(absl::StrCat(
        "Restart client not available for hardware module '", Name(), "'."));
  }
  return restart_client_->Trigger(deadline);
}

RealtimeStatusOr<RemoteTriggerClient::AsyncRequest>
HardwareModuleProxy::RestartAsync() const {
  if (GetHardwareModuleState()->code() ==
      intrinsic_fbs::StateCode::kMotionEnabled) {
    return FailedPreconditionError(
        "Cannot restart a hardware module while it is enabled.");
  }
  if (!restart_client_) {
    return FailedPreconditionError(RealtimeStatus::StrCat(
        "Restart client not available for hardware module '", Name(), "'."));
  }
  return restart_client_->TriggerAsync();
}

const intrinsic_fbs::HardwareModuleState*
HardwareModuleProxy::GetHardwareModuleState() const {
  return *hardware_module_state_;
}

absl::StatusOr<HardwareModuleProxy> WaitForHardwareModule(
    absl::string_view shared_memory_namespace, absl::string_view module_name,
    absl::Duration connection_timeout,
    std::optional<intrinsic::Duration> expected_control_period) {
  const absl::Time deadline = absl::Now() + connection_timeout;

  INTR_ASSIGN_OR_RETURN(
      auto hw_module,
      HardwareModuleProxy::Create(shared_memory_namespace, module_name));
  auto connection_status = absl::InternalError("Unknown connection error.");
  LOG(INFO) << "Waiting for hardware module '" << module_name << "'.";
  // Try to connect once.
  do {
    connection_status = hw_module.Connect(connection_timeout);
    if (connection_status.ok()) {
      bool init_failed = hw_module.GetHardwareModuleState()->code() ==
                         intrinsic_fbs::StateCode::kInitFailed;

      if (expected_control_period.has_value() && !init_failed) {
        INTR_RETURN_IF_ERROR(
            ValidateControlPeriod(*expected_control_period, hw_module));
      }
      return hw_module;
    }
    // Return if the error is not retryable.
    if (connection_status.code() != absl::StatusCode::kDeadlineExceeded) {
      absl::Status connection_status_with_module_name(
          connection_status.code(),
          absl::StrCat("'", module_name,
                       "' failed to connect: ", connection_status.message()));
      LOG(ERROR) << connection_status_with_module_name;
      return connection_status_with_module_name;
    }
    LOG(INFO) << "Failed to connect to hardware module '" << module_name
              << "' with status: " << connection_status << ". Retrying until "
              << absl::FormatTime(deadline);
    absl::SleepFor(absl::Seconds(1));
  } while (absl::Now() <= deadline);
  absl::Status connection_status_with_module_name(
      connection_status.code(),
      absl::StrCat("'", module_name,
                   "' failed to connect: ", connection_status.message()));
  LOG(ERROR) << connection_status_with_module_name;
  return connection_status_with_module_name;
}

}  // namespace intrinsic::icon
