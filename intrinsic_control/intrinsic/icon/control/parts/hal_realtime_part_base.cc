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

#include "intrinsic/icon/control/parts/hal_realtime_part_base.h"

#include <optional>
#include <string>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/cc_client/operational_status.h"
#include "intrinsic/icon/control/parts/feature_interface_registry.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/hal/hardware_module_manager.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {
FeatureInterfaceRegistry& HalRealtimePartBase::GetFeatureInterfaces() {
  return interface_registry_;
}

const FeatureInterfaceRegistry& HalRealtimePartBase::GetFeatureInterfaces()
    const {
  return interface_registry_;
}

absl::Status HalRealtimePartBase::AddHardwareModule(
    absl::string_view hardware_module_name) {
  hardware_module_names_.insert(std::string(hardware_module_name));
  // Check that no part is created that uses both operational and cell control
  // hardware modules, to ensure that all enabled hardware modules receive a
  // command every cycle.
  std::optional<std::string> operational_hardware_module_name;
  std::optional<std::string> cell_control_hardware_module_name;
  for (const std::string& hardware_module_name : hardware_module_names_) {
    INTR_ASSIGN_OR_RETURN(auto config,
                          hardware_module_manager_->GetHardwareModuleConfig(
                              hardware_module_name));
    if (config.cell_control_hardware_keep_enabled) {
      cell_control_hardware_module_name = hardware_module_name;
    } else {
      operational_hardware_module_name = hardware_module_name;
    }
  }
  if (operational_hardware_module_name.has_value() &&
      cell_control_hardware_module_name.has_value()) {
    return absl::FailedPreconditionError(absl::StrCat(
        "A part cannot use both operational and cell control hardware "
        "modules. Configured both operational module_name: ",
        *operational_hardware_module_name,
        " and cell control (by setting cell_control_hardware) module_name: ",
        *cell_control_hardware_module_name));
  }
  if (operational_hardware_module_name.has_value()) {
    depends_on_hardware_.operational_hardware = true;
  }
  if (cell_control_hardware_module_name.has_value()) {
    depends_on_hardware_.cell_control_hardware = true;
  }
  return absl::OkStatus();
}

RealtimeStatus HalRealtimePartBase::ReadStatus(ReadStatusParameters params) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(auto new_status, GetOperationalStatus());
  OperationalState new_state = ToOperationalState(new_status.state);

  // On first iteration in enabled state, reset feature interfaces.
  if (new_state == OperationalState::kEnabled &&
      previous_operational_state_ != OperationalState::kEnabled) {
    for (auto& interface : feature_interfaces_) {
      INTRINSIC_RT_RETURN_IF_ERROR(interface->Reset());
    }
  }
  previous_operational_state_ = new_state;

  for (auto& interface : feature_interfaces_) {
    INTRINSIC_RT_RETURN_IF_ERROR(interface->ReadStatus(params));
  }
  // HardwareModuleManager
  // (intrinsic/icon/hal/hardware_module_manager.h) takes care of
  // calling ReadStatus() on all HWMs, so we don't have to do that here.
  return OkStatus();
}

RealtimeStatus HalRealtimePartBase::ApplyCommand(
    ApplyCommandParameters params) {
  for (auto& interface : feature_interfaces_) {
    INTRINSIC_RT_RETURN_IF_ERROR(interface->ApplyCommand(params));
  }
  // HardwareModuleManager
  // (intrinsic/icon/hal/hardware_module_manager.h) takes care of
  // calling ApplyCommand() on all HWMs, so we don't have to do that here.
  return OkStatus();
}

RealtimeStatusOr<RealtimeOperationalStatus>
HalRealtimePartBase::GetOperationalStatus() const {
  RealtimeOperationalStatus relevant_status = {
      .state = RealtimeOperationalState::kEnabled};
  for (const auto& hardware_module_name : hardware_module_names_) {
    auto state =
        hardware_module_manager_->GetHardwareModuleState(hardware_module_name);
    if (!state.has_value()) {
      return NotFoundError(RealtimeStatus::StrCat(
          "Could not find hardware module named '", hardware_module_name, "'"));
    }
    RealtimeOperationalStatus hwm_status = ToRealtimeOperationalStatus(*state);
    constexpr auto relevance = [](const RealtimeOperationalStatus& status) {
      switch (status.state) {
        case RealtimeOperationalState::kEnabled:
          return 0;
        case RealtimeOperationalState::kDisabled:
          return 1;
        case RealtimeOperationalState::kFaultedConnected:
          return 2;
        case RealtimeOperationalState::kFatallyFaulted:
          return 3;
      }
    };
    if (relevance(hwm_status) > relevance(relevant_status)) {
      relevant_status = hwm_status;
    }
  }
  return relevant_status;
}

}  // namespace intrinsic::icon
