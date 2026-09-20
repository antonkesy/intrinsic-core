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


#include "intrinsic/icon/hal/hardware_module_manager.h"

#include <cstddef>
#include <list>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/container/flat_hash_map.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/icon/control/realtime_operational_status.h"
#include "intrinsic/icon/control/realtime_state_manager_interface.h"
#include "intrinsic/icon/hal/hardware_module_proxy.h"
#include "intrinsic/icon/hal/hardware_module_util.h"
#include "intrinsic/icon/hal/interfaces/hardware_module_state.fbs.h"
#include "intrinsic/icon/hal/interfaces/hardware_module_state_utils.h"
#include "intrinsic/icon/interprocess/remote_trigger/remote_trigger_client.h"
#include "intrinsic/icon/server/config/icon_main_config.pb.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/duration.h"
#include "intrinsic/icon/utils/fixed_str_cat.h"
#include "intrinsic/icon/utils/fixed_string.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/util/thread/rt_trace.h"

namespace intrinsic::icon {
namespace {

RemoteTriggerClient::AsyncRequest ReadStatus(
    const HardwareModuleProxy& hwm_proxy,
    FixedString<RealtimeStatus::kMaxMessageLength>& message) {
  auto read_return_status = hwm_proxy.ReadStatusAsync();
  if (!read_return_status.ok()) {
    if (!message.empty()) {
      message.append("; ");
    }
    message.append(icon::RealtimeStatus::StrCat(
        "'", hwm_proxy.Name(), "': ", read_return_status.status().message()));
    return {};
  }
  return std::move(*read_return_status);
}

bool IsInGroup(const HardwareModuleManager::HardwareModuleConfig& config,
               RealtimeStateManagerInterface::Group group) {
  switch (group) {
    case RealtimeStateManagerInterface::Group::kAllHardware:
      return true;
    case RealtimeStateManagerInterface::Group::kOperationalHardware:
      return !config.cell_control_hardware_keep_enabled;
    case RealtimeStateManagerInterface::Group::kCellControlHardware:
      return config.cell_control_hardware_keep_enabled;
  }
}

}  // namespace

RealtimeOperationalStatus ToRealtimeOperationalStatus(
    const intrinsic_fbs::HardwareModuleState& state) {
  RealtimeOperationalStatus result{
      .state = ToRealtimeOperationalState(state.code())};
  if (result.state == RealtimeOperationalState::kFatallyFaulted ||
      result.state == RealtimeOperationalState::kFaultedConnected) {
    result.fault_reason = RealtimeOperationalStatus::FaultReasonString(
        intrinsic_fbs::GetMessage(&state));
  }
  return result;
}

absl::Status HardwareModuleManager::Add(
    HardwareModuleProxy&& proxy, const bool proxy_drives_clock,
    const intrinsic_proto::icon::HardwareConfig& hardware_config) {
  if (proxy_drives_clock && hwm_proxy_that_drives_clock_ != nullptr) {
    return absl::FailedPreconditionError(absl::StrCat(
        "Tried to add proxy `", proxy.Name(),
        "` with option to drive the clock but proxy `",
        hwm_proxy_that_drives_clock_->Name(), "` is already driving clock."));
  }
  auto proxy_ptr = std::make_unique<HardwareModuleProxy>(std::move(proxy));
  HardwareModuleProxyData proxy_data{
      .proxy = proxy_ptr.get(),
      .async_request = RemoteTriggerClient::AsyncRequest(),
      .rt_async_request = RemoteTriggerClient::AsyncRequest(),
      .config = HardwareModuleConfig{
          .cell_control_hardware_keep_enabled =
              hardware_config.has_cell_control_hardware()}};
  hwm_proxy_data_.emplace_back(std::move(proxy_data));

  if (proxy_drives_clock) {
    hwm_proxy_that_drives_clock_ = std::move(proxy_ptr);
  } else {
    hwm_proxies_that_follow_clock_.emplace_back(std::move(proxy_ptr));
  }
  return absl::OkStatus();
}

RealtimeStatus HardwareModuleManager::Read(absl::Time deadline) {
  INTRINSIC_TRACE_SCOPED("HardwareModuleManager::Read");
  FixedString<RealtimeStatus::kMaxMessageLength> error_message;
  // For the simulation, we need to call the hardware module that drives the
  // clock first, see b/292538881.
  if (hwm_proxy_that_drives_clock_) {
    auto request = ReadStatus(*hwm_proxy_that_drives_clock_, error_message);
    if (request.Valid()) {
      if (const auto status = request.WaitUntil(deadline); !status.ok()) {
        if (!error_message.empty()) {
          error_message.append("; ");
        }
        error_message.append(icon::RealtimeStatus::StrCat(
            "'", hwm_proxy_that_drives_clock_->Name(),
            "': ", status.message()));
      }
    }
  }

  for (auto& hwm_proxy_entry : hwm_proxy_data_) {
    // For the simulation, we called the hardware module that drives the
    // clock first, see b/292538881. So we skip it here.
    if (hwm_proxy_that_drives_clock_.get() == hwm_proxy_entry.proxy) {
      continue;
    }
    const HardwareModuleProxy* hwm_proxy = hwm_proxy_entry.proxy;
    if (hwm_proxy == nullptr) {
      return FailedPreconditionError(
          RealtimeStatus::StrCat("Proxy for a hardware module is null."));
    }
    auto request = ReadStatus(*hwm_proxy, error_message);
    if (request.Valid()) {
      hwm_proxy_entry.rt_async_request = std::move(request);
    } else {
      hwm_proxy_entry.rt_async_request = {};
    }
  }
  for (auto& hwm_proxy_entry : hwm_proxy_data_) {
    if (hwm_proxy_that_drives_clock_.get() == hwm_proxy_entry.proxy) {
      continue;
    }
    const HardwareModuleProxy* ptr = hwm_proxy_entry.proxy;
    auto& request = hwm_proxy_entry.rt_async_request;
    if (!request.Valid()) {
      continue;
    }
    if (const auto status = request.WaitUntil(deadline); !status.ok()) {
      if (!error_message.empty()) {
        error_message.append("; ");
      }
      error_message.append(icon::RealtimeStatus::StrCat(
          "'", ptr->Name(), "': ", status.message()));
    }
    request = {};
  }
  if (!error_message.empty()) {
    return AbortedError(
        RealtimeStatus::StrCat("ReadStatus failed: ", error_message));
  }
  return icon::OkStatus();
}

namespace {
RemoteTriggerClient::AsyncRequest ApplyCommand(
    HardwareModuleProxy* hwm_proxy, intrinsic_fbs::StateCode state,
    FixedString<RealtimeStatus::kMaxMessageLength>& message) {
  if (state != intrinsic_fbs::StateCode::kMotionEnabled) {
    return {};
  }
  auto write_return_status = hwm_proxy->ApplyCommandAsync();
  if (!write_return_status.ok()) {
    if (!message.empty()) {
      message.append("; ");
    }
    message.append(icon::RealtimeStatus::StrCat(
        "'", hwm_proxy->Name(),
        "' failed: ", write_return_status.status().message()));
    return {};
  }
  return std::move(*write_return_status);
}
}  // namespace

RealtimeStatus HardwareModuleManager::Write(absl::Time deadline) {
  INTRINSIC_TRACE_SCOPED("HardwareModuleManager::Write");

  FixedString<RealtimeStatus::kMaxMessageLength> error_message;
  intrinsic_fbs::StateCode clock_driver_state =
      intrinsic_fbs::StateCode::kInitFailed;
  for (auto& hwm_proxy_entry : hwm_proxy_data_) {
    if (hwm_proxy_that_drives_clock_.get() == hwm_proxy_entry.proxy) {
      // For the simulation, we need to call the hardware module that drives the
      // clock last, see b/292538881. So we skip it here.
      clock_driver_state = hwm_proxy_entry.cached_state.code();
      continue;
    }
    auto request =
        ApplyCommand(hwm_proxy_entry.proxy, hwm_proxy_entry.cached_state.code(),
                     error_message);
    if (request.Valid()) {
      hwm_proxy_entry.rt_async_request = std::move(request);
    } else {
      hwm_proxy_entry.rt_async_request = {};
    }
  }
  for (auto& hwm_proxy_entry : hwm_proxy_data_) {
    const HardwareModuleProxy* ptr = hwm_proxy_entry.proxy;
    auto& request = hwm_proxy_entry.rt_async_request;
    if (!request.Valid()) {
      continue;
    }
    if (const auto status = request.WaitUntil(deadline); !status.ok()) {
      if (!error_message.empty()) {
        error_message.append("; ");
      }
      error_message.append(icon::RealtimeStatus::StrCat(
          "'", ptr->Name(), "': ", status.message()));
    }
    request = {};
  }
  // For the simulation, we need to call the hardware module that drives the
  // clock last, see b/292538881.
  if (hwm_proxy_that_drives_clock_) {
    auto request = ApplyCommand(hwm_proxy_that_drives_clock_.get(),
                                clock_driver_state, error_message);
    if (request.Valid()) {
      if (const auto status = request.WaitUntil(deadline); !status.ok()) {
        if (!error_message.empty()) {
          error_message.append("; ");
        }
        error_message.append(icon::RealtimeStatus::StrCat(
            "'", hwm_proxy_that_drives_clock_->Name(),
            "': ", status.message()));
      }
    }
  }
  if (!error_message.empty()) {
    return AbortedError(
        RealtimeStatus::StrCat("ApplyCommand failed: ", error_message));
  }
  return icon::OkStatus();
}

RealtimeStatusOr<std::optional<HardwareModuleManager::CommandId>>
HardwareModuleManager::RequestTransition(
    RealtimeStateManagerInterface::TransitionRequest request) {
  if (pending_transition_request_.has_value()) {
    if (pending_transition_request_.value() == request) {
      return icon::OkStatus();
    }
    return ResourceExhaustedError(RealtimeStatus::StrCat(
        "A different transition request (",
        ToString(pending_transition_request_.value()), ") with id ",
        current_command_id_.value(), " is already pending."));
  }

  // Ensure there is at least one HWM without active request.
  if (auto it =
          absl::c_find_if(hwm_proxy_data_,
                          [](const auto& proxy_and_request) {
                            return !proxy_and_request.async_request.Valid();
                          });
      !hwm_proxy_data_.empty() && it == hwm_proxy_data_.cend()) {
    return ResourceExhaustedError(RealtimeStatus::StrCat(
        "All hardware modules have an ongoing transition request."));
  }

  // Check that the transition is allowed.
  INTRINSIC_RT_ASSIGN_OR_RETURN(auto guard_result,
                                CheckTransitionGuard(request));
  if (guard_result == TransitionGuardResult::kNoOp) {
    return std::optional<HardwareModuleManager::CommandId>();
  }

  INTRINSIC_RT_LOG(INFO) << "Requesting transition to " << ToString(request);
  pending_transition_request_ = request;
  return std::optional<HardwareModuleManager::CommandId>(++current_command_id_);
}

std::optional<RealtimeStateManagerInterface::TransitionRequest>
HardwareModuleManager::PendingTransitionRequest(CommandId command_id) const {
  if (command_id != current_command_id_) {
    return std::nullopt;
  }
  return pending_transition_request_;
}

RealtimeStateManagerInterface::RequestState
HardwareModuleManager::GetRequestState() const {
  if (pending_transition_request_.has_value()) {
    return RequestState::kActive;
  }
  return cached_request_state_;
}

RealtimeStatusOr<HardwareModuleManager::RequestState>
HardwareModuleManager::GetRequestState(CommandId command_id) const {
  if (command_id != current_command_id_) {
    return FailedPreconditionError(RealtimeStatus::StrCat(
        "The current hardware module CommandId is not ", command_id.value(),
        ". It is ", current_command_id_.value()));
  }
  return GetRequestState();
}

RealtimeStatus HardwareModuleManager::ProcessTransitionRequestAsync() {
  if (pause_transitions_) {
    return icon::OkStatus();
  }

  // Check if any ongoing request is done or failed.
  for (HardwareModuleProxyData& data : hwm_proxy_data_) {
    if (data.async_request.Valid() && data.async_request.Ready()) {
      INTRINSIC_RT_LOG(INFO)
          << "Request done for: " << data.proxy->Name() << " new state: "
          << intrinsic_fbs::EnumNameStateCode(
                 data.proxy->GetHardwareModuleState()->code())
          << " Duration: "
          << ToInt64Milliseconds(intrinsic::Clock::Now() -
                                 data.async_request_start_time)
          << " ms";

      // 'WaitUntil' automatically resets the async request.
      INTRINSIC_RT_RETURN_IF_ERROR(data.async_request.WaitUntil());
    } else if (data.async_request.Valid() &&
               (intrinsic::Clock::Now() - data.async_request_start_time) >
                   request_timeout_) {
      INTRINSIC_RT_LOG(INFO) << "Request timed out for: " << data.proxy->Name();
      return DeadlineExceededError(RealtimeStatus::StrCat(
          "Request timed out for: ", data.proxy->Name()));
    }
  }

  if (!pending_transition_request_.has_value()) {
    // There is no pending transition request to process.
    return icon::OkStatus();
  }

  for (auto& [module, request, rt_request, start_time, module_state, config] :
       hwm_proxy_data_) {
    // Don't modify ongoing requests.
    if (request.Valid()) {
      continue;
    }
    INTRINSIC_RT_LOG(INFO) << "Executing request "
                           << ToString(pending_transition_request_.value())
                           << " for HWM: " << module->Name()
                           << " current state: "
                           << intrinsic_fbs::EnumNameStateCode(
                                  module_state.code());
    switch (pending_transition_request_.value()) {
      case RealtimeStateManagerInterface::TransitionRequest::kEnableMotion: {
        INTRINSIC_RT_ASSIGN_OR_RETURN(request, module->EnableMotionAsync());
        start_time = intrinsic::Clock::Now();
        break;
      }
      case RealtimeStateManagerInterface::TransitionRequest::kDisableMotion: {
        INTRINSIC_RT_ASSIGN_OR_RETURN(request, module->DisableMotionAsync());
        start_time = intrinsic::Clock::Now();
        break;
      }
      case RealtimeStateManagerInterface::TransitionRequest::
          kDisableMotionSkipCellControlHardware: {
        if (config.cell_control_hardware_keep_enabled) continue;
        INTRINSIC_RT_ASSIGN_OR_RETURN(request, module->DisableMotionAsync());
        start_time = intrinsic::Clock::Now();
        break;
      }
      case RealtimeStateManagerInterface::TransitionRequest::kClearFaults: {
        INTRINSIC_RT_ASSIGN_OR_RETURN(request, module->ClearFaultsAsync());
        start_time = intrinsic::Clock::Now();
        break;
      }

      default:
        break;
    }
  }
  pending_transition_request_.reset();

  return icon::OkStatus();
}

absl::Status HardwareModuleManager::Prepare(absl::Time deadline) {
  return BlockingMultiHardwareModuleCall(&HardwareModuleProxy::PrepareAsync,
                                         "Prepare", deadline,
                                         intrinsic_fbs::StateCode::kPreparing,
                                         intrinsic_fbs::StateCode::kPrepared);
}

RealtimeStatus HardwareModuleManager::Activate(absl::Time deadline) {
  return BlockingMultiHardwareModuleCallRT(
      &HardwareModuleProxy::ActivateAsync, "Activate", deadline,
      {intrinsic_fbs::StateCode::kActivated});
}

RealtimeStatus HardwareModuleManager::Deactivate(absl::Time deadline) {
  return BlockingMultiHardwareModuleCallRT(
      &HardwareModuleProxy::DeactivateAsync, "Deactivate", deadline,
      {intrinsic_fbs::StateCode::kDeactivated});
}

RealtimeStatus HardwareModuleManager::TryDeactivate(absl::Time deadline) {
  return BlockingMultiHardwareModuleCallRT(
      &HardwareModuleProxy::DeactivateAsync, "TryDeactivate", deadline,
      {intrinsic_fbs::StateCode::kDeactivated,
       intrinsic_fbs::StateCode::kFatallyFaulted,
       intrinsic_fbs::StateCode::kInitFailed});
}

absl::Status HardwareModuleManager::EnableMotionTestOnly(absl::Time deadline) {
  return BlockingMultiHardwareModuleCall(
      &HardwareModuleProxy::EnableMotionAsync, "EnableMotion", deadline,
      intrinsic_fbs::StateCode::kMotionEnabling,
      intrinsic_fbs::StateCode::kMotionEnabled);
}

absl::Status HardwareModuleManager::DisableMotionTestOnly(absl::Time deadline) {
  return BlockingMultiHardwareModuleCall(
      &HardwareModuleProxy::DisableMotionAsync, "DisableMotion", deadline,
      intrinsic_fbs::StateCode::kMotionDisabling,
      intrinsic_fbs::StateCode::kActivated);
}

absl::Status HardwareModuleManager::ClearFaultsTestOnly(absl::Time deadline) {
  return BlockingMultiHardwareModuleCall(
      &HardwareModuleProxy::ClearFaultsAsync, "ClearFaults", deadline,
      intrinsic_fbs::StateCode::kClearingFaults, {});
}

const HardwareModuleProxy* HardwareModuleManager::GetHardwareModuleProxy(
    const std::string_view module_name) const {
  if (auto it = absl::c_find_if(
          hwm_proxy_data_,
          [module_name](const HardwareModuleProxyData& proxy_and_request) {
            return proxy_and_request.proxy->Name() == module_name;
          });
      it != hwm_proxy_data_.end()) {
    return it->proxy;
  }
  return nullptr;
}

std::optional<intrinsic_fbs::HardwareModuleState>
HardwareModuleManager::GetHardwareModuleState(
    std::string_view module_name) const {
  if (auto it = absl::c_find_if(
          hwm_proxy_data_,
          [module_name](const HardwareModuleProxyData& proxy_and_request) {
            return proxy_and_request.proxy != nullptr &&
                   proxy_and_request.proxy->Name() == module_name;
          });
      it != hwm_proxy_data_.end()) {
    return it->cached_state;
  }
  return {};
}

absl::StatusOr<HardwareModuleManager::HardwareModuleConfig>
HardwareModuleManager::GetHardwareModuleConfig(
    std::string_view module_name) const {
  auto it = absl::c_find_if(
      hwm_proxy_data_,
      [module_name](const HardwareModuleProxyData& proxy_and_request) {
        return proxy_and_request.proxy != nullptr &&
               proxy_and_request.proxy->Name() == module_name;
      });
  if (it == hwm_proxy_data_.end()) {
    return absl::NotFoundError(
        absl::StrCat("Hardware module ", module_name, " not found."));
  }
  return it->config;
}

absl::StatusOr<absl::flat_hash_map<std::string, std::vector<std::string>>>
HardwareModuleManager::GetUnusedRequiredInterfaceNames() const {
  absl::flat_hash_map<std::string, std::vector<std::string>>
      unused_interface_names;
  for (const auto& proxy_data : hwm_proxy_data_) {
    INTR_ASSIGN_OR_RETURN(const auto missing,
                          proxy_data.proxy->GetUnusedRequiredInterfaceNames());
    if (!missing.empty()) {
      unused_interface_names[proxy_data.proxy->Name()] = missing;
    }
  }
  return unused_interface_names;
}

bool HardwareModuleManager::CheckAnyInOneOfStates(
    absl::Span<const intrinsic_fbs::StateCode> states,
    RealtimeStateManagerInterface::Group group) const {
  return absl::c_any_of(hwm_proxy_data_, [&](const auto& data) {
    if (!data.proxy) {
      return false;
    }
    if (!IsInGroup(data.config, group)) {
      return false;
    }
    return absl::c_any_of(states, [&](const auto& state) {
      return data.cached_state.code() == state;
    });
  });
}

bool HardwareModuleManager::CheckAllInOneOfStates(
    absl::Span<const intrinsic_fbs::StateCode> states,
    RealtimeStateManagerInterface::Group group) const {
  return absl::c_all_of(hwm_proxy_data_, [&](const auto& data) {
    if (!data.proxy) {
      return true;
    }
    if (!IsInGroup(data.config, group)) {
      return true;
    }
    return absl::c_any_of(states, [&](const auto& state) {
      return data.cached_state.code() == state;
    });
  });
}

RealtimeOperationalState HardwareModuleManager::GetCombinedOperationalState()
    const {
  RealtimeOperationalState combined_state = RealtimeOperationalState::kEnabled;
  for (const auto& data : hwm_proxy_data_) {
    RealtimeOperationalState state =
        ToRealtimeOperationalState(data.cached_state.code());
    combined_state = GetMaximumSeverity(combined_state, state);
  }
  return combined_state;
}

RealtimeOperationalState HardwareModuleManager::GetCellControlState() const {
  RealtimeOperationalState combined_state = RealtimeOperationalState::kEnabled;
  for (const auto& data : hwm_proxy_data_) {
    if (!data.config.cell_control_hardware_keep_enabled) continue;
    RealtimeOperationalState state =
        ToRealtimeOperationalState(data.cached_state.code());
    combined_state = GetMaximumSeverity(combined_state, state);
  }
  return combined_state;
}

std::optional<HardwareModuleManager::FaultReasonString>
HardwareModuleManager::GetFaultReason() const {
  return combined_fault_reason_;
}

FixedString<RealtimeStateManagerInterface::kStateSummaryMaxLength>
HardwareModuleManager::StateSummary() const {
  FixedString<kStateSummaryMaxLength> result;
  result.append(
      RealtimeStatus::StrCat("HWMs (", hwm_proxy_data_.size(), "):\n"));
  for (const auto& [module, request, rt_request, start_time, state,
                    keep_enabled] : hwm_proxy_data_) {
    if (!module) {
      result.append("hwm nullptr\n");
      continue;
    }

    FixedString<RealtimeStatus::kMaxMessageLength> ready_str;
    if (request.Valid()) {
      ready_str = RealtimeStatus::StrCat(
          ", ready: ", request.Ready() ? "true" : "false", ", duration: ",
          intrinsic::ToInt64Milliseconds(intrinsic::Clock::Now() - start_time),
          "ms");
    }

    result.append(RealtimeStatus::StrCat(
        module->Name(), " ", intrinsic_fbs::EnumNameStateCode(state.code()),
        ", request: ", request.Valid() ? "valid" : "invalid", ready_str, "\n"));
  }
  return result;
}

RealtimeStatus HardwareModuleManager::UpdateState() {
  // Query the request state first and then the HWM state since the HWM state
  // has been written before the request finishes. Reading it in the other order
  // would be a race condition.

  cached_request_state_ = [this] {
    size_t busy_requests = 0;
    for (const auto& data : hwm_proxy_data_) {
      if (data.async_request.Valid()) [[unlikely]] {
        busy_requests++;
      }
    }
    if (busy_requests == hwm_proxy_data_.size()) {
      return RequestState::kActive;
    } else if (busy_requests == 0) [[likely]] {
      return RequestState::kDone;
    } else {
      return RequestState::kPartiallyDone;
    }
  }();

  // Now update all Hardware Module states.
  for (auto& data : hwm_proxy_data_) {
    if (!data.proxy) {
      continue;
    }
    data.cached_state = *data.proxy->GetHardwareModuleState();
  }

  // Update the fault reason if necessary.
  {
    FaultReasonString tmp_fault_reason;
    bool found_faulted_hwm = false;
    for (const auto& data : hwm_proxy_data_) {
      if (!data.proxy) {
        continue;
      }
      const intrinsic_fbs::HardwareModuleState& state = data.cached_state;
      intrinsic_fbs::StateCode code = state.code();
      const bool faulted = code == intrinsic_fbs::StateCode::kFaulted ||
                           code == intrinsic_fbs::StateCode::kInitFailed ||
                           code == intrinsic_fbs::StateCode::kFatallyFaulted;
      // The fault reason is empty, when clearing faults, but we only want to
      // reset the fault reason, when the faults are cleared.
      found_faulted_hwm |=
          code == intrinsic_fbs::StateCode::kClearingFaults || faulted;
      if (faulted) [[unlikely]] {
        std::string_view message = intrinsic_fbs::GetMessage(&state);
        if (!message.empty()) {
          auto full_message = FixedStrCat<FaultReasonString::max_size()>(
              "'", data.proxy->Name(), "': ", message);
          if (!tmp_fault_reason.empty()) {
            tmp_fault_reason.append("; ");
          }
          tmp_fault_reason.append(full_message);
        }
      }
    }
    if (!found_faulted_hwm) [[likely]] {
      // Clear the fault reason as soon as no HWM is faulted anymore.
      combined_fault_reason_ = std::nullopt;
    } else if (!tmp_fault_reason.empty()) {
      // Only overwrite if there is a value to keep the old value until the
      // fault has been cleared.
      combined_fault_reason_ = tmp_fault_reason;
    }
  }

  return OkStatus();
}

RealtimeStatusOr<TransitionGuardResult>
HardwareModuleManager::CheckTransitionGuard(
    RealtimeStateManagerInterface::TransitionRequest request) {
  if (hwm_proxy_data_.empty()) {
    return TransitionGuardResult::kNoOp;
  }

  intrinsic_fbs::StateCode to;
  bool skip_cell_control_hardware = false;
  switch (request) {
    case RealtimeStateManagerInterface::TransitionRequest::kEnableMotion:
      to = intrinsic_fbs::StateCode::kMotionEnabling;
      break;
    case RealtimeStateManagerInterface::TransitionRequest::kDisableMotion:
      to = intrinsic_fbs::StateCode::kMotionDisabling;
      break;
    case RealtimeStateManagerInterface::TransitionRequest::
        kDisableMotionSkipCellControlHardware:
      to = intrinsic_fbs::StateCode::kMotionDisabling;
      skip_cell_control_hardware = true;
      break;
    case RealtimeStateManagerInterface::TransitionRequest::kClearFaults:
      to = intrinsic_fbs::StateCode::kClearingFaults;
      break;
    default:
      return InternalError(RealtimeStatus::StrCat(
          "Unexpected transition request: ", ToString(request)));
  }

  TransitionGuardResult merged_transition_guard_result =
      TransitionGuardResult::kNoOp;
  for (auto& data : hwm_proxy_data_) {
    if (skip_cell_control_hardware &&
        data.config.cell_control_hardware_keep_enabled) {
      continue;
    }
    intrinsic_fbs::StateCode from = data.cached_state.code();
    TransitionGuardResult result = HardwareModuleTransitionGuard(from, to);
    // If any HWM is in a state that allows the transition, this function will
    // accept the request. If no HWM allows the transition, this function will
    // reject the request.
    if (merged_transition_guard_result == TransitionGuardResult::kAllowed ||
        result == TransitionGuardResult::kAllowed) {
      merged_transition_guard_result = TransitionGuardResult::kAllowed;
    } else if (merged_transition_guard_result ==
                   TransitionGuardResult::kProhibited ||
               result == TransitionGuardResult::kProhibited) {
      merged_transition_guard_result = TransitionGuardResult::kProhibited;
    }
  }

  if (merged_transition_guard_result == TransitionGuardResult::kProhibited) {
    FixedString<RealtimeStatus::kMaxMessageLength> message;
    for (auto& data : hwm_proxy_data_) {
      if (!message.empty()) {
        message.append(", ");
      }
      message.append(
          intrinsic_fbs::EnumNameStateCode(data.cached_state.code()));
    }
    // Switching is not allowed in at least one of the current states.
    return FailedPreconditionError(
        RealtimeStatus::StrCat("Switching to ", EnumNameStateCode(to),
                               " is not allowed in states (", message, ")!"));
  } else if (merged_transition_guard_result == TransitionGuardResult::kNoOp) {
    return TransitionGuardResult::kNoOp;
  }
  return TransitionGuardResult::kAllowed;
}

absl::Status HardwareModuleManager::BlockingMultiHardwareModuleCall(
    AsyncFunc async_func, absl::string_view human_readable_function_name,
    absl::Time deadline, intrinsic_fbs::StateCode transient_state,
    std::optional<intrinsic_fbs::StateCode> final_state) {
  const absl::Time start_time = absl::Now();
  absl::flat_hash_map<const HardwareModuleProxy*, absl::Status> statuses;
  std::list<std::pair<RemoteTriggerClient::AsyncRequest, HardwareModuleProxy*>>
      async_requests;
  auto combine_statuses =
      [&](const absl::flat_hash_map<const HardwareModuleProxy*, absl::Status>&
              statuses) {
        std::string error_messages;
        for (const auto& [proxy, status] : statuses) {
          absl::StrAppend(&error_messages, status.message(), "\n");
        }
        const absl::StatusCode code = !statuses.empty()
                                          ? statuses.begin()->second.code()
                                          : absl::StatusCode::kUnknown;
        return absl::Status(code, error_messages);
      };
  for (auto& hw_module_proxy_and_request : hwm_proxy_data_) {
    // Clear up any finished requests.
    if (hw_module_proxy_and_request.async_request.Ready() &&
        hw_module_proxy_and_request.async_request.Valid()) {
      INTRINSIC_RT_RETURN_IF_ERROR(
          hw_module_proxy_and_request.async_request.WaitUntil());
    }
    auto request = ((*hw_module_proxy_and_request.proxy).*async_func)();
    if (request.ok()) {
      async_requests.push_back(std::make_pair(
          std::move(request.value()), hw_module_proxy_and_request.proxy));
    } else {
      auto status = absl::AbortedError(
          absl::StrCat(human_readable_function_name, " hardware module '",
                       hw_module_proxy_and_request.proxy->Name(),
                       "' async call failed: ", request.status().message()));
      LOG(ERROR) << status.message();
      statuses[hw_module_proxy_and_request.proxy] = std::move(status);
    }
  }
  if (!statuses.empty()) {
    return combine_statuses(statuses);
  }
  // Check
  // - if requests are done
  // - if HWM state has changed to transient state
  // - if deadline is exceeded
  while (absl::Now() < deadline) {
    if (async_requests.empty()) {
      break;
    }
    for (auto it = async_requests.begin(); it != async_requests.end();
         // not incrementing here, since finished requests are erased in the
         // loop and the iterator is incremented at the end of the loop.
    ) {
      auto& [request, proxy] = *it;

      const bool ready = request.Ready();
      if (ready) {
        // Clear up the request. Since it is ready, it will return immediately.
        // This WaitUntil is necessary to reset the binary futex.
        INTRINSIC_RT_RETURN_IF_ERROR(request.WaitUntil());
        it = async_requests.erase(it);
        continue;
      } else if (absl::Now() > deadline) {
        auto status = absl::AbortedError(
            absl::StrCat(human_readable_function_name, " on hardware module '",
                         proxy->Name(), "' failed to complete."));
        LOG(ERROR) << status.message();
        statuses[proxy] =
            OverwriteIfNotInError(statuses[proxy], (std::move(status)));
      } else {
        const auto state = proxy->GetHardwareModuleState()->code();
        // Timeout until the action should have been started on the hardware
        // module. Otherwise, the hardware module manager might not realize
        // that the hardware module is not responding until the full timeout
        // has passed.
        // TODO(b/327596638): Reduce this timeout to 1 second after ICON
        // reliably only starts when the HWM is ready.
        constexpr auto kActionStartTimeout = absl::Seconds(10);
        if (absl::Now() > start_time + kActionStartTimeout &&
            state != transient_state && state != final_state) {
          statuses[proxy] = OverwriteIfNotInError(
              statuses[proxy],
              absl::UnavailableError(absl::StrCat(
                  "Hardware module '", proxy->Name(),
                  "' did not change its state to ",
                  intrinsic_fbs::EnumNameStateCode(transient_state), " within ",
                  kActionStartTimeout, ". Actually in state: ",
                  intrinsic_fbs::EnumNameStateCode(state))));
          return combine_statuses(statuses);
        }
      }
      ++it;
    }
    absl::SleepFor(absl::Milliseconds(10));
  }
  if (!statuses.empty()) {
    return combine_statuses(statuses);
  }
  // If we reach this, then either
  // - we have exceeded the deadline
  // - all async requests have finished
  if (absl::Now() > deadline) {
    return DeadlineExceededError(absl::StrCat(
        "Deadline exceeded while waiting for hardware modules to finish ",
        human_readable_function_name, "."));
  }
  if (final_state.has_value()) {
    for (const auto& hw_module_proxy_and_request : hwm_proxy_data_) {
      const HardwareModuleProxy* proxy = hw_module_proxy_and_request.proxy;
      if (proxy->GetHardwareModuleState()->code() != final_state) {
        statuses[proxy] = absl::UnavailableError(absl::StrCat(
            "'", proxy->Name(), "' is in state ",
            intrinsic_fbs::EnumNameStateCode(
                proxy->GetHardwareModuleState()->code()),
            " instead of ", intrinsic_fbs::EnumNameStateCode(*final_state),
            ": ", GetMessage(proxy->GetHardwareModuleState())));
      }
    }
  }
  if (!statuses.empty()) {
    return combine_statuses(statuses);
  }
  return absl::OkStatus();
}

RealtimeStatus HardwareModuleManager::BlockingMultiHardwareModuleCallRT(
    AsyncFunc async_func, absl::string_view human_readable_function_name,
    absl::Time deadline,
    absl::Span<const intrinsic_fbs::StateCode> final_states) {
  FixedString<RealtimeStatus::kMaxMessageLength> joined_status_messages;
  std::optional<absl::StatusCode> first_error_code;
  // Convenience function to append a status to the status messages and only use
  // the first error code that happened.
  auto append_status = [&](RealtimeStatus status) {
    if (!first_error_code.has_value()) {
      first_error_code = status.code();
    }
    if (!joined_status_messages.empty()) {
      joined_status_messages.append(", ");
    }
    joined_status_messages.append(status.message());
  };
  for (auto& hw_module_proxy_and_request : hwm_proxy_data_) {
    // Clear up any finished requests.
    if (hw_module_proxy_and_request.async_request.Ready() &&
        hw_module_proxy_and_request.async_request.Valid()) {
      INTRINSIC_RT_RETURN_IF_ERROR(
          hw_module_proxy_and_request.async_request.WaitUntil(absl::Now()));
    }

    // We overwrite existing requests. This is not ideal design, but the number
    // of hardware modules is unbound and we cannot allocate here. However, this
    // function must not be called while the lockstep is active and in case
    // another request is still running, we do not really care anymore about its
    // result. Since in our uses cases this can only be Deactivate while
    // another, different call is running, we are willing to accept this
    // downside.
    RealtimeStatusOr<RemoteTriggerClient::AsyncRequest> request =
        ((*hw_module_proxy_and_request.proxy).*async_func)();
    if (request.ok()) {
      hw_module_proxy_and_request.async_request = std::move(*request);
    } else {
      hw_module_proxy_and_request.async_request = {};
      auto status = AbortedError(RealtimeStatus::StrCat(
          "Async call of ", human_readable_function_name,
          " on hardware module '", hw_module_proxy_and_request.proxy->Name(),
          "' failed: ", request.status().message()));
      INTRINSIC_RT_LOG(ERROR) << status.message();
      append_status(status);
    }
  }
  if (first_error_code.has_value()) {
    return RealtimeStatus(first_error_code.value(), joined_status_messages);
  }

  bool found_valid_request = false;
  for (auto& hw_module_proxy_and_request : hwm_proxy_data_) {
    // Only if we started the request successfully, we should check its status.
    if (hw_module_proxy_and_request.async_request.Valid()) {
      found_valid_request |= true;
      const auto request_status =
          hw_module_proxy_and_request.async_request.WaitUntil(deadline);
      if (!request_status.ok()) {
        auto status = RealtimeStatus(
            request_status.code(),
            RealtimeStatus::StrCat(human_readable_function_name,
                                   " on hardware module '",
                                   hw_module_proxy_and_request.proxy->Name(),
                                   "' failed: ", request_status.message()));
        INTRINSIC_RT_LOG(ERROR) << status.message();
        append_status(status);
      }
    }
  }
  if (!found_valid_request && !hwm_proxy_data_.empty()) {
    return InternalError(RealtimeStatus::StrCat(
        "No valid request found while executing ", human_readable_function_name,
        ". This is a bug!"));
  }
  if (first_error_code.has_value()) {
    return RealtimeStatus(first_error_code.value(), joined_status_messages);
  }

  if (!final_states.empty()) {
    FixedString<RealtimeStatus::kMaxMessageLength> joined_final_state_messages;
    for (const auto& final_state : final_states) {
      if (!joined_final_state_messages.empty()) {
        joined_final_state_messages.append("|");
      }
      joined_final_state_messages.append(
          intrinsic_fbs::EnumNameStateCode(final_state));
    }
    for (const auto& hw_module_proxy_and_request : hwm_proxy_data_) {
      const HardwareModuleProxy* proxy = hw_module_proxy_and_request.proxy;
      if (!absl::c_contains(final_states,
                            proxy->GetHardwareModuleState()->code())) {
        append_status(UnavailableError(RealtimeStatus::StrCat(
            "'", proxy->Name(), "' is in state ",
            intrinsic_fbs::EnumNameStateCode(
                proxy->GetHardwareModuleState()->code()),
            " instead of ", joined_final_state_messages, ": ",
            GetMessage(proxy->GetHardwareModuleState()))));
      }
    }
  }
  // Check if the deadline has been exceeded by now. Should only happen
  // - if our thread got suspended too long
  // - if `request.WaitUntil(deadline)` returned just before the deadline was
  //   exceeded.
  // - if there is a bug in `request.WaitUntil(deadline);`
  // We could just take the success result, but since we are in a realtime
  // context, it makes sense to check the deadline again and fail if we exceeded
  // it.
  if (absl::Now() > deadline) {
    return DeadlineExceededError(RealtimeStatus::StrCat(
        "Deadline exceeded while waiting for hardware modules to finish ",
        human_readable_function_name, "."));
  }
  if (first_error_code.has_value()) {
    return RealtimeStatus(first_error_code.value(), joined_status_messages);
  }
  return OkStatus();
}

}  // namespace intrinsic::icon
