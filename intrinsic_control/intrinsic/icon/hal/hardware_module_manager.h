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

#ifndef INTRINSIC_ICON_HAL_HARDWARE_MODULE_MANAGER_H_
#define INTRINSIC_ICON_HAL_HARDWARE_MODULE_MANAGER_H_

#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/control/realtime_operational_status.h"
#include "intrinsic/icon/control/realtime_state_manager_interface.h"
#include "intrinsic/icon/hal/hardware_module_proxy.h"
#include "intrinsic/icon/hal/hardware_module_util.h"
#include "intrinsic/icon/hal/interfaces/hardware_module_state.fbs.h"
#include "intrinsic/icon/interprocess/remote_trigger/remote_trigger_client.h"
#include "intrinsic/icon/server/config/icon_main_config.pb.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/duration.h"
#include "intrinsic/icon/utils/fixed_string.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

// Manages a list of hardware module proxies.
class HardwareModuleManager final : public RealtimeStateManagerInterface {
 public:
  // Default timeout for all hardware module requests that is higher than any
  // hardware module operation should take, but low enough to give an error
  // message in a reasonable amount of time in case the hardware module is
  // hanging.
  static constexpr intrinsic::Duration kDefaultRequestTimeout =
      intrinsic::Seconds(60);

  struct HardwareModuleConfig {
    // If true, this is a cell control hardware module and should not be
    // disabled by faults in operational hardware modules.
    bool cell_control_hardware_keep_enabled = false;
  };

  struct HardwareModuleProxyData {
    // Pointer to the proxy. Not owned by this struct.
    HardwareModuleProxy* proxy = nullptr;
    // The async request object used for all non-rt requests associated with the
    // HWM in `proxy`.
    RemoteTriggerClient::AsyncRequest async_request;
    // The async request object used for all rt requests associated with the HWM
    // in `proxy`, used in `Read()` and `Write()`. We need to store this here,
    // since the number of hardware modules is only known at runtime. We do
    // the requests async all at once and collect the results afterwards.
    // Therefore, we need store the requests briefly in a container that can
    // only be allocated in a non-realtime context.
    RemoteTriggerClient::AsyncRequest rt_async_request;
    // The time when the async request started.
    intrinsic::Time async_request_start_time = intrinsic::Clock::Now();
    // The cached state of the HWM in `proxy`. Will be updated by
    // `UpdateState()`.
    intrinsic_fbs::HardwareModuleState cached_state = {
        intrinsic_fbs::StateCode::kDeactivated};
    HardwareModuleConfig config = {};
  };

  HardwareModuleManager() = default;

  // Adds a hardware module proxy. If `proxy_drives_clock` the `proxy` is
  // assigned to drive the clock and is read from first and written to last.
  // Returns FailedPrecondition if another proxy has already been assigned to
  // drive the clock.
  // `hardware_config` contains hardware module specific config, whether it is
  // an operational or a cell control hardware module.
  absl::Status Add(
      HardwareModuleProxy&& proxy, bool proxy_drives_clock,
      const intrinsic_proto::icon::HardwareConfig& hardware_config);

  // Calls ReadStatus on all hardware module proxies.
  RealtimeStatus Read(absl::Time deadline);
  // Calls ApplyCommand on all hardware module proxies.
  RealtimeStatus Write(absl::Time deadline);

  RealtimeStatus UpdateState() override;

  // Starts any pending transition request (if no request is currently active)
  // and checks whether ongoing requests are done.
  RealtimeStatus ProcessTransitionRequestAsync() override;

  // RealtimeStateInterface implementations.

  // Requests a new transition. The transition is pending until
  // `ProcessTransitionRequestAsync` is called.
  RealtimeStatusOr<std::optional<CommandId>> RequestTransition(
      RealtimeStateManagerInterface::TransitionRequest request) override;
  RequestState GetRequestState() const override;
  RealtimeStatusOr<RequestState> GetRequestState(
      CommandId command_id) const override;
  FixedString<kStateSummaryMaxLength> StateSummary() const override;
  std::optional<RealtimeStateManagerInterface::TransitionRequest>
  PendingTransitionRequest(CommandId command_id) const override;
  void PauseTransitions(bool pause) override { pause_transitions_ = pause; }
  RealtimeOperationalState GetCombinedOperationalState() const override;
  RealtimeOperationalState GetCellControlState() const override;
  std::optional<FaultReasonString> GetFaultReason() const override;
  // Returns true if any of the hardware modules in `group` is in any of the
  // given
  // states. Ignores proxy nullptr entries.
  bool CheckAnyInOneOfStates(
      absl::Span<const intrinsic_fbs::StateCode> states,
      RealtimeStateManagerInterface::Group group) const override;
  // Returns true if all hardware modules in `group` are in any the given
  // states. Ignores nullptr proxy entries.
  bool CheckAllInOneOfStates(
      absl::Span<const intrinsic_fbs::StateCode> states,
      RealtimeStateManagerInterface::Group group) const override;

  // Returns a pointer to a hardware module proxy from its name.
  const HardwareModuleProxy* GetHardwareModuleProxy(
      std::string_view module_name) const;

  // Returns the cached state of a hardware module from its name.
  std::optional<intrinsic_fbs::HardwareModuleState> GetHardwareModuleState(
      std::string_view module_name) const;

  // Returns the config of a hardware module.
  absl::StatusOr<HardwareModuleConfig> GetHardwareModuleConfig(
      std::string_view module_name) const INTRINSIC_NON_REALTIME_ONLY;

  // Calls Prepare and waits for completion or error/timeout on all hardware
  // module proxies in parallel.
  absl::Status Prepare(absl::Time deadline);

  // Calls Activate and waits for completion or error/timeout on all hardware
  // module proxies in parallel.
  RealtimeStatus Activate(absl::Time deadline) INTRINSIC_CHECK_REALTIME_SAFE;

  // Calls Deactivate and waits for completion or error/timeout on all hardware
  // module proxies in parallel. Always sends out deactivate calls to all
  // hardware modules, even if another request is ongoing. Even if a timeout is
  // returned, hardware modules may still have received the command and
  // deactivate later.
  RealtimeStatus Deactivate(absl::Time deadline) INTRINSIC_CHECK_REALTIME_SAFE;
  // Attempts to deactivate all hardware modules, but does not return an error
  // if some hardware modules are in a fatal/init fault since they cannot be
  // deactivated in this case.
  RealtimeStatus TryDeactivate(absl::Time deadline)
      INTRINSIC_CHECK_REALTIME_SAFE;

  // Calls EnableMotion and waits for completion or error on all hardware
  // module proxies in parallel. Only for tests, use `RequestTransition()` for
  // real usage.
  absl::Status EnableMotionTestOnly(absl::Time deadline);
  // Calls DisableMotion and waits for completion or error on all hardware
  // module proxies in parallel. Only for tests, use `RequestTransition()` for
  // real usage.
  absl::Status DisableMotionTestOnly(absl::Time deadline);
  // Calls ClearFaults and waits for completion or error on all hardware
  // module proxies in parallel. Only for tests, use `RequestTransition()` for
  // real usage.
  absl::Status ClearFaultsTestOnly(absl::Time deadline);

  // Returns a map from hardware_module_name to list of interface names that are
  // marked as required, but not used by ICON.
  // Empty when all required interfaces are used.
  // Forwards parsing errors of interface_names.
  absl::StatusOr<absl::flat_hash_map<std::string, std::vector<std::string>>>
  GetUnusedRequiredInterfaceNames() const;

 private:
  // Checks that `request` is allowed at least for one HWM using the cached HWM
  // state of each HWM.
  //
  // Returns FailedPrecondition if the transition is not allowed.
  // Returns AbortedError if the transition would be a noop.
  RealtimeStatusOr<TransitionGuardResult> CheckTransitionGuard(
      RealtimeStateManagerInterface::TransitionRequest request);

  // A pointer to a member function of HardwareModuleProxy, with the signature
  // `RealtimeStatusOr<RemoteTriggerClient::AsyncRequest>()`.
  // In particular, this type matches ActivateAsync, DeactivateAsync,
  // EnableMotionAsync and DisableMotionAsync.
  using AsyncFunc = RealtimeStatusOr<RemoteTriggerClient::AsyncRequest> (
      HardwareModuleProxy::*)() const;

  // Calls the given `async_func` on all HWM proxies and then waits for all
  // results. In case of an error, `human_readable_function_name` is used to
  // print the error messages. Will wait up until reaching `deadline`.
  // `transient_state` is the expected state of the HWM proxies after the
  // function call within 1 second, for example `kMotionEnabling` for
  // `kMotionEnabled`. This helps detecting early if the HWM is not responding.
  // `final_state` is the expected state of the HWM proxies after the function
  // call.
  //
  // Note: Must not be called if the ICON lockstep is running and do not call in
  // parallel for the same `async_func`.
  absl::Status BlockingMultiHardwareModuleCall(
      AsyncFunc async_func, absl::string_view human_readable_function_name,
      absl::Time deadline, intrinsic_fbs::StateCode transient_state,
      std::optional<intrinsic_fbs::StateCode> final_state)
      INTRINSIC_NON_REALTIME_ONLY;

  // Calls the given `async_func` on all HWM proxies and then waits for all
  // results. This function is blocking, i.e. waits for completion, but meant
  // for realtime use, e.g. for `Activate()`. It doesn't allocate memory on the
  // heap and does not have any sleeps.
  //
  // In case of an error, `human_readable_function_name` is used to
  // print the error messages. Will wait up until reaching `deadline`.
  // final_states` is a list of acceptable states of the HWM proxies after the
  // function call. If any of HWMs is not in one of these states, the function
  // returns an UnavailableError.
  //
  // Note: Must not be called if the ICON lockstep is running and do not call in
  // parallel for the same `async_func`.
  RealtimeStatus BlockingMultiHardwareModuleCallRT(
      AsyncFunc async_func, absl::string_view human_readable_function_name,
      absl::Time deadline,
      absl::Span<const intrinsic_fbs::StateCode> final_states)
      INTRINSIC_CHECK_REALTIME_SAFE;

  std::vector<std::unique_ptr<HardwareModuleProxy>>
      hwm_proxies_that_follow_clock_;
  std::unique_ptr<HardwareModuleProxy> hwm_proxy_that_drives_clock_;
  // HardwareModuleProxyData contains pointers to all proxies and their async
  // requests.
  std::vector<HardwareModuleProxyData> hwm_proxy_data_;
  // The pending transition request that will be processed on next call to
  // `ProcessTransitionRequestAsync()`.
  std::optional<RealtimeStateManagerInterface::TransitionRequest>
      pending_transition_request_ = std::nullopt;
  // The current command id that increases on every new request. Integer
  // overruns won't affect the behavior negatively.
  CommandId current_command_id_ = CommandId(0);
  intrinsic::Duration request_timeout_ = kDefaultRequestTimeout;
  // See `PauseTransitions()`. Refer to its doc in interface.
  bool pause_transitions_ = false;
  // Updated in `UpdateState()`. Refer to its doc in interface.
  RequestState cached_request_state_ = RequestState::kUnknown;
  std::optional<HardwareModuleManager::FaultReasonString>
      combined_fault_reason_;
};

constexpr RealtimeOperationalState ToRealtimeOperationalState(
    intrinsic_fbs::StateCode code) {
  if (code == intrinsic_fbs::StateCode::kMotionEnabled) {
    return RealtimeOperationalState::kEnabled;
  }
  if (code == intrinsic_fbs::StateCode::kFatallyFaulted ||
      code == intrinsic_fbs::StateCode::kInitFailed) {
    return RealtimeOperationalState::kFatallyFaulted;
  }
  if (code == intrinsic_fbs::StateCode::kFaulted ||
      code == intrinsic_fbs::StateCode::kClearingFaults) {
    return RealtimeOperationalState::kFaultedConnected;
  }
  return RealtimeOperationalState::kDisabled;
}

RealtimeOperationalStatus ToRealtimeOperationalStatus(
    const intrinsic_fbs::HardwareModuleState& state);

}  // namespace intrinsic::icon
#endif  // INTRINSIC_ICON_HAL_HARDWARE_MODULE_MANAGER_H_
