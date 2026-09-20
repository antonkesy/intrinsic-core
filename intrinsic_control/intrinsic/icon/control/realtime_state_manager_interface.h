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

#ifndef INTRINSIC_ICON_CONTROL_REALTIME_STATE_MANAGER_INTERFACE_H_
#define INTRINSIC_ICON_CONTROL_REALTIME_STATE_MANAGER_INTERFACE_H_
#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <optional>

#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "intrinsic/icon/control/realtime_operational_status.h"
#include "intrinsic/icon/hal/interfaces/hardware_module_state.fbs.h"
#include "intrinsic/icon/utils/fixed_string.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "ortools/base/strong_int.h"

namespace intrinsic::icon {

// Interface for managing the state of real-time sub-components such as hardware
// modules under real-time constraints.
class RealtimeStateManagerInterface {
 public:
  DEFINE_STRONG_INT_TYPE(CommandId, int64_t);
  enum class RequestState {
    kUnknown,
    kActive,         // The RealtimeStateManager processes currently a request.
    kPartiallyDone,  // Some but not all requests on the sub-components are done
                     // (when an implementation manages multiple
                     // sub-components).
    kDone            // All requests on the sub-components are done.
  };

  enum class TransitionRequest {
    kEnableMotion,
    kDisableMotion,
    kDisableMotionSkipCellControlHardware,
    kClearFaults,
  };

  // A group of hardware modules to filter by. `kCellControlHardware` is the
  // group of hardware modules that have
  // `IconMainConfig.hardware_config.cell_control_hardware` configured,
  // `kOperationalHardware` is the rest.
  enum class Group {
    kAllHardware,
    kOperationalHardware,
    kCellControlHardware,
  };

  static constexpr size_t kStateSummaryMaxLength = 1024;
  using FaultReasonString =
      FixedString<RealtimeOperationalStatus::kFaultReasonMaxLength>;

  virtual ~RealtimeStateManagerInterface() = default;

  // Updates the internal states (request state, sub-component states and fault
  // reason). To be called at a specific point in the cycle so that one state is
  // used per cycle instead of depending on the timing of a call. A class that
  // implements this method should store the RequestStates for any ongoing
  // requests in member variables. The other methods of this interface should
  // refer to those stored states, and not query for or compute an updated state
  // themselves. Implementations must consider that different requests to
  // sub-components can be active at the same time, but only one request per
  // sub-component. The responsibility to correctly cache the state lies with
  // the implementation of this interface.
  virtual RealtimeStatus UpdateState() = 0;

  // Returns the current state of the most recent request.
  virtual RequestState GetRequestState() const = 0;

  // Returns the current state of the request that matches the provided command
  // id.
  // Returns `FailedPreconditionError` if the `command_id` does not match the id
  // of the active request or if there is no active request.
  // This function does *not* consider whether there never existed a request or
  // the provided request has already been finished and cleaned up.
  virtual RealtimeStatusOr<RequestState> GetRequestState(
      CommandId command_id) const = 0;

  // Requests a new transition. This function must not block or do any of the
  // actual processing. Processing happens in ProcessTransitionRequestAsync.
  // - Returns a new command id if the request would not have any effect.
  // - Returns std::nullopt if the request would not have any effect.
  // - Returns InvalidArgumentError if `request` contains an invalid value.
  // - Returns ResourceExhaustedError if a request is already pending.
  // - Returns ResourceExhaustedError if all sub-components are already
  // processing a request.
  // - Returns FailedPreconditionError when the request is not allowed in the
  // current state.
  //
  // If only some sub-components are busy, forwards the request
  // to all idle sub-components. This overriding of an active request
  // is needed to disable sub-components when the operation succeeded on one
  // sub-component, failed on another and a third sub-component is still
  // processing. This function marks the request as kActive/kPartiallyDone
  // until also the new request is finished, but increases the `command_id`
  // counter of `GetRequestState()` already. This means the user can consider
  // the original request failed.
  //
  // If the requested transition will have no effect due to the current state
  // (e.g. requesting kEnableMotion when already in MotionEnabled), the function
  // returns AbortedError.
  virtual RealtimeStatusOr<std::optional<CommandId>> RequestTransition(
      TransitionRequest request) = 0;

  // Returns the combined operational state of all sub-components:
  // - If any sub-component is in any of the faulted states, the return value
  //   is `Faulted`.
  // - If all are enabled, the return value is `Enabled`.
  // - Otherwise `Disabled`.
  virtual RealtimeOperationalState GetCombinedOperationalState() const = 0;

  // Returns the combined state of cell control hardware modules.
  // - If any cell control hardware  is in any of the faulted states, the return
  //   value is `Faulted`.
  // - If all are enabled, the return value is `Enabled`.
  // - Otherwise `Disabled`.
  virtual RealtimeOperationalState GetCellControlState() const = 0;

  // Returns the combined reason (joined by `;`) for the faults of all
  // sub-components, or nullopt if none of the sub-components are currently
  // faulted.
  virtual std::optional<FaultReasonString> GetFaultReason() const = 0;

  // Returns the currently pending transition request.
  // Returns std::nullopt if no transition is currently requested (by a call to
  // `RequestTransition()`) or if `command_id` does not match the current
  // command id.
  virtual std::optional<TransitionRequest> PendingTransitionRequest(
      CommandId command_id) const = 0;

  // Processes transitions requests. Must not block.
  // Returns DeadlineExceededError if the request times out.
  virtual RealtimeStatus ProcessTransitionRequestAsync() = 0;

  // Pauses (or resumes) processing transitions. The pause-state does not affect
  // whether `RequestTransition()` will accept new requests.
  virtual void PauseTransitions(bool pause) = 0;

  // Returns true if any of the hardware modules in `group` is in any of the
  // given states.
  virtual bool CheckAnyInOneOfStates(
      absl::Span<const intrinsic_fbs::StateCode> states, Group group) const = 0;

  // Returns true if all hardware modules in `group` are in any the given
  // states.
  virtual bool CheckAllInOneOfStates(
      absl::Span<const intrinsic_fbs::StateCode> states, Group group) const = 0;

  // Returns a summary of the current state of all sub-components for debugging
  // purposes.
  virtual FixedString<kStateSummaryMaxLength> StateSummary() const {
    return {};
  }
};

constexpr absl::string_view ToString(
    RealtimeStateManagerInterface::TransitionRequest request) {
  switch (request) {
    case RealtimeStateManagerInterface::TransitionRequest::kEnableMotion:
      return "kEnableMotion";
    case RealtimeStateManagerInterface::TransitionRequest::kDisableMotion:
      return "kDisableMotion";
    case RealtimeStateManagerInterface::TransitionRequest::
        kDisableMotionSkipCellControlHardware:
      return "kDisableMotionSkipCellControlHardware";
    case RealtimeStateManagerInterface::TransitionRequest::kClearFaults:
      return "kClearFaults";
  }
}

template <typename Sink>
void AbslStringify(Sink& sink,
                   RealtimeStateManagerInterface::TransitionRequest request) {
  absl::Format(&sink, "%s", ToString(request));
}

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_REALTIME_STATE_MANAGER_INTERFACE_H_
