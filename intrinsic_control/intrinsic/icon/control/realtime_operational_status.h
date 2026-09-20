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

#ifndef INTRINSIC_ICON_CONTROL_REALTIME_OPERATIONAL_STATUS_H_
#define INTRINSIC_ICON_CONTROL_REALTIME_OPERATIONAL_STATUS_H_

#include <cstddef>

#include "absl/strings/string_view.h"
#include "intrinsic/icon/cc_client/operational_status.h"
#include "intrinsic/icon/proto/part_status.pb.h"
#include "intrinsic/icon/utils/fixed_string.h"
#include "intrinsic/icon/utils/log.h"

namespace intrinsic::icon {

enum class OperationalStateCommand {
  kNone,
  kEnable,
  kDisable,
  kDisableSkipCellControlHardware,
  kClearFaults
};

// Enum of possible operational states in the real-time control layer.
// The control layer has more states than the client and public interface in
// intrinsic/icon/cc_client/operational_status.h
// kFaulted and kFatallyFaulted are mapped to OperationalState::kFaulted.
enum class RealtimeOperationalState {
  // Indicates that server is not ready for sessions to start, expect for
  // read-only sessions which are possible.
  // Part status is being published.
  // Call `icon_client.Enable()`, or wait for the server to auto-enable to allow
  // starting sessions.
  kDisabled,

  // Indicates that at least one part is in an erroneous state,
  // but all hardware modules are connected and the real-time clock is running.
  // Read-only sessions can continue if their parts are not faulted.
  // Follow instructions in the `fault_reason` message, then call
  // `icon_client.ClearFaults()` (possible followed by `Enable()`) to re-enable
  // control.
  // Part status is published but may be incomplete for faulted parts.
  // One example is that a hardware module reports an emergency stop.
  kFaultedConnected,

  // Indicates that the control loop and real-time clock have stopped.
  // Similar to kFaultedConnected, a `fault_reason` message is given
  // and `ClearFaults()` can be attempted.
  // The published part status topic only contains an error message.
  // One example is that a hardware module stopped responding.
  kFatallyFaulted,

  // Indicates that the server is ready for a session to begin and all parts are
  // enabled.
  // Part status is being published.
  kEnabled
};

constexpr RealtimeOperationalState GetMaximumSeverity(
    RealtimeOperationalState state1, RealtimeOperationalState state2) {
  constexpr auto severity = [](const RealtimeOperationalState& state) {
    switch (state) {
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
  if (severity(state1) > severity(state2)) return state1;
  return state2;
}

inline OperationalState ToOperationalState(RealtimeOperationalState state) {
  switch (state) {
    case RealtimeOperationalState::kDisabled:
      return OperationalState::kDisabled;
    case RealtimeOperationalState::kFaultedConnected:
    case RealtimeOperationalState::kFatallyFaulted:
      return OperationalState::kFaulted;
    case RealtimeOperationalState::kEnabled:
      return OperationalState::kEnabled;
  }
  INTRINSIC_RT_LOG_THROTTLED(ERROR)
      << "Unexpected RealtimeOperationalState: " << state;
  return OperationalState::kFaulted;
}

// A real-time safe structure representing an operational status.
struct RealtimeOperationalStatus {
  static constexpr size_t kFaultReasonMaxLength = 1024;
  using FaultReasonString = FixedString<kFaultReasonMaxLength>;

  // Current operational state.
  RealtimeOperationalState state;
  // Human-readable reason for the fault. This should be empty/ignored if
  // `state != kFaultedConnected && state != kFatallyFaulted`.
  FaultReasonString fault_reason;
};

// A set of hardware module groups (operational and cell control hardware).
// A part can depend on none or a single group. A session can depend on any
// combination of groups (or none). One group shares the same fault behavior.
// For example, operational hardware faulting will disable all operational
// hardware, but does not propagate to cell control hardware.
struct HardwareGroupSet {
  bool operational_hardware = false;
  bool cell_control_hardware = false;
};

inline absl::string_view ToString(OperationalStateCommand command) {
  switch (command) {
    case OperationalStateCommand::kNone:
      return "none";
    case OperationalStateCommand::kEnable:
      return "enable";
    case OperationalStateCommand::kDisable:
      return "disable";
    case OperationalStateCommand::kDisableSkipCellControlHardware:
      return "disable_skip_cell_control_hardware";
    case OperationalStateCommand::kClearFaults:
      return "clear_faults";
    default:
      return "unknown";
  }
}

inline absl::string_view ToString(RealtimeOperationalState state) {
  switch (state) {
    case RealtimeOperationalState::kDisabled:
      return "disabled";
    case RealtimeOperationalState::kFaultedConnected:
      return "faulted_connected";
    case RealtimeOperationalState::kFatallyFaulted:
      return "fatally_faulted";
    case RealtimeOperationalState::kEnabled:
      return "enabled";
    default:
      return "unknown";
  }
}

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_REALTIME_OPERATIONAL_STATUS_H_
