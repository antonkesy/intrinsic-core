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

#include "intrinsic/icon/control/realtime_operational_state_bridge.h"

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "intrinsic/icon/cc_client/operational_status.h"
#include "intrinsic/icon/control/realtime_operational_status.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

absl::Status RealtimeOperationalStateBridge::Enable() {
  INTRINSIC_ASSERT_NON_REALTIME();
  INTR_ASSIGN_OR_RETURN(
      RealtimeStatus status,
      operational_state_command_bridge_.Call(OperationalStateCommand::kEnable));
  return status;
}

absl::Status RealtimeOperationalStateBridge::Disable(
    bool skip_cell_control_hardware) {
  INTRINSIC_ASSERT_NON_REALTIME();
  INTR_ASSIGN_OR_RETURN(
      RealtimeStatus status,
      operational_state_command_bridge_.Call(
          skip_cell_control_hardware
              ? OperationalStateCommand::kDisableSkipCellControlHardware
              : OperationalStateCommand::kDisable));
  return status;
}

absl::Status RealtimeOperationalStateBridge::ClearFaults() {
  INTRINSIC_ASSERT_NON_REALTIME();
  INTR_ASSIGN_OR_RETURN(RealtimeStatus status,
                        operational_state_command_bridge_.Call(
                            OperationalStateCommand::kClearFaults));
  return status;
}

absl::StatusOr<OperationalStatus> RealtimeOperationalStateBridge::GetStatus() {
  INTRINSIC_ASSERT_NON_REALTIME();
  absl::MutexLock l(operational_status_mutex_);
  OperationalCellControlStatus* active_buffer_ptr;
  operational_cell_control_status_buffer_.GetActiveBuffer(&active_buffer_ptr);
  switch (active_buffer_ptr->operational.state) {
    case RealtimeOperationalState::kDisabled:
      return OperationalStatus::Disabled();
    case RealtimeOperationalState::kFaultedConnected:
    case RealtimeOperationalState::kFatallyFaulted:
      return OperationalStatus::Faulted(
          active_buffer_ptr->operational.fault_reason);
    case RealtimeOperationalState::kEnabled:
      return OperationalStatus::Enabled();
  }
  return absl::InternalError(
      "Unexpected OperationalState; Should never reach here.");
}

absl::StatusOr<OperationalStatus>
RealtimeOperationalStateBridge::GetCellControlStatus() {
  INTRINSIC_ASSERT_NON_REALTIME();
  absl::MutexLock l(operational_status_mutex_);
  OperationalCellControlStatus* active_buffer_ptr;
  operational_cell_control_status_buffer_.GetActiveBuffer(&active_buffer_ptr);
  switch (active_buffer_ptr->cell_control.state) {
    case RealtimeOperationalState::kDisabled:
      return OperationalStatus::Disabled();
    case RealtimeOperationalState::kFaultedConnected:
      return OperationalStatus::Faulted(
          active_buffer_ptr->cell_control.fault_reason);
    case RealtimeOperationalState::kFatallyFaulted:
      return OperationalStatus::Faulted(
          active_buffer_ptr->cell_control.fault_reason);
    case RealtimeOperationalState::kEnabled:
      return OperationalStatus::Enabled();
  }
  return absl::InternalError(
      "Unexpected OperationalState; Should never reach here.");
}

RealtimeOperationalStatus RealtimeOperationalStateBridge::GetInternalStatus() {
  INTRINSIC_ASSERT_NON_REALTIME();
  absl::MutexLock l(operational_status_mutex_);
  OperationalCellControlStatus* active_buffer_ptr;
  operational_cell_control_status_buffer_.GetActiveBuffer(&active_buffer_ptr);
  return active_buffer_ptr->operational;
}

}  // namespace intrinsic::icon
