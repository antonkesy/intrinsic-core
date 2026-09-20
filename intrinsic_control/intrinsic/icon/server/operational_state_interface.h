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

#ifndef INTRINSIC_ICON_SERVER_OPERATIONAL_STATE_INTERFACE_H_
#define INTRINSIC_ICON_SERVER_OPERATIONAL_STATE_INTERFACE_H_

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/icon/cc_client/operational_status.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"

namespace intrinsic {
namespace icon {

// OperationalState allows a non-realtime caller to query and modify the
// operational state of an ICON instance.
//
// Classes implementing this interface must be thread safe!
class OperationalStateInterface {
 public:
  virtual ~OperationalStateInterface() = default;

  // Attempts to enable the server, preparing parts for sessions to be created.
  // Blocks until all parts are done enabling (or an error occurs).
  virtual absl::Status Enable() = 0;

  // Attempts to disable the server, bringing parts to a fully disabled state.
  // Terminates all active sessions.
  // Blocks until all parts are done disabling (or an error occurs).
  virtual absl::Status Disable(bool skip_cell_control_hardware) = 0;

  // Attempts to clear faults present on the server and return to a disabled
  // state.
  // Blocks until all parts are done clearing faults (or an error occurs). A new
  // fault can occur while clearing the previous one, causing ICON to stay in a
  // faulted state (potentially with a new fault_reason string).
  virtual absl::Status ClearFaults() = 0;

  // Attempts to return the current operational status. Implementations must be
  // thread-safe.
  virtual absl::StatusOr<OperationalStatus> GetStatus() = 0;
  // Attempts to return the current status of cell control hardware
  // modules. Implementations must be thread-safe.
  virtual absl::StatusOr<OperationalStatus> GetCellControlStatus() = 0;

  // Similar to GetStatus() but with more detailed RealtimeOperationalState.
  virtual RealtimeOperationalStatus GetInternalStatus() = 0;
};

}  // namespace icon
}  // namespace intrinsic

#endif  // INTRINSIC_ICON_SERVER_OPERATIONAL_STATE_INTERFACE_H_
