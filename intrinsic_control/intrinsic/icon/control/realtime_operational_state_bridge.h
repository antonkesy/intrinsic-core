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

#ifndef INTRINSIC_ICON_CONTROL_REALTIME_OPERATIONAL_STATE_BRIDGE_H_
#define INTRINSIC_ICON_CONTROL_REALTIME_OPERATIONAL_STATE_BRIDGE_H_

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "intrinsic/icon/cc_client/operational_status.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/control/realtime_function_bridge.h"
#include "intrinsic/icon/control/realtime_operational_status.h"
#include "intrinsic/icon/server/operational_state_interface.h"
#include "intrinsic/icon/utils/async_buffer.h"
#include "intrinsic/icon/utils/realtime_status.h"

namespace intrinsic::icon {

// Implements OperationalStateInterface using real-time safe communication
// channels.
// Note that calls to this will block indefinitely unless another thread handles
// the calls to `operational_state_command_bridge_`.
class RealtimeOperationalStateBridge final : public OperationalStateInterface {
 public:
  struct OperationalCellControlStatus {
    RealtimeOperationalStatus operational;
    RealtimeOperationalStatus cell_control;
  };

  RealtimeOperationalStateBridge(
      RealtimeFunctionBridge<RealtimeStatus(OperationalStateCommand cmd)>&
          operational_state_command_bridge,
      AsyncBuffer<OperationalCellControlStatus>&
          operational_cell_control_status_buffer)
      : operational_cell_control_status_buffer_(
            operational_cell_control_status_buffer),
        operational_state_command_bridge_(operational_state_command_bridge) {}

  absl::Status Enable() override;

  absl::Status Disable(bool skip_cell_control_hardware) override;

  absl::Status ClearFaults() override;

  absl::StatusOr<OperationalStatus> GetStatus() override;

  absl::StatusOr<OperationalStatus> GetCellControlStatus() override;

  RealtimeOperationalStatus GetInternalStatus() override;

 private:
  absl::Mutex operational_status_mutex_;
  // Reading from an AsyncBuffer is not thread safe by itself, since another
  // thread may invalidate the pointer we're reading from.
  AsyncBuffer<OperationalCellControlStatus>&
      operational_cell_control_status_buffer_
          ABSL_GUARDED_BY(operational_status_mutex_);
  // RealtimeBridge is thread safe on its own.
  RealtimeFunctionBridge<RealtimeStatus(OperationalStateCommand cmd)>&
      operational_state_command_bridge_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_REALTIME_OPERATIONAL_STATE_BRIDGE_H_
