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

#ifndef INTRINSIC_ICON_SERVER_TESTING_FAKE_OPERATIONAL_STATE_H_
#define INTRINSIC_ICON_SERVER_TESTING_FAKE_OPERATIONAL_STATE_H_

#include "absl/base/thread_annotations.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "intrinsic/icon/cc_client/operational_status.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/server/operational_state_interface.h"

namespace intrinsic {
namespace icon {
class FakeOperationalState : public OperationalStateInterface {
 public:
  FakeOperationalState() : status_(OperationalStatus::Enabled()) {}

  absl::Status Enable() override {
    absl::MutexLock l(status_mutex_);
    if (IsFaulted(status_)) {
      return absl::FailedPreconditionError(
          "FAULTED: you must ClearFaults() before calling Enable()");
    }
    status_ = OperationalStatus::Enabled();
    return absl::OkStatus();
  }

  absl::Status Disable(bool skip_cell_control_hardware) override {
    if (skip_cell_control_hardware) {
      return absl::UnimplementedError("not implemented");
    }
    {
      absl::MutexLock l(status_mutex_);
      if (IsFaulted(status_)) {
        return absl::FailedPreconditionError(
            "FAULTED: you must ClearFaults() instead of Disable()");
      }
      status_ = OperationalStatus::Disabled();
    }
    // Keep the fake disabled for at least 10ms to ensure that ICON is ticked
    // at least once.
    absl::SleepFor(absl::Milliseconds(10));
    LOG(INFO) << "Auto-enabling ICON fake after Disable()...";
    return Enable();
  }

  absl::Status ClearFaults() override {
    {
      absl::MutexLock l(status_mutex_);
      status_ = OperationalStatus::Disabled();
    }
    LOG(INFO) << "Auto-enabling ICON fake after ClearFaults()...";
    return Enable();
  }

  absl::StatusOr<OperationalStatus> GetStatus() override {
    absl::MutexLock l(status_mutex_);
    return status_;
  }

  absl::StatusOr<OperationalStatus> GetCellControlStatus() override {
    return GetStatus();
  }

  RealtimeOperationalStatus GetInternalStatus() override {
    absl::MutexLock l(status_mutex_);
    switch (status_.state()) {
      case OperationalState::kDisabled:
        return RealtimeOperationalStatus{
            .state = RealtimeOperationalState::kDisabled};
      case OperationalState::kEnabled:
        return RealtimeOperationalStatus{
            .state = RealtimeOperationalState::kEnabled};
      case OperationalState::kFaulted:
        return RealtimeOperationalStatus{
            .state = RealtimeOperationalState::kFaultedConnected,
            .fault_reason = RealtimeOperationalStatus::FaultReasonString(
                absl::string_view(status_.fault_reason()))};
    }
    LOG(FATAL) << "Unexpected OperationalState";
  }

  void Fault(absl::string_view reason) {
    absl::MutexLock l(status_mutex_);
    status_ = OperationalStatus::Faulted(reason);
  }

  absl::Status SetDisabled() {
    absl::MutexLock l(status_mutex_);
    if (IsFaulted(status_)) {
      return absl::FailedPreconditionError(
          "FAULTED: you must ClearFaults() instead of SetDisabled()");
    }
    status_ = OperationalStatus::Disabled();
    return absl::OkStatus();
  }

 private:
  absl::Mutex status_mutex_;
  OperationalStatus status_ ABSL_GUARDED_BY(status_mutex_);
};
}  // namespace icon
}  // namespace intrinsic

#endif  // INTRINSIC_ICON_SERVER_TESTING_FAKE_OPERATIONAL_STATE_H_
