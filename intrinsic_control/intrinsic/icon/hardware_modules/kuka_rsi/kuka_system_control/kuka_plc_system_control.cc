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

#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_control/kuka_plc_system_control.h"

#include <functional>
#include <memory>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_util.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_control/kuka_plc_client.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::kuka {

constexpr absl::Duration kRetryTimeout = absl::Seconds(10);
constexpr absl::Duration kStatePollingInterval = absl::Milliseconds(50);

absl::Status KukaPlcSystemControl::StartRSI(absl::Duration timeout) {
  std::function<absl::Status()> start_rsi = [this, timeout]() -> absl::Status {
    INTR_ASSIGN_OR_RETURN(auto plc_state, plc_client_->getState());

    switch (plc_state) {
      case KukaPlcState::kRunning:
        LOG(INFO) << "Calling KukaRsiClient::StartRSI but RSI is already "
                     "'started' (state 'Running'). This has no effect.";
        return absl::OkStatus();

      case KukaPlcState::kEStop:
        return absl::FailedPreconditionError(
            "Calling KukaRsiClient::StartRSI but the system is in state "
            "'E-STOP'. An operator intervention is required.");

      case KukaPlcState::kStopMessagesActive:
        LOG(INFO) << "Calling KukaRsiClient::StartRSI but stop messages are "
                     "active (errors). Attempting to clear errors";
        INTR_RETURN_IF_ERROR(plc_client_->RequestAcknowledgeErrors());

        [[fallthrough]];

      case KukaPlcState::kWaitForPrgActive:
        // The PLC might be currently starting the program. So wait a bit for it
        // to become ready.
        INTR_RETURN_IF_ERROR(plc_client_->WaitForState(
            KukaPlcState::kReady, timeout, kStatePollingInterval))
            << "While waiting for PLC to become ready";

        [[fallthrough]];
      case KukaPlcState::kReady:
        INTR_RETURN_IF_ERROR(plc_client_->RequestStartRSI());
        INTR_RETURN_IF_ERROR(plc_client_->WaitForState(
            KukaPlcState::kRunning, timeout, kStatePollingInterval));
        break;

      default: {
        INTR_ASSIGN_OR_RETURN(auto plc_state_string,
                              plc_client_->getStateAsString());

        return absl::FailedPreconditionError(absl::StrFormat(
            "Calling KukaRsiClient::StartRSI but the system is in an "
            "unexpected state (0x%x: %s). Operator intervention is "
            "required.",
            static_cast<int>(plc_state), plc_state_string));
      }
    }
    return absl::OkStatus();
  };
  return RetryUntilTimeout(start_rsi, "StartRSI", kRetryTimeout);
}

absl::Status KukaPlcSystemControl::StopRSI() {
  std::function<absl::Status()> stop_rsi = [this]() -> absl::Status {
    INTR_ASSIGN_OR_RETURN(auto state, plc_client_->getState());

    if (state != KukaPlcState::kRunning) {
      LOG(INFO) << "Attempting to stop RSI but it is already stopped";

      return absl::OkStatus();
    }

    INTR_RETURN_IF_ERROR(plc_client_->RequestStopRSI());
    INTR_RETURN_IF_ERROR(plc_client_->WaitForState(
        KukaPlcState::kReady, absl::Seconds(2), kStatePollingInterval));
    LOG(INFO) << "RSI is now stopped.";
    return absl::OkStatus();
  };
  return RetryUntilTimeout(stop_rsi, "StopRSI", kRetryTimeout);
}

absl::Status KukaPlcSystemControl::ClearFaults() {
  const auto deadline = absl::Now() + kRetryTimeout;
  std::function<absl::Status()> clear_faults = [this,
                                                deadline]() -> absl::Status {
    INTR_RETURN_IF_ERROR(plc_client_->RequestAcknowledgeErrors());
    while (absl::Now() < deadline) {
      INTR_ASSIGN_OR_RETURN(auto state, plc_client_->getState());
      if (state != KukaPlcState::kStopMessagesActive) {
        return absl::OkStatus();
      }
    }
    return absl::DeadlineExceededError(absl ::StrCat(
        "ClearFaults timed out after ", absl::FormatDuration(kRetryTimeout)));
  };
  return RetryUntilTimeout(clear_faults, "ClearFaults", kRetryTimeout);
}

absl::StatusOr<KukaErrorFlagsMask> KukaPlcSystemControl::ActiveErrorFlags()
    const {
  std::function<absl::StatusOr<KukaErrorFlagsMask>()> get_state =
      [this]() -> absl::StatusOr<KukaErrorFlagsMask> {
    INTR_ASSIGN_OR_RETURN(auto plc_state, plc_client_->getState());

    switch (plc_state) {
      case KukaPlcState::kReady:
        break;
      case KukaPlcState::kStopMessagesActive:
        return KukaErrorFlagsMask::kMessagesPresent;
      case KukaPlcState::kWaitForPrgNoReq:
        break;
      case KukaPlcState::kRunning:
        break;
      case KukaPlcState::kWaitForApplRun:
        break;
      case KukaPlcState::kEStop:
        return KukaErrorFlagsMask::kEmergencyStopActiveOrWrongOpMode;
      default:
        break;
    }
    return KukaErrorFlagsMask::kNone;
  };
  return RetryUntilTimeout(get_state, "GetState", kRetryTimeout);
}
}  // namespace intrinsic::kuka
