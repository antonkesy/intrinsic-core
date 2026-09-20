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

#ifndef INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_SYSTEM_CONTROL_KUKA_PLC_SYSTEM_CONTROL_H_
#define INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_SYSTEM_CONTROL_KUKA_PLC_SYSTEM_CONTROL_H_

#include <functional>
#include <memory>
#include <utility>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_util.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_control/kuka_plc_client.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_system_control/kuka_system_control_interface.h"
#include "intrinsic/util/fixed_vector.h"

namespace intrinsic::kuka {

// Uses OPC-UA to communicate with a PLC that is connected to the KUKA robot
// using IOs.
class KukaPlcSystemControl : public KukaSystemControlInterface {
 public:
  explicit KukaPlcSystemControl(std::unique_ptr<KukaPlcClient>&& plc_client)
      : plc_client_(std::move(plc_client)) {}
  FixedVector<OpMode, OpModeCount> CompatibleOperationModes() const override {
    return {OpMode::kExternal};
  }
  absl::Status StartRSI(absl::Duration timeout) override;
  absl::Status StopRSI() override;
  absl::Status ClearFaults() override;
  absl::StatusOr<KukaErrorFlagsMask> ActiveErrorFlags() const override;

 private:
  absl::Status ToStatus(const absl::Status& status) const { return status; }
  template <typename ReturnType>
  absl::Status ToStatus(const absl::StatusOr<ReturnType>& status) const {
    return status.status();
  }
  template <typename ReturnType>
  ReturnType RetryUntilTimeout(std::function<ReturnType()>& func,
                               absl::string_view function_name,
                               absl::Duration timeout) const {
    auto result = func();
    if (result.ok() ||
        // OPC-UA problems are returned as
        // UnavailableError. Only retry in those cases.
        !absl::IsUnavailable(ToStatus(result))) {
      return result;
    }
    // Retry at least once. More times only when below timeout.
    absl::Time deadline = absl::Now() + timeout;
    absl::Status status;
    do {
      LOG(INFO)
          << "Attempting to reconnect to OPC-UA PLC for perform function `"
          << function_name << "`.";
      status = plc_client_->Reconnect();
      if (!status.ok()) {
        absl::SleepFor(absl::Seconds(1));
        continue;
      }
      auto result = func();
      auto func_status = ToStatus(result);
      if (func_status.ok() ||
          !absl::IsUnavailable(
              func_status)  // OPC-UA problems are returned as
                            // UnavailableError. Only retry in those cases.
      ) {
        return result;
      }
      status = func_status;
    } while (absl::Now() < deadline);
    if (status.ok()) {
      return absl::DeadlineExceededError(absl ::StrCat(
          function_name, " timed out after ", absl::FormatDuration(timeout)));
    }
    return absl::DeadlineExceededError(absl ::StrCat(
        function_name, " timed out after ", absl::FormatDuration(timeout),
        ". error: ", status.message()));
  }
  std::unique_ptr<KukaPlcClient> plc_client_;
};
}  // namespace intrinsic::kuka

#endif  // INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_SYSTEM_CONTROL_KUKA_PLC_SYSTEM_CONTROL_H_
