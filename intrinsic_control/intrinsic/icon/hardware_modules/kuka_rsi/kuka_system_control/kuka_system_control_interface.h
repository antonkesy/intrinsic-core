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

#ifndef INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_SYSTEM_CONTROL_KUKA_SYSTEM_CONTROL_INTERFACE_H_
#define INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_SYSTEM_CONTROL_KUKA_SYSTEM_CONTROL_INTERFACE_H_

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_util.h"
#include "intrinsic/util/fixed_vector.h"

namespace intrinsic::kuka {

class KukaSystemControlInterface {
 public:
  virtual ~KukaSystemControlInterface() = default;
  // Returns the compatible KUKA operation modes to use this
  // KukaSystemControlInterface control implementation. Usually, it is
  // kExternal, but can be multiple.
  virtual FixedVector<OpMode, OpModeCount> CompatibleOperationModes() const = 0;
  // Triggers starting the RSI on the KUKA side. Doesn't wait for the RSI to
  // actually be running.
  virtual absl::Status StartRSI(absl::Duration timeout) = 0;
  // Triggers stopping the RSI on the KUKA side.
  virtual absl::Status StopRSI() = 0;
  // Clear all present faults, if possible. Should block until faults are
  // cleared. If this function returns earlier, it can happen that old faults
  // are still reported as active after clearing is done.
  virtual absl::Status ClearFaults() = 0;
  // Returns the active error flags as reported by the robot.
  virtual absl::StatusOr<KukaErrorFlagsMask> ActiveErrorFlags() const = 0;
};

// This implementation does nothing and expects the user to do the changes via
// the teach pendant.
class NoOpKukaSystemControl : public KukaSystemControlInterface {
 public:
  // This mode does not care if it is in external or automatic mode. The control
  // of the system state must happen externally (e.g. human with teach pendant
  // or 3rd party PLC).
  FixedVector<OpMode, OpModeCount> CompatibleOperationModes() const override {
    return {OpMode::kAutomatic, OpMode::kExternal};
  }
  absl::Status StartRSI(absl::Duration) override { return absl::OkStatus(); }
  absl::Status StopRSI() override { return absl::OkStatus(); }
  absl::Status ClearFaults() override { return absl::OkStatus(); }
  absl::StatusOr<KukaErrorFlagsMask> ActiveErrorFlags() const override {
    return KukaErrorFlagsMask::kNone;
  }
};

}  // namespace intrinsic::kuka

#endif  // INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_SYSTEM_CONTROL_KUKA_SYSTEM_CONTROL_INTERFACE_H_
