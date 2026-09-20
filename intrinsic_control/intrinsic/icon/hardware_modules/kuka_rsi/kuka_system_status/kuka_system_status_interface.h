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

#ifndef INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_SYSTEM_STATUS_KUKA_SYSTEM_STATUS_INTERFACE_H_
#define INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_SYSTEM_STATUS_KUKA_SYSTEM_STATUS_INTERFACE_H_

#include <netdb.h>

#include <cstdint>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/hardware_modules/kuka_rsi/kuka_rsi_util.h"
#include "intrinsic/icon/testing/realtime_annotations.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/world/robot_payload/robot_payload_base.h"

namespace intrinsic::kuka {
constexpr absl::string_view kUnknownKukaModelName = "UnknownModel";

class KukaSystemStatusInterface {
 public:
  virtual ~KukaSystemStatusInterface() = default;

  // Name of the implementation.
  virtual std::string Name() const = 0;
  // Returns the current operation mode as reported by the robot, if possible.
  virtual absl::StatusOr<OpMode> CurrentOpMode() const = 0;
  // Returns the active error flags as reported by the robot.
  virtual absl::StatusOr<KukaErrorFlagsMask> ActiveErrorFlags() const = 0;
  // Returns the string representation of error messages as reported by the
  // robot.
  virtual std::vector<std::string> ErrorMessages() const = 0;
  // Returns the current position as reported by the robot in radians. This
  // position will be used when RSI is not active. If this function returns an
  // error, the reported position will not be updated when RSI is not active.
  virtual icon::RealtimeStatusOr<eigenmath::Vectord<kKukaNumJoints>>
  CurrentPosition() const INTRINSIC_CHECK_REALTIME_SAFE = 0;
  // Returns the robot serial number as reported by the robot. When no data is
  // available yet, the function should return an UnavailableError. If the
  // implementation cannot provide the data, it should return 0.
  virtual absl::StatusOr<uint32_t> RobotSerialNumber() const = 0;
  // Returns the robot serial number as reported by the robot. When no data is
  // available yet, the function should return an UnavailableError. If the
  // implementation cannot provide the data, it should return
  // kUnknownKukaModelName.
  virtual absl::StatusOr<std::string> RobotModel() const = 0;
  // Waits until new data is available. Returns true, if new data is available
  // before the timeout. Returns false when no new data is available and the
  // timeout has been reached.
  virtual bool WaitForNewData(absl::Duration timeout) const = 0;
  // Resets the internal storage of faults until new data is received. Does not
  // reset faults on the KUKA KRC.
  virtual void ResetFaultStorage() = 0;
  // Sets the tcp payload on the robot controller. Shall wait until the payload
  // is set and return an error if the payload could not be set.
  virtual absl::Status SetPayload(const RobotPayloadBase& payload) = 0;
};

// This implementation does nothing and returns empty, error values or
// placeholder values.
class NoOpKukaSystemStatus : public KukaSystemStatusInterface {
 public:
  std::string Name() const override { return "NoOp"; }
  absl::StatusOr<OpMode> CurrentOpMode() const override {
    return absl::UnavailableError("");
  }
  absl::StatusOr<KukaErrorFlagsMask> ActiveErrorFlags() const override {
    return KukaErrorFlagsMask::kNone;
  }
  std::vector<std::string> ErrorMessages() const override { return {}; }
  icon::RealtimeStatusOr<eigenmath::Vectord<kKukaNumJoints>> CurrentPosition()
      const override INTRINSIC_CHECK_REALTIME_SAFE {
    return icon::UnavailableError("");
  }
  absl::StatusOr<uint32_t> RobotSerialNumber() const override { return 0; }
  absl::StatusOr<std::string> RobotModel() const override {
    return std::string(kUnknownKukaModelName);
  }
  bool WaitForNewData(absl::Duration timeout) const override { return true; }
  void ResetFaultStorage() override {}
  absl::Status SetPayload(const RobotPayloadBase& payload) override {
    return absl::UnavailableError(
        "Payload setting unavailable. Use a KukaSystemStatusInterface that "
        "supports payload setting, e.g. KUKA EKI.");
  }
};

}  // namespace intrinsic::kuka

#endif  // INTRINSIC_ICON_HARDWARE_MODULES_KUKA_RSI_KUKA_SYSTEM_STATUS_KUKA_SYSTEM_STATUS_INTERFACE_H_
