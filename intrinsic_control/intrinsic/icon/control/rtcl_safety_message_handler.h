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

#ifndef INTRINSIC_ICON_CONTROL_RTCL_SAFETY_MESSAGE_HANDLER_H_
#define INTRINSIC_ICON_CONTROL_RTCL_SAFETY_MESSAGE_HANDLER_H_

#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/control/context.h"
#include "intrinsic/icon/control/safety/extern/safety_status.fbs.h"
#include "intrinsic/icon/control/safety/safety_messages.fbs.h"
#include "intrinsic/icon/control/safety/safety_messages_utils.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/interfaces/io_controller.fbs.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/util/fixed_vector.h"

namespace intrinsic::icon {

// Reads and combines all configured `safety_status` messages by passing trough
// the most significant entry.
// Sets the exported SafetyStatus handle to `RequestedBehavior::PAUSE` when the
// configured signal is active and no more significant behavior is requested.
// Move only.
class SafetyMessageHandler {
 public:
  // Arbitrarily limit the number of signals to parse in one control step.
  // When changed, adjust comment in
  // intrinsic_control/intrinsic/icon/server/config/realtime_control_config.proto
  static constexpr size_t kMaxPatterns = 32;

  // Maximum number of safety hardware interfaces that can be multiplexed.
  static constexpr size_t kMaxSafetyHardwareInterfaces = 32;

  struct BehaviorOverrideRequestSignal {
    HardwareInterfaceHandle<intrinsic_fbs::DIOStatus>
        digital_input_hardware_interface;
    FixedVector<intrinsic_proto::fieldbus::v1::ValuePattern, kMaxPatterns>
        match_patterns;
  };

  struct Config {
    // There is no explicit upper limit of hardware modules.
    absl::FixedArray<
        HardwareInterfaceHandle<intrinsic_fbs::SafetyStatusMessage>>
        input_hardware_interfaces;
    // Extend when more `BehaviorOverrideRequest` are added to
    // intrinsic_control/intrinsic/icon/proto/v1/types.proto
    std::optional<BehaviorOverrideRequestSignal>
        behavior_override_request_pause_signal;
  };
  // Class is move only.
  SafetyMessageHandler(const SafetyMessageHandler&) = delete;
  SafetyMessageHandler& operator=(const SafetyMessageHandler&) = delete;
  SafetyMessageHandler(SafetyMessageHandler&&) = default;
  SafetyMessageHandler& operator=(SafetyMessageHandler&&) = delete;
  ~SafetyMessageHandler() = default;

  static absl::StatusOr<std::unique_ptr<SafetyMessageHandler>> Create(
      const intrinsic_proto::icon::SafetyMessageHandlerConfig& config,
      const Context& context) INTRINSIC_NON_REALTIME_ONLY;

  // Reads the configured safety interfaces.
  // Multiplexes safety_status_messages.
  // Updates "exported" safety interface.
  RealtimeStatus Update() INTRINSIC_CHECK_REALTIME_SAFE;

  // Returns a list of BehaviorOverrideRequests that can be triggered by this
  // SafetyMessageHandler.
  std::vector<intrinsic_proto::icon::v1::BehaviorOverrideRequest>
  GetConfiguredBehaviorOverrides() const INTRINSIC_CHECK_REALTIME_SAFE;

  // Returns the current merged safety status. Call `Update` to read and update
  // all safety_status_messages.
  const intrinsic_fbs::SafetyStatusMessage& SafetyStatusHandle() const
      INTRINSIC_CHECK_REALTIME_SAFE {
    return *output_safety_status_;
  }

 private:
  explicit SafetyMessageHandler(Config config) INTRINSIC_NON_REALTIME_ONLY;

  Config config_;

  flatbuffers::DetachedBuffer output_safety_status_buffer_;
  intrinsic_fbs::SafetyStatusMessage* output_safety_status_;
};
}  // namespace intrinsic::icon
#endif  // INTRINSIC_ICON_CONTROL_RTCL_SAFETY_MESSAGE_HANDLER_H_
