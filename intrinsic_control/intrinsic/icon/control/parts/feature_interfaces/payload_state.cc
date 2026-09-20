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

#include "intrinsic/icon/control/parts/feature_interfaces/payload_state.h"

#include <optional>
#include <utility>

#include "absl/status/statusor.h"
#include "intrinsic/icon/control/parts/payload_property.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/control/parts/realtime_robot_payload.h"
#include "intrinsic/icon/hal/interfaces/robot_payload_utils.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/world/robot_payload/robot_payload_base.h"

namespace intrinsic::icon {

absl::StatusOr<PayloadStateFeature> PayloadStateFeature::Create(
    PayloadStateHardwareInterface payload_state_hardware_interface,
    const PayloadProperty& payload_property) {
  return PayloadStateFeature(std::move(payload_state_hardware_interface),
                             payload_property);
}

PayloadStateFeature::PayloadStateFeature(
    PayloadStateHardwareInterface payload_state_hardware_interface,
    const PayloadProperty& payload_property)
    : payload_state_hardware_interface_(
          std::move(payload_state_hardware_interface)),
      payload_property_(payload_property) {}

RealtimeStatus PayloadStateFeature::ReadStatus(
    RealtimePartInterface::ReadStatusParameters params) {
  std::optional<RobotPayloadBase> payload;
  INTRINSIC_RT_RETURN_IF_ERROR(
      CopyTo(*payload_state_hardware_interface_->full_payload(), payload));
  INTRINSIC_RT_RETURN_IF_ERROR(
      payload_property_.Write(params.part_properties, payload));
  active_payload_ = std::optional<RealtimeRobotPayload>(payload);
  return OkStatus();
}

RealtimeStatus PayloadStateFeature::ApplyCommand(
    RealtimePartInterface::ApplyCommandParameters params) {
  return OkStatus();
}

std::optional<RealtimeRobotPayload> PayloadStateFeature::GetActivePayload()
    const {
  return active_payload_;
}

}  // namespace intrinsic::icon
