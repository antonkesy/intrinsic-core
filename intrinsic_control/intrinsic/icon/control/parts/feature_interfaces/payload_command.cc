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

#include "intrinsic/icon/control/parts/feature_interfaces/payload_command.h"

#include <optional>
#include <utility>

#include "absl/status/statusor.h"
#include "intrinsic/icon/control/parts/payload_property.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/control/parts/realtime_robot_payload.h"
#include "intrinsic/icon/hal/interfaces/robot_payload_utils.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/world/robot_payload/robot_payload_base.h"

namespace intrinsic::icon {

absl::StatusOr<PayloadCommandFeature> PayloadCommandFeature::Create(
    PayloadCommandHardwareInterface payload_command_hardware_interface,
    const PayloadProperty& payload_property,
    const std::optional<RobotPayloadBase>& initial_payload) {
  INTRINSIC_RT_RETURN_IF_ERROR(intrinsic_fbs::CopyTo(
      initial_payload,
      *payload_command_hardware_interface->mutable_full_payload()));
  return PayloadCommandFeature(std::move(payload_command_hardware_interface),
                               payload_property);
}

PayloadCommandFeature::PayloadCommandFeature(
    PayloadCommandHardwareInterface payload_command_hardware_interface,
    const PayloadProperty& payload_property)
    : payload_command_hardware_interface_(
          std::move(payload_command_hardware_interface)),
      payload_property_(payload_property) {}

RealtimeStatus PayloadCommandFeature::ReadStatus(
    RealtimePartInterface::ReadStatusParameters params) {
  return OkStatus();
}

RealtimeStatus PayloadCommandFeature::ApplyCommand(
    RealtimePartInterface::ApplyCommandParameters params) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(std::optional<RealtimeRobotPayload> payload,
                                payload_property_.Read(params.part_properties));
  INTRINSIC_RT_RETURN_IF_ERROR(intrinsic_fbs::CopyTo(
      payload, *payload_command_hardware_interface_->mutable_full_payload()));

  payload_command_hardware_interface_.UpdatedAt(Clock::now());
  return OkStatus();
}

}  // namespace intrinsic::icon
