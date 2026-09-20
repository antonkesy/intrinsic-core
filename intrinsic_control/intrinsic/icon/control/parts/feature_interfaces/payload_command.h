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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_PAYLOAD_COMMAND_H_
#define INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_PAYLOAD_COMMAND_H_

#include <optional>

#include "absl/status/statusor.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interfaces/hal_feature_interface_base.h"
#include "intrinsic/icon/control/parts/payload_property.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/interfaces/payload_command.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/world/robot_payload/robot_payload_base.h"

namespace intrinsic::icon {

// The PayloadCommandFeature is a feature interface that allows to set the
// payload of a robot. It reads the payload from the `PayloadProperty` and
// writes it to the `PayloadCommandHardwareInterface` in every cycle.
//
// The hardware module should set the payload in the "Enabling" state.
class PayloadCommandFeature : public HalFeatureInterfaceBase, public Payload {
  using PayloadCommandHardwareInterface =
      MutableHardwareInterfaceHandle<intrinsic_fbs::PayloadCommand>;

 public:
  // Creates a PayloadCommandFeature. Requires a `PayloadProperty` to access the
  // payload in the part properties.
  static absl::StatusOr<PayloadCommandFeature> Create(
      PayloadCommandHardwareInterface payload_command_hardware_interface,
      const PayloadProperty& payload_property,
      const std::optional<RobotPayloadBase>& initial_payload);

  PayloadCommandFeature(const PayloadCommandFeature&) = delete;
  PayloadCommandFeature& operator=(const PayloadCommandFeature&) = delete;
  PayloadCommandFeature(PayloadCommandFeature&& other) = default;
  PayloadCommandFeature& operator=(PayloadCommandFeature&& other) = delete;
  ~PayloadCommandFeature() override = default;

  RealtimeStatus ReadStatus(
      RealtimePartInterface::ReadStatusParameters params) override;

  RealtimeStatus ApplyCommand(
      RealtimePartInterface::ApplyCommandParameters params) override;

 private:
  PayloadCommandFeature(
      PayloadCommandHardwareInterface payload_command_hardware_interface,
      const PayloadProperty& payload_property);
  PayloadCommandHardwareInterface payload_command_hardware_interface_;
  const PayloadProperty payload_property_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_PAYLOAD_COMMAND_H_
