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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_PAYLOAD_STATE_H_
#define INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_PAYLOAD_STATE_H_

#include <optional>

#include "absl/status/statusor.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interfaces/hal_feature_interface_base.h"
#include "intrinsic/icon/control/parts/payload_property.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/control/parts/realtime_robot_payload.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/interfaces/payload_state.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"

namespace intrinsic::icon {

// The PayloadStateFeature is a feature interface that reads the active payload
// from the hardware module and provides it to actions and as a part property.
//
// The hardware module should set the payload in the "Enabling" state.
class PayloadStateFeature : public HalFeatureInterfaceBase,
                            public PayloadState {
  using PayloadStateHardwareInterface =
      MutableHardwareInterfaceHandle<intrinsic_fbs::PayloadState>;

 public:
  // Creates a PayloadStateFeature. Requires a `PayloadProperty` to access the
  // payload in the part properties.
  static absl::StatusOr<PayloadStateFeature> Create(
      PayloadStateHardwareInterface payload_state_hardware_interface,
      const PayloadProperty& payload_property);

  PayloadStateFeature(const PayloadStateFeature&) = delete;
  PayloadStateFeature& operator=(const PayloadStateFeature&) = delete;
  PayloadStateFeature(PayloadStateFeature&& other) = default;
  PayloadStateFeature& operator=(PayloadStateFeature&& other) = delete;
  ~PayloadStateFeature() override = default;

  RealtimeStatus ReadStatus(
      RealtimePartInterface::ReadStatusParameters params) override;

  RealtimeStatus ApplyCommand(
      RealtimePartInterface::ApplyCommandParameters params) override;

  std::optional<RealtimeRobotPayload> GetActivePayload() const override;

 private:
  PayloadStateFeature(
      PayloadStateHardwareInterface payload_state_hardware_interface,
      const PayloadProperty& payload_property);
  PayloadStateHardwareInterface payload_state_hardware_interface_;
  const PayloadProperty payload_property_;
  std::optional<RealtimeRobotPayload> active_payload_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_PAYLOAD_STATE_H_
