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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_HANDGUIDING_COMMAND_H_
#define INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_HANDGUIDING_COMMAND_H_

#include "absl/status/statusor.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interfaces/hal_feature_interface_base.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/interfaces/joint_command.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"

namespace intrinsic::icon {

class HandGuidingCommandFeature : public HalFeatureInterfaceBase,
                                  public HandGuiding {
  using HandGuidingCommandHardwareInterface =
      MutableHardwareInterfaceHandle<::intrinsic_fbs::HandGuidingCommand>;

 public:
  static absl::StatusOr<HandGuidingCommandFeature> Create(
      HandGuidingCommandHardwareInterface&&
          hand_guiding_command_hardware_interface);

  HandGuidingCommandFeature(const HandGuidingCommandFeature&) = delete;
  HandGuidingCommandFeature& operator=(const HandGuidingCommandFeature&) =
      delete;
  HandGuidingCommandFeature(HandGuidingCommandFeature&& other) = default;
  HandGuidingCommandFeature& operator=(HandGuidingCommandFeature&& other) =
      default;
  ~HandGuidingCommandFeature() override = default;

  RealtimeStatus CommandHandGuiding() override;

 private:
  explicit HandGuidingCommandFeature(
      HandGuidingCommandHardwareInterface
          hand_guiding_command_hardware_interface);

  HandGuidingCommandHardwareInterface hand_guiding_command_hardware_interface_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_HANDGUIDING_COMMAND_H_
