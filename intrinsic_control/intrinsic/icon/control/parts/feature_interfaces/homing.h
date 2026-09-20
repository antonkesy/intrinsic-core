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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_HOMING_H_
#define INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_HOMING_H_

#include <cstdint>
#include <string>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interfaces/hal_feature_interface_base.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/interfaces/electrical_motor.fbs.h"
#include "intrinsic/icon/utils/fixed_string.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

// Implementation of the Homing FeatureInterface.
//
// This class takes care of sending and receiving homing related commands and
// statuses to and from a hardware module.
class HomingFeature : public HalFeatureInterfaceBase, public Homing {
 public:
  // Hardware interface for sending a homing command.
  using HomingCommandHardwareInterface =
      MutableHardwareInterfaceHandle<::intrinsic_fbs::HomeCommand>;
  // Hardware interface for reading the homing status.
  using HomingStatusHardwareInterface =
      HardwareInterfaceHandle<::intrinsic_fbs::HomingStatus>;

  // Default homing method, which is used if no homing method is specified.
  static constexpr int8_t kNoHomingMethod = 0;

  // Struct containing all required hardware interfaces for homing.
  struct HomingInterfaces {
    HomingCommandHardwareInterface command;
    HomingStatusHardwareInterface status;
  };

  // Creates a HomingFeature.
  //
  // Args:
  //   `homing_interfaces`: A map of drive names to homing interfaces.
  static absl::StatusOr<HomingFeature> Create(
      absl::flat_hash_map<std::string, HomingInterfaces>&& homing_interfaces);

  // Commands the drive with `drive_name` to start homing. The parameters
  // `homing_method`, `search_speed`, `creep_speed`, `acceleration` and `offset`
  // are used to configure the homing motion.
  //
  // `drive_name`: The name of the drive to command for homing.
  // `homing_method`: The homing method to use. There are a few standard
  //                  methods, but the drive manufacturer can define custom
  //                  methods as well (then often using negative values).
  // `search_speed`: The speed at which the drive will search a switch point.
  //                 For the unit please see the manufacturer's documentation.
  // `creep_speed`: The speed at which the drive will approach the zero point.
  //                For the unit please see the manufacturer's documentation.
  // `acceleration`: The acceleration at which the drive will approach the zero
  //                 point. For the unit please see the manufacturer's
  //                 documentation.
  // `offset`: The offset that'll be applied to the new home position after
  //           homing. For the unit please see the manufacturer's documentation.
  //
  // Returns:
  //   - `OkStatus` if the homing command was successfully sent.
  //   - `InternalError` if the homing command could not be sent.
  RealtimeStatus CommandHoming(absl::string_view drive_name,
                               int8_t homing_method, double search_speed,
                               double creep_speed, double acceleration,
                               double offset) override;

  // Returns true if the drive with `drive_name` is currently homing.
  //
  // `drive_name`: The name of the drive to check.
  bool IsHoming(absl::string_view drive_name) const override;

  // Returns true if the drive with `drive_name` is done homing.
  //
  // `drive_name`: The name of the drive to check.
  //
  // Returns:
  //   - `true` if the homing is done.
  //   - `false` if the homing is not done.
  //   - `InternalError` if the status of the homing could not be determined.
  RealtimeStatusOr<bool> IsHomingDone(
      absl::string_view drive_name) const override;

  // Reads the current status of the hardware interfaces.
  RealtimeStatus ReadStatus(
      RealtimePartInterface::ReadStatusParameters params) override;

 private:
  // Private constructor.
  explicit HomingFeature(
      absl::flat_hash_map<std::string, HomingInterfaces>&& homing_interfaces);

  // Map of drive names to homing interfaces.
  absl::flat_hash_map<std::string, HomingInterfaces> homing_interfaces_;
  // A fixed string that holds all configured drive names.
  const FixedString<RealtimeStatus::kMaxMessageLength> drive_names_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_HOMING_H_
