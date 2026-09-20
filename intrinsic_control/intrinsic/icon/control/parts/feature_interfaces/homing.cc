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

#include "intrinsic/icon/control/parts/feature_interfaces/homing.h"

#include <cstdint>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "gloop/util/gtl/iterator_adaptors.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interfaces/hal_feature_interface_base.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/flatbuffers/fixed_string.h"
#include "intrinsic/icon/hal/interfaces/electrical_motor.fbs.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/fixed_str_cat.h"
#include "intrinsic/icon/utils/log.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

absl::StatusOr<HomingFeature> HomingFeature::Create(
    absl::flat_hash_map<std::string, HomingInterfaces>&& homing_interfaces) {
  return HomingFeature(std::move(homing_interfaces));
}

RealtimeStatus HomingFeature::CommandHoming(
    absl::string_view drive_name, int8_t homing_method, double search_speed,
    double creep_speed, double acceleration, double offset) {
  if (!homing_interfaces_.contains(drive_name)) {
    return NotFoundError(FixedStrCat<RealtimeStatus::kMaxMessageLength>(
        "Drive name ", drive_name,
        " not found. Expected one of: ", drive_names_));
  }
  INTRINSIC_RT_LOG(INFO) << "Commanding homing for drive " << drive_name
                         << " with method " << homing_method;
  homing_interfaces_.at(drive_name).command->mutate_method(homing_method);
  homing_interfaces_.at(drive_name).command->mutate_search_speed(search_speed);
  homing_interfaces_.at(drive_name).command->mutate_creep_speed(creep_speed);
  homing_interfaces_.at(drive_name).command->mutate_acceleration(acceleration);
  homing_interfaces_.at(drive_name).command->mutate_offset(offset);
  homing_interfaces_.at(drive_name).command.UpdatedAt(Clock::now());
  return OkStatus();
}

bool HomingFeature::IsHoming(absl::string_view drive_name) const {
  if (!homing_interfaces_.contains(drive_name)) {
    return false;
  }
  return homing_interfaces_.at(drive_name).status->state() ==
         intrinsic_fbs::HomingStatusFlag::HomingInProgress;
}

RealtimeStatusOr<bool> HomingFeature::IsHomingDone(
    absl::string_view drive_name) const {
  if (!homing_interfaces_.contains(drive_name)) {
    return icon::InvalidArgumentError(
        RealtimeStatus::StrCat("Drive name ", drive_name, " not found."));
  }
  // Check if there are any errors.
  if (homing_interfaces_.at(drive_name).status->state() ==
      intrinsic_fbs::HomingStatusFlag::HomingError) {
    return AbortedError(RealtimeStatus::StrCat(
        "drive `", drive_name, "` homing: ",
        intrinsic_fbs::StringView(
            &homing_interfaces_.at(drive_name).status->error_message())));
  }

  // Check if the drive reports homing attained.
  return homing_interfaces_.at(drive_name).status->state() ==
         intrinsic_fbs::HomingStatusFlag::HomingAttained;
}

RealtimeStatus HomingFeature::ReadStatus(
    RealtimePartInterface::ReadStatusParameters params) {
  // Set all command interfaces to NONE. They will be overwritten by any running
  // homing action. But if none is running, the homing implementation (inside
  // the hardware module) will know that there's no (more) homing request.
  for (auto& [drive_name, homing_interfaces] : homing_interfaces_) {
    homing_interfaces.command->mutate_method(kNoHomingMethod);
    homing_interfaces.command.UpdatedAt(Clock::now());
  }
  return HalFeatureInterfaceBase::ReadStatus(params);
}

HomingFeature::HomingFeature(
    absl::flat_hash_map<std::string, HomingInterfaces>&& homing_interfaces)
    : Homing(),
      homing_interfaces_(std::move(homing_interfaces)),
      drive_names_(absl::StrJoin(gtl::key_view(homing_interfaces_), ",")) {}

}  // namespace intrinsic::icon
