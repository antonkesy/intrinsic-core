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

#include "intrinsic/icon/control/parts/feature_interfaces/handguiding_command.h"

#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"

namespace intrinsic::icon {

absl::StatusOr<HandGuidingCommandFeature> HandGuidingCommandFeature::Create(
    HandGuidingCommandHardwareInterface&&
        hand_guiding_command_hardware_interface) {
  if (*hand_guiding_command_hardware_interface == nullptr) {
    return absl::InvalidArgumentError(
        "Hardware interface handle for HandGuidingCommand is not initialized.");
  }

  return HandGuidingCommandFeature(
      std::move(hand_guiding_command_hardware_interface));
}

RealtimeStatus HandGuidingCommandFeature::CommandHandGuiding() {
  hand_guiding_command_hardware_interface_.UpdatedAt(Clock::Now());
  return icon::OkStatus();
}

HandGuidingCommandFeature::HandGuidingCommandFeature(
    HandGuidingCommandHardwareInterface hand_guiding_command_hardware_interface)
    : hand_guiding_command_hardware_interface_(
          std::move(hand_guiding_command_hardware_interface)) {}

}  // namespace intrinsic::icon
