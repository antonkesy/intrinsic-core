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

#include "intrinsic/icon/control/parts/feature_interfaces/control_mode_state.h"

#include <utility>

#include "absl/log/log.h"
#include "absl/status/statusor.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/hal/interfaces/control_mode.fbs.h"
#include "intrinsic/icon/utils/log.h"

namespace intrinsic::icon {

/*static*/
absl::StatusOr<ControlModeStateFeature> ControlModeStateFeature::Create(
    ControlModeStateHardwareInterface control_mode_state_hardware_interface) {
  return ControlModeStateFeature(
      std::move(control_mode_state_hardware_interface));
}

ControlModeExporter::ControlMode
ControlModeStateFeature::GetCurrentControlMode() const {
  switch (control_mode_state_hardware_interface_->status()) {
    case intrinsic_fbs::ControlMode::kCyclicPosition:
      return ControlModeExporter::ControlMode::kCyclicPosition;
    case intrinsic_fbs::ControlMode::kCyclicVelocity:
      return ControlModeExporter::ControlMode::kCyclicVelocity;
    case intrinsic_fbs::ControlMode::kCyclicTorque:
      return ControlModeExporter::ControlMode::kCyclicTorque;
    case intrinsic_fbs::ControlMode::kHandguiding:
      return ControlModeExporter::ControlMode::kHandGuiding;
    case intrinsic_fbs::ControlMode::kUnknown:
      return ControlModeExporter::ControlMode::kUnknown;
  }
  INTRINSIC_RT_LOG(ERROR)
      << "Failed to map control mode: "
      << intrinsic_fbs::EnumNameControlMode(
             control_mode_state_hardware_interface_->status())
      << " Using kUnknown.";
  return ControlModeExporter::ControlMode::kUnknown;
}

ControlModeStateFeature::ControlModeStateFeature(
    ControlModeStateHardwareInterface control_mode_state_hardware_interface)
    : control_mode_state_hardware_interface_(
          std::move(control_mode_state_hardware_interface)) {}

}  // namespace intrinsic::icon
