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

#include "intrinsic/icon/control/parts/feature_interfaces/joint_position_state.h"

#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/hal/interfaces/joint_state.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

absl::StatusOr<JointPositionStateFeature> JointPositionStateFeature::Create(
    JointPositionStateHardwareInterface
        joint_position_state_hardware_interface) {
  if (*joint_position_state_hardware_interface == nullptr) {
    return absl::InvalidArgumentError(
        "Hardware interface handle for JointPositionStateFeature is not "
        "initialized.");
  }

  JointStateP sensed_position;
  INTR_RETURN_IF_ERROR(sensed_position.SetSize(
      joint_position_state_hardware_interface->position()->size()));

  return JointPositionStateFeature(
      std::move(joint_position_state_hardware_interface),
      std::move(sensed_position));
}

JointPositionStateFeature::JointPositionStateFeature(
    JointPositionStateHardwareInterface joint_position_state_hardware_interface,
    JointStateP sensed_position)
    : joint_position_state_hardware_interface_(
          std::move(joint_position_state_hardware_interface)),
      sensed_position_(std::move(sensed_position)) {}

RealtimeStatus JointPositionStateFeature::ReadStatus(
    RealtimePartInterface::ReadStatusParameters params) {
  for (int i = 0; i < sensed_position_.size(); ++i) {
    sensed_position_.position[i] =
        joint_position_state_hardware_interface_->position()->Get(i);
  }
  return OkStatus();
}

JointStateP JointPositionStateFeature::GetSensedPosition() const {
  return sensed_position_;
}

}  // namespace intrinsic::icon
