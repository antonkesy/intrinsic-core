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

#include "intrinsic/icon/control/parts/feature_interfaces/joint_acceleration_state.h"

#include <utility>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/hal/interfaces/joint_state.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

absl::StatusOr<JointAccelerationStateFeature>
JointAccelerationStateFeature::Create(
    JointAccelerationStateHardwareInterface
        joint_acceleration_state_hardware_interface) {
  if (*joint_acceleration_state_hardware_interface == nullptr) {
    return absl::InvalidArgumentError(
        "Hardware interface handle for JointAccelerationStateFeature is not "
        "initialized.");
  }

  JointStateA sensed_acceleration;
  INTR_RETURN_IF_ERROR(sensed_acceleration.SetSize(
      joint_acceleration_state_hardware_interface->acceleration()->size()));

  return JointAccelerationStateFeature(
      std::move(joint_acceleration_state_hardware_interface),
      std::move(sensed_acceleration));
}

JointAccelerationStateFeature::JointAccelerationStateFeature(
    JointAccelerationStateHardwareInterface
        joint_acceleration_state_hardware_interface,
    JointStateA sensed_acceleration)
    : joint_acceleration_state_hardware_interface_(
          std::move(joint_acceleration_state_hardware_interface)) {
  CHECK_OK(sensed_acceleration_.SetSize(
      joint_acceleration_state_hardware_interface_->acceleration()->size()));
}

RealtimeStatus JointAccelerationStateFeature::ReadStatus(
    RealtimePartInterface::ReadStatusParameters params) {
  for (int i = 0; i < sensed_acceleration_.size(); ++i) {
    sensed_acceleration_.acceleration[i] =
        joint_acceleration_state_hardware_interface_->acceleration()->Get(i);
  }
  return OkStatus();
}

JointStateA JointAccelerationStateFeature::GetAccelerationEstimate() const {
  return sensed_acceleration_;
}

}  // namespace intrinsic::icon
