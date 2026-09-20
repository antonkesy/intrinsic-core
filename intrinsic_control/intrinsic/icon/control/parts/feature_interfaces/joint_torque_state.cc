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

#include "intrinsic/icon/control/parts/feature_interfaces/joint_torque_state.h"

#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/hal/interfaces/joint_state.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

absl::StatusOr<JointTorqueStateFeature> JointTorqueStateFeature::Create(
    JointTorqueStateHardwareInterface joint_torque_state_hardware_interface) {
  if (*joint_torque_state_hardware_interface == nullptr) {
    return absl::InvalidArgumentError(
        "Hardware interface handle for JointTorqueStateFeature is not "
        "initialized.");
  }

  JointStateT sensed_torque;
  INTR_RETURN_IF_ERROR(sensed_torque.SetSize(
      joint_torque_state_hardware_interface->torque()->size()));

  return JointTorqueStateFeature(
      std::move(joint_torque_state_hardware_interface),
      std::move(sensed_torque));
}

JointTorqueStateFeature::JointTorqueStateFeature(
    JointTorqueStateHardwareInterface joint_torque_state_hardware_interface,
    JointStateT sensed_torque)
    : joint_torque_state_hardware_interface_(
          std::move(joint_torque_state_hardware_interface)),
      sensed_torque_(std::move(sensed_torque)) {}

RealtimeStatus JointTorqueStateFeature::ReadStatus(
    RealtimePartInterface::ReadStatusParameters params) {
  for (int i = 0; i < sensed_torque_.size(); ++i) {
    sensed_torque_.torque[i] =
        joint_torque_state_hardware_interface_->torque()->Get(i);
  }
  return OkStatus();
}

JointStateT JointTorqueStateFeature::GetSensedTorque() const {
  return sensed_torque_;
}

}  // namespace intrinsic::icon
