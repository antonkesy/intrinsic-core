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

#include "intrinsic/icon/control/parts/feature_interfaces/joint_torque_command.h"

#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/flatbuffers/transform_view.h"
#include "intrinsic/icon/hal/interfaces/joint_command.fbs.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"

namespace intrinsic::icon {

absl::StatusOr<JointTorqueCommandFeature> JointTorqueCommandFeature::Create(
    JointTorqueCommandHardwareInterface
        joint_torque_command_hardware_interface) {
  if (*joint_torque_command_hardware_interface == nullptr) {
    return absl::InvalidArgumentError(
        "Hardware interface handle for JointTorqueCommandFeature is not "
        "initialized.");
  }
  eigenmath::VectorNd previous_torque_setpoints =
      ::intrinsic_fbs::ViewAs<::intrinsic::eigenmath::VectorNd>(
          joint_torque_command_hardware_interface->torque());
  return JointTorqueCommandFeature(
      std::move(joint_torque_command_hardware_interface),
      std::move(previous_torque_setpoints));
}

JointTorqueCommandFeature::JointTorqueCommandFeature(
    JointTorqueCommandHardwareInterface joint_torque_command_hardware_interface,
    eigenmath::VectorNd previous_torque_setpoints)
    : joint_torque_command_hardware_interface_(
          std::move(joint_torque_command_hardware_interface)),
      previous_torque_setpoints_(std::move(previous_torque_setpoints)) {}

RealtimeStatus JointTorqueCommandFeature::SetTorqueSetpoints(
    const eigenmath::VectorNd& setpoints) {
  if (setpoints.size() !=
      joint_torque_command_hardware_interface_->torque()->size()) {
    return icon::InvalidArgumentError(
        "Invalid setpoints: Wrong number of DoFs.");
  }

  for (int i = 0; i < setpoints.size(); ++i) {
    joint_torque_command_hardware_interface_->mutable_torque()->Mutate(
        i, setpoints[i]);
  }
  joint_torque_command_hardware_interface_.UpdatedAt(Clock::now());
  previous_torque_setpoints_ = setpoints;
  return OkStatus();
}

RealtimeStatus JointTorqueCommandFeature::ReadStatus(
    RealtimePartInterface::ReadStatusParameters params) {
  previous_torque_setpoints_ =
      ::intrinsic_fbs::ViewAs<::intrinsic::eigenmath::VectorNd>(
          joint_torque_command_hardware_interface_->torque());
  return OkStatus();
}

eigenmath::VectorNd JointTorqueCommandFeature::PreviousTorqueSetpoints() const {
  return previous_torque_setpoints_;
}

}  // namespace intrinsic::icon
