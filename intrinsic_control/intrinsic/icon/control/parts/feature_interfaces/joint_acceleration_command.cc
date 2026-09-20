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

#include "intrinsic/icon/control/parts/feature_interfaces/joint_acceleration_command.h"

#include <optional>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/control/joint_acceleration_command.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/flatbuffers/transform_view.h"
#include "intrinsic/icon/hal/interfaces/joint_command.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"

namespace intrinsic::icon {

absl::StatusOr<JointAccelerationCommandFeature>
JointAccelerationCommandFeature::Create(
    JointAccelerationAndTorqueCommandHardwareInterface hardware_interface) {
  if (*hardware_interface == nullptr) {
    return absl::InvalidArgumentError(
        "Hardware interface handle for JointAccelerationCommandFeature is not "
        "initialized.");
  }
  eigenmath::VectorNd previous_acceleration_setpoints =
      ::intrinsic_fbs::ViewAs<::intrinsic::eigenmath::VectorNd>(
          hardware_interface->acceleration());
  eigenmath::VectorNd previous_torque_setpoints =
      ::intrinsic_fbs::ViewAs<::intrinsic::eigenmath::VectorNd>(
          hardware_interface->torque());
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      JointAccelerationCommand previous_setpoints,
      JointAccelerationCommand::Create(previous_acceleration_setpoints,
                                       previous_torque_setpoints));
  return JointAccelerationCommandFeature(std::move(hardware_interface),
                                         std::move(previous_setpoints));
}

JointAccelerationCommandFeature::JointAccelerationCommandFeature(
    JointAccelerationAndTorqueCommandHardwareInterface hardware_interface,
    JointAccelerationCommand previous_setpoints)
    : hardware_interface_(std::move(hardware_interface)),
      previous_setpoints_(std::move(previous_setpoints)) {}

RealtimeStatus JointAccelerationCommandFeature::ReadStatus(
    RealtimePartInterface::ReadStatusParameters params) {
  eigenmath::VectorNd previous_acceleration_setpoints =
      ::intrinsic_fbs::ViewAs<::intrinsic::eigenmath::VectorNd>(
          hardware_interface_->acceleration());
  eigenmath::VectorNd previous_torque_setpoints =
      ::intrinsic_fbs::ViewAs<::intrinsic::eigenmath::VectorNd>(
          hardware_interface_->torque());
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      previous_setpoints_,
      JointAccelerationCommand::Create(previous_acceleration_setpoints,
                                       previous_torque_setpoints));
  return OkStatus();
}

RealtimeStatus JointAccelerationCommandFeature::SetAccelerationSetpoints(
    const JointAccelerationCommand& setpoints) {
  if (setpoints.Size() != hardware_interface_->acceleration()->size()) {
    return icon::InvalidArgumentError(
        "Invalid setpoints: Wrong number of DoFs.");
  }

  for (int i = 0; i < setpoints.Size(); ++i) {
    hardware_interface_->mutable_acceleration()->Mutate(
        i, setpoints.acceleration()[i]);

    if (setpoints.torque().has_value()) {
      hardware_interface_->mutable_torque()->Mutate(
          i, setpoints.torque().value()[i]);
    } else {
      hardware_interface_->mutable_torque()->Mutate(i, 0.0);
    }
  }

  hardware_interface_.UpdatedAt(Clock::now());
  return OkStatus();
}

JointAccelerationCommand
JointAccelerationCommandFeature::PreviousAccelerationSetpoints() const {
  return previous_setpoints_;
}

}  // namespace intrinsic::icon
