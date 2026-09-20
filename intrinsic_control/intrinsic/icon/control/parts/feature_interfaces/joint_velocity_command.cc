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

#include "intrinsic/icon/control/parts/feature_interfaces/joint_velocity_command.h"

#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/icon/hal/interfaces/joint_command.fbs.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"

namespace intrinsic::icon {

absl::StatusOr<JointVelocityCommandFeature> JointVelocityCommandFeature::Create(
    JointVelocityCommandHardwareInterface
        joint_velocity_command_hardware_interface) {
  if (*joint_velocity_command_hardware_interface == nullptr) {
    return absl::InvalidArgumentError(
        "Hardware interface handle for JointVelocityCommandFeature is not "
        "initialized.");
  }

  return JointVelocityCommandFeature(
      std::move(joint_velocity_command_hardware_interface));
}

JointVelocityCommandFeature::JointVelocityCommandFeature(
    JointVelocityCommandHardwareInterface
        joint_velocity_command_hardware_interface)
    : joint_velocity_command_hardware_interface_(
          std::move(joint_velocity_command_hardware_interface)) {}

RealtimeStatus JointVelocityCommandFeature::SetVelocitySetpoints(
    const eigenmath::VectorNd& setpoints) {
  if (setpoints.size() !=
      joint_velocity_command_hardware_interface_->velocity()->size()) {
    return icon::InvalidArgumentError(
        "Invalid setpoints: Wrong number of DoFs.");
  }

  for (int i = 0; i < setpoints.size(); ++i) {
    joint_velocity_command_hardware_interface_->mutable_velocity()->Mutate(
        i, setpoints[i]);
  }

  joint_velocity_command_hardware_interface_.UpdatedAt(Clock::now());

  return OkStatus();
}

}  // namespace intrinsic::icon
