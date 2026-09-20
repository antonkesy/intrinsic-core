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

#include "intrinsic/icon/control/parts/feature_interfaces/linear_gripper.h"

#include <optional>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/hal/linear_gripper_part/hal_linear_gripper_part_config.pb.h"
#include "intrinsic/icon/utils/realtime_status.h"

namespace intrinsic::icon {

absl::StatusOr<LinearGripperFeature> LinearGripperFeature::Create(
    LinearGripperStateHardwareInterface linear_gripper_state_hardware_interface,
    LinearGripperCommandHardwareInterface
        linear_gripper_command_hardware_interface,
    const intrinsic_proto::icon::HalLinearGripperPartConfig&
        linear_gripper_config) {
  if (*linear_gripper_state_hardware_interface == nullptr) {
    return absl::InvalidArgumentError(
        "Hardware interface handle for LinearGripperStateFeature is not "
        "initialized.");
  }
  if (*linear_gripper_command_hardware_interface == nullptr) {
    return absl::InvalidArgumentError(
        "Hardware interface handle for LinearGripperCommandFeature is not "
        "initialized.");
  }

  if (!linear_gripper_config.has_default_grasp_width()) {
    return absl::FailedPreconditionError("Must provide default_grasp_width.");
  }
  if (!linear_gripper_config.has_default_release_width()) {
    return absl::FailedPreconditionError("Must provide default_release_width.");
  }

  // Preconditions for default_grasp_width and default_release_width:
  // min_width <= default_grasp_width
  // default_grasp_width < default_release_width
  // default_release_width <= max_width
  if (linear_gripper_config.default_release_width() <=
      linear_gripper_config.default_grasp_width()) {
    return absl::FailedPreconditionError(absl::StrCat(
        "default_grasp_width (", linear_gripper_config.default_grasp_width(),
        ") must to be smaller than default_release_width (",
        linear_gripper_config.default_release_width(),
        "). Otherwise grasping is impossible."));
  }
  if (linear_gripper_config.default_grasp_width() <
      linear_gripper_config.min_width()) {
    return absl::FailedPreconditionError(absl::StrCat(
        "default_grasp_width (", linear_gripper_config.default_grasp_width(),
        ") is smaller than min_width (", linear_gripper_config.min_width(),
        ")."));
  }
  if (linear_gripper_config.default_release_width() >
      linear_gripper_config.max_width()) {
    return absl::FailedPreconditionError(
        absl::StrCat("default_release_width (",
                     linear_gripper_config.default_release_width(),
                     ") is bigger than max_width (",
                     linear_gripper_config.max_width(), ")."));
  }

  return LinearGripperFeature(
      std::move(linear_gripper_state_hardware_interface),
      std::move(linear_gripper_command_hardware_interface),
      linear_gripper_config);
}

LinearGripperFeature::LinearGripperFeature(
    LinearGripperStateHardwareInterface linear_gripper_state_hardware_interface,
    LinearGripperCommandHardwareInterface
        linear_gripper_command_hardware_interface,
    const intrinsic_proto::icon::HalLinearGripperPartConfig& gripper_config)
    : state_interface_(std::move(linear_gripper_state_hardware_interface)),
      command_interface_(std::move(linear_gripper_command_hardware_interface)),
      gripper_config_(gripper_config) {}

RealtimeStatus LinearGripperFeature::SetGripperCommand(
    double width, std::optional<double> force, std::optional<double> speed) {
  if (state_interface_->in_error()) {
    return FailedPreconditionError(
        "LinearGripper is in error state, not accepting commands.");
  }
  if (width < gripper_config_.min_width() ||
      width > gripper_config_.max_width()) {
    return OutOfRangeError(RealtimeStatus::StrCat(
        "Commanded width ", width, "is outside limits (",
        gripper_config_.min_width(), ",", gripper_config_.max_width(), "."));
  }
  if (!force.has_value()) {
    force = gripper_config_.default_force();
  }
  if (*force < gripper_config_.min_force() ||
      *force > gripper_config_.max_force()) {
    return OutOfRangeError(RealtimeStatus::StrCat(
        "Commanded force ", *force, "is outside limits (",
        gripper_config_.min_force(), ",", gripper_config_.max_force(), "."));
  }
  if (!speed.has_value()) {
    speed = gripper_config_.default_speed();
  }
  if (*speed < gripper_config_.min_speed() ||
      *speed > gripper_config_.max_speed()) {
    return OutOfRangeError(RealtimeStatus::StrCat(
        "Commanded speed ", *speed, "is outside limits (",
        gripper_config_.min_speed(), ",", gripper_config_.max_speed(), "."));
  }
  command_interface_->mutate_width(width);
  command_interface_->mutate_force(*force);
  command_interface_->mutate_speed(*speed);
  return OkStatus();
}

double LinearGripperFeature::GetGripperWidth() const {
  return state_interface_->width();
}

RealtimeStatus LinearGripperFeature::SetGripperCommand(
    const SimpleGripper::GripperCommand& command) {
  switch (command) {
    case SimpleGripper::GripperCommand::kGrasp: {
      return SetGripperCommand(/*width=*/gripper_config_.default_grasp_width(),
                               /*force=*/gripper_config_.default_force(),
                               /*speed=*/gripper_config_.default_speed());
    }
    case SimpleGripper::GripperCommand::kRelease: {
      return SetGripperCommand(
          /*width=*/gripper_config_.default_release_width(),
          /*force=*/gripper_config_.default_force(),
          /*speed=*/gripper_config_.default_speed());
    }
    default:
      return UnknownError(RealtimeStatus::StrCat(
          "Unknown SimpleGripper Command received (hex: ", absl::Hex(command),
          ")."));
  }
  return OkStatus();
}

SimpleGripper::GripperState LinearGripperFeature::GetGripperState() const {
  const double current_width = GetGripperWidth();
  if (current_width <= gripper_config_.default_grasp_width()) {
    return GripperState::kGrasped;
  }
  if (current_width >= gripper_config_.default_release_width()) {
    return GripperState::kReleased;
  }
  return GripperState::kUnknown;
}

}  // namespace intrinsic::icon
