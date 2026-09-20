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

#include "intrinsic/icon/control/parts/feature_interfaces/laser_tracker_position_state.h"

#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/flatbuffers/control_types_copy.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic::icon {
absl::StatusOr<LaserTrackerPositionStateFeature>
LaserTrackerPositionStateFeature::Create(
    LaserTrackerStateHardwareInterface
        laser_stracker_state_hardware_interface) {
  if (*laser_stracker_state_hardware_interface == nullptr) {
    return absl::InvalidArgumentError(
        "Hardware interface handle for LaserTrackerPositionStateFeature is not "
        "initialized.");
  }
  return LaserTrackerPositionStateFeature(
      std::move(laser_stracker_state_hardware_interface));
}

LaserTrackerPositionStateFeature::LaserTrackerPositionStateFeature(
    LaserTrackerStateHardwareInterface laser_tracker_state_hardware_interface)
    : state_hardware_interface_(
          std::move(laser_tracker_state_hardware_interface)) {}

RealtimeStatus LaserTrackerPositionStateFeature::ReadStatus(
    RealtimePartInterface::ReadStatusParameters params) {
  CopyTo(&sensed_pose_, *state_hardware_interface_->pose_sensed());
  return OkStatus();
}

Pose3d LaserTrackerPositionStateFeature::GetSensedPose() const {
  return sensed_pose_;
}

}  // namespace intrinsic::icon
