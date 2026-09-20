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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_LASER_TRACKER_POSITION_STATE_H_
#define INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_LASER_TRACKER_POSITION_STATE_H_

#include "absl/status/statusor.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interfaces/hal_feature_interface_base.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/interfaces/laser_tracker_state.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic::icon {

// Implementation of the LaserTrackerPositionState feature interface.
class LaserTrackerPositionStateFeature : public HalFeatureInterfaceBase,
                                         public CartesianPositionState {
  using LaserTrackerStateHardwareInterface =
      HardwareInterfaceHandle<intrinsic_fbs::LaserTrackerState>;

 public:
  static absl::StatusOr<LaserTrackerPositionStateFeature> Create(
      LaserTrackerStateHardwareInterface
          laser_stracker_state_hardware_interface);

  LaserTrackerPositionStateFeature(const LaserTrackerPositionStateFeature&) =
      delete;
  LaserTrackerPositionStateFeature& operator=(
      const LaserTrackerPositionStateFeature&) = delete;
  LaserTrackerPositionStateFeature(LaserTrackerPositionStateFeature&& other) =
      default;
  LaserTrackerPositionStateFeature& operator=(
      LaserTrackerPositionStateFeature&& other) = default;
  ~LaserTrackerPositionStateFeature() override = default;

  RealtimeStatus ReadStatus(
      RealtimePartInterface::ReadStatusParameters params) override;

  Pose3d GetSensedPose() const override;

 private:
  explicit LaserTrackerPositionStateFeature(
      LaserTrackerStateHardwareInterface
          laser_tracker_state_hardware_interface);

  LaserTrackerStateHardwareInterface state_hardware_interface_;

  Pose3d sensed_pose_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_LASER_TRACKER_POSITION_STATE_H_
