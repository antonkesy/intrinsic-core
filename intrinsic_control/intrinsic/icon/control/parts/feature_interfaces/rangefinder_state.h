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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_RANGEFINDER_STATE_H_
#define INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_RANGEFINDER_STATE_H_

#include "absl/status/statusor.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interfaces/hal_feature_interface_base.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/interfaces/rangefinder.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic::icon {

class RangefinderStateFeature : public HalFeatureInterfaceBase,
                                public RangeFinder {
  using RangefinderHardwareInterface =
      HardwareInterfaceHandle<intrinsic_fbs::RangeFinderStatus>;

 public:
  static absl::StatusOr<RangefinderStateFeature> Create(
      RangefinderHardwareInterface rangefinder_hardware_interface,
      const Pose3d& flange_t_sensor);

  RangefinderStateFeature(const RangefinderStateFeature&) = delete;
  RangefinderStateFeature& operator=(const RangefinderStateFeature&) = delete;
  RangefinderStateFeature(RangefinderStateFeature&& other) = default;
  RangefinderStateFeature& operator=(RangefinderStateFeature&& other) = default;
  ~RangefinderStateFeature() override = default;

  RealtimeStatus ReadStatus(
      RealtimePartInterface::ReadStatusParameters params) override;

  double GetSensedDistance() const override { return sensed_distance_; };
  bool IsMeasurementValid() const override { return measurement_valid_; }
  Pose3d GetPoseInTCPFrame() const override { return flange_t_sensor_; }

 private:
  explicit RangefinderStateFeature(
      RangefinderHardwareInterface rangefinder_hardware_interface,
      const Pose3d& flange_t_sensor);

  RangefinderHardwareInterface rangefinder_hardware_interface_;

  double sensed_distance_ = 0.0;
  bool measurement_valid_ = false;
  Pose3d flange_t_sensor_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_RANGEFINDER_STATE_H_
