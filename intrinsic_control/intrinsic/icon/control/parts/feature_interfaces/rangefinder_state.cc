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

#include "intrinsic/icon/control/parts/feature_interfaces/rangefinder_state.h"

#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/hal/interfaces/rangefinder.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/math/pose3.h"

namespace intrinsic::icon {

using ::intrinsic_fbs::RangeFinderError;

absl::StatusOr<RangefinderStateFeature> RangefinderStateFeature::Create(
    RangefinderHardwareInterface rangefinder_hardware_interface,
    const Pose3d& flange_t_sensor) {
  if (*rangefinder_hardware_interface == nullptr) {
    return absl::InvalidArgumentError(
        "Hardware interface handle for RangefinderStateFeature is not "
        "initialized.");
  }
  return RangefinderStateFeature(std::move(rangefinder_hardware_interface),
                                 flange_t_sensor);
}

RangefinderStateFeature::RangefinderStateFeature(
    RangefinderHardwareInterface rangefinder_hardware_interface,
    const Pose3d& flange_t_sensor)
    : rangefinder_hardware_interface_(
          std::move(rangefinder_hardware_interface)),
      flange_t_sensor_(flange_t_sensor) {}

RealtimeStatus RangefinderStateFeature::ReadStatus(
    RealtimePartInterface::ReadStatusParameters params) {
  sensed_distance_ = rangefinder_hardware_interface_->distance();

  measurement_valid_ =
      rangefinder_hardware_interface_->error() == RangeFinderError::NoError;
  return OkStatus();
}

}  // namespace intrinsic::icon
