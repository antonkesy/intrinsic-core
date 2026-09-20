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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_HAL_LASER_TRACKER_PART_HAL_LASER_TRACKER_PART_H_
#define INTRINSIC_ICON_CONTROL_PARTS_HAL_LASER_TRACKER_PART_HAL_LASER_TRACKER_PART_H_

#include "absl/status/statusor.h"
#include "intrinsic/icon/control/parts/hal/laser_tracker_part/hal_laser_tracker_part_config.pb.h"
#include "intrinsic/icon/control/parts/hal_realtime_part_base.h"
#include "intrinsic/icon/control/parts/realtime_part_factory_common.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/hardware_module_manager.h"
#include "intrinsic/icon/hal/interfaces/laser_tracker_state.fbs.h"

namespace intrinsic::icon {

// A realtime part that exposes the laser tracker state.
class HalLaserTrackerPart final : public HalRealtimePartBase {
 public:
  using PositionStatusHardwareInterface =
      intrinsic::icon::HardwareInterfaceHandle<
          intrinsic_fbs::LaserTrackerState>;

  static constexpr char kPartTypeName[] = "HalLaserTrackerPart";

  // Builds a HalLaserTrackerPart from the given configuration proto.
  static absl::StatusOr<PartPtrAndGenericConfig> FromProto(
      PartFactoryContext context,
      const intrinsic_proto::icon::HalLaserTrackerPartConfig& config);

  explicit HalLaserTrackerPart(HardwareModuleManager* manager);
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_HAL_LASER_TRACKER_PART_HAL_LASER_TRACKER_PART_H_
