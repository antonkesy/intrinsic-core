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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_HAL_IMU_PART_HAL_IMU_PART_H_
#define INTRINSIC_ICON_CONTROL_PARTS_HAL_IMU_PART_HAL_IMU_PART_H_

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/control/parts/hal/imu_part/hal_imu_part_config.pb.h"
#include "intrinsic/icon/control/parts/hal_realtime_part_base.h"
#include "intrinsic/icon/control/parts/realtime_part_factory_common.h"
#include "intrinsic/icon/hal/hardware_module_manager.h"
#include "intrinsic/icon/hal/interfaces/hardware_module_state.fbs.h"

namespace intrinsic::icon {

class HalImuPart final : public HalRealtimePartBase {
 public:
  static constexpr char kPartTypeName[] = "HalImuPart";

  // Builds a HalImuPart from the given configuration proto as defined
  // in
  // intrinsic/icon/control/parts/hal/imu_part/hal_imu_part_config.proto.
  //
  // Returns an error if the configuration is invalid, or on parsing errors.
  static absl::StatusOr<PartPtrAndGenericConfig> FromProto(
      PartFactoryContext context,
      const intrinsic_proto::icon::HalImuPartConfig& config);

  HalImuPart(absl::string_view name, HardwareModuleManager* manager);
};
}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_HAL_IMU_PART_HAL_IMU_PART_H_
