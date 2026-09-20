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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_HAL_LINEAR_GRIPPER_PART_HAL_LINEAR_GRIPPER_PART_H_
#define INTRINSIC_ICON_CONTROL_PARTS_HAL_LINEAR_GRIPPER_PART_HAL_LINEAR_GRIPPER_PART_H_

#include <limits>
#include <memory>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "intrinsic/icon/control/parts/feature_interface_registry.h"
#include "intrinsic/icon/control/parts/feature_interfaces/linear_gripper.h"
#include "intrinsic/icon/control/parts/hal/linear_gripper_part/hal_linear_gripper_part_config.pb.h"
#include "intrinsic/icon/control/parts/hal_realtime_part_base.h"
#include "intrinsic/icon/control/parts/realtime_part_factory_common.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/hardware_module_manager.h"
#include "intrinsic/icon/hal/interfaces/gripper.fbs.h"

namespace intrinsic::icon {

// Provides access to a gripper by connecting to a subordinate that exposes
// the GripperCommand and GripperStatus messages.
class HalLinearGripperPart final : public HalRealtimePartBase {
 public:
  static constexpr char kPartTypeName[] = "HalLinearGripperPart";

  struct PartConfig {
    double min_width = 0.0;
    double max_width = std::numeric_limits<double>::max();

    double default_force = 0.0;
    double min_force = 0.0;
    double max_force = std::numeric_limits<double>::max();

    double default_speed = 0.0;
    double min_speed = 0.0;
    double max_speed = std::numeric_limits<double>::max();

    double default_grasp_width = 0;
    double default_release_width = 0;
  };

  static absl::StatusOr<PartPtrAndGenericConfig> FromProto(
      PartFactoryContext context,
      const intrinsic_proto::icon::HalLinearGripperPartConfig& config);

  static absl::StatusOr<std::unique_ptr<HalLinearGripperPart>>
  CreateFromHandles(
      HardwareModuleManager* manager,
      MutableHardwareInterfaceHandle<intrinsic_fbs::GripperCommand>

          command,
      HardwareInterfaceHandle<intrinsic_fbs::GripperStatus> status,
      const intrinsic_proto::icon::HalLinearGripperPartConfig& config);

 private:
  explicit HalLinearGripperPart(HardwareModuleManager* manager);
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_HAL_LINEAR_GRIPPER_PART_HAL_LINEAR_GRIPPER_PART_H_
