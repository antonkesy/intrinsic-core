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

#include "intrinsic/icon/control/parts/hal/linear_gripper_part/hal_linear_gripper_part.h"

#include <limits>
#include <memory>
#include <utility>

#include "absl/log/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/control/context.h"
#include "intrinsic/icon/control/parts/feature_interfaces/linear_gripper.h"
#include "intrinsic/icon/control/parts/hal/linear_gripper_part/hal_linear_gripper_part_config.pb.h"
#include "intrinsic/icon/control/parts/hal/v1/hal_part_config.pb.h"
#include "intrinsic/icon/control/parts/hal_realtime_part_base.h"
#include "intrinsic/icon/control/parts/realtime_part_factory_common.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/hardware_interface_traits.h"
#include "intrinsic/icon/hal/hardware_module_manager.h"
#include "intrinsic/icon/hal/interfaces/gripper.fbs.h"
#include "intrinsic/icon/hal/interfaces/gripper_utils.h"
#include "intrinsic/icon/proto/generic_part_config.pb.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

HalLinearGripperPart::HalLinearGripperPart(HardwareModuleManager* manager)
    : HalRealtimePartBase(manager) {}

absl::StatusOr<PartPtrAndGenericConfig> HalLinearGripperPart::FromProto(
    PartFactoryContext context,
    const intrinsic_proto::icon::HalLinearGripperPartConfig& config) {
  INTRINSIC_ASSERT_NON_REALTIME();

  // We apply some defaults on the passed config to make it consistent with the
  // old constructor.
  intrinsic_proto::icon::HalLinearGripperPartConfig config_copy = config;
  if (!config_copy.has_max_width()) {
    config_copy.set_max_width(std::numeric_limits<double>::max());
  }
  if (!config_copy.has_max_force()) {
    config_copy.set_max_force(std::numeric_limits<double>::max());
  }
  if (!config_copy.has_max_speed()) {
    config_copy.set_max_speed(std::numeric_limits<double>::max());
  }

  INTR_ASSIGN_OR_RETURN(
      auto status,
      context.context.GetHardwareInterfaceHandle<intrinsic_fbs::GripperStatus>(
          config_copy.status_interface().module_name(),
          config_copy.status_interface().interface_name()));
  INTR_ASSIGN_OR_RETURN(
      auto command,
      context.context
          .GetMutableHardwareInterfaceHandle<intrinsic_fbs::GripperCommand>(
              config_copy.command_interface().module_name(),
              config_copy.command_interface().interface_name()));

  if (!config_copy.has_default_grasp_width()) {
    return absl::FailedPreconditionError("Must provide default_grasp_width.");
  }
  if (!config_copy.has_default_release_width()) {
    return absl::FailedPreconditionError("Must provide default_release_width.");
  }

  // TODO(b/291207170): Add additional tests for consistency between parameters.
  PartConfig part_config;
  part_config.min_width = config_copy.min_width();
  part_config.max_width = config_copy.max_width();
  part_config.default_force = config_copy.default_force();
  part_config.min_force = config_copy.min_force();
  part_config.max_force = config_copy.max_force();
  part_config.default_speed = config_copy.default_speed();
  part_config.min_speed = config_copy.min_speed();
  part_config.max_speed = config_copy.max_speed();
  part_config.default_grasp_width = config_copy.default_grasp_width();
  part_config.default_release_width = config_copy.default_release_width();

  INTR_ASSIGN_OR_RETURN(
      auto part_ptr,
      CreateFromHandles(context.context.GetHardwareModuleManager(),
                        std::move(command), std::move(status), config_copy));
  INTR_RETURN_IF_ERROR(part_ptr->AddHardwareModule(
      config_copy.status_interface().module_name()));
  INTR_RETURN_IF_ERROR(part_ptr->AddHardwareModule(
      config_copy.command_interface().module_name()));
  // Fill the GenericPartConfig proto.
  intrinsic_proto::icon::GenericPartConfig config_proto;
  // Touch the SimpleGripper config_copy so that it is valid.
  config_proto.mutable_simple_gripper_config();
  intrinsic_proto::icon::GenericLinearGripperConfig& linear_gripper_config =
      *config_proto.mutable_linear_gripper_config();
  linear_gripper_config.set_min_width_m(part_config.min_width);
  linear_gripper_config.set_max_width_m(part_config.max_width);
  linear_gripper_config.set_min_force_newton(part_config.min_force);
  linear_gripper_config.set_max_force_newton(part_config.max_force);
  linear_gripper_config.set_min_speed_meters_per_second(part_config.min_speed);
  linear_gripper_config.set_max_speed_meters_per_second(part_config.max_speed);
  return PartPtrAndGenericConfig{.part_ptr = std::move(part_ptr),
                                 .config = std::move(config_proto)};
}

absl::StatusOr<std::unique_ptr<HalLinearGripperPart>>
HalLinearGripperPart::CreateFromHandles(
    HardwareModuleManager* manager,
    MutableHardwareInterfaceHandle<intrinsic_fbs::GripperCommand> command,
    HardwareInterfaceHandle<intrinsic_fbs::GripperStatus> status,
    const intrinsic_proto::icon::HalLinearGripperPartConfig& part_config) {
  auto part = absl::WrapUnique(new HalLinearGripperPart(manager));

  INTR_ASSIGN_OR_RETURN(
      auto linear_gripper_feature,
      LinearGripperFeature::Create(std::move(status), std::move(command),
                                   part_config));

  INTR_RETURN_IF_ERROR(
      part->RegisterInterface(std::move(linear_gripper_feature)));
  return part;
}

namespace hardware_interface_traits {
INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::GripperStatus,
                                 intrinsic_fbs::BuildGripperStatus,
                                 "intrinsic_fbs.GripperStatus")

INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::GripperCommand,
                                 intrinsic_fbs::BuildGripperCommand,
                                 "intrinsic_fbs.GripperCommand")
}  // namespace hardware_interface_traits

}  // namespace intrinsic::icon
