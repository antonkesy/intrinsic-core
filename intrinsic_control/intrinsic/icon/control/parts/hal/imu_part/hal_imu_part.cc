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

#include "intrinsic/icon/control/parts/hal/imu_part/hal_imu_part.h"

#include <memory>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/control/parts/feature_interfaces/imu_state.h"
#include "intrinsic/icon/control/parts/hal/imu_part/hal_imu_part_config.pb.h"
#include "intrinsic/icon/control/parts/hal/v1/hal_part_config.pb.h"
#include "intrinsic/icon/control/parts/hal_realtime_part_base.h"
#include "intrinsic/icon/control/parts/realtime_part_factory_common.h"
#include "intrinsic/icon/hal/default_hardware_interfaces.h"  // IWYU pragma: keep
#include "intrinsic/icon/hal/hardware_module_manager.h"
#include "intrinsic/icon/hal/interfaces/imu.fbs.h"
#include "intrinsic/icon/proto/generic_part_config.pb.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

HalImuPart::HalImuPart(absl::string_view name, HardwareModuleManager* manager)
    : HalRealtimePartBase(manager) {}

absl::StatusOr<PartPtrAndGenericConfig> HalImuPart::FromProto(
    PartFactoryContext context,
    const intrinsic_proto::icon::HalImuPartConfig& config) {
  if (!config.has_interface()) {
    return absl::FailedPreconditionError(
        "Config is missing hardware module interface.");
  }

  auto part = std::make_unique<HalImuPart>(
      context.part_name, context.context.GetHardwareModuleManager());
  const auto& module_name = config.interface().module_name();
  const auto& interface_name = config.interface().interface_name();
  INTR_ASSIGN_OR_RETURN(
      auto imu_handle,
      context.context.GetHardwareInterfaceHandle<intrinsic_fbs::ImuStatus>(
          module_name, interface_name));

  INTR_ASSIGN_OR_RETURN(Pose3d flange_t_sensor,
                        intrinsic_proto::FromProto(config.flange_t_sensor()));
  INTR_ASSIGN_OR_RETURN(auto feature_interface,
                        InertialMeasurementUnitStateFeature::Create(
                            std::move(imu_handle), flange_t_sensor));

  INTR_RETURN_IF_ERROR(part->RegisterInterface(std::move(feature_interface)));
  INTR_RETURN_IF_ERROR(
      part->AddHardwareModule(config.interface().module_name()));

  intrinsic_proto::icon::GenericPartConfig generic_config =
      ExtractGenericConfig(part->GetFeatureInterfaces());

  return PartPtrAndGenericConfig{.part_ptr = std::move(part),
                                 .config = std::move(generic_config)};
}

}  // namespace intrinsic::icon
