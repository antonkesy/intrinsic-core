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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_HAL_FORCE_TORQUE_SENSOR_PART_HAL_FORCE_TORQUE_SENSOR_PART_H_
#define INTRINSIC_ICON_CONTROL_PARTS_HAL_FORCE_TORQUE_SENSOR_PART_HAL_FORCE_TORQUE_SENSOR_PART_H_

#include <memory>
#include <string>
#include <variant>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/control/parts/feature_interface_registry.h"
#include "intrinsic/icon/control/parts/feature_interfaces/force_torque_sensor.h"
#include "intrinsic/icon/control/parts/feature_interfaces/standalone_force_torque_sensor.h"
#include "intrinsic/icon/control/parts/hal/force_torque_sensor_part/hal_force_torque_sensor_part_config.pb.h"
#include "intrinsic/icon/control/parts/hal_realtime_part_base.h"
#include "intrinsic/icon/control/parts/payload_property.h"
#include "intrinsic/icon/control/parts/realtime_part_factory_common.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/hardware_module_manager.h"
#include "intrinsic/icon/hal/hardware_module_proxy.h"
#include "intrinsic/icon/hal/interfaces/force_sensor.fbs.h"
#include "intrinsic/icon/hal/interfaces/force_torque.fbs.h"
#include "intrinsic/icon/hal/interfaces/hardware_module_state.fbs.h"
#include "intrinsic/icon/hal/interfaces/joint_state.fbs.h"
#include "intrinsic/icon/utils/fixed_string.h"
#include "intrinsic/icon/utils/realtime_status.h"

namespace intrinsic::icon {

// A force-torque sensor part which makes use of the ForceSensorController to
// provide essential FT-functionality like taring or low-pass filtering.
class HalForceTorqueSensorPart final : public HalRealtimePartBase {
  using ForceTorqueStatusHardwareInterface =
      intrinsic::icon::HardwareInterfaceHandle<
          intrinsic_fbs::ForceTorqueStatus>;
  using JointTorqueStateHardwareInterface =
      intrinsic::icon::HardwareInterfaceHandle<intrinsic_fbs::JointTorqueState>;
  using ForceTorqueCommandHardwareInterface =
      intrinsic::icon::MutableHardwareInterfaceHandle<
          intrinsic_fbs::ForceTorqueCommand>;
  using JointPositionStateHardwareInterface =
      HardwareInterfaceHandle<intrinsic_fbs::JointPositionState>;
  using JointVelocityStateHardwareInterface =
      HardwareInterfaceHandle<intrinsic_fbs::JointVelocityState>;
  using JointAccelerationStateHardwareInterface =
      HardwareInterfaceHandle<intrinsic_fbs::JointAccelerationState>;

 public:
  //  Bundles the PartPropertyIds that make up the force torque sensor part
  //  configuration. The part needs the IDs to read from and write to the part
  //  property values in its ReadStatus()/ApplyCommand() methods.
  struct PartPropertyIds {
    PayloadProperty mounted_payload;
    PayloadProperty grasped_payload;
  };

  static constexpr char kPartTypeName[] = "HalForceTorqueSensorPart";
  static constexpr char kMountedPayloadPropertyName[] = "mounted_payload";
  static constexpr char kGraspedPayloadPropertyName[] = "grasped_payload";

  // Builds a HalForceTorqueSensorPart from the given configuration proto.
  static absl::StatusOr<PartPtrAndGenericConfig> FromProto(
      PartFactoryContext context,
      const intrinsic_proto::icon::HalForceTorqueSensorPartConfig& config);

  HalForceTorqueSensorPart(absl::string_view name,
                           HardwareModuleManager* manager);

  RealtimeStatus ReadStatus(ReadStatusParameters params) override;
  RealtimeStatus ApplyCommand(ApplyCommandParameters params) override;

 private:
  absl::Status RegisterForceTorqueInterface(
      std::variant<std::unique_ptr<ForceTorqueSensorFeature>,
                   std::unique_ptr<StandaloneForceTorqueSensorFeature>>
          feature_interface);

  std::string name_;

  std::variant<std::unique_ptr<ForceTorqueSensorFeature>,
               std::unique_ptr<StandaloneForceTorqueSensorFeature>>
      force_torque_sensor_feature_;

  // IDs needed for access to the part properties from the RT thread.
  PartPropertyIds ft_sensor_part_property_ids_;

  intrinsic_fbs::StateCode state_ = intrinsic_fbs::StateCode::kActivating;

  // Proxy to read the hardware module status from.
  const HardwareModuleProxy* hwm_state_proxy_ = nullptr;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_HAL_FORCE_TORQUE_SENSOR_PART_HAL_FORCE_TORQUE_SENSOR_PART_H_
