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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_HAL_ARM_PART_HAL_ARM_PART_H_
#define INTRINSIC_ICON_CONTROL_PARTS_HAL_ARM_PART_HAL_ARM_PART_H_

#include <memory>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/control/parts/feature_interface_registry.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/hal/arm_part/hal_arm_part_config.pb.h"
#include "intrinsic/icon/control/parts/hal_realtime_part_base.h"
#include "intrinsic/icon/control/parts/new_manipulator_kinematics.h"
#include "intrinsic/icon/control/parts/payload_property.h"
#include "intrinsic/icon/control/parts/realtime_part_factory_common.h"
#include "intrinsic/icon/dynamics/rigid_body_interface.h"
#include "intrinsic/icon/hal/hardware_module_manager.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

class HalArmPart final : public HalRealtimePartBase {
 public:
  // All part property ids for the arm part.
  struct PartPropertyIds {
    PayloadProperty full_payload;
  };

  static constexpr char kPartTypeName[] = "HalArmPart";
  static constexpr char kFullPayloadPropertyName[] = "full_payload";

  // Builds an arm specific part for a hardware module.
  //
  // The actual proto configuration contains information about which hardware
  // interfaces the part should claim.
  // See `hal_part_config.proto` for details on how to specify these hardware
  // interface values.
  static absl::StatusOr<PartPtrAndGenericConfig> FromProto(
      PartFactoryContext context,
      const intrinsic_proto::icon::HalArmPartConfig& config);

  HalArmPart(absl::string_view name, HardwareModuleManager* manager);

 private:
  // Helper class to allow HalArmPart to optionally provide Dynamics.
  class DynamicsImpl final : public Dynamics {
   public:
    explicit DynamicsImpl(
        std::unique_ptr<RigidBodyInterface> dynamics_interface)
        : dynamics_interface_(std::move(dynamics_interface)) {}

    RigidBodyInterface& GetRigidBodyInterface() override {
      return *dynamics_interface_;
    }

   private:
    std::unique_ptr<RigidBodyInterface> dynamics_interface_;
  };

  absl::Status RegisterManipulatorInterface(
      std::unique_ptr<NewManipulatorKinematicsImpl> feature_interface) {
    std::unique_ptr<NewManipulatorKinematicsImpl> feature_interface_ptr =
        std::move(feature_interface);
    INTR_RETURN_IF_ERROR(interface_registry_.RegisterAsCompatibleInterfaces(
        feature_interface_ptr.get()));
    manipulator_kinematics_ = std::move(feature_interface_ptr);
    return absl::OkStatus();
  }

  absl::Status RegisterDynamicsInterface(
      std::unique_ptr<DynamicsImpl> feature_interface) {
    std::unique_ptr<DynamicsImpl> feature_interface_ptr =
        std::move(feature_interface);
    INTR_RETURN_IF_ERROR(interface_registry_.RegisterAsCompatibleInterfaces(
        feature_interface_ptr.get()));
    dynamics_ = std::move(feature_interface_ptr);
    return absl::OkStatus();
  }

  const std::string name_;  // part name
  // The part implementation does not care about which feature interfaces it
  // registers exactly, but they must all inherit from HalFeatureInterfaceBase,
  // so we can call ReadStatus() and ApplyCommand() on them.
  std::unique_ptr<NewManipulatorKinematicsImpl> manipulator_kinematics_;
  std::unique_ptr<DynamicsImpl> dynamics_ = nullptr;

  PartPropertyIds part_property_ids_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_HAL_ARM_PART_HAL_ARM_PART_H_
