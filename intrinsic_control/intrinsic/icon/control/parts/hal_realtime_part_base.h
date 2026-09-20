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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_HAL_REALTIME_PART_BASE_H_
#define INTRINSIC_ICON_CONTROL_PARTS_HAL_REALTIME_PART_BASE_H_

#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "absl/base/nullability.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/cc_client/operational_status.h"
#include "intrinsic/icon/control/parts/feature_interface_registry.h"
#include "intrinsic/icon/control/parts/feature_interfaces/hal_feature_interface_base.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/hal/hardware_module_manager.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {

// This class implements the RealtimePartInterface interface functions with
// typical functionality of most real-time parts. This avoids code duplication
// and keeps the actual implementations clean.
class HalRealtimePartBase : public RealtimePartInterface {
 public:
  explicit HalRealtimePartBase(
      const HardwareModuleManager* absl_nonnull hardware_module_manager)
      : hardware_module_manager_(hardware_module_manager) {}
  ~HalRealtimePartBase() override = default;

  // Adds a hardware module to the set of used hardware modules. All used
  // hardware modules *must* be added using this function, otherwise
  // `GetOperationalState()` won't return a correct state.
  absl::Status AddHardwareModule(absl::string_view hardware_module_name);

  HardwareGroupSet GetHardwareDependencies() const override {
    return depends_on_hardware_;
  }

  // Computes the operational state from all associated hardware modules.
  RealtimeStatusOr<RealtimeOperationalStatus> GetOperationalStatus()
      const override;

  FeatureInterfaceRegistry& GetFeatureInterfaces() override;
  const FeatureInterfaceRegistry& GetFeatureInterfaces() const override;

  RealtimeStatus ReadStatus(ReadStatusParameters params) override;
  RealtimeStatus ApplyCommand(ApplyCommandParameters params) override;

  // Registers `feature_interface` to be exported by the part and be used by an
  // action.
  // Takes ownership of `feature_interface` and exports its interface via the
  // registry.
  template <typename FeatureInterfaceImplT,
            typename = std::enable_if_t<std::is_base_of_v<
                HalFeatureInterfaceBase, FeatureInterfaceImplT>>>
  absl::Status RegisterInterface(FeatureInterfaceImplT&& feature_interface) {
    auto feature_interface_ptr = std::make_unique<FeatureInterfaceImplT>(
        std::forward<FeatureInterfaceImplT>(feature_interface));
    INTR_RETURN_IF_ERROR(interface_registry_.RegisterAsCompatibleInterfaces(
        feature_interface_ptr.get()));
    feature_interfaces_.emplace_back(std::move(feature_interface_ptr));

    return absl::OkStatus();
  }

 protected:
  std::vector<std::unique_ptr<HalFeatureInterfaceBase>> feature_interfaces_;
  FeatureInterfaceRegistry interface_registry_;
  const HardwareModuleManager* absl_nonnull hardware_module_manager_;
  OperationalState previous_operational_state_ = OperationalState::kDisabled;

 private:
  // A set of names of the hardware modules used by this part.
  absl::flat_hash_set<std::string> hardware_module_names_;
  HardwareGroupSet depends_on_hardware_;
};
}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_HAL_REALTIME_PART_BASE_H_
