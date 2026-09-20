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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_CONTROL_MODE_STATE_H_
#define INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_CONTROL_MODE_STATE_H_

#include "absl/status/statusor.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interfaces/hal_feature_interface_base.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/interfaces/control_mode.fbs.h"

namespace intrinsic::icon {

class ControlModeStateFeature : public HalFeatureInterfaceBase,
                                public ControlModeExporter {
  using ControlModeStateHardwareInterface =
      HardwareInterfaceHandle<::intrinsic_fbs::ControlModeStatus>;

 public:
  static absl::StatusOr<ControlModeStateFeature> Create(
      ControlModeStateHardwareInterface control_mode_state_hardware_interface);

  ControlModeStateFeature(const ControlModeStateFeature&) = delete;
  ControlModeStateFeature& operator=(const ControlModeStateFeature&) = delete;
  ControlModeStateFeature(ControlModeStateFeature&& other) = default;
  ControlModeStateFeature& operator=(ControlModeStateFeature&& other) = default;
  ~ControlModeStateFeature() override = default;

  ControlModeExporter::ControlMode GetCurrentControlMode() const override;

 private:
  explicit ControlModeStateFeature(
      ControlModeStateHardwareInterface control_mode_state_hardware_interface);

  ControlModeStateHardwareInterface control_mode_state_hardware_interface_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_CONTROL_MODE_STATE_H_
