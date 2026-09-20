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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_PROCESS_WRENCH_AT_ENDEFFECTOR_FEATURE_H_
#define INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_PROCESS_WRENCH_AT_ENDEFFECTOR_FEATURE_H_

#include "absl/status/statusor.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interfaces/hal_feature_interface_base.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/flatbuffers/transform_types.fbs.h"
#include "intrinsic/icon/flatbuffers/transform_types.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/math/twist.h"

namespace intrinsic::icon {

class ProcessWrenchAtEndeffectorFeature : public HalFeatureInterfaceBase,
                                          public ProcessWrenchAtEndeffector {
  using ProcessWrenchAtEndeffectorHardwareInterface =
      MutableHardwareInterfaceHandle<intrinsic_fbs::Wrench>;

 public:
  static absl::StatusOr<ProcessWrenchAtEndeffectorFeature> Create(
      ProcessWrenchAtEndeffectorHardwareInterface
          process_wrench_at_endeffector_hardware_interface);

  ProcessWrenchAtEndeffectorFeature(const ProcessWrenchAtEndeffectorFeature&) =
      delete;
  ProcessWrenchAtEndeffectorFeature& operator=(
      const ProcessWrenchAtEndeffectorFeature&) = delete;
  ProcessWrenchAtEndeffectorFeature(ProcessWrenchAtEndeffectorFeature&& other) =
      default;
  ProcessWrenchAtEndeffectorFeature& operator=(
      ProcessWrenchAtEndeffectorFeature&& other) = default;
  ~ProcessWrenchAtEndeffectorFeature() override = default;

  // Resets the currently set process force at the endeffector to zero.
  RealtimeStatus Reset() override;

  // This function uses the wrench from the most recent call to
  // `SetProcessWrenchAtTip()` and writes it to the hardware module.
  RealtimeStatus ApplyCommand(
      RealtimePartInterface::ApplyCommandParameters params) override;

  void SetProcessWrenchAtTip(const Wrench& wrench_at_tip) override;

  Wrench GetProcessWrenchAtTip() const override {
    return process_wrench_at_tip_;
  }

 private:
  explicit ProcessWrenchAtEndeffectorFeature(
      ProcessWrenchAtEndeffectorHardwareInterface
          process_wrench_at_endeffector_hardware_interface);

  ProcessWrenchAtEndeffectorHardwareInterface
      process_wrench_at_endeffector_hardware_interface_;
  Wrench process_wrench_at_tip_ = Wrench::ZERO;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_PROCESS_WRENCH_AT_ENDEFFECTOR_FEATURE_H_
