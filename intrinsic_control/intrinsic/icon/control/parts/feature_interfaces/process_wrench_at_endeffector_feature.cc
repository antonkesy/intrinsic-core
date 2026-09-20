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

#include "intrinsic/icon/control/parts/feature_interfaces/process_wrench_at_endeffector_feature.h"

#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/math/twist.h"

namespace intrinsic::icon {

/*static*/
absl::StatusOr<ProcessWrenchAtEndeffectorFeature>
ProcessWrenchAtEndeffectorFeature::Create(
    ProcessWrenchAtEndeffectorHardwareInterface
        process_wrench_at_endeffector_hardware_interface) {
  if (*process_wrench_at_endeffector_hardware_interface == nullptr) {
    return absl::InvalidArgumentError(
        "Hardware interface handle for ProcessWrenchAtEndeffectorFeature is "
        "not initialized.");
  }

  return ProcessWrenchAtEndeffectorFeature(
      std::move(process_wrench_at_endeffector_hardware_interface));
}

ProcessWrenchAtEndeffectorFeature::ProcessWrenchAtEndeffectorFeature(
    ProcessWrenchAtEndeffectorHardwareInterface
        process_wrench_at_endeffector_hardware_interface)
    : process_wrench_at_endeffector_hardware_interface_(
          std::move(process_wrench_at_endeffector_hardware_interface)) {}

RealtimeStatus ProcessWrenchAtEndeffectorFeature::Reset() {
  process_wrench_at_tip_.setZero();
  return OkStatus();
}

RealtimeStatus ProcessWrenchAtEndeffectorFeature::ApplyCommand(
    RealtimePartInterface::ApplyCommandParameters params) {
  process_wrench_at_endeffector_hardware_interface_->mutate_x(
      process_wrench_at_tip_(0));
  process_wrench_at_endeffector_hardware_interface_->mutate_y(
      process_wrench_at_tip_(1));
  process_wrench_at_endeffector_hardware_interface_->mutate_z(
      process_wrench_at_tip_(2));
  process_wrench_at_endeffector_hardware_interface_->mutate_rx(
      process_wrench_at_tip_(3));
  process_wrench_at_endeffector_hardware_interface_->mutate_ry(
      process_wrench_at_tip_(4));
  process_wrench_at_endeffector_hardware_interface_->mutate_rz(
      process_wrench_at_tip_(5));

  return OkStatus();
}

void ProcessWrenchAtEndeffectorFeature::SetProcessWrenchAtTip(
    const Wrench& wrench_at_tip) {
  process_wrench_at_tip_ = wrench_at_tip;
  process_wrench_at_endeffector_hardware_interface_.UpdatedAt(Clock::now());
}

}  // namespace intrinsic::icon
