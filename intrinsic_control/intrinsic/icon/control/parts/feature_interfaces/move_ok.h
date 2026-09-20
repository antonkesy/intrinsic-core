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

#ifndef INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_MOVE_OK_H_
#define INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_MOVE_OK_H_

#include <memory>

#include "absl/status/statusor.h"
#include "intrinsic/icon/control/joint_position_command.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/feature_interfaces/hal_feature_interface_base.h"
#include "intrinsic/icon/control/parts/move_checker.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/hal/hardware_interface_handle.h"
#include "intrinsic/icon/hal/interfaces/joint_command.fbs.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/kinematics/types/joint_limits.h"

namespace intrinsic::icon {

// Implementation of MoveOk.
class MoveOkFeature : public HalFeatureInterfaceBase, public MoveOk {
 public:
  // Creates a MoveOk FeatureInterface. Note that this takes a `JointPosition`
  // (FeatureInterface) pointer, rather than a hardware interface reference.
  //
  // This is to ensure compatibility with "wrapper" feature interfaces that
  // provide a joint position API over a hardware interface that might not (like
  // `JointPositionPidTorqueControllerFeature`).
  static absl::StatusOr<MoveOkFeature> Create(
      double control_frequency_hz, const JointLimits& application_limits,
      const JointLimits& system_limits,
      const JointPosition* joint_position_command_feature_interface);

  RealtimeStatus ReadStatus(
      RealtimePartInterface::ReadStatusParameters params) override;

  bool IsMoveOkWithPartLimits(const JointPositionCommand& setpoint) override;

  bool IsMoveOkWithUserLimits(const JointPositionCommand& setpoint,
                              const JointLimits& joint_limits) override;

  // Set both the position setpoints and the previous position setpoints.
  RealtimeStatus UpdatePreviousPositionSetpoints(
      const JointPositionCommand& setpoints);

  // Set the System and Application Joint limits used by the feature interface.
  RealtimeStatus UpdateJointLimits(const JointLimits& application_limits,
                                   const JointLimits& system_limits);

 private:
  explicit MoveOkFeature(
      std::unique_ptr<MoveChecker> move_checker,
      const JointLimits& application_limits, const JointLimits& system_limits,
      const JointPosition* joint_position_command_feature_interface);

  const JointPosition* joint_position_command_feature_interface_;
  std::unique_ptr<MoveChecker> move_checker_;
  JointLimits application_limits_;
  JointLimits system_limits_;
};

}  // namespace intrinsic::icon
#endif  // INTRINSIC_ICON_CONTROL_PARTS_FEATURE_INTERFACES_MOVE_OK_H_
